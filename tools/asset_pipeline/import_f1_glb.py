"""Convert the "Ferrari F1 2019" glTF model (Sketchfab, valvetin, CC-BY-4.0) into Godot-ready parts.

Source: FerrariF1/ferrari_f1_2019.glb. Named nodes: "Tire FL/F../RL/RL.001" and "Rim ..." per wheel, "Steering Wheel"
plus its buttons and label decals, tyre-sidewall decals "Decal Tire FL.00x" at each hub, and the body parts.

Output frame (Godot, same as the Mini): car faces -Z, left = -X, metres, origin on the ground directly below the CG.
There is no MATLAB model of an F1 car; the CG sits CG_TO_REAR_SHARE × wheelbase ahead of the rear axle (provisional
front-axle weight share 0.45, VehicleParameters.F1Provisional). Scale: the overall width becomes 2.000 m, the 2019
maximum that F1 cars are built to (FIA Technical Regulations art. 3). Scaling instead to the 660 mm 2019 tyre diameter
gave a 2.16 m wide, 3.93 m wheelbase car (the model's tyres are ~7 % undersized); the width scale gives a 3.64 m
wheelbase and ~5.6 m length, matching the real cars. The resulting dimensions are reported and the profile takes the
wheelbase and tracks from the metadata. The physics keeps the 0.33 m regulation rolling radius.

Written: f1.glb with nodes Body (static parts), WheelFL/FR/RL/RR (at the hub) -> Spin -> Mesh (tyre, rim and sidewall
decals relative to the hub, so they can spin about X and the fronts can steer about Y), SteeringWheel (at its centre)
-> Mesh. Node transforms are baked into the vertices (mirrored source nodes get their winding and tangent sign
flipped); primitives of one part are merged per material. Materials, textures and images are copied unchanged.
f1_meta.json: scale, hubs, wheelbase, tracks, tyre radii, AABBs, steering-wheel centre/axis, camera eyes, exhaust.

Usage: python3 -I tools/asset_pipeline/import_f1_glb.py <in.glb> <out_dir>
"""
import json
import os
import struct
import sys
from collections import defaultdict

import numpy as np

OVERALL_WIDTH = 2.000          # m, 2019 F1 maximum overall width (cars are built to it)
CG_TO_REAR_SHARE = 0.45        # CG ahead of the rear axle as a share of the wheelbase (front-axle weight share)
WHEELS = ("fl", "fr", "rl", "rr")

COMPONENT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


# --- glTF reading ---------------------------------------------------------------------------------------------------
def load_glb(path):
    data = open(path, "rb").read()
    magic, _, _ = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF":
        raise ValueError("not a GLB file")
    off, gltf, binary = 12, None, b""
    while off < len(data):
        length, kind = struct.unpack_from("<I4s", data, off)
        chunk = data[off + 8: off + 8 + length]
        if kind == b"JSON":
            gltf = json.loads(chunk)
        elif kind == b"BIN\x00":
            binary = chunk
        off += 8 + length
    return gltf, binary


def read_accessor(g, binary, index):
    a = g["accessors"][index]
    dtype = COMPONENT[a["componentType"]]
    n, w = a["count"], WIDTH[a["type"]]
    if "bufferView" not in a:
        return np.zeros((n, w), dtype=np.float32)
    bv = g["bufferViews"][a["bufferView"]]
    start = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
    item = np.dtype(dtype).itemsize * w
    stride = bv.get("byteStride", item)
    if stride == item:
        arr = np.frombuffer(binary, dtype=dtype, count=n * w, offset=start).reshape(n, w)
    else:
        raw = np.frombuffer(binary, dtype=np.uint8, count=stride * (n - 1) + item, offset=start)
        arr = np.lib.stride_tricks.as_strided(raw, shape=(n, item), strides=(stride, 1)).copy().view(dtype).reshape(n, w)
    arr = arr.astype(np.float64 if dtype == np.float32 else arr.dtype)
    if a.get("normalized") and dtype != np.float32:
        arr = arr.astype(np.float64) / np.iinfo(dtype).max
    return arr


def local_matrix(node):
    if "matrix" in node:
        return np.array(node["matrix"], dtype=np.float64).reshape(4, 4).T
    t = np.array(node.get("translation", [0, 0, 0]), dtype=np.float64)
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    s = np.diag(node.get("scale", [1, 1, 1]))
    m = np.eye(4)
    m[:3, :3] = r @ s
    m[:3, 3] = t
    return m


def world_matrices(g):
    out = {}

    def walk(i, parent):
        m = parent @ local_matrix(g["nodes"][i])
        out[i] = m
        for c in g["nodes"][i].get("children", []):
            walk(c, m)

    for root in g["scenes"][g.get("scene", 0)]["nodes"]:
        walk(root, np.eye(4))
    return out


class Part:
    """Triangles of one output part, grouped by (material, attribute set)."""

    def __init__(self):
        self.groups = defaultdict(lambda: defaultdict(list))

    def add(self, material, attrs, indices):
        key = (material, tuple(sorted(attrs)))
        grp = self.groups[key]
        base = sum(len(p) for p in grp["POSITION"])
        for k, v in attrs.items():
            grp[k].append(v)
        grp["_indices"].append(indices + base)

    def points(self):
        return np.vstack([np.vstack(g["POSITION"]) for g in self.groups.values()])


def mesh_primitives(g, binary, mesh_index, world):
    """Primitives of a mesh with positions/normals/tangents in world space (winding fixed for mirrored nodes)."""
    lin = world[:3, :3]
    det = np.linalg.det(lin)
    normal_m = np.linalg.inv(lin).T
    for prim in g["meshes"][mesh_index]["primitives"]:
        if prim.get("mode", 4) != 4:
            continue
        attrs = {}
        for name, acc in prim["attributes"].items():
            v = read_accessor(g, binary, acc)
            if name == "POSITION":
                v = v @ lin.T + world[:3, 3]
            elif name == "NORMAL":
                v = v @ normal_m.T
                v /= np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-12)
            elif name == "TANGENT":
                xyz = v[:, :3] @ lin.T
                xyz /= np.maximum(np.linalg.norm(xyz, axis=1, keepdims=True), 1e-12)
                v = np.hstack([xyz, v[:, 3:4] * (1 if det > 0 else -1)])
            attrs[name] = v
        if "indices" in prim:
            idx = read_accessor(g, binary, prim["indices"]).reshape(-1).astype(np.int64)
        else:
            idx = np.arange(len(attrs["POSITION"]), dtype=np.int64)
        tri = idx.reshape(-1, 3)
        if det < 0:
            tri = tri[:, [0, 2, 1]]
        yield prim.get("material"), attrs, tri.reshape(-1)


# --- glTF writing ---------------------------------------------------------------------------------------------------
class GlbOut:
    def __init__(self, src, src_bin):
        self.src, self.src_bin = src, src_bin
        self.bin = bytearray()
        self.g = {"asset": {"version": "2.0", "generator": "import_f1_glb.py",
                            "extras": src.get("asset", {}).get("extras", {})},
                  "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [], "meshes": [], "accessors": [],
                  "bufferViews": [], "buffers": []}
        for key in ("materials", "textures", "samplers"):
            if key in src:
                self.g[key] = json.loads(json.dumps(src[key]))
        self._fix_alpha_modes()
        self.g["images"] = []
        for img in src.get("images", []):
            img = dict(img)
            if "bufferView" in img:
                bv = src["bufferViews"][img["bufferView"]]
                o = bv.get("byteOffset", 0)
                img["bufferView"] = self._view(src_bin[o: o + bv["byteLength"]], None)
            self.g["images"].append(img)

    def _image_has_alpha(self, texture_index):
        from io import BytesIO
        from PIL import Image
        img = self.src["images"][self.src["textures"][texture_index]["source"]]
        bv = self.src["bufferViews"][img["bufferView"]]
        o = bv.get("byteOffset", 0)
        im = Image.open(BytesIO(self.src_bin[o: o + bv["byteLength"]]))
        if im.mode in ("RGBA", "LA") or (im.mode == "P" and "transparency" in im.info):
            lo, _ = im.convert("RGBA").getchannel("A").getextrema()
            return lo < 250
        return False

    def _fix_alpha_modes(self):
        """The Sketchfab export marks every material BLEND. A material whose base-colour texture has no alpha becomes
        OPAQUE (Godot would otherwise render the whole car as transparent: no SSR/SSAO, sorting artefacts); one with
        alpha (the decals) becomes MASK, which needs no sorting against the body it sits on."""
        for m in self.g.get("materials", []):
            tex = m.get("pbrMetallicRoughness", {}).get("baseColorTexture", {}).get("index")
            if m.get("alphaMode") != "BLEND" or tex is None:
                continue
            if self._image_has_alpha(tex):
                m["alphaMode"] = "MASK"
                m["alphaCutoff"] = 0.5
            else:
                m["alphaMode"] = "OPAQUE"
            print("material", m.get("name"), "->", m["alphaMode"])

    def _view(self, payload, target):
        while len(self.bin) % 4:
            self.bin.append(0)
        view = {"buffer": 0, "byteOffset": len(self.bin), "byteLength": len(payload)}
        if target:
            view["target"] = target
        self.bin.extend(payload)
        self.g["bufferViews"].append(view)
        return len(self.g["bufferViews"]) - 1

    def _accessor(self, arr, kind, comp, target, minmax=False):
        view = self._view(np.ascontiguousarray(arr).tobytes(), target)
        acc = {"bufferView": view, "componentType": comp, "count": int(arr.shape[0]), "type": kind}
        if minmax:
            acc["min"] = arr.min(0).tolist()
            acc["max"] = arr.max(0).tolist()
        self.g["accessors"].append(acc)
        return len(self.g["accessors"]) - 1

    def mesh(self, name, part, offset):
        prims = []
        for (material, keys), grp in sorted(part.groups.items(), key=lambda kv: (kv[0][0] if kv[0][0] is not None else -1, kv[0][1])):
            attrs = {}
            for k in keys:
                v = np.vstack(grp[k])
                if k == "POSITION":
                    v = v - offset
                v = v.astype(np.float32)
                kind = {1: "SCALAR", 2: "VEC2", 3: "VEC3", 4: "VEC4"}[v.shape[1]]
                attrs[k] = self._accessor(v, kind, 5126, 34962, minmax=(k == "POSITION"))
            idx = np.concatenate(grp["_indices"]).astype(np.uint32)
            prim = {"attributes": attrs, "indices": self._accessor(idx.reshape(-1, 1), "SCALAR", 5125, 34963)}
            if material is not None:
                prim["material"] = material
            prims.append(prim)
        self.g["meshes"].append({"name": name, "primitives": prims})
        return len(self.g["meshes"]) - 1

    def node(self, name, translation=None, mesh=None, children=None):
        n = {"name": name}
        if translation is not None:
            n["translation"] = [float(x) for x in translation]
        if mesh is not None:
            n["mesh"] = mesh
        if children:
            n["children"] = children
        self.g["nodes"].append(n)
        return len(self.g["nodes"]) - 1

    def save(self, path):
        while len(self.bin) % 4:
            self.bin.append(0)
        self.g["buffers"] = [{"byteLength": len(self.bin)}]
        js = json.dumps(self.g, separators=(",", ":")).encode()
        js += b" " * ((4 - len(js) % 4) % 4)
        with open(path, "wb") as f:
            f.write(struct.pack("<4sII", b"glTF", 2, 12 + 8 + len(js) + 8 + len(self.bin)))
            f.write(struct.pack("<I4s", len(js), b"JSON") + js)
            f.write(struct.pack("<I4s", len(self.bin), b"BIN\x00") + bytes(self.bin))


# --- conversion -----------------------------------------------------------------------------------------------------
def wheel_key(name):
    """'Tire FL' / 'Rim RL.001' -> fl/fr/rl/rr. The source names the right-hand wheels 'F??' (garbled) and 'RL.001'."""
    n = name.lower()
    if not (n.startswith("tire ") or n.startswith("rim ")):
        return None
    tag = n.split(" ", 1)[1]
    if tag == "fl":
        return "fl"
    if tag.startswith("f"):
        return "fr"
    if tag == "rl":
        return "rl"
    if tag.startswith("rl."):
        return "rr"
    return None


def main(src_path, out_dir):
    g, binary = load_glb(src_path)
    world = world_matrices(g)
    mesh_nodes = [(i, g["nodes"][i]) for i in world if "mesh" in g["nodes"][i]]

    def parent_name(i):
        for j, n in enumerate(g["nodes"]):
            if i in n.get("children", []):
                return n.get("name", "")
        return ""

    # Hubs in source world space: origin of each "Tire XX" node.
    hubs_src = {}
    for i, n in enumerate(g["nodes"]):
        k = wheel_key(n.get("name", ""))
        if k and n["name"].lower().startswith("tire") and i in world:
            hubs_src[k] = world[i][:3, 3].copy()
    if sorted(hubs_src) != sorted(WHEELS):
        raise RuntimeError(f"wheel nodes not found: {sorted(hubs_src)}")

    # Car frame from the hubs: forward = rear axle -> front axle, left = right -> left wheel, up = forward x left.
    front = (hubs_src["fl"] + hubs_src["fr"]) / 2
    rear = (hubs_src["rl"] + hubs_src["rr"]) / 2
    fwd = front - rear
    left = ((hubs_src["fl"] - hubs_src["fr"]) + (hubs_src["rl"] - hubs_src["rr"])) / 2
    up = np.cross(fwd, left)
    left = np.cross(up, fwd)
    fwd, left, up = (v / np.linalg.norm(v) for v in (fwd, left, up))
    # Godot: forward = -Z, left = -X, up = +Y.
    rot = np.vstack([-left, up, -fwd])      # rows map source vectors onto Godot axes

    # Collect geometry per part in the rotated (still unscaled, unshifted) frame.
    parts = {"body": Part(), "steering": Part(), **{w: Part() for w in WHEELS}}
    raw = []
    for i, node in mesh_nodes:
        m = np.eye(4)
        m[:3, :3] = rot
        w = m @ world[i]
        name = parent_name(i) or node.get("name", "")
        prims = list(mesh_primitives(g, binary, node["mesh"], w))
        pts = np.vstack([a["POSITION"] for _, a, _ in prims])
        raw.append((name, prims, pts))

    hubs = {k: rot @ v for k, v in hubs_src.items()}
    sw_node = next(p for p in raw if p[0] == "Steering Wheel")
    sw_centre_raw = (sw_node[2].min(0) + sw_node[2].max(0)) / 2
    sw_radius_raw = np.linalg.norm(sw_node[2].max(0) - sw_node[2].min(0)) / 2

    # Scale from the overall width (x extent of everything).
    width_raw = np.ptp(np.vstack([p[2] for p in raw])[:, 0])
    scale = OVERALL_WIDTH / width_raw

    for name, prims, pts in raw:
        k = wheel_key(name)
        centre = (pts.min(0) + pts.max(0)) / 2
        if k is None and name.lower().startswith("decal tire"):
            k = min(WHEELS, key=lambda w: np.linalg.norm(hubs[w] - centre))
        if k is None and np.linalg.norm(centre - sw_centre_raw) < 1.2 * sw_radius_raw and np.ptp(pts, axis=0).max() < 2.2 * sw_radius_raw:
            k = "steering"
        target = parts[k or "body"]
        for material, attrs, idx in prims:
            target.add(material, attrs, idx)

    # Metric frame: scale, ground at the tyres' lowest point, x centred on the axles, z: CG = origin.
    def metric(p):
        return p * scale

    for part in parts.values():
        for grp in part.groups.values():
            grp["POSITION"] = [metric(v) for v in grp["POSITION"]]
    hubs = {k: metric(v) for k, v in hubs.items()}
    ground = min(parts[w].points()[:, 1].min() for w in WHEELS)
    wheelbase = float(np.linalg.norm(((hubs["fl"] + hubs["fr"]) - (hubs["rl"] + hubs["rr"])) / 2))
    rear_z = (hubs["rl"][2] + hubs["rr"][2]) / 2
    cg_z = rear_z - CG_TO_REAR_SHARE * wheelbase              # forward is -Z
    centre_x = np.mean([h[0] for h in hubs.values()])
    shift = np.array([centre_x, ground, cg_z])
    for part in parts.values():
        for grp in part.groups.values():
            grp["POSITION"] = [v - shift for v in grp["POSITION"]]
    hubs = {k: v - shift for k, v in hubs.items()}

    body_pts = parts["body"].points()
    all_pts = np.vstack([p.points() for p in parts.values()])
    sw_pts = parts["steering"].points()
    sw_centre = (sw_pts.min(0) + sw_pts.max(0)) / 2
    # Steering-column axis: an F1 wheel is wider than tall and its grips/paddles give it depth, so its thinnest principal
    # direction is not the column. ASSUMPTION: column 15 deg above horizontal, pointing back at the driver (+Z).
    axis = np.array([0.0, np.sin(np.radians(15)), np.cos(np.radians(15))])
    # Cameras: driver's eye behind and above the steering wheel; T-cam above the airbox (highest body point).
    eye = sw_centre + np.array([0.0, 0.22, 0.48])
    top = body_pts[np.argmax(body_pts[:, 1])]
    tcam = np.array([0.0, top[1] + 0.12, top[2] - 0.05])
    tyre_r = {w: float(np.ptp(parts[w].points()[:, 1]) / 2) for w in WHEELS}

    out = GlbOut(g, binary)
    children = [out.node("Body", mesh=out.mesh("Body", parts["body"], np.zeros(3)))]
    for w in WHEELS:
        mesh = out.mesh(f"Wheel{w.upper()}", parts[w], hubs[w])
        inner = out.node("Mesh", mesh=mesh)
        spin = out.node("Spin", children=[inner])
        children.append(out.node(f"Wheel{w.upper()}", translation=hubs[w], children=[spin]))
    sw_mesh = out.mesh("SteeringWheel", parts["steering"], sw_centre)
    children.append(out.node("SteeringWheel", translation=sw_centre, children=[out.node("Mesh", mesh=sw_mesh)]))
    root = {"name": "F1", "children": children}
    out.g["nodes"].insert(0, root)
    # Inserting the root at 0 shifts every other node index by one.
    for n in out.g["nodes"]:
        if "children" in n:
            n["children"] = [c + 1 for c in n["children"]]
    os.makedirs(out_dir, exist_ok=True)
    out.save(os.path.join(out_dir, "f1.glb"))

    def r4(v):
        return [round(float(x), 4) for x in v]

    meta = {
        "source": os.path.relpath(src_path),
        "credit": "Ferrari F1 2019 by valvetin (https://sketchfab.com/valvetin), CC-BY-4.0, "
                  "https://sketchfab.com/3d-models/ferrari-f1-2019-1b050bffe4e749b586b4782ee7ff4fd0",
        "frame": "Godot: car faces -Z, left = -X, metres, origin on the ground below the CG "
                 f"({CG_TO_REAR_SHARE} x wheelbase ahead of the rear axle)",
        "scale_basis": f"overall width {OVERALL_WIDTH} m (2019 F1 maximum)",
        "scale_to_metres_from_source_world": scale,
        "wheelbase_m": wheelbase,
        "cg_to_rear_m": CG_TO_REAR_SHARE * wheelbase,
        "track_front_m": float(abs(hubs["fl"][0] - hubs["fr"][0])),
        "track_rear_m": float(abs(hubs["rl"][0] - hubs["rr"][0])),
        "tyre_radius_m": tyre_r,
        "hubs": {k: r4(v) for k, v in hubs.items()},
        "body_aabb_min": r4(body_pts.min(0)), "body_aabb_max": r4(body_pts.max(0)),
        "overall_aabb_min": r4(all_pts.min(0)), "overall_aabb_max": r4(all_pts.max(0)),
        "steering_wheel": {"centre": r4(sw_centre), "axis": r4(axis)},
        "cockpit_eye": r4(eye),
        "tcam_eye": r4(tcam),
        "triangles": {k: int(sum(len(np.concatenate(gr["_indices"])) // 3 for gr in p.groups.values())) for k, p in parts.items()},
    }
    with open(os.path.join(out_dir, "f1_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print("length", round(float(np.ptp(all_pts[:, 2])), 3), "height", round(float(all_pts[:, 1].max()), 3),
          "tyre diameter", round(2 * tyre_r["fl"], 3))
    print(json.dumps({k: meta[k] for k in ("scale_to_metres_from_source_world", "wheelbase_m", "track_front_m",
                                           "track_rear_m", "tyre_radius_m", "overall_aabb_min", "overall_aabb_max",
                                           "steering_wheel", "cockpit_eye", "tcam_eye", "triangles")}, indent=1))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
