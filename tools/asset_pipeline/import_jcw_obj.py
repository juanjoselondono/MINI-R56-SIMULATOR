"""Convert the "2011 Mini John Cooper Works (R56) GP" OBJ package into Godot-ready parts.

Source frame (as delivered): front = -X, up = +Y, left = +Z (right-handed; the steering wheel is on +Z, i.e. LHD),
arbitrary units, tyres touching Y = 0.
Output frame (Godot): car faces -Z, left = -X, metres, origin on the ground directly below the CG, which model.m places
car.c = 1.529 m ahead of the rear axle. The scale makes the model's wheelbase equal model.m's car.L = 2.467 m, so the
visual axles sit exactly where the physics puts them.

Parts: body.obj (everything static), wheel_{fl,fr,rl,rr}.obj (tyre + rim + disc, relative to the hub, so they can
spin and steer), steering_wheel.obj (relative to its centre; rotation axis in the metadata). A cleaned MTL is written:
map_d / map_Ka are dropped (the source maps alpha from every diffuse texture, which would make the whole car
transparent); only the glass keeps d < 1.

Usage: python3 -I tools/asset_pipeline/import_jcw_obj.py <package_dir> <out_dir>
"""
import json
import os
import shutil
import sys
from collections import OrderedDict

import numpy as np

WHEELBASE = 2.467      # model.m car.L
CG_TO_REAR = 1.529     # model.m car.c
GLASS_MATERIAL = "Doorrfok0051Mtl"
TYRE_GROUP_PREFIX = "Wheel"
STEERING_GROUP = "Movsteer101"


def parse_obj(path):
    v, vt, vn = [], [], []
    faces = OrderedDict()           # (group, material) -> list of [(vi, ti, ni) x3]
    group, mtl = "default", None
    with open(path, "r", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line or line[0] == "#":
                continue
            tag, _, rest = line.partition(" ")
            if tag == "v":
                v.append([float(x) for x in rest.split()[:3]])
            elif tag == "vt":
                vt.append([float(x) for x in rest.split()[:2]])
            elif tag == "vn":
                vn.append([float(x) for x in rest.split()[:3]])
            elif tag in ("g", "o"):
                group = rest.strip()
            elif tag == "usemtl":
                mtl = rest.strip()
            elif tag == "f":
                corners = []
                for tok in rest.split():
                    a = (tok.split("/") + ["", ""])[:3]
                    corners.append(tuple(int(x) - 1 if x else -1 for x in a))
                for k in range(1, len(corners) - 1):          # fan-triangulate (source is all triangles)
                    faces.setdefault((group, mtl), []).append([corners[0], corners[k], corners[k + 1]])
    return np.array(v), np.array(vt), np.array(vn), faces


def parse_mtl(path):
    mats = OrderedDict()
    cur = None
    with open(path, "r", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if line.startswith("newmtl"):
                cur = line.split(None, 1)[1]
                mats[cur] = {}
            elif cur and line and not line.startswith("#"):
                key, _, val = line.partition(" ")
                mats[cur][key] = val.strip()
    return mats


def group_points(V, tris):
    return V[np.array([c[0] for t in tris for c in t])]


def main(pkg, out):
    src = os.path.join(pkg, "source", "Mini JCW GP")
    V, VT, VN, faces = parse_obj(os.path.join(src, "Mini JCW GP.obj"))
    mats = parse_mtl(os.path.join(src, "Mini JCW GP.mtl"))

    # --- wheels: cluster the Wheel* groups by position into 4 hubs -------------------------------------------------
    wheel_keys = [k for k in faces if k[0].startswith(TYRE_GROUP_PREFIX)]
    tyre_keys = [k for k in wheel_keys if k[1] == "Wheel1Mtl"]          # the tyre gives the true rolling radius
    hubs_src = {}
    for k in tyre_keys:
        P = group_points(V, faces[k])
        centre = (P.min(0) + P.max(0)) / 2
        front = centre[0] < -20          # front axle is at more negative X
        left = centre[2] > -14           # left side is +Z
        name = ("f" if front else "r") + ("l" if left else "r")
        hubs_src[name] = {"centre": centre, "diameter": float(P[:, 1].max() - P[:, 1].min()),
                          "width": float(P[:, 2].max() - P[:, 2].min())}
    assert sorted(hubs_src) == ["fl", "fr", "rl", "rr"], hubs_src.keys()
    wheel_of = {}
    for k in wheel_keys:
        c = group_points(V, faces[k]).mean(0)
        wheel_of[k] = min(hubs_src, key=lambda n: np.linalg.norm(hubs_src[n]["centre"][[0, 2]] - c[[0, 2]]))

    front_x = (hubs_src["fl"]["centre"][0] + hubs_src["fr"]["centre"][0]) / 2
    rear_x = (hubs_src["rl"]["centre"][0] + hubs_src["rr"]["centre"][0]) / 2
    mid_z = np.mean([h["centre"][2] for h in hubs_src.values()])
    wheelbase_src = rear_x - front_x
    s = WHEELBASE / wheelbase_src
    # CG along the car axis: c ahead of the rear axle (towards -X in the source frame).
    cg_x = rear_x - CG_TO_REAR / s

    def to_godot(P):
        P = np.asarray(P, float)
        x, y, z = P[..., 0] - cg_x, P[..., 1], P[..., 2] - mid_z
        return np.stack([-z * s, y * s, x * s], axis=-1)       # (x, y, z)_src -> (-z, y, x)_godot

    def rot(N):
        N = np.asarray(N, float)
        return np.stack([-N[..., 2], N[..., 1], N[..., 0]], axis=-1)

    hubs = {n: to_godot(h["centre"]) for n, h in hubs_src.items()}

    # --- steering wheel: centre and column axis (smallest-variance direction of its vertices) ---------------------
    sw_keys = [k for k in faces if k[0] == STEERING_GROUP]
    Psw = to_godot(group_points(V, [t for k in sw_keys for t in faces[k]]))
    sw_centre = (Psw.min(0) + Psw.max(0)) / 2
    _, _, vecs = np.linalg.svd(Psw - Psw.mean(0), full_matrices=False)
    sw_axis = vecs[-1]
    if sw_axis[2] > 0:                    # point the axis towards the driver (+Z = rearwards)
        sw_axis = -sw_axis
    sw_axis = -sw_axis                    # from the dashboard towards the driver

    # --- write parts --------------------------------------------------------------------------------------------
    os.makedirs(out, exist_ok=True)
    tex_dir = os.path.join(out, "textures")
    os.makedirs(tex_dir, exist_ok=True)
    used_mtls = sorted({k[1] for k in faces})
    write_mtl(os.path.join(out, "mini_jcw.mtl"), mats, used_mtls, src, tex_dir)

    parts = {"body": [], "steering_wheel": [], "wheel_fl": [], "wheel_fr": [], "wheel_rl": [], "wheel_rr": []}
    for k, tris in faces.items():
        if k in wheel_of:
            parts["wheel_" + wheel_of[k]].append((k, tris))
        elif k[0] == STEERING_GROUP:
            parts["steering_wheel"].append((k, tris))
        else:
            parts["body"].append((k, tris))

    offsets = {"body": np.zeros(3), "steering_wheel": sw_centre}
    offsets.update({f"wheel_{n}": hubs[n] for n in hubs})
    counts = {}
    for name, items in parts.items():
        counts[name] = write_part(os.path.join(out, name + ".obj"), name, items, V, VT, VN, to_godot, rot, offsets[name])

    # --- metadata (measured, not chosen) ------------------------------------------------------------------------
    body = to_godot(group_points(V, [t for _, tris in parts["body"] for t in tris]))
    chassis = to_godot(group_points(V, [t for k, tris in faces.items() if k[0].startswith(("Chassis", "Bumpfront", "Bumprear", "Bonnet", "Boot")) for t in tris]))

    def centre_of(prefix, material=None):
        P = to_godot(group_points(V, [t for k, tris in faces.items() if k[0].startswith(prefix) and (material is None or k[1] == material) for t in tris]))
        return P

    tail = centre_of("Tailight", "Tailight0031Mtl")
    head = centre_of("Frontlight")
    seat = centre_of("Seat1")
    meta = OrderedDict(
        source="2011-mini-john-cooper-works-r56/source/Mini JCW GP/Mini JCW GP.obj",
        frame="Godot: car faces -Z, left = -X, metres, origin on the ground below the CG (car.c ahead of the rear axle)",
        scale_to_metres=s,
        wheelbase_source_units=wheelbase_src,
        wheelbase_m=WHEELBASE,
        tyre_radius_m=float(np.mean([h["diameter"] for h in hubs_src.values()]) * s / 2),
        tyre_width_m=float(np.mean([h["width"] for h in hubs_src.values()]) * s),
        hubs={n: [round(float(c), 4) for c in hubs[n]] for n in sorted(hubs)},
        track_front_m=float(abs(hubs["fl"][0] - hubs["fr"][0])),
        track_rear_m=float(abs(hubs["rl"][0] - hubs["rr"][0])),
        body_aabb_min=[round(float(c), 4) for c in body.min(0)],
        body_aabb_max=[round(float(c), 4) for c in body.max(0)],
        chassis_aabb_min=[round(float(c), 4) for c in chassis.min(0)],
        chassis_aabb_max=[round(float(c), 4) for c in chassis.max(0)],
        steering_wheel=dict(centre=[round(float(c), 4) for c in sw_centre], axis=[round(float(c), 5) for c in sw_axis]),
        tail_lights=dict(left=[round(float(c), 3) for c in tail[tail[:, 0] < 0].mean(0)],
                         right=[round(float(c), 3) for c in tail[tail[:, 0] > 0].mean(0)],
                         rear_z=round(float(tail[:, 2].max()), 3)),
        head_lights=dict(left=[round(float(c), 3) for c in head[head[:, 0] < 0].mean(0)],
                         right=[round(float(c), 3) for c in head[head[:, 0] > 0].mean(0)],
                         front_z=round(float(head[:, 2].min()), 3)),
        seats_aabb=[[round(float(c), 3) for c in seat.min(0)], [round(float(c), 3) for c in seat.max(0)]],
        triangles=counts,
    )
    with open(os.path.join(out, "mini_jcw_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print(json.dumps(meta, indent=2))


def write_mtl(path, mats, used, src_dir, tex_dir):
    with open(path, "w") as f:
        f.write("# cleaned by tools/asset_pipeline/import_jcw_obj.py: no map_d/map_Ka; only the glass is translucent\n")
        for name in used:
            m = mats.get(name, {})
            f.write(f"\nnewmtl {name}\n")
            f.write(f"Kd {m.get('Kd', '1 1 1')}\n")
            f.write(f"Ks {m.get('Ks', '0 0 0')}\n")
            f.write(f"Ns {m.get('Ns', '0')}\n")
            f.write(f"d {m.get('d', '1') if name == GLASS_MATERIAL else '1'}\n")
            if "map_Kd" in m:
                tex = m["map_Kd"]
                shutil.copyfile(os.path.join(src_dir, tex), os.path.join(tex_dir, tex))
                f.write(f"map_Kd textures/{tex}\n")


def write_part(path, name, items, V, VT, VN, to_godot, rot, offset):
    # Re-index only the vertices this part uses.
    vmap, tmap, nmap = {}, {}, {}
    vs, ts, ns, out_faces = [], [], [], []
    for (group, mtl), tris in items:
        block = []
        for t in tris:
            corners = []
            for vi, ti, ni in t:
                if vi not in vmap:
                    vmap[vi] = len(vs) + 1
                    vs.append(vi)
                if ti >= 0 and ti not in tmap:
                    tmap[ti] = len(ts) + 1
                    ts.append(ti)
                if ni >= 0 and ni not in nmap:
                    nmap[ni] = len(ns) + 1
                    ns.append(ni)
                corners.append(f"{vmap[vi]}/{tmap.get(ti, '')}/{nmap.get(ni, '')}")
            block.append(corners)
        out_faces.append((group, mtl, block))
    P = to_godot(V[vs]) - offset
    N = rot(VN[ns]) if ns else np.zeros((0, 3))
    T = VT[ts] if ts else np.zeros((0, 2))
    with open(path, "w") as f:
        f.write(f"# {name}: generated by tools/asset_pipeline/import_jcw_obj.py\nmtllib mini_jcw.mtl\no {name}\n")
        f.write("".join(f"v {p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n" for p in P))
        f.write("".join(f"vt {t[0]:.5f} {t[1]:.5f}\n" for t in T))
        f.write("".join(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n" for n in N))
        # One block per material (not per source group): Godot makes one surface, i.e. one draw call, per block.
        by_mtl = OrderedDict()
        for group, mtl, block in out_faces:
            by_mtl.setdefault(mtl, []).extend(block)
        for mtl, block in by_mtl.items():
            f.write(f"usemtl {mtl}\n")
            f.write("".join("f " + " ".join(c) + "\n" for c in block))
    return sum(len(b) for _, _, b in out_faces)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
