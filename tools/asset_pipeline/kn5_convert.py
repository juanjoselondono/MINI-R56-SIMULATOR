"""Convert an Assetto Corsa track folder (kn5 + ai + data) into Godot-ready assets.

Outputs (all in Godot coordinates; AC's x, y, z map to Godot unchanged, verified: the racing line runs clockwise as the
track's ui_track.json states, and both use clockwise front faces):
  <out>/<name>_visual.glb        render geometry only, merged per (material, 200 m cell, size class) to cut draw calls;
                                 node names carry render hints for the post-import script (__ns = no shadows,
                                 __vr<m> = visibility range end)
  <out>/textures/*.dds           the embedded DDS textures with normalised headers (Godot loads DDS natively, keeping the
                                 GPU block compression and mipmaps)
  <out>/materials/*.tres         one StandardMaterial3D per AC material (albedo / alpha / normal map / roughness)
  <out>/<name>_physics.obj       low-poly collision mesh, one `usemtl` group per AC surface key (TARMAC, WALL, ...); when
                                 the physics file has no kerb surface (Monaco), a KERB group derived from the visual kerb
                                 meshes
  <out>/<name>_track.json        start grid, pits, timing gates, racing line (fast_lane.ai) with track-edge distances,
                                 surface table: friction (data/surfaces.ini, null when the key is one of AC's global
                                 surfaces, which are not part of the track) and class (road / kerb / grass / wall)

Multi-layout tracks (models_<layout>.ini, <layout>/ai, <layout>/data, ui/<layout>): pass --layout <layout>. Every
MODEL_n of the layout that is not the physics or the spawn file is rendered; several visual kn5 files are merged into
the one GLB (materials and textures renamed only on a name clash between files).

Usage: python3 -I tools/asset_pipeline/kn5_convert.py <ac_track_dir> <out_dir> <name> [--layout <layout>]
"""
import configparser
import hashlib
import json
import math
import os
import re
import struct
import sys
import tempfile
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kn5 import Kn5  # noqa: E402

CELL = 200.0                 # m, spatial merge cell
SMALL_CELL = 50.0            # m, merge cell for minor objects: Godot measures visibility ranges from the node's AABB
                             # centre, so a minor object must stay close to the centre of its merged group
SMALL_RADIUS = 4.0           # m, meshes smaller than this are "minor" objects


def small_visibility_end(radius):
    """Visibility end [m] for minor objects, by size: ~250 radii (a 2 m object ends ~500 m away, about 4 px tall at
    1080p, then fades), at least 400 m, at most 1200 m."""
    return int(min(1200, max(400, 250 * radius)))
NO_SHADOW_PATTERN = re.compile(r'tree|fence|net|grass|bush|crowd|people|flag|banner|cable|wire|leaf|plant|palm|shadow',
                               re.I)
KERB_PATTERN = re.compile(r'curb|kerb', re.I)
# Surface classes of physics keys (game side: wall = barrier contact, kerb = curb rolling/grip, grass = off-road rolling
# resistance and, without a surfaces.ini friction, the game's grass grip factor; road = everything else drivable).
SURFACE_CLASSES = (('wall', re.compile(r'^WALL')),
                   ('kerb', re.compile(r'KERB|CURB|KURB|RUMBLE|RMBL|3DIMP|XBLOCK')),
                   ('grass', re.compile(r'GRASS|SAND|GRAVEL|GREEN|DIRT')))


def surface_class(key):
    return next((cls for cls, pattern in SURFACE_CLASSES if pattern.search(key)), 'road')


def sanitize(name):
    return re.sub(r'[^A-Za-z0-9_.]', '_', name)


# --- DDS ------------------------------------------------------------------------------------------------------------
def normalise_dds(data):
    """Make DDS header flags consistent with the payload so Godot's strict loader accepts them."""
    if data[:4] != b'DDS ':
        return data
    b = bytearray(data)
    flags, height, width = struct.unpack_from('<III', b, 8)
    mips = struct.unpack_from('<I', b, 28)[0]
    pf_flags = struct.unpack_from('<I', b, 80)[0]
    fourcc = bytes(b[84:88])
    DDSD_PITCH, DDSD_LINEARSIZE, DDSD_MIPMAPCOUNT = 0x8, 0x80000, 0x20000
    if pf_flags & 0x4:  # compressed (FOURCC)
        block = 8 if fourcc == b'DXT1' else 16
        size = max(1, (width + 3) // 4) * max(1, (height + 3) // 4) * block
        flags = (flags | DDSD_LINEARSIZE) & ~DDSD_PITCH
        struct.pack_into('<I', b, 20, size)
    else:
        bpp = struct.unpack_from('<I', b, 88)[0]
        flags = (flags | DDSD_PITCH) & ~DDSD_LINEARSIZE
        struct.pack_into('<I', b, 20, (width * bpp + 7) // 8)
    flags = (flags | DDSD_MIPMAPCOUNT) if mips > 1 else (flags & ~DDSD_MIPMAPCOUNT)
    struct.pack_into('<I', b, 8, flags)
    return bytes(b)


def dds_payload_complete(data):
    """True if Godot can load it: block-compressed sizes multiple of 4, and no mipmaps or a COMPLETE chain down to 1x1
    that is fully present in the file (Godot treats any DDS with mipmaps as a full chain and rejects shorter payloads)."""
    height, width = struct.unpack_from('<II', data, 12)
    mips = max(1, struct.unpack_from('<I', data, 28)[0])
    if struct.unpack_from('<I', data, 80)[0] & 0x4 and (width % 4 or height % 4):
        return False   # block-compressed with a partial block row/column: Godot rejects the header's linear size
    full_levels = int(math.floor(math.log2(max(width, height)))) + 1
    if mips > 1 and mips != full_levels:
        return False
    pf_flags = struct.unpack_from('<I', data, 80)[0]
    fourcc = data[84:88]
    need = 0
    w, h = width, height
    for _ in range(mips):
        if pf_flags & 0x4:
            need += max(1, (w + 3) // 4) * max(1, (h + 3) // 4) * (8 if fourcc == b'DXT1' else 16)
        else:
            need += w * h * struct.unpack_from('<I', data, 88)[0] // 8
        w, h = max(1, w // 2), max(1, h // 2)
    return len(data) - 128 >= need


def write_texture(tex_dir, tname, data):
    """DDS with normalised header; files with truncated mip chains are decoded (Pillow) and written as PNG so that
    Godot regenerates complete mipmaps on import. Returns the file name written."""
    base = sanitize(os.path.splitext(tname)[0]).lower()
    data = normalise_dds(data)
    if dds_payload_complete(data):
        fn = base + '.dds'
        with open(os.path.join(tex_dir, fn), 'wb') as f:
            f.write(data)
        return fn
    from io import BytesIO
    from PIL import Image
    fn = base + '.png'
    Image.open(BytesIO(data)).convert('RGBA').save(os.path.join(tex_dir, fn))
    write_png_import(os.path.join(tex_dir, fn))
    return fn


def write_png_import(path):
    """Godot import settings for a re-encoded texture: VRAM uncompressed, full mip chain (these are the textures whose
    DDS mip chain was incomplete). Godot fills in the remaining keys on import."""
    with open(path + '.import', 'w') as f:
        f.write('[remap]\n\nimporter="texture"\ntype="CompressedTexture2D"\n\n[params]\n\n'
                'compress/mode=3\ncompress/high_quality=true\nmipmaps/generate=true\nmipmaps/limit=-1\n'
                'process/fix_alpha_border=true\ndetect_3d/compress_to=0\n')


# --- materials --------------------------------------------------------------------------------------------------------
TEXTURE_FILES = {}   # AC texture name -> written file name (of the kn5 being converted)


def texture_file(name):
    return TEXTURE_FILES.get(name, sanitize(os.path.splitext(name)[0]).lower() + '.dds')


def write_material(path, m, tex_res_dir, available):
    """StandardMaterial3D in Godot's text resource format."""
    def prop(key, default):
        return m.props[key][0] if key in m.props else default

    ext = []
    diffuse = m.textures.get('txDiffuse')
    normal = m.textures.get('txNormal') if 'NM' in m.shader else None
    lines = []
    for slot, tex in (('albedo', diffuse), ('normal', normal)):
        if tex and texture_file(tex) in available:
            ext.append((slot, f'{tex_res_dir}/{texture_file(tex)}'))
    header = f'[gd_resource type="StandardMaterial3D" load_steps={len(ext) + 1} format=3]\n\n'
    for i, (_, res) in enumerate(ext):
        header += f'[ext_resource type="Texture2D" path="{res}" id="{i + 1}"]\n'
    lines.append(f'resource_name = "{sanitize(m.name)}"')
    alpha_test = m.alpha_tested or m.blend_mode == 2 or 'AT' in m.shader or m.shader == 'ksTree'
    if m.blend_mode == 1 and not alpha_test:
        lines.append('transparency = 1')
    elif alpha_test:
        lines.append('transparency = 2')
        lines.append(f'alpha_scissor_threshold = {min(0.9, max(0.1, prop("ksAlphaRef", 0.5))):.3f}')
        lines.append('cull_mode = 2')
    if alpha_test or m.blend_mode == 1:
        lines.append('alpha_antialiasing_mode = 1' if alpha_test else 'depth_draw_mode = 1')
    if m.shader == 'ksTree':
        # Two-sided leaf cards: without back-lighting, the side facing away from the sun renders almost black.
        lines.append('backlight_enabled = true')
        lines.append('backlight = Color(0.55, 0.6, 0.45, 1)')
    for i, (slot, _) in enumerate(ext):
        if slot == 'albedo':
            lines.append(f'albedo_texture = ExtResource("{i + 1}")')
        else:
            lines.append('normal_enabled = true')
            lines.append(f'normal_texture = ExtResource("{i + 1}")')
    if not any(s == 'albedo' for s, _ in ext):
        lines.append('albedo_color = Color(0.55, 0.55, 0.55, 1)')
    spec_exp = max(1.0, prop('ksSpecularEXP', 10.0))
    rough = min(0.95, max(0.3, 1.0 - math.log2(spec_exp) / 12))
    lines.append(f'roughness = {rough:.3f}')
    emissive = m.props.get('ksEmissive', (0, (0, 0), (0, 0, 0), (0, 0, 0, 0)))[2]
    if max(emissive) > 0.01:
        lines.append('emission_enabled = true')
        lines.append(f'emission = Color({min(1, emissive[0]):.3f}, {min(1, emissive[1]):.3f}, {min(1, emissive[2]):.3f}, 1)')
    if 'UVMult' in m.shader and 'diffuseMult' in m.props:
        mult = m.props['diffuseMult'][0]
        lines.append(f'uv1_scale = Vector3({mult:.3f}, {mult:.3f}, {mult:.3f})')
    lines.append('texture_filter = 5')
    with open(path, 'w') as f:
        f.write(header + '\n[resource]\n' + '\n'.join(lines) + '\n')


# --- glTF -----------------------------------------------------------------------------------------------------------
class GlbWriter:
    def __init__(self):
        # The binary chunk is spooled to a temporary file: a multi-file track has several hundred MB of vertex data.
        self.bin = tempfile.TemporaryFile()
        self.bin_len = 0
        self.gltf = {'asset': {'version': '2.0', 'generator': 'kn5_convert.py'}, 'scene': 0,
                     'scenes': [{'nodes': []}], 'nodes': [], 'meshes': [], 'materials': [], 'accessors': [],
                     'bufferViews': [], 'buffers': []}
        self.material_index = {}

    def _view(self, arr, target):
        pad = (4 - self.bin_len % 4) % 4
        self.bin.write(b'\0' * pad)
        off = self.bin_len + pad
        data = arr.tobytes()
        self.bin.write(data)
        self.bin_len = off + len(data)
        self.gltf['bufferViews'].append({'buffer': 0, 'byteOffset': off, 'byteLength': arr.nbytes, 'target': target})
        return len(self.gltf['bufferViews']) - 1

    def _accessor(self, arr, comp, typ, target, minmax=False):
        acc = {'bufferView': self._view(arr, target), 'componentType': comp, 'count': len(arr), 'type': typ}
        if minmax:
            acc['min'] = arr.min(0).tolist()
            acc['max'] = arr.max(0).tolist()
        self.gltf['accessors'].append(acc)
        return len(self.gltf['accessors']) - 1

    def material(self, name, alpha_mode):
        if name not in self.material_index:
            mat = {'name': name, 'pbrMetallicRoughness': {'metallicFactor': 0.0}}
            if alpha_mode != 'OPAQUE':
                mat['alphaMode'] = alpha_mode
            self.gltf['materials'].append(mat)
            self.material_index[name] = len(self.gltf['materials']) - 1
        return self.material_index[name]

    def add(self, node_name, material, pos, nrm, uv, idx):
        prim = {'attributes': {
            'POSITION': self._accessor(pos.astype(np.float32), 5126, 'VEC3', 34962, True),
            'NORMAL': self._accessor(nrm.astype(np.float32), 5126, 'VEC3', 34962),
            'TEXCOORD_0': self._accessor(uv.astype(np.float32), 5126, 'VEC2', 34962)},
            'indices': self._accessor(idx.astype(np.uint32), 5125, 'SCALAR', 34963), 'material': material}
        self.gltf['meshes'].append({'name': node_name, 'primitives': [prim]})
        self.gltf['nodes'].append({'name': node_name, 'mesh': len(self.gltf['meshes']) - 1})
        self.gltf['scenes'][0]['nodes'].append(len(self.gltf['nodes']) - 1)

    def save(self, path):
        pad = (4 - self.bin_len % 4) % 4
        self.bin.write(b'\0' * pad)
        self.bin_len += pad
        self.gltf['buffers'] = [{'byteLength': self.bin_len}]
        js = json.dumps(self.gltf, separators=(',', ':')).encode()
        js += b' ' * ((4 - len(js) % 4) % 4)
        with open(path, 'wb') as f:
            f.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + self.bin_len))
            f.write(struct.pack('<II', len(js), 0x4E4F534A) + js)
            f.write(struct.pack('<II', self.bin_len, 0x004E4942))
            self.bin.seek(0)
            while chunk := self.bin.read(1 << 24):
                f.write(chunk)
        self.bin.close()


def convert_visual(k, out, glb, stats, written, prefix=''):
    """Adds one visual kn5 to the GLB. <written> maps written texture / material file names to the source that wrote
    them, so that a second kn5 reusing a name gets <prefix> instead of overwriting (identical textures are shared)."""
    tex_dir = os.path.join(out, 'textures')
    mat_dir = os.path.join(out, 'materials')
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(mat_dir, exist_ok=True)
    TEXTURE_FILES.clear()
    available = written.setdefault('available', set())
    for tname, data in k.textures.items():
        if data[:4] != b'DDS ':
            continue
        source = tname
        digest = hashlib.md5(data).hexdigest()
        base = sanitize(os.path.splitext(tname)[0]).lower()
        if written['textures'].get(base, digest) != digest:
            tname, base = prefix + tname, sanitize(os.path.splitext(prefix + tname)[0]).lower()   # clash: rename
        if written['textures'].get(base) == digest:
            fn = written['texture_files'][base]                                                  # same image: share
        else:
            fn = write_texture(tex_dir, tname, data)
            written['textures'][base] = digest
            written['texture_files'][base] = fn
            stats['textures_png_fallback'] += fn.endswith('.png')
        TEXTURE_FILES[source] = fn
        available.add(fn)
    k.textures.clear()   # free the texture bytes before the geometry is merged
    res_dir = 'res://' + os.path.relpath(out, find_project_root(out)).replace(os.sep, '/')
    mat_names = {}
    for m in k.materials:
        mname = sanitize(m.name)
        if mname in written['materials']:
            mname = sanitize(prefix + m.name)
        written['materials'].add(mname)
        mat_names[id(m)] = mname
        m.name = mname
        write_material(os.path.join(mat_dir, mname + '.tres'), m, res_dir + '/textures', available)

    groups = defaultdict(list)
    for mesh in k.meshes:
        if not mesh.visible or not mesh.renderable or len(mesh.indices) == 0:
            continue
        lo, hi = mesh.positions.min(0), mesh.positions.max(0)
        centre, radius = (lo + hi) / 2, float(np.linalg.norm(hi - lo) / 2)
        mat = k.materials[mesh.material]
        small = radius < SMALL_RADIUS
        size_cell = SMALL_CELL if small else CELL
        no_shadow = (not mesh.cast_shadows or mesh.transparent or mat.blend_mode == 1 or mat.shader == 'ksTree'
                     or bool(NO_SHADOW_PATTERN.search(mesh.name + ' ' + mat.name)) or small)
        cell = (int(centre[0] // size_cell), int(centre[2] // size_cell))
        groups[(mesh.material, cell, small, no_shadow)].append(mesh)

    stats['source_meshes'] += len(k.meshes)
    for (mat_id, cell, small, no_shadow), meshes in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        mat = k.materials[mat_id]
        pos = np.vstack([m.positions for m in meshes])
        nrm = np.vstack([m.normals for m in meshes])
        uv = np.vstack([m.uvs for m in meshes])
        offsets = np.cumsum([0] + [len(m.positions) for m in meshes[:-1]])
        idx = np.concatenate([m.indices + o for m, o in zip(meshes, offsets)])
        alpha = 'MASK' if (mat.alpha_tested or 'AT' in mat.shader or mat.shader == 'ksTree') else (
            'BLEND' if mat.blend_mode == 1 else 'OPAQUE')
        node = f'{mat.name}__c{cell[0]}_{cell[1]}'
        if no_shadow:
            node += '__ns'
            stats['no_shadow_nodes'] += 1
        if small:
            radius = max(float(np.linalg.norm(m.positions.max(0) - m.positions.min(0)) / 2) for m in meshes)
            node += f'__vr{small_visibility_end(radius)}'
            stats['small_nodes'] += 1
        glb.add(node, glb.material(mat.name, alpha), pos, nrm, uv, idx)
        stats['nodes'] += 1
        stats['triangles'] += len(idx) // 3
    stats['textures'] = len(available)
    stats['materials'] += len(k.materials)


def convert_visuals(paths, out, name):
    """All visual kn5 files of the layout, one at a time (memory), into <name>_visual.glb."""
    glb = GlbWriter()
    stats = {'files': 0, 'source_meshes': 0, 'nodes': 0, 'triangles': 0, 'no_shadow_nodes': 0, 'small_nodes': 0,
             'textures': 0, 'textures_png_fallback': 0, 'materials': 0}
    written = {'textures': {}, 'texture_files': {}, 'materials': set()}
    for i, path in enumerate(paths):
        stem = sanitize(os.path.splitext(os.path.basename(path))[0])
        convert_visual(Kn5(path), out, glb, stats, written, prefix='' if i == 0 else stem + '__')
        stats['files'] += 1
        print(f'  visual {os.path.basename(path)}: {stats["triangles"] / 1e6:.2f} M triangles so far', flush=True)
    glb.save(os.path.join(out, f'{name}_visual.glb'))
    return stats


def find_project_root(path):
    p = os.path.abspath(path)
    while p != '/':
        if os.path.exists(os.path.join(p, 'project.godot')):
            return p
        p = os.path.dirname(p)
    raise RuntimeError('project.godot not found above ' + path)


# --- physics --------------------------------------------------------------------------------------------------------
def physics_key(mesh_name, keys):
    """AC surface key of a physics mesh: the name is <digits><KEY>...; the longest surfaces.ini key that the name (or
    the name without its digits) starts with, else the leading capitals (AC's global keys: ROAD, GRASS, WALL, ...).
    Names without leading digits are not physical in AC (None)."""
    m = re.match(r'^(\d+)(.*)$', mesh_name)
    if not m:
        return None
    rest = m.group(2)
    candidates = [k for k in keys if mesh_name.startswith(k) or rest.startswith(k)]
    if candidates:
        return max(candidates, key=len)
    token = re.match(r'[A-Z][A-Z_]*[A-Z]|[A-Z]', rest)
    return token.group(0) if token else 'TARMAC'


def convert_physics(phys, visual_paths, out, name, keys):
    by_key = defaultdict(list)
    skipped = 0
    for mesh in phys.meshes:
        key = physics_key(mesh.name, keys)
        if key is None:
            skipped += 1
            continue
        by_key[key].append(mesh)
    if not any(surface_class(k) == 'kerb' for k in by_key):
        # No kerb surface in the physics file (Monaco): take the visual kerb meshes.
        for path in visual_paths:
            visual = Kn5(path, False)
            for mesh in visual.meshes:
                mat = visual.materials[mesh.material]
                if KERB_PATTERN.search(mat.name + ' ' + str(mat.textures.get('txDiffuse', ''))):
                    by_key['KERB'].append(mesh)
    path = os.path.join(out, f'{name}_physics.obj')
    counts = {}
    with open(os.path.join(out, f'{name}_physics.mtl'), 'w') as f:
        for key in by_key:
            f.write(f'newmtl {key}\nKd 0.5 0.5 0.5\n\n')
    with open(path, 'w') as f:
        f.write(f'# {name} collision/surface mesh: one usemtl group per AC surface key (generated by kn5_convert.py)\n')
        f.write(f'mtllib {name}_physics.mtl\n')
        base = 1
        for key, meshes in by_key.items():
            f.write(f'o {key}\nusemtl {key}\n')
            tri = 0
            for mesh in meshes:
                f.write(''.join(f'v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n' for p in mesh.positions))
                ids = mesh.indices.reshape(-1, 3) + base
                f.write(''.join(f'f {a} {b} {c}\n' for a, b, c in ids))
                base += len(mesh.positions)
                tri += len(ids)
            counts[key] = tri
    if skipped:
        counts['(not physical: no digit prefix)'] = skipped
    return counts, list(by_key)


# --- track data -----------------------------------------------------------------------------------------------------
def read_ai(path):
    d = open(path, 'rb').read()
    version, n = struct.unpack('<ii', d[:8])
    pts = np.frombuffer(d[16:16 + 20 * n], dtype=np.dtype([('p', '<3f4'), ('len', '<f4'), ('id', '<i4')]))
    off = 16 + 20 * n
    extra = None
    if version >= 7 and struct.unpack('<i', d[off:off + 4])[0] == n:
        extra = np.frombuffer(d[off + 4:off + 4 + 72 * n], dtype='<f4').reshape(n, 18)
    return pts['p'].astype(float), extra


def dummies(k):
    out = {}
    for dm in k.dummies:
        m = dm.matrix
        out[dm.name] = {'position': m[3, :3].round(4).tolist(), 'forward': (m[2, :3] / np.linalg.norm(m[2, :3])).round(5).tolist()}
    return out


def track_data(ac_dir, layout_dir, ui_path, start_kn5, out, name, physics_keys):
    pts, extra = read_ai(os.path.join(layout_dir, 'ai', 'fast_lane.ai'))
    starts = dummies(start_kn5)
    left = np.array(starts['AC_TIME_0_L']['position'])
    right = np.array(starts['AC_TIME_0_R']['position'])
    gate_mid = (left + right) / 2
    # Re-index the racing line so that point 0 is the one nearest the start/finish gate.
    i0 = int(np.argmin(np.linalg.norm(pts - gate_mid, axis=1)))
    order = np.r_[i0:len(pts), 0:i0]
    pts = pts[order]
    line = {'x': pts[:, 0].round(4).tolist(), 'y': pts[:, 1].round(4).tolist(), 'z': pts[:, 2].round(4).tolist()}
    if extra is not None:
        extra = extra[order]
        line['side_left'] = extra[:, 5].round(3).tolist()
        line['side_right'] = extra[:, 6].round(3).tolist()
    surfaces = {}
    for key, value in surfaces_ini(layout_dir).items():
        surfaces[key] = dict(value, **{'class': surface_class(key)})
    for key in physics_keys:
        # One of AC's global surfaces (system/data/surfaces.ini, not part of the track): no friction value here; the
        # game uses its own grip factor for the class.
        surfaces.setdefault(key, {'friction': None, 'valid_track': surface_class(key) != 'grass', 'class': surface_class(key)})
    info = {}
    if os.path.exists(ui_path):
        info = json.load(open(ui_path, encoding='utf-8-sig'))
    data = {
        'name': info.get('name', name), 'run': info.get('run'), 'length_m': info.get('length'),
        'frame': 'Godot (AC coordinates map unchanged: Y up)',
        'start_grid': [starts[f'AC_START_{i}'] for i in range(64) if f'AC_START_{i}' in starts],
        'pits': [starts[f'AC_PIT_{i}'] for i in range(64) if f'AC_PIT_{i}' in starts],
        'timing_gates': [{'left': starts[f'AC_TIME_{i}_L']['position'], 'right': starts[f'AC_TIME_{i}_R']['position']}
                         for i in range(16) if f'AC_TIME_{i}_L' in starts and f'AC_TIME_{i}_R' in starts],
        'racing_line': line,
        'surfaces': surfaces,
    }
    with open(os.path.join(out, f'{name}_track.json'), 'w') as f:
        json.dump(data, f, indent=1)
    return {'grid': len(data['start_grid']), 'gates': len(data['timing_gates']), 'line_points': len(pts),
            'surfaces': list(surfaces)}


def surfaces_ini(layout_dir):
    surfaces = {}
    ini = configparser.ConfigParser()
    ini.read(os.path.join(layout_dir, 'data', 'surfaces.ini'), encoding='utf-8-sig')
    for sec in ini.sections():
        surfaces[ini[sec]['KEY']] = {'friction': float(ini[sec].get('FRICTION', 1)),
                                     'valid_track': ini[sec].get('IS_VALID_TRACK', '1') == '1'}
    return surfaces


def main(ac_dir, out, name, layout=None):
    os.makedirs(out, exist_ok=True)
    layout_dir = os.path.join(ac_dir, layout) if layout else ac_dir
    ui_path = os.path.join(ac_dir, 'ui', layout, 'ui_track.json') if layout else os.path.join(ac_dir, 'ui', 'ui_track.json')
    models = configparser.ConfigParser()
    models.read(os.path.join(ac_dir, f'models_{layout}.ini' if layout else 'models.ini'), encoding='utf-8-sig')
    files = [models[s]['FILE'] for s in models.sections() if s.startswith('MODEL_')]
    missing = [f for f in files if not os.path.exists(os.path.join(ac_dir, f))]
    if missing:
        print('missing (listed in the models file, skipped):', missing)
    files = [f for f in files if f not in missing]
    physics_file = next(f for f in files if 'physics' in f)
    start_file = next(f for f in files if 'start' in f or 'spawn' in f)
    visual_files = [os.path.join(ac_dir, f) for f in files if f not in (physics_file, start_file)]
    print('visual:', convert_visuals(visual_files, out, name))
    counts, keys = convert_physics(Kn5(os.path.join(ac_dir, physics_file), False), visual_files, out, name,
                                   list(surfaces_ini(layout_dir)))
    print('physics:', counts)
    print('track:', track_data(ac_dir, layout_dir, ui_path, Kn5(os.path.join(ac_dir, start_file), False), out, name, keys))


if __name__ == '__main__':
    args = sys.argv[1:]
    lay = None
    if '--layout' in args:
        k = args.index('--layout')
        lay = args[k + 1]
        del args[k:k + 2]
    main(args[0], args[1], args[2], lay)
