"""Minimal reader for Assetto Corsa .kn5 model files (versions 5-6).

Layout (little endian; strings are int32 length + bytes):
  "sc6969", int32 version, [int32 extra if version > 5]
  int32 textureCount × { int32 active, string name, int32 size, bytes data (DDS/PNG) }
  int32 materialCount × { string name, string shader, byte blendMode, byte alphaTested, [int32 depthMode if version > 4],
                          int32 propCount × { string name, float a, float[2] b, float[3] c, float[4] d },
                          int32 mapCount × { string sampler, int32 slot, string textureName } }
  node tree, depth first: int32 class, string name, int32 childCount, byte active, then by class:
    1 (transform): float[16] row-major matrix (translation in the last row)
    2 (mesh): byte castShadows, byte visible, byte transparent, uint32 vertexCount ×
              { float3 position, float3 normal, float2 uv, float3 tangent }, uint32 indexCount × uint16,
              uint32 materialId, uint32 layer, float lodIn, float lodOut, float3 sphereCentre, float radius,
              byte renderable
    3 (skinned mesh): byte×3, uint32 boneCount × { string, float[16] }, uint32 vertexCount ×
              { float3, float3, float2, float3, float4 weights, float4 indices }, uint32 indexCount × uint16,
              uint32 materialId, uint32 layer, 8 bytes
Mesh vertices are in the mesh node's frame; world = product of the ancestors' transforms.
"""
import struct
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Material:
    name: str
    shader: str
    blend_mode: int
    alpha_tested: bool
    depth_mode: int
    props: dict = field(default_factory=dict)      # name -> (a, b[2], c[3], d[4])
    textures: dict = field(default_factory=dict)   # sampler -> texture name


@dataclass
class Mesh:
    name: str
    path: str
    material: int
    positions: np.ndarray   # (n, 3) world, kn5 frame
    normals: np.ndarray     # (n, 3) world, kn5 frame
    uvs: np.ndarray         # (n, 2)
    indices: np.ndarray     # (m,) uint32
    cast_shadows: bool
    visible: bool
    transparent: bool
    renderable: bool
    lod_in: float
    lod_out: float


@dataclass
class Dummy:
    name: str
    path: str
    matrix: np.ndarray      # 4x4 world, row-vector convention (p' = p @ M)


class Kn5:
    def __init__(self, path, load_texture_data=True):
        self._f = open(path, 'rb')
        self.version = 0
        self.textures = {}      # name -> bytes (or size if not loaded)
        self.materials = []
        self.meshes = []
        self.dummies = []
        self._parse(load_texture_data)
        self._f.close()

    # --- primitives -----------------------------------------------------------------------------------------------
    def _i32(self):
        return struct.unpack('<i', self._f.read(4))[0]

    def _u32(self):
        return struct.unpack('<I', self._f.read(4))[0]

    def _f32(self, n=1):
        v = struct.unpack(f'<{n}f', self._f.read(4 * n))
        return v[0] if n == 1 else v

    def _u8(self):
        return self._f.read(1)[0]

    def _str(self):
        n = self._i32()
        return self._f.read(n).decode('utf-8', errors='replace')

    # --- structure ------------------------------------------------------------------------------------------------
    def _parse(self, load_texture_data):
        if self._f.read(6) != b'sc6969':
            raise ValueError('not a kn5 file')
        self.version = self._i32()
        if self.version > 5:
            self._i32()
        for _ in range(self._i32()):
            self._i32()
            name = self._str()
            size = self._i32()
            if load_texture_data:
                self.textures[name] = self._f.read(size)
            else:
                self._f.seek(size, 1)
                self.textures[name] = size
        for _ in range(self._i32()):
            m = Material(self._str(), self._str(), self._u8(), self._u8() != 0, self._i32() if self.version > 4 else 0)
            for _ in range(self._i32()):
                pname = self._str()
                m.props[pname] = (self._f32(), self._f32(2), self._f32(3), self._f32(4))
            for _ in range(self._i32()):
                sampler = self._str()
                self._i32()
                m.textures[sampler] = self._str()
            self.materials.append(m)
        self._node(np.eye(4), '')

    def _node(self, parent, path):
        cls = self._i32()
        name = self._str()
        children = self._i32()
        self._u8()
        here = f'{path}/{name}'
        world = parent
        if cls == 1:
            local = np.array(self._f32(16), dtype=np.float64).reshape(4, 4)
            world = local @ parent
            self.dummies.append(Dummy(name, here, world))
        elif cls == 2:
            cast, vis, transp = self._u8(), self._u8(), self._u8()
            n = self._u32()
            raw = np.frombuffer(self._f.read(44 * n), dtype='<f4').reshape(n, 11).astype(np.float64)
            m = self._u32()
            idx = np.frombuffer(self._f.read(2 * m), dtype='<u2').astype(np.uint32)
            mat, _layer = self._u32(), self._u32()
            lod_in, lod_out = self._f32(), self._f32()
            self._f32(4)
            renderable = self._u8() != 0
            pos = np.c_[raw[:, 0:3], np.ones(n)] @ parent
            nrm = raw[:, 3:6] @ parent[:3, :3]
            ln = np.linalg.norm(nrm, axis=1, keepdims=True)
            nrm = nrm / np.where(ln == 0, 1, ln)
            self.meshes.append(Mesh(name, here, mat, pos[:, :3], nrm, raw[:, 6:8], idx,
                                    cast != 0, vis != 0, transp != 0, renderable, lod_in, lod_out))
        elif cls == 3:
            self._u8(), self._u8(), self._u8()
            for _ in range(self._u32()):
                self._str()
                self._f32(16)
            n = self._u32()
            self._f.seek(76 * n, 1)
            m = self._u32()
            self._f.seek(2 * m, 1)
            self._u32(), self._u32()
            self._f.seek(8, 1)
        else:
            raise ValueError(f'unknown node class {cls} at {here}')
        for _ in range(children):
            self._node(world, here)
