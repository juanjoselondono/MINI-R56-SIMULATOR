using System;
using System.Collections.Generic;
using Godot;
using Simulator.Dynamics;
using VehicleDynamics.Core.Track;

namespace Simulator.Track;

/// <summary>
/// Builds the road, curb, runoff and barrier meshes by sampling the core <see cref="TrackGeometry"/>, with the same
/// cross-section and the same s-continuous barrier offsets the physics uses, so the rendered walls are where the
/// collision model puts them. Physics wall surface = inner face of the rendered barrier.
/// </summary>
public static class TrackMeshBuilder
{
    public const float BarrierHeight = 1.0f;
    public const float BarrierThickness = 0.4f;

    /// <summary>Arc-length stations: straights every 4 m, curves every ≤ 1° of heading (0.4–2 m).</summary>
    public static List<double> Stations(TrackGeometry t)
    {
        var s = new List<double>();
        foreach (var seg in t.Segments)
        {
            double spacing = seg.Kind == SegmentKind.Straight ? 4.0 : Math.Clamp(seg.Radius * Mathf.DegToRad(1), 0.4, 2.0);
            if (seg.Kind == SegmentKind.Clothoid) spacing = 0.5;
            int n = Math.Max(1, (int)Math.Ceiling(seg.Length / spacing));
            for (int i = 0; i < n; i++) s.Add(seg.StartS + seg.Length * i / n);
        }
        // Barrier offsets blend linearly over ±20 m around joints; add the blend ends so straights stay exact.
        foreach (var seg in t.Segments)
        {
            s.Add(t.NormalizeS(seg.StartS + TrackGeometry.BarrierTransitionLength));
            s.Add(t.NormalizeS(seg.StartS - TrackGeometry.BarrierTransitionLength));
        }
        s.Sort();
        var unique = new List<double>(s.Count + 1);
        foreach (double v in s)
            if (unique.Count == 0 || v - unique[^1] > 1e-6) unique.Add(v);
        unique.Add(t.Length);
        return unique;
    }

    public static Vector3 Point(TrackGeometry t, double s, double lateral, float height = 0)
    {
        var (x, y) = t.PointAt(s, lateral);
        return VehicleModelAdapter.ToGodotPosition(x, y, height);
    }

    /// <summary>Godot direction of the model-frame left normal at s.</summary>
    public static Vector3 LeftNormal(TrackGeometry t, double s)
    {
        double psi = t.Heading(s);
        return VehicleModelAdapter.ToGodotPosition(-Math.Sin(psi), Math.Cos(psi)).Normalized();
    }

    public static ArrayMesh BuildSurfaces(TrackGeometry t, Material road, Material curb, Material grass, Material barrier,
        out Vector3[] barrierFaces)
    {
        var st = Stations(t);
        var sec = t.Section;
        double rw = sec.RoadHalfWidth, cw = sec.CurbWidth;

        var roadB = new Strip();
        var curbB = new Strip();
        var grassB = new Strip();
        var wallB = new Strip();
        var faces = new List<Vector3>();

        for (int i = 0; i + 1 < st.Count; i++)
        {
            double s0 = st[i], s1 = st[i + 1];
            // Horizontal bands (lateral ranges); the runoff ends at the s-dependent barrier offsets.
            roadB.Band(t, s0, s1, -rw, -rw, rw, rw);
            curbB.Band(t, s0, s1, rw, rw, rw + cw, rw + cw);
            curbB.Band(t, s0, s1, -rw - cw, -rw - cw, -rw, -rw);
            grassB.Band(t, s0, s1, rw + cw, rw + cw, t.BarrierOffsetLeftAt(s0), t.BarrierOffsetLeftAt(s1));
            grassB.Band(t, s0, s1, -t.BarrierOffsetRightAt(s0), -t.BarrierOffsetRightAt(s1), -rw - cw, -rw - cw);

            wallB.Wall(t, s0, s1, +1, faces);
            wallB.Wall(t, s0, s1, -1, faces);
        }

        var mesh = new ArrayMesh();
        roadB.Commit(mesh, road);
        curbB.Commit(mesh, curb);
        grassB.Commit(mesh, grass);
        wallB.Commit(mesh, barrier);
        barrierFaces = faces.ToArray();
        return mesh;
    }

    /// <summary>Triangle accumulator; winding is chosen from the intended normal so faces are never culled wrongly.</summary>
    private sealed class Strip
    {
        private readonly List<Vector3> _v = new();
        private readonly List<Vector3> _n = new();
        private readonly List<Vector2> _uv = new();
        private readonly List<int> _i = new();

        public void Band(TrackGeometry t, double s0, double s1, double a0, double a1, double b0, double b1)
        {
            // Quad corners: (s0, a0) (s0, b0) (s1, a1) (s1, b1); a = right edge, b = left edge (lateral + left).
            Quad(Point(t, s0, a0), Point(t, s0, b0), Point(t, s1, a1), Point(t, s1, b1), Vector3.Up,
                new Vector2((float)a0, (float)s0), new Vector2((float)b0, (float)s0),
                new Vector2((float)a1, (float)s1), new Vector2((float)b1, (float)s1));
        }

        /// <summary>Barrier on one side (side +1 = left): inner face at the physical offset, top, outer face.</summary>
        public void Wall(TrackGeometry t, double s0, double s1, int side, List<Vector3> faces)
        {
            double o0 = side > 0 ? t.BarrierOffsetLeftAt(s0) : -t.BarrierOffsetRightAt(s0);
            double o1 = side > 0 ? t.BarrierOffsetLeftAt(s1) : -t.BarrierOffsetRightAt(s1);
            double th = side * BarrierThickness;
            Vector3 inward = -side * (LeftNormal(t, 0.5 * (s0 + s1)));
            float h = BarrierHeight;
            var uS0 = (float)s0;
            var uS1 = (float)s1;

            // Inner face (faces the track).
            Vector3 p00 = Point(t, s0, o0), p01 = Point(t, s0, o0, h), p10 = Point(t, s1, o1), p11 = Point(t, s1, o1, h);
            Quad(p00, p01, p10, p11, inward, new Vector2(0, uS0), new Vector2(h, uS0), new Vector2(0, uS1), new Vector2(h, uS1));
            faces.AddRange(new[] { p00, p01, p10, p10, p01, p11 });
            // Top.
            Vector3 q01 = Point(t, s0, o0 + th, h), q11 = Point(t, s1, o1 + th, h);
            Quad(p01, q01, p11, q11, Vector3.Up, new Vector2(h, uS0), new Vector2(h + 0.4f, uS0), new Vector2(h, uS1), new Vector2(h + 0.4f, uS1));
            // Outer face.
            Vector3 q00 = Point(t, s0, o0 + th), q10 = Point(t, s1, o1 + th);
            Quad(q00, q01, q10, q11, -inward, new Vector2(0, uS0), new Vector2(h, uS0), new Vector2(0, uS1), new Vector2(h, uS1));
        }

        private void Quad(Vector3 a0, Vector3 b0, Vector3 a1, Vector3 b1, Vector3 normal,
            Vector2 ua0, Vector2 ub0, Vector2 ua1, Vector2 ub1)
        {
            int k = _v.Count;
            _v.Add(a0); _v.Add(b0); _v.Add(a1); _v.Add(b1);
            _uv.Add(ua0); _uv.Add(ub0); _uv.Add(ua1); _uv.Add(ub1);
            for (int j = 0; j < 4; j++) _n.Add(normal);
            Tri(k, k + 1, k + 2, normal);
            Tri(k + 2, k + 1, k + 3, normal);
        }

        private void Tri(int a, int b, int c, Vector3 normal)
        {
            // Godot front faces are clockwise as seen from the front, i.e. (b - a) x (c - a) points away from the viewer.
            var cross = (_v[b] - _v[a]).Cross(_v[c] - _v[a]);
            if (cross.Dot(normal) > 0) (b, c) = (c, b);
            _i.Add(a); _i.Add(b); _i.Add(c);
        }

        public void Commit(ArrayMesh mesh, Material material)
        {
            if (_v.Count == 0) return;
            var arrays = new Godot.Collections.Array();
            arrays.Resize((int)Mesh.ArrayType.Max);
            arrays[(int)Mesh.ArrayType.Vertex] = _v.ToArray();
            arrays[(int)Mesh.ArrayType.Normal] = _n.ToArray();
            arrays[(int)Mesh.ArrayType.TexUV] = _uv.ToArray();
            arrays[(int)Mesh.ArrayType.Index] = _i.ToArray();
            mesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);
            mesh.SurfaceSetMaterial(mesh.GetSurfaceCount() - 1, material);
        }
    }
}
