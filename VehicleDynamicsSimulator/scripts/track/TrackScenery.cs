using System;
using Godot;
using VehicleDynamics.Core.Track;

namespace Simulator.Track;

/// <summary>
/// Decorative scenery placed from the track geometry (start gantry, grandstands, braking boards, trees).
/// Everything is outside the barriers, so none of it takes part in the vehicle physics.
/// </summary>
public static class TrackScenery
{
    public static void Build(Node3D parent, TrackGeometry t)
    {
        var root = new Node3D { Name = "Scenery" };
        parent.AddChild(root);
        StartGantry(root, t);
        Grandstand(root, t, 40, 220);
        BrakingBoards(root, t);
        Trees(root, t);
    }

    private static StandardMaterial3D Mat(Color c, float rough = 0.8f, float metal = 0f)
        => new() { AlbedoColor = c, Roughness = rough, Metallic = metal };

    /// <summary>Transform at (s, lateral, height) whose -Z axis points along the track direction (Label3D text faces +Z, i.e. approaching cars).</summary>
    private static Transform3D Frame(TrackGeometry t, double s, double lateral, float height = 0)
    {
        var origin = TrackMeshBuilder.Point(t, s, lateral, height);
        var basis = new Basis(Vector3.Up, (float)(t.Heading(s) - Math.PI / 2));
        return new Transform3D(basis, origin);
    }

    private static void Box(Node3D parent, Transform3D frame, Vector3 localPos, Vector3 size, Material mat)
    {
        parent.AddChild(new MeshInstance3D
        {
            Mesh = new BoxMesh { Size = size, Material = mat },
            Transform = frame * new Transform3D(Basis.Identity, localPos),
        });
    }

    private static void StartGantry(Node3D root, TrackGeometry t)
    {
        double left = t.BarrierOffsetLeftAt(0) + 0.9, right = t.BarrierOffsetRightAt(0) + 0.9;
        var f = Frame(t, 0, 0);
        var steel = Mat(new Color(0.2f, 0.21f, 0.23f), 0.4f, 0.7f);
        float span = (float)(left + right);
        float centre = (float)((left - right) / 2);
        // Car frame: -X is left of travel; lateral + (left) maps to local -X.
        Box(root, f, new Vector3((float)-left, 3.5f, 0), new Vector3(0.5f, 7f, 0.5f), steel);
        Box(root, f, new Vector3((float)right, 3.5f, 0), new Vector3(0.5f, 7f, 0.5f), steel);
        Box(root, f, new Vector3(-centre, 6.6f, 0), new Vector3(span, 1.2f, 0.6f), Mat(new Color(0.08f, 0.08f, 0.1f), 0.5f));
        var label = new Label3D
        {
            Text = "START  ·  FINISH",
            FontSize = 160,
            PixelSize = 0.01f,
            Modulate = new Color(1, 0.85f, 0.2f),
            OutlineSize = 0,
            Transform = f * new Transform3D(Basis.Identity, new Vector3(-centre, 6.6f, 0.32f)),
        };
        root.AddChild(label);
    }

    private static void Grandstand(Node3D root, TrackGeometry t, double s0, double s1)
    {
        var concrete = Mat(new Color(0.55f, 0.55f, 0.57f));
        var seats = Mat(new Color(0.1f, 0.25f, 0.55f), 0.6f);
        var roof = Mat(new Color(0.85f, 0.86f, 0.88f), 0.3f, 0.5f);
        for (double s = s0; s < s1; s += 20)
        {
            double off = t.BarrierOffsetRightAt(s) + 8;
            var f = Frame(t, s + 10, -off);
            for (int k = 0; k < 7; k++)
                Box(root, f, new Vector3(1.6f * k + 0.8f, 0.6f * k + 0.3f, 0), new Vector3(1.6f, 0.6f * (k + 1), 20f), k % 2 == 0 ? concrete : seats);
            Box(root, f, new Vector3(5.6f, 6.5f, 0), new Vector3(12f, 0.2f, 20f), roof);
            Box(root, f, new Vector3(11.4f, 3.4f, 0), new Vector3(0.4f, 6.4f, 20f), concrete);
        }
    }

    /// <summary>150/100/50 m boards before every corner that follows a straight of at least 150 m.</summary>
    private static void BrakingBoards(Node3D root, TrackGeometry t)
    {
        var board = Mat(new Color(0.95f, 0.95f, 0.95f), 0.5f);
        for (int i = 0; i < t.Segments.Count; i++)
        {
            var seg = t.Segments[i];
            if (seg.Kind == SegmentKind.Straight) continue;
            var prev = t.Segments[(i - 1 + t.Segments.Count) % t.Segments.Count];
            if (prev.Kind != SegmentKind.Straight || prev.Length < 150) continue;
            // Outside of the corner: right side when the corner turns left.
            int side = seg.K0 > 0 ? -1 : 1;
            foreach (int d in new[] { 150, 100, 50 })
            {
                double s = seg.StartS - d;
                double off = (side > 0 ? t.BarrierOffsetLeftAt(s) : t.BarrierOffsetRightAt(s)) + 1.2;
                var f = Frame(t, s, side * off);
                Box(root, f, new Vector3(0, 1.4f, 0), new Vector3(1.4f, 1.0f, 0.1f), board);
                Box(root, f, new Vector3(0, 0.45f, 0), new Vector3(0.12f, 0.9f, 0.12f), board);
                root.AddChild(new Label3D
                {
                    Text = d.ToString(),
                    FontSize = 96,
                    PixelSize = 0.008f,
                    Modulate = new Color(0.05f, 0.05f, 0.05f),
                    OutlineSize = 0,
                    Transform = f * new Transform3D(Basis.Identity, new Vector3(0, 1.4f, 0.06f)),
                });
            }
        }
    }

    private static void Trees(Node3D root, TrackGeometry t)
    {
        var bounds = new Aabb();
        bool first = true;
        foreach (var (s, _) in t.Sample(20))
        {
            var p = TrackMeshBuilder.Point(t, s, 0);
            if (first) { bounds = new Aabb(p, Vector3.Zero); first = false; } else bounds = bounds.Expand(p);
        }
        bounds = bounds.Grow(220);

        const int count = 1400;
        var trunk = new MultiMesh { TransformFormat = MultiMesh.TransformFormatEnum.Transform3D, UseColors = true, InstanceCount = count };
        var crown = new MultiMesh { TransformFormat = MultiMesh.TransformFormatEnum.Transform3D, UseColors = true, InstanceCount = count };
        trunk.Mesh = new CylinderMesh { TopRadius = 0.18f, BottomRadius = 0.28f, Height = 3f, RadialSegments = 6, Rings = 1,
            Material = new StandardMaterial3D { VertexColorUseAsAlbedo = true, Roughness = 0.9f } };
        crown.Mesh = new CylinderMesh { TopRadius = 0f, BottomRadius = 2.2f, Height = 6.5f, RadialSegments = 8, Rings = 2,
            Material = new StandardMaterial3D { VertexColorUseAsAlbedo = true, Roughness = 0.85f } };

        var rng = new RandomNumberGenerator { Seed = 56 };
        int placed = 0, attempts = 0;
        while (placed < count && attempts < count * 20)
        {
            attempts++;
            var pos = new Vector3(rng.RandfRange(bounds.Position.X, bounds.End.X), 0, rng.RandfRange(bounds.Position.Z, bounds.End.Z));
            var pr = t.Project(pos.X, -pos.Z);
            double wall = pr.Lateral >= 0 ? t.BarrierOffsetLeftAt(pr.S) : t.BarrierOffsetRightAt(pr.S);
            if (Math.Abs(pr.Lateral) < wall + 14) continue;
            float scale = rng.RandfRange(0.7f, 1.5f);
            var basis = new Basis(Vector3.Up, rng.RandfRange(0, Mathf.Tau)).Scaled(new Vector3(scale, scale, scale));
            trunk.SetInstanceTransform(placed, new Transform3D(basis, pos + new Vector3(0, 1.5f * scale, 0)));
            crown.SetInstanceTransform(placed, new Transform3D(basis, pos + new Vector3(0, 5.6f * scale, 0)));
            trunk.SetInstanceColor(placed, new Color(0.3f, 0.2f, 0.12f));
            float g = rng.RandfRange(0.75f, 1.15f);
            crown.SetInstanceColor(placed, new Color(0.1f * g, 0.3f * g, 0.12f * g));
            placed++;
        }
        trunk.VisibleInstanceCount = placed;
        crown.VisibleInstanceCount = placed;
        root.AddChild(new MultiMeshInstance3D { Name = "TreeTrunks", Multimesh = trunk });
        root.AddChild(new MultiMeshInstance3D { Name = "TreeCrowns", Multimesh = crown });
    }
}
