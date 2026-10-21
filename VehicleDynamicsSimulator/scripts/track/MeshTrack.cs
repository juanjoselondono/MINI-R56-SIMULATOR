using System;
using System.Collections.Generic;
using System.Linq;
using Godot;
using Simulator.Dynamics;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;

namespace Simulator.Track;

/// <summary>Ground under the car from the collision mesh: height/attitude for rendering, surface and grade for physics.</summary>
public readonly record struct GroundSample(bool Hit, float Height, Vector3 Up, Surface Surface, string SurfaceKey, double Grip, double Grade);

/// <summary>
/// A track converted from Assetto Corsa (tools/asset_pipeline/kn5_convert.py): the visual GLB is rendered as is, and the
/// low-poly physics OBJ becomes one hidden StaticBody3D per surface key (trimesh shapes, never rendered):
/// WALL on <see cref="TrackManager.BarrierCollisionLayer"/>, TARMAC on <see cref="TarmacLayer"/>, KERB on
/// <see cref="KerbLayer"/>. Each body carries "surface" and "friction" (surfaces.ini) metadata.
/// The vehicle dynamics core never sees the meshes; per physics step this node answers two questions with ray/shape
/// queries: the ground under the car (surface → μ, local grade, height/attitude) and wall contacts (→ E3 response).
/// </summary>
public partial class MeshTrack : Node3D
{
    public const uint TarmacLayer = 1u << 2;
    public const uint KerbLayer = 1u << 3;
    /// <summary>Layer of the vehicle's trigger volume (detected by the start/finish Area3D).</summary>
    public const uint VehicleTriggerLayer = 1u << 4;

    [Export] public string DataPath { get; set; } = "res://assets/tracks/monaco/monaco_track.json";
    [Export] public string PhysicsMeshPath { get; set; } = "res://assets/tracks/monaco/monaco_physics.obj";

    public string DisplayName { get; private set; } = "";
    public PolylineTrack Line { get; private set; } = null!;
    public (double X, double Y, double Psi) Spawn { get; private set; }
    public float SpawnHeight { get; private set; }
    public Vector3 GateLeft { get; private set; }
    public Vector3 GateRight { get; private set; }
    public IReadOnlyDictionary<string, double> Friction => _friction;
    public Aabb Bounds { get; private set; }
    /// <summary>Raised when the vehicle's trigger volume enters the start/finish Area3D (Godot physics tick).</summary>
    public event Action? StartFinishEntered;

    private readonly Dictionary<string, double> _friction = new();
    private BoxShape3D _footprint = new();
    private PhysicsShapeQueryParameters3D _wallQuery = new();
    private float _groundHeight;
    private RayCast3D _probe = null!;

    public void Load()
    {
        if (Line != null) return;
        var data = Json.ParseString(FileAccess.GetFileAsString(DataPath)).AsGodotDictionary();
        DisplayName = data["name"].AsString();
        foreach (var (key, value) in data["surfaces"].AsGodotDictionary())
            _friction[key.AsString()] = value.AsGodotDictionary()["friction"].AsDouble();

        // Racing line in the model frame (X = x, Y = -z); s = 0 at the start/finish gate (converter re-indexed it).
        var line = data["racing_line"].AsGodotDictionary();
        double[] gx = line["x"].AsFloat64Array(), gz = line["z"].AsFloat64Array();
        Line = new PolylineTrack(DisplayName, gx, gz.Select(z => -z).ToArray(),
            line.ContainsKey("side_left") ? line["side_left"].AsFloat64Array() : null,
            line.ContainsKey("side_right") ? line["side_right"].AsFloat64Array() : null);

        var pole = data["start_grid"].AsGodotArray()[0].AsGodotDictionary();
        var p = ToVector(pole["position"]);
        var f = ToVector(pole["forward"]);
        Spawn = (p.X, -p.Z, Math.Atan2(-f.Z, f.X));
        SpawnHeight = p.Y;
        _groundHeight = p.Y;

        var gate = data["timing_gates"].AsGodotArray()[0].AsGodotDictionary();
        GateLeft = ToVector(gate["left"]);
        GateRight = ToVector(gate["right"]);

        BuildCollision();
        BuildStartFinishArea();
        if (GetNodeOrNull<Node3D>("Visual") is { } visual) Bounds = ComputeBounds(visual);
        _probe = new RayCast3D { Name = "GroundProbe", Enabled = false, CollisionMask = TarmacLayer | KerbLayer, HitBackFaces = true };
        AddChild(_probe);
    }

    /// <summary>
    /// Vertical RayCast3D down the whole height of the track (from above the highest geometry to below the lowest)
    /// against the tarmac/kerb collision mesh: elevation of the highest drivable surface at (x, y), or null if the
    /// column has none. Used for spawns and resets, where the previous height says nothing about the new position.
    /// </summary>
    public float? ProbeGroundHeight(double x, double y)
    {
        float top = Bounds.End.Y + 50f, bottom = Bounds.Position.Y - 50f;
        var at = VehicleModelAdapter.ToGodotPosition(x, y, top);
        _probe.GlobalPosition = at;
        _probe.TargetPosition = new Vector3(0, bottom - top, 0);
        _probe.ForceRaycastUpdate();
        return _probe.IsColliding() ? _probe.GetCollisionPoint().Y : null;
    }

    /// <summary>
    /// Ground elevation at (x, y) searched around <paramref name="hint"/> (+4 / −12 m), so a line that runs through the
    /// tunnel finds the road, not the roof; falls back to the full-height probe. For walking along a line.
    /// </summary>
    public float? GroundHeightNear(double x, double y, float hint)
    {
        var hit = Ray(VehicleModelAdapter.ToGodotPosition(x, y, hint + 4f), VehicleModelAdapter.ToGodotPosition(x, y, hint - 12f), TarmacLayer | KerbLayer);
        return hit.Count > 0 ? ((Vector3)hit["position"]).Y : ProbeGroundHeight(x, y);
    }

    /// <summary>
    /// Snaps the ground tracking to (x, y) after a teleport (restart, recover): the per-step rays then search around
    /// the probed elevation. Returns that elevation.
    /// </summary>
    public float? SnapToGround(double x, double y)
    {
        var h = ProbeGroundHeight(x, y);
        if (h is { } v) _groundHeight = v;
        return h;
    }

    private static Vector3 ToVector(Variant v)
    {
        var a = v.AsFloat64Array();
        return new Vector3((float)a[0], (float)a[1], (float)a[2]);
    }

    private void BuildCollision()
    {
        var mesh = GD.Load<ArrayMesh>(PhysicsMeshPath);
        for (int i = 0; i < mesh.GetSurfaceCount(); i++)
        {
            string key = mesh.SurfaceGetName(i);
            var arrays = mesh.SurfaceGetArrays(i);
            var verts = arrays[(int)Mesh.ArrayType.Vertex].AsVector3Array();
            var index = arrays[(int)Mesh.ArrayType.Index].AsInt32Array();
            var faces = new Vector3[index.Length];
            for (int k = 0; k < index.Length; k++) faces[k] = verts[index[k]];
            uint layer = key switch { "WALL" => TrackManager.BarrierCollisionLayer, "KERB" => KerbLayer, _ => TarmacLayer };
            var body = new StaticBody3D { Name = $"Physics_{key}", CollisionLayer = layer, CollisionMask = 0 };
            body.SetMeta("surface", key);
            body.SetMeta("friction", _friction.TryGetValue(key, out var mu) ? mu : 1.0);
            body.AddChild(new CollisionShape3D { Shape = new ConcavePolygonShape3D { Data = faces, BackfaceCollision = true } });
            AddChild(body);
        }
    }

    private void BuildStartFinishArea()
    {
        var mid = (GateLeft + GateRight) / 2;
        var across = GateRight - GateLeft;
        var width = new Vector3(across.X, 0, across.Z).Length();
        var area = new Area3D
        {
            Name = "StartFinish", CollisionLayer = 0, CollisionMask = VehicleTriggerLayer, Monitorable = false,
            Transform = new Transform3D(new Basis(Vector3.Up, Mathf.Atan2(-across.Z, across.X)), mid + Vector3.Up * 2f),
        };
        area.AddChild(new CollisionShape3D { Shape = new BoxShape3D { Size = new Vector3(width, 4f, 1f) } });
        area.AreaEntered += _ => StartFinishEntered?.Invoke();
        AddChild(area);
    }

    private static Aabb ComputeBounds(Node3D root)
    {
        var box = new Aabb();
        bool first = true;
        foreach (var mi in root.FindChildren("*", nameof(MeshInstance3D), true, false).OfType<MeshInstance3D>())
        {
            var b = mi.GlobalTransform * mi.GetAabb();
            box = first ? b : box.Merge(b);
            first = false;
        }
        return box;
    }

    private Godot.Collections.Dictionary Ray(Vector3 from, Vector3 to, uint mask)
        => GetWorld3D().DirectSpaceState.IntersectRay(PhysicsRayQueryParameters3D.Create(from, to, mask));

    /// <summary>
    /// Ground at the car's pose (model frame): heights under the front axle centre and the two rear wheels give the
    /// visual attitude and the local grade along the heading (E6); the surface under the CG (kerb if a kerb lies on top
    /// of the tarmac) gives the surfaces.ini friction for μ (E2). The AC physics mesh ends at the track limits: a wheel
    /// beyond it takes the mean height of the others, and a CG beyond it reads OFFTRACK (surfaces.ini friction,
    /// asphalt rolling resistance — at Monaco that is pavement, not grass). Hit = false only if no wheel finds ground.
    /// </summary>
    /// <param name="updateHint">Physics steps move the ray window with the car; render-time queries (interpolated
    /// pose) pass false so the physics never depends on the frame rate.</param>
    public GroundSample Sample(double x, double y, double psi, VehicleParameters p, bool updateHint = true)
    {
        double c = Math.Cos(psi), s = Math.Sin(psi);
        double lf = p.Wheelbase - p.CgToRear, lr = p.CgToRear, half = 0.5 * p.TrackRear;
        float top = _groundHeight + 4f, bottom = _groundHeight - 12f;
        Vector3 At(double ox, double oy, float h) => VehicleModelAdapter.ToGodotPosition(x + ox * c - oy * s, y + ox * s + oy * c, h);
        float? Height(double ox, double oy, uint mask)
        {
            var hit = Ray(At(ox, oy, top), At(ox, oy, bottom), mask);
            return hit.Count > 0 ? ((Vector3)hit["position"]).Y : null;
        }
        const uint ground = TarmacLayer | KerbLayer;
        float? hFront = Height(lf, 0, ground), hRearL = Height(-lr, half, ground), hRearR = Height(-lr, -half, ground);
        var found = new[] { hFront, hRearL, hRearR }.Where(h => h.HasValue).Select(h => h!.Value).ToArray();
        if (found.Length == 0 && ProbeGroundHeight(x, y) is { } reacquired && Math.Abs(reacquired - _groundHeight) > 1f)
        {
            // Ground lost by the search window (it follows the last height): re-acquire from a full-height probe.
            float saved = _groundHeight;
            _groundHeight = reacquired;
            var retry = Sample(x, y, psi, p, updateHint);
            if (!updateHint) _groundHeight = saved;
            return retry;
        }
        if (found.Length == 0)
            return new GroundSample(false, _groundHeight, Vector3.Up, Surface.Asphalt, "OFFTRACK", Friction.GetValueOrDefault("OFFTRACK", 1), 0);
        float fill = found.Average();
        float hf = hFront ?? fill, hl = hRearL ?? fill, hr = hRearR ?? fill;

        var front = At(lf, 0, hf);
        var rearL = At(-lr, half, hl);
        var rearR = At(-lr, -half, hr);
        var up = (rearR - front).Cross(rearL - front).Normalized();
        if (up.Y < 0) up = -up;
        float rearMid = 0.5f * (hl + hr);
        float hCg = rearMid + (hf - rearMid) * (float)(lr / p.Wheelbase);
        if (updateHint) _groundHeight = hCg;
        double grade = Math.Atan2(hf - rearMid, p.Wheelbase);

        float? tarmac = Height(0, 0, TarmacLayer), kerb = Height(0, 0, KerbLayer);
        // A CG ray that misses while all three wheels have ground is a seam in the AC mesh, not off-track.
        string key = kerb is { } k && (tarmac is not { } t || k >= t - 0.15f) ? "KERB"
            : tarmac.HasValue || found.Length == 3 ? "TARMAC" : "OFFTRACK";
        var surface = key == "KERB" ? Surface.Curb : Surface.Asphalt;
        return new GroundSample(true, hCg, up, surface, key, Friction.GetValueOrDefault(key, 1), grade);
    }

    /// <summary>
    /// Wall contact for the E3 collision response: the car's footprint box (body height above the ground) queried against
    /// the WALL layer. Normal = horizontal part of the wall normal, oriented towards the car; penetration = deepest
    /// contact pair. Model frame in and out.
    /// </summary>
    public BarrierContactResult? WallContact(double x, double y, double psi, double halfLength, double halfWidth)
    {
        _footprint.Size = new Vector3(2 * (float)halfWidth, 1.0f, 2 * (float)halfLength);
        _wallQuery.Shape = _footprint;
        _wallQuery.CollisionMask = TrackManager.BarrierCollisionLayer;
        var centre = VehicleModelAdapter.ToGodotPosition(x, y, _groundHeight + 0.7f);
        _wallQuery.Transform = new Transform3D(new Basis(Vector3.Up, (float)(psi - Math.PI / 2)), centre);
        var space = GetWorld3D().DirectSpaceState;
        var pairs = space.CollideShape(_wallQuery, 16);
        if (pairs.Count < 2) return null;
        float depth = 0;
        Vector3 point = pairs[1];
        for (int i = 0; i + 1 < pairs.Count; i += 2)
        {
            float d = pairs[i].DistanceTo(pairs[i + 1]);
            if (d > depth) { depth = d; point = pairs[i + 1]; }
        }
        var rest = space.GetRestInfo(_wallQuery);
        var n = rest.Count > 0 ? (Vector3)rest["normal"] : (centre - point);
        var planar = new Vector2(n.X, n.Z);
        if (planar.LengthSquared() < 1e-8f) planar = new Vector2(centre.X - point.X, centre.Z - point.Z);
        planar = planar.Normalized();
        if (planar.Dot(new Vector2(centre.X - point.X, centre.Z - point.Z)) < 0) planar = -planar;
        return new BarrierContactResult(planar.X, -planar.Y, Math.Max(depth, 0.005), point.X, -point.Z, false, 0, 0);
    }
}
