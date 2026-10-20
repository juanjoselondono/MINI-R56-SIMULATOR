using System;
using Godot;
using VehicleDynamics.Core.Track;

namespace Simulator.Track;

/// <summary>
/// Loads a track from the core <see cref="TrackLibrary"/> and builds its 3D representation from that same geometry.
/// The core geometry is the single source for rendering, surfaces, barrier contact and lap timing.
/// </summary>
public partial class TrackManager : Node3D
{
    /// <summary>Physics layer of the barrier collision body (camera ray queries only; car contact is analytic).</summary>
    public const uint BarrierCollisionLayer = 1u << 1;

    [Export] public string TrackName { get; set; } = "Circuit";
    /// <summary>
    /// Arc length of the grid slot [m]. 0 = CG on the start/finish line, which is also model.m's oval start point
    /// (beginning of straight 1); the lap timer uses a standing start from there.
    /// </summary>
    [Export] public float SpawnS { get; set; } = 0f;
    [Export] public bool BuildScenery { get; set; } = true;

    public TrackGeometry Geometry { get; private set; } = null!;
    public (double X, double Y, double Psi) SpawnPose { get; private set; }
    public Aabb Bounds { get; private set; }

    public static readonly string[] Available = { "Circuit", "Oval" };

    public static TrackGeometry Create(string name) => name switch
    {
        "Oval" => TrackLibrary.Oval(),
        "Circuit" => TrackLibrary.Circuit(),
        _ => throw new ArgumentException($"Unknown track '{name}'. Available: {string.Join(", ", Available)}"),
    };

    /// <summary>Build <see cref="TrackName"/> on ready. Off in Main, where SimulationManager chooses the track.</summary>
    [Export] public bool LoadOnReady { get; set; }

    public override void _Ready()
    {
        if (LoadOnReady && Geometry == null) LoadTrack(TrackName);
    }

    public void LoadTrack(string name)
    {
        foreach (var child in GetChildren()) { RemoveChild(child); child.QueueFree(); }
        TrackName = name;
        Geometry = Create(name);
        var p = Geometry.Pose(SpawnS);
        SpawnPose = (p.X, p.Y, p.Psi);

        var road = Load("road.gdshader");
        road.SetShaderParameter("road_half_width", Geometry.Section.RoadHalfWidth);
        road.SetShaderParameter("track_length", Geometry.Length);
        var grass = Load("grass.gdshader");
        var mesh = TrackMeshBuilder.BuildSurfaces(Geometry, road, Load("curb.gdshader"), grass, Load("barrier.gdshader"),
            out var barrierFaces);
        AddChild(new MeshInstance3D { Name = "TrackSurface", Mesh = mesh });

        var body = new StaticBody3D { Name = "BarrierCollision", CollisionLayer = BarrierCollisionLayer, CollisionMask = 0 };
        body.AddChild(new CollisionShape3D { Shape = new ConcavePolygonShape3D { Data = barrierFaces, BackfaceCollision = true } });
        AddChild(body);

        Bounds = mesh.GetAabb();
        var terrainMat = Load("grass.gdshader");
        terrainMat.SetShaderParameter("mow_stripes", 0.0);
        float size = Mathf.Max(Bounds.Size.X, Bounds.Size.Z) + 1200f;
        AddChild(new MeshInstance3D
        {
            Name = "Terrain",
            Mesh = new PlaneMesh { Size = new Vector2(size, size), Material = terrainMat },
            Position = Bounds.GetCenter() with { Y = -0.05f },
            CastShadow = GeometryInstance3D.ShadowCastingSetting.Off,
        });

        if (BuildScenery) TrackScenery.Build(this, Geometry);
    }

    private static ShaderMaterial Load(string file)
        => new() { Shader = GD.Load<Shader>($"res://assets/materials/{file}") };
}
