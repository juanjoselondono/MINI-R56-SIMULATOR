using System;
using System.Collections.Generic;
using System.Linq;
using Godot;
using Simulator.Simulation;
using VehicleDynamics.Core.Track;

namespace Simulator.Track;

/// <summary>
/// Loads a track: either a procedural one from the core <see cref="TrackLibrary"/> (3D built from that same analytic
/// geometry, which is then the single source for rendering, surfaces, barrier contact and lap timing), or a converted
/// mesh track (<see cref="MeshTrack"/>: surfaces/walls from its physics mesh, timing on its AC racing line).
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

    /// <summary>Analytic geometry of a procedural track (null for converted mesh tracks).</summary>
    public TrackGeometry? Geometry { get; private set; }
    /// <summary>Converted Assetto Corsa track (null for procedural tracks).</summary>
    public MeshTrack? Mesh { get; private set; }
    /// <summary>Reference line for lap timing, track position and scripted driving (every track has one).</summary>
    public ITrackLine Line { get; private set; } = null!;
    public (double X, double Y, double Psi) SpawnPose { get; private set; }
    public Aabb Bounds { get; private set; }

    /// <summary>A converted Assetto Corsa track: its scene (MeshTrack + visual GLB) and the converted data it needs.</summary>
    public sealed record MeshTrackInfo(string Name, string Scene, string Data, string Physics, string Description);

    /// <summary>
    /// Converted tracks. To add one: convert it (tools/asset_pipeline/kn5_convert.py → assets/tracks/&lt;name&gt;/), give it
    /// a scene like scenes/TrackMonaco.tscn (MeshTrack script, DataPath/PhysicsMeshPath, the visual GLB as child
    /// "Visual") and list it here. It appears in the menu once its data file exists.
    /// </summary>
    public static readonly MeshTrackInfo[] MeshTrackList =
    {
        new("Monaco", "res://scenes/TrackMonaco.tscn", "res://assets/tracks/monaco/monaco_track.json",
            "res://assets/tracks/monaco/monaco_physics.obj",
            "Monaco street circuit — converted Assetto Corsa track (mesh surfaces, walls, elevation)"),
        new("Nurburgring", "res://scenes/TrackNurburgring.tscn", "res://assets/tracks/nurburgring/nurburgring_track.json",
            "res://assets/tracks/nurburgring/nurburgring_physics.obj",
            "Nürburgring 24h (Nordschleife + GP) — converted Assetto Corsa track, 25.4 km"),
    };

    public static readonly Dictionary<string, MeshTrackInfo> MeshTracks = MeshTrackList.ToDictionary(t => t.Name);

    /// <summary>Heavy resources of a track (scene with its visual GLB, physics mesh) for background loading.</summary>
    public static IEnumerable<string> ResourcesFor(string name)
    {
        if (!MeshTracks.TryGetValue(name, out var t)) yield break;
        yield return t.Scene;
        yield return t.Physics;
    }

    /// <summary>
    /// Procedural tracks always; converted tracks only when their assets are present (they are generated locally by
    /// tools/asset_pipeline/kn5_convert.py and are not in version control).
    /// </summary>
    public static readonly string[] Available = new[] { "Circuit", "Oval" }
        .Concat(MeshTrackList.Where(t => FileAccess.FileExists(t.Data)).Select(t => t.Name)).ToArray();

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
        if (LoadOnReady && Line == null) LoadTrack(TrackName);
    }

    public void LoadTrack(string name)
    {
        foreach (var child in GetChildren()) { RemoveChild(child); child.QueueFree(); }
        TrackName = name;
        Mesh = null;
        if (MeshTracks.TryGetValue(name, out var info))
        {
            string scenePath = info.Scene;
            if (!Available.Contains(name))
                throw new InvalidOperationException($"Track '{name}' is not converted yet: run tools/asset_pipeline/kn5_convert.py (docs/run_instructions.md).");
            Geometry = null;
            PackedScene packed;
            using (LoadTimer.Stage("track scene load (glb, textures)")) packed = GD.Load<PackedScene>(scenePath);
            using (LoadTimer.Stage("track scene instantiate")) Mesh = packed.Instantiate<MeshTrack>();
            AddChild(Mesh);
            using (LoadTimer.Stage("track data + collision build")) Mesh.Load();
            Line = Mesh.Line;
            SpawnPose = Mesh.Spawn;
            Bounds = Mesh.Bounds;
            return;
        }
        var geometry = Create(name);
        Geometry = geometry;
        Line = geometry;
        var p = geometry.Pose(SpawnS);
        SpawnPose = (p.X, p.Y, p.Psi);

        var road = Load("road.gdshader");
        road.SetShaderParameter("road_half_width", geometry.Section.RoadHalfWidth);
        road.SetShaderParameter("track_length", geometry.Length);
        var grass = Load("grass.gdshader");
        var mesh = TrackMeshBuilder.BuildSurfaces(geometry, road, Load("curb.gdshader"), grass, Load("barrier.gdshader"),
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

        if (BuildScenery) TrackScenery.Build(this, geometry);
    }

    private static ShaderMaterial Load(string file)
        => new() { Shader = GD.Load<Shader>($"res://assets/materials/{file}") };
}
