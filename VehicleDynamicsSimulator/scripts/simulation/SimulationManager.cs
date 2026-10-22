using System;
using System.Globalization;
using System.Linq;
using Godot;
using Simulator.CameraRig;
using Simulator.Dynamics;
using Simulator.Effects;
using Simulator.Race;
using Simulator.Track;
using Simulator.UI;
using Simulator.Vehicle;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;

namespace Simulator.Simulation;

/// <summary>Choices carried from the main menu into the simulation scene.</summary>
public static class GameSession
{
    public static string TrackName { get; set; } = "Circuit";
    /// <summary>Optional configuration JSON (absolute path) to load at start.</summary>
    public static string? ConfigPath { get; set; }
    /// <summary>Racing-line driver aid shown (L / A / D-pad up); kept across tracks and restarts.</summary>
    public static bool RacingLineVisible { get; set; } = true;
    /// <summary>Where the racing line comes from (setup panel, or -- --racing-line=optimised|reference|&lt;csv&gt;).</summary>
    public static RacingLineSource RacingLineSource { get; set; } = RacingLineSource.Optimised;
    /// <summary>Telemetry CSV with the best lap, for <see cref="RacingLineSource.RecordedLap"/>.</summary>
    public static string? RacingLineCsv { get; set; }
}

public enum RacingLineSource
{
    /// <summary>Minimum-curvature line inside the track limits (RacingLineOptimizer).</summary>
    Optimised,
    /// <summary>The track's own reference line: AC fast_lane (Monaco) or the centreline (procedural tracks).</summary>
    TrackReference,
    /// <summary>Fastest valid lap of a telemetry CSV exported by the game.</summary>
    RecordedLap,
}

/// <summary>
/// Wires the per-step hooks of the vehicle's simulation loop: surface lookup before each step (E2), barrier
/// collisions (E3), lap timing and telemetry after each step. Also handles reset / recover / restart and routes
/// impact events to effects, audio and telemetry. Runs on the main thread like everything else.
/// </summary>
public partial class SimulationManager : Node
{
    [Export] public NodePath TrackPath { get; set; } = new("../Track");
    [Export] public NodePath VehiclePath { get; set; } = new("../Vehicle");
    [Export] public NodePath CameraPath { get; set; } = new("../Camera");

    public TrackManager Track { get; private set; } = null!;
    public VehicleController Vehicle { get; private set; } = null!;
    public ChaseCamera? Camera { get; private set; }
    public CollisionManager Collisions { get; private set; } = null!;
    public LapTimer Laps { get; private set; } = null!;
    public TelemetryService Telemetry { get; } = new();
    public Surface CurrentSurface { get; private set; }
    /// <summary>Last ground sample on a converted mesh track (null on procedural tracks).</summary>
    public GroundSample? CurrentGround { get; private set; }
    public double LastImpactSeverity { get; private set; }
    public string? LastMessage { get; private set; }
    public double LastMessageTime { get; private set; } = -1e9;

    public event Action<ImpactEvent>? Impact;
    public event Action? SimulationReset;

    public static string RunsDirectory => ProjectSettings.GlobalizePath("user://runs");
    public static string ConfigsDirectory => ProjectSettings.GlobalizePath("user://configs");

    private int _lastGear = 1;
    private bool _blownLogged;

    public override void _Ready()
    {
        Track = GetNode<TrackManager>(TrackPath);
        Vehicle = GetNode<VehicleController>(VehiclePath);
        Camera = GetNodeOrNull<ChaseCamera>(CameraPath);
        AddToGroup("simulation_manager");

        if (GameSession.ConfigPath is { } cfg)
        {
            if (ConfigSerializer.TryLoad(cfg, out var p, out var err)) Vehicle.ApplyParameters(p!);
            else Notify($"Configuration not loaded: {err}");
        }
        var trackArg = OS.GetCmdlineUserArgs().FirstOrDefault(a => a.StartsWith("--track="));
        if (trackArg != null) GameSession.TrackName = trackArg["--track=".Length..];
        if (OS.GetCmdlineUserArgs().FirstOrDefault(a => a.StartsWith("--racing-line=")) is { } rlArg)
        {
            string v = rlArg["--racing-line=".Length..];
            GameSession.RacingLineSource = v switch { "optimised" => RacingLineSource.Optimised, "reference" => RacingLineSource.TrackReference, _ => RacingLineSource.RecordedLap };
            if (GameSession.RacingLineSource == RacingLineSource.RecordedLap) GameSession.RacingLineCsv = v;
        }
        LoadTrack(GameSession.TrackName);

        Input.JoyConnectionChanged += OnJoyConnectionChanged;
        foreach (int id in Input.GetConnectedJoypads()) OnJoyConnectionChanged(id, true);

        var args = OS.GetCmdlineUserArgs();
        if (args.Contains("--selftest")) AddChild(new Dev.SelfTest { Name = "SelfTest" });
        Dev.ScreenshotRig.AttachIfRequested(this);
        Dev.PerfProbe.AttachIfRequested(this);
        if (args.Contains("--demo"))
        {
            // Scripted centreline driver through the normal input path (screenshots / attract mode).
            var driver = new Dev.CentrelineDriver(RacingLineTrack);
            Vehicle.AutoShift = true;
            Vehicle.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters);
        }
    }

    public void LoadTrack(string name)
    {
        Track.LoadTrack(name);
        GameSession.TrackName = name;
        Collisions = Track.Mesh is { } meshTrack
            ? new CollisionManager(meshTrack.WallContact)
            : new CollisionManager(Track.Geometry!);
        Collisions.Impact += OnImpact;
        Laps = new LapTimer(Track.Line);
        Laps.LapCompleted += lap =>
        {
            Telemetry.AddEvent(FormattableString.Invariant($"lap:{lap.Lap}:{lap.Time:F3}:{(lap.Valid ? "valid" : "invalid")}"));
            Notify(FormattableString.Invariant($"Lap {lap.Lap}: {FormatTime(lap.Time)}{(lap.Valid ? "" : " (invalid)")}"));
        };

        if (Track.Mesh is { } mesh)
        {
            // Converted track: surface (→ μ from surfaces.ini) and local grade from the physics mesh under the car;
            // the same sample places the car body on the ground. The Area3D gate is logged as a cross-check of the
            // step-exact lap timer (which stays authoritative: it runs at the 200 Hz physics rate).
            mesh.StartFinishEntered += () => Telemetry.AddEvent(FormattableString.Invariant($"sf_gate:lap={Laps.Lap}"));
            Vehicle.BeforeStep = (d, u) =>
            {
                var g = mesh.Sample(d.State.X, d.State.Y, d.State.Psi, Vehicle.Parameters);
                CurrentGround = g;
                u.Surface = g.Surface;
                u.SurfaceGrip = g.Grip;
                u.RoadGrade = g.Grade;
                CurrentSurface = u.Surface;
                return u;
            };
            Vehicle.GroundSnap = (x, y) => mesh.SnapToGround(x, y);
            Vehicle.GroundPose = (x, y, psi) =>
                mesh.Sample(x, y, psi, Vehicle.Parameters, updateHint: false) is { Hit: true } g ? (g.Height, g.Up) : null;
        }
        else
        {
            var geometry = Track.Geometry!;
            CurrentGround = null;
            Vehicle.BeforeStep = (d, u) =>
            {
                u.Surface = geometry.SurfaceAt(d.State.X, d.State.Y);
                CurrentSurface = u.Surface;
                return u;
            };
            Vehicle.GroundPose = null;
            Vehicle.GroundSnap = null;
        }
        Vehicle.AfterStep = (d, u) =>
        {
            Collisions.AfterStep(d);
            Laps.AfterStep(d.State, Vehicle.Loop.Clock.FixedDt);
            if (d.State.Gear != _lastGear)
            {
                Telemetry.AddEvent($"shift:{_lastGear}->{d.State.Gear}");
                _lastGear = d.State.Gear;
            }
            if (d.Outputs.TransmissionDamagedThisStep)
            {
                Telemetry.AddEvent(FormattableString.Invariant($"transmission_damaged:v={d.State.V * 3.6:F1}kmh"));
                Laps.InvalidateLap();
            }
            if (d.State.EngineBlown && !_blownLogged)
            {
                Telemetry.AddEvent("engine_blown");
                _blownLogged = true;
            }
            Telemetry.OnStep(d.State, d.Outputs, u, CurrentTrackSample(), Vehicle.Loop.Clock.FixedDt);
        };
        Vehicle.TrackContext = _ => CurrentTrackSample();

        BuildRacingLine();
        var (x, y, psi) = Track.SpawnPose;
        Vehicle.SetSpawn(x, y, psi);
        RestartSimulation();
    }

    /// <summary>Driver-aid racing line for the current track and vehicle parameters.</summary>
    public RacingLine RacingLine { get; private set; } = null!;

    /// <summary>The line the racing-line aid draws (and the demo driver follows).</summary>
    public ITrackLine RacingLineTrack { get; private set; } = null!;
    /// <summary>Human-readable source of <see cref="RacingLineTrack"/>.</summary>
    public string RacingLineInfo { get; private set; } = "";

    private static readonly System.Collections.Generic.Dictionary<string, ITrackLine> OptimisedCache = new();

    /// <summary>Re-reads the racing-line source (GameSession) and rebuilds the line and its colours.</summary>
    public void BuildRacingLine()
    {
        if (RacingLine == null)
        {
            RacingLine = new RacingLine { Name = "RacingLine" };
            AddChild(RacingLine);
        }
        RacingLineTrack = ResolveRacingLine(out string info);
        RacingLineInfo = info;
        RacingLine.Build(RacingLineTrack, Vehicle.Parameters, Track.Mesh);
        RacingLine.Visible = GameSession.RacingLineVisible;
    }

    private ITrackLine ResolveRacingLine(out string info)
    {
        if (GameSession.RacingLineSource == RacingLineSource.RecordedLap && GameSession.RacingLineCsv is { } csv)
        {
            try
            {
                var rec = RecordedLapLine.FromTelemetryCsv(csv);
                info = FormattableString.Invariant($"best lap {rec.Lap}{(rec.LapTime is { } t ? $" ({FormatTime(t)})" : "")} from {System.IO.Path.GetFileName(csv)}");
                return rec.Line;
            }
            catch (Exception e) when (e is System.IO.IOException or System.IO.InvalidDataException or FormatException or UnauthorizedAccessException)
            {
                Notify("Racing line CSV not usable: " + e.Message + " — using the optimised line");
            }
        }
        if (GameSession.RacingLineSource == RacingLineSource.TrackReference)
        {
            info = Track.Mesh != null ? "Assetto Corsa AI line" : "track centreline";
            return Track.Line;
        }
        // Margin from the limits to the CG line: half the body width (0.84 m) plus 0.36 m of clearance.
        if (!OptimisedCache.TryGetValue(Track.TrackName, out var line))
        {
            // Mesh tracks: optimise inside the wall-bounded corridor, not the bare AC edges.
            ITrackLine corridor = Track.Mesh?.WallBoundedCorridor() ?? Track.Line;
            line = RacingLineOptimizer.MinimumCurvature(corridor, margin: 1.2).Line;
            OptimisedCache[Track.TrackName] = line;
        }
        info = "minimum-curvature line";
        return line;
    }

    private TrackSample CurrentTrackSample() => new()
    {
        S = Laps.Projection.S,
        Lateral = Laps.Projection.Lateral,
        Surface = CurrentSurface,
        Lap = Laps.Lap,
        LapTime = Laps.CurrentLapTime,
        CollisionSeverity = Collisions.LastSeverity,
    };

    /// <summary>Car back on the grid at rest, laps and contacts cleared. Recording (if any) continues with an event.</summary>
    public void RestartSimulation()
    {
        Vehicle.ResetVehicle();
        Collisions.Reset();
        Laps.Reset();
        Telemetry.ClearLive();
        _lastGear = Vehicle.Dynamics.State.Gear;
        _blownLogged = false;
        Telemetry.AddEvent("restart");
        Camera?.Snap();
        SimulationReset?.Invoke();
    }

    /// <summary>
    /// Recover to track: an explicit reset at rest on the centreline at the nearest arc length, heading along the
    /// track. The lap in progress becomes invalid.
    /// </summary>
    public void RecoverToTrack()
    {
        var st = Vehicle.Dynamics.State;
        var pr = Track.Line.Project(st.X, st.Y);
        var pose = Track.Line.Pose(pr.S);
        Vehicle.ResetVehicleAt(pose.X, pose.Y, pose.Psi, keepEngineDamage: true);
        Collisions.Reset();
        Laps.InvalidateLap();
        _lastGear = 1;
        Telemetry.AddEvent(FormattableString.Invariant($"recover:s={pr.S:F1}"));
        Notify("Recovered to track (lap invalid)");
        Camera?.Snap();
    }

    public void ToggleRecording()
    {
        if (Telemetry.Recording)
        {
            var e = Telemetry.Stop(Vehicle);
            Notify($"Recording stopped: {e?.Run.Samples.Count ?? 0} samples");
        }
        else
        {
            Telemetry.Start(Vehicle, Track.TrackName);
            Notify("Recording started");
        }
    }

    public string? ExportLatestRun()
    {
        if (Telemetry.Recording) Telemetry.Stop(Vehicle);
        var last = Telemetry.Runs.LastOrDefault();
        if (last == null) { Notify("Nothing recorded yet (F5 to record)"); return null; }
        var path = TelemetryService.Export(last, RunsDirectory);
        Notify($"Exported {path}");
        return path;
    }

    /// <summary>Applies parameters between steps. A dt change restarts the recording so each CSV has one sample interval.</summary>
    public void ApplyParameters(VehicleParameters p)
    {
        bool dtChanged = p.Dt != Vehicle.Parameters.Dt;
        bool wasRecording = Telemetry.Recording;
        if (dtChanged && wasRecording) Telemetry.Stop(Vehicle);
        Vehicle.ApplyParameters(p);
        if (dtChanged && wasRecording) Telemetry.Start(Vehicle, Track.TrackName);
        Telemetry.AddEvent("parameters_applied");
        BuildRacingLine();   // braking/acceleration zones follow the new setup
        Notify("Setup applied");
    }

    public override void _ExitTree() => Input.JoyConnectionChanged -= OnJoyConnectionChanged;

    /// <summary>
    /// Announces gamepads. Godot maps known controllers (SDL database) to the standard XInput layout; an unknown one
    /// is usually a Logitech F710 in DirectInput mode (switch on the back set to "D").
    /// </summary>
    private void OnJoyConnectionChanged(long device, bool connected)
    {
        int id = (int)device;
        if (!connected) { Notify($"Gamepad {id} disconnected"); return; }
        string name = Input.GetJoyName(id);
        Notify(Input.IsJoyKnown(id)
            ? $"Gamepad connected: {name} (RT throttle, LT brake, left stick steer, RB/LB shift)"
            : $"Gamepad '{name}' has no standard mapping: set the F710 switch to X (XInput)");
    }

    public void Notify(string message)
    {
        LastMessage = message;
        LastMessageTime = Time.GetTicksMsec() / 1000.0;
        GD.Print(message);
    }

    public override void _Process(double delta)
    {
        if (UiFocus.IsTyping(GetViewport())) return;
        if (Input.IsActionJustPressed(InputSetup.ResetVehicle)) RestartSimulation();
        if (Input.IsActionJustPressed(InputSetup.Recover)) RecoverToTrack();
        if (Input.IsActionJustPressed(InputSetup.ToggleRecording)) ToggleRecording();
        if (Input.IsActionJustPressed(InputSetup.CycleGraphics) && Camera != null) Notify("Graphics: " + Camera.CycleGraphics());
        if (Input.IsActionJustPressed(InputSetup.ToggleRacingLine))
        {
            GameSession.RacingLineVisible = !GameSession.RacingLineVisible;
            RacingLine.Visible = GameSession.RacingLineVisible;
            Notify(GameSession.RacingLineVisible
                ? FormattableString.Invariant($"Racing line on: {RacingLineInfo} (reference lap {FormatTime(RacingLine.LapTime)}, quasi-steady estimate)")
                : "Racing line off");
        }
        if (Input.IsActionJustPressed(InputSetup.Menu)) GetTree().ChangeSceneToFile("res://scenes/MainMenu.tscn");
    }

    private void OnImpact(ImpactEvent e)
    {
        Laps.InvalidateLap();
        LastImpactSeverity = e.Severity;
        Telemetry.AddEvent(FormattableString.Invariant($"impact:{e.Severity:F2}"));
        var point = VehicleModelAdapter.ToGodotPosition(e.PointX, e.PointY, 0.5);
        var normal = VehicleModelAdapter.ToGodotPosition(e.NormalX, e.NormalY);
        ImpactEffects.Sparks(this, point, normal, e.Severity);
        Camera?.AddShake((float)e.Severity / 8f);
        Impact?.Invoke(e);
    }

    public static string FormatTime(double? t)
    {
        if (t is not { } v) return "--:--.---";
        int m = (int)(v / 60);
        return string.Create(CultureInfo.InvariantCulture, $"{m}:{v - 60 * m:00.000}");
    }
}
