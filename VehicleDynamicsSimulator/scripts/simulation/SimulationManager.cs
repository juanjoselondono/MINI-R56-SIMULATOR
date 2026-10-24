using System;
using System.Globalization;
using System.Linq;
using System.Threading.Tasks;
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
    /// <summary>
    /// Optional named setup (absolute path, in the vehicle's own folder) to load at start instead of the vehicle's
    /// current setup (setup_&lt;key&gt;.json). It must belong to <see cref="VehicleProfile"/>.
    /// </summary>
    public static string? ConfigPath { get; set; }
    /// <summary>Start with the profile's factory defaults instead of its saved setup (main menu choice).</summary>
    public static bool FactoryDefaults { get; set; }
    /// <summary>
    /// Saved setups are used (and written) only in interactive play: command-line test modes always drive the profile
    /// defaults so their results do not depend on what the player saved.
    /// </summary>
    public static bool UseSavedSetups(string[] args) => !IsTestRun(args);

    public static bool IsTestRun(string[] args) => args.Any(a => a.StartsWith("--autodrive") || a == "--selftest" || a.StartsWith("--selftest-only")
        || a == "--demo" || a.StartsWith("--track=") || a.StartsWith("--shots=") || a.StartsWith("--perf=") || a == "--via-loading-screen");
    /// <summary>Vehicle profile to drive (VehicleProfiles id), chosen in the main menu or the setup panel.</summary>
    public static string VehicleProfile { get; set; } = VehicleDynamics.Core.Simulation.VehicleProfiles.MiniR56;
    /// <summary>
    /// Set by <see cref="SceneManager.StartSession"/>: the main scene builds its session in stages behind the loading
    /// screen (racing line on a worker thread). Command-line modes leave it false and load synchronously.
    /// </summary>
    public static bool StagedLoad { get; set; }
    /// <summary>Racing-line driver aid shown (L / A / D-pad up); kept across tracks and restarts.</summary>
    public static bool RacingLineVisible { get; set; } = true;
    /// <summary>Track minimap shown (F6); kept across tracks and restarts.</summary>
    public static bool MinimapVisible { get; set; } = true;
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
    /// <summary>All-time top-10 laps per track and vehicle, with webcam podium photos.</summary>
    public LeaderboardService Leaderboard { get; private set; } = null!;
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
    private static bool _setupsMigrated;

    /// <summary>
    /// Per-vehicle setup files under <see cref="ConfigsDirectory"/> (setup_mini.json, setup_f1.json, mini/, f1/). The
    /// first access moves configurations saved before the split into their vehicle's folder.
    /// </summary>
    public static VehicleSetupStore Setups
    {
        get
        {
            var store = new VehicleSetupStore(ConfigsDirectory);
            if (!_setupsMigrated)
            {
                _setupsMigrated = true;
                try
                {
                    int moved = store.MigrateLegacy();
                    if (moved > 0) GD.Print($"Setups: moved {moved} configuration(s) into their vehicle folders under {ConfigsDirectory}");
                }
                catch (Exception ex) when (ex is System.IO.IOException or UnauthorizedAccessException) { GD.PushWarning("Setup migration: " + ex.Message); }
            }
            return store;
        }
    }

    /// <summary>The parameters a vehicle starts with: its saved setup in play, its defaults in test runs.</summary>
    public VehicleParameters SetupFor(string profileId)
    {
        if (!GameSession.UseSavedSetups(OS.GetCmdlineUserArgs())) return VehicleProfiles.Create(profileId);
        var p = Setups.LoadActive(profileId, out var err);
        if (err != null) Notify($"Saved setup ignored ({err}); using the defaults");
        return p;
    }

    /// <summary>Stores the running parameters as the current vehicle's own setup (setup_&lt;key&gt;.json).</summary>
    public string? SaveCurrentSetup()
    {
        if (!GameSession.UseSavedSetups(OS.GetCmdlineUserArgs())) return null;
        try
        {
            return Setups.SaveActive(Vehicle.Parameters);
        }
        catch (Exception ex) when (ex is System.IO.IOException or UnauthorizedAccessException or ArgumentException)
        {
            Notify("Setup not saved: " + ex.Message);
            return null;
        }
    }

    private int _lastGear = 1;
    private bool _blownLogged;

    public override void _Ready()
    {
        Track = GetNode<TrackManager>(TrackPath);
        Vehicle = GetNode<VehicleController>(VehiclePath);
        Camera = GetNodeOrNull<ChaseCamera>(CameraPath);
        AddToGroup("simulation_manager");
        bool testRun = GameSession.IsTestRun(OS.GetCmdlineUserArgs());
        // Dev: `--leaderboard-file=PATH` shows another board (read-only in test runs; photos in PATH's folder/photos).
        string? boardFile = OS.GetCmdlineUserArgs().FirstOrDefault(x => x.StartsWith("--leaderboard-file="))?["--leaderboard-file=".Length..];
        Leaderboard = boardFile == null
            ? new LeaderboardService { Name = "Leaderboard", Enabled = !testRun, CameraAllowed = !testRun }
            : new LeaderboardService
            {
                Name = "Leaderboard", Enabled = !testRun, CameraAllowed = !testRun, FilePath = boardFile,
                PhotosDirectory = System.IO.Path.Combine(System.IO.Path.GetDirectoryName(boardFile) ?? ".", "photos"),
            };
        AddChild(Leaderboard);

        if (OS.GetCmdlineUserArgs().FirstOrDefault(a => a.StartsWith("--vehicle=")) is { } vArg)
            GameSession.VehicleProfile = vArg["--vehicle=".Length..];
        bool configured = false;
        if (GameSession.ConfigPath is { } cfg)
        {
            try
            {
                Vehicle.ApplyParameters(VehicleSetupStore.LoadFor(GameSession.VehicleProfile, cfg));
                configured = true;
            }
            catch (Exception ex) when (ex is ArgumentException or System.IO.IOException or UnauthorizedAccessException)
            {
                Notify($"Configuration not loaded: {ex.Message}");
            }
        }
        if (!configured)
            using (LoadTimer.Stage("vehicle " + GameSession.VehicleProfile))
                Vehicle.ApplyParameters(GameSession.FactoryDefaults ? VehicleProfiles.Create(GameSession.VehicleProfile) : SetupFor(GameSession.VehicleProfile));
        Vehicle.VehicleChanged += OnVehicleChanged;
        var trackArg = OS.GetCmdlineUserArgs().FirstOrDefault(a => a.StartsWith("--track="));
        if (trackArg != null) GameSession.TrackName = trackArg["--track=".Length..];
        if (OS.GetCmdlineUserArgs().FirstOrDefault(a => a.StartsWith("--racing-line=")) is { } rlArg)
        {
            string v = rlArg["--racing-line=".Length..];
            GameSession.RacingLineSource = v switch { "optimised" => RacingLineSource.Optimised, "reference" => RacingLineSource.TrackReference, _ => RacingLineSource.RecordedLap };
            if (GameSession.RacingLineSource == RacingLineSource.RecordedLap) GameSession.RacingLineCsv = v;
        }
        if (GameSession.StagedLoad && SceneManager.Instance is { } loader)
        {
            // Menu start: the assets are already loaded (SceneManager); build in stages behind the loading screen.
            GameSession.StagedLoad = false;
            Loading = true;
            using (LoadTimer.Stage("track core (staged)")) LoadTrackCore(GameSession.TrackName);
            EnsureRacingLineNode();
            FinishTrackLoad();
            Vehicle.SetPaused(true);
            _ = CompleteStagedLoadAsync(loader);
        }
        else using (LoadTimer.Stage("session total (track + line)")) LoadTrack(GameSession.TrackName);

        Input.JoyConnectionChanged += OnJoyConnectionChanged;
        foreach (int id in Input.GetConnectedJoypads()) OnJoyConnectionChanged(id, true);

        var args = OS.GetCmdlineUserArgs();
        if (args.Contains("--selftest")) AddChild(new Dev.SelfTest { Name = "SelfTest" });
        Dev.ScreenshotRig.AttachIfRequested(this);
        Dev.PerfProbe.AttachIfRequested(this);
        if (args.Contains("--demo"))
        {
            // Scripted centreline driver through the normal input path (screenshots / attract mode).
            var driver = new Dev.CentrelineDriver(RacingLineTrack)
            {
                LookaheadTime = Vehicle.Parameters.Profile == VehicleProfiles.F1 ? 0.45 : 0.7,
            };
            Vehicle.AutoShift = true;
            Vehicle.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters);
        }
    }

    /// <summary>True while a session or vehicle change is being built behind the loading screen (input ignored).</summary>
    public bool Loading { get; private set; }

    private async Task CompleteStagedLoadAsync(SceneManager loader)
    {
        loader.Report(0.80, "Racing line");
        await loader.NextFrame();
        await BuildRacingLineAsync(f => loader.ReportFromWorker(0.80 + 0.18 * f,
            "Optimising the racing line (first time for this track and car; cached afterwards)"));
        RestartSimulation();
        Vehicle.SetPaused(false);
        Loading = false;
        loader.Finish();
    }

    /// <summary>Synchronous track load (command-line modes, self-test): core, racing line, spawn.</summary>
    public void LoadTrack(string name)
    {
        LoadTrackCore(name);
        BuildRacingLine();
        FinishTrackLoad();
    }

    /// <summary>Track scene, collisions, lap timer and the per-step hooks (everything but the racing line).</summary>
    private void LoadTrackCore(string name)
    {
        using (LoadTimer.Stage("track " + name)) Track.LoadTrack(name);
        GameSession.TrackName = name;
        Collisions = Track.Mesh is { } meshTrack
            ? new CollisionManager(meshTrack.WallContact)
            : new CollisionManager(Track.Geometry!);
        Collisions.Impact += OnImpact;
        ConfigureCollisions();
        Laps = new LapTimer(Track.Line);
        Laps.LapCompleted += lap =>
        {
            // ":valid" kept in the event so telemetry CSVs stay readable by RecordedLapLine (every lap counts now).
            Telemetry.AddEvent(FormattableString.Invariant($"lap:{lap.Lap}:{lap.Time:F3}:valid"));
            Notify(FormattableString.Invariant($"Lap {lap.Lap}: {FormatTime(lap.Time)}"));
            if (Leaderboard.Record(Track.TrackName, Vehicle.Parameters, lap.Time) is { } podium)
                Notify(FormattableString.Invariant($"TOP 10 — P{podium.Rank} all time on {Track.TrackName}: {FormatTime(lap.Time)}"));
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
            }
            if (d.State.EngineBlown && !_blownLogged)
            {
                Telemetry.AddEvent("engine_blown");
                _blownLogged = true;
            }
            Telemetry.OnStep(d.State, d.Outputs, u, CurrentTrackSample(), Vehicle.Loop.Clock.FixedDt);
        };
        Vehicle.TrackContext = _ => CurrentTrackSample();
    }

    private void FinishTrackLoad()
    {
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

    private void EnsureRacingLineNode()
    {
        if (RacingLine != null) return;
        RacingLine = new RacingLine { Name = "RacingLine" };
        AddChild(RacingLine);
    }

    /// <summary>Re-reads the racing-line source (GameSession) and rebuilds the line and its colours (synchronous).</summary>
    public void BuildRacingLine()
    {
        EnsureRacingLineNode();
        var line = ResolveNonOptimised(out string info);
        if (line == null)
        {
            var prep = PrepareOptimised();
            line = prep.Line ?? StoreOptimised(prep, Optimise(prep, null));
            info = "minimum-curvature line";
        }
        ShowRacingLine(line, info);
    }

    /// <summary>Same, with the optimisation (if it is not cached) on a worker thread; <paramref name="progress"/> is called there.</summary>
    public async Task BuildRacingLineAsync(Action<double>? progress)
    {
        EnsureRacingLineNode();
        var line = ResolveNonOptimised(out string info);
        if (line == null)
        {
            var prep = PrepareOptimised();
            if (prep.Line != null) line = prep.Line;
            else
            {
                var computed = await Task.Run(() => Optimise(prep, progress));
                line = StoreOptimised(prep, computed);
            }
            info = "minimum-curvature line";
        }
        ShowRacingLine(line, info);
    }

    private void ShowRacingLine(ITrackLine line, string info)
    {
        RacingLineTrack = line;
        RacingLineInfo = info;
        using (LoadTimer.Stage("racing line: heights + reference lap + mesh")) RacingLine.Build(RacingLineTrack, Vehicle.Parameters, Track.Mesh);
        RacingLine.Visible = GameSession.RacingLineVisible;
    }

    /// <summary>The recorded-lap or track-reference line, or null when the optimised line is wanted.</summary>
    private ITrackLine? ResolveNonOptimised(out string info)
    {
        info = "";
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
        return null;
    }

    private sealed record OptimisedRequest(ITrackLine? Line, ITrackLine Corridor, double Margin, string MemoryKey, string DiskKey);

    /// <summary>
    /// Main thread: margin, corridor (mesh tracks: raycasts to the walls) and the cache lookups (memory, then disk).
    /// Line is set when no optimisation is needed.
    /// </summary>
    private OptimisedRequest PrepareOptimised()
    {
        // Margin from the limits to the CG line: half the car's width plus 0.36 m of clearance (Mini 1.20 m, F1 1.36 m).
        double margin = Vehicle.Visual.HalfWidth + 0.36;
        string memoryKey = FormattableString.Invariant($"{Track.TrackName}|{margin:F2}");
        if (OptimisedCache.TryGetValue(memoryKey, out var cached)) return new OptimisedRequest(cached, Track.Line, margin, memoryKey, "");
        ITrackLine corridor;
        using (LoadTimer.Stage("racing line: wall corridor")) corridor = Track.Mesh?.WallBoundedCorridor() ?? Track.Line;
        string diskKey = RacingLineCache.Key(Track.TrackName, margin, corridor);
        if (RacingLineCache.TryLoad(diskKey, Track.Line.Name + " (racing line)", out var fromDisk))
        {
            GD.Print("[load] racing line: from disk cache " + diskKey);
            OptimisedCache[memoryKey] = fromDisk;
            return new OptimisedRequest(fromDisk, corridor, margin, memoryKey, diskKey);
        }
        return new OptimisedRequest(null, corridor, margin, memoryKey, diskKey);
    }

    /// <summary>Pure computation (any thread).</summary>
    private static PolylineTrack Optimise(OptimisedRequest r, Action<double>? progress)
    {
        using (LoadTimer.Stage("racing line: minimum-curvature optimiser"))
            return RacingLineOptimizer.MinimumCurvature(r.Corridor, margin: r.Margin, progress: progress).Line;
    }

    private ITrackLine StoreOptimised(OptimisedRequest r, PolylineTrack line)
    {
        OptimisedCache[r.MemoryKey] = line;
        RacingLineCache.Save(r.DiskKey, line);
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
    /// track. The lap in progress keeps running.
    /// </summary>
    public void RecoverToTrack()
    {
        var st = Vehicle.Dynamics.State;
        var pr = Track.Line.Project(st.X, st.Y);
        var pose = Track.Line.Pose(pr.S);
        Vehicle.ResetVehicleAt(pose.X, pose.Y, pose.Psi, keepEngineDamage: true);
        Collisions.Reset();
        _lastGear = 1;
        Telemetry.AddEvent(FormattableString.Invariant($"recover:s={pr.S:F1}"));
        Notify("Recovered to track");
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
    /// <summary>Drive another vehicle profile: its own saved setup (or defaults), 3D model and footprint; back to the grid.</summary>
    public void SwitchVehicle(string profileId)
    {
        if (profileId == Vehicle.Parameters.Profile) return;
        ApplyParameters(SetupFor(profileId));
    }

    /// <summary>
    /// Change of car during a session behind the loading screen: the vehicle model loads on Godot's loader threads, the
    /// racing line (new car width) is optimised on a worker thread unless cached, then back to the grid.
    /// <paramref name="done"/> runs afterwards on the main thread. Without the SceneManager autoload: synchronous.
    /// </summary>
    public async void ChangeVehicleAsync(VehicleParameters p, Action? done = null)
    {
        if (Loading) return;
        var loader = SceneManager.Instance;
        if (loader == null || p.Profile == Vehicle.Parameters.Profile)
        {
            ApplyParameters(p);
            done?.Invoke();
            return;
        }
        Loading = true;
        loader.Show($"{Track.TrackName}  ·  {VehicleProfiles.DisplayName(p.Profile)}");
        await loader.NextFrame();
        if (!await loader.LoadThreaded(new[] { VehicleCatalog.Get(p.Profile).VisualScene }, 0, 0.6, "Loading the vehicle model"))
        {
            Loading = false;
            return;
        }
        loader.Report(0.62, "Vehicle setup");
        await loader.NextFrame();
        if (Telemetry.Recording) Telemetry.Stop(Vehicle);
        Vehicle.ApplyParameters(p);
        Vehicle.SetPaused(true);
        await BuildRacingLineAsync(f => loader.ReportFromWorker(0.65 + 0.33 * f,
            "Optimising the racing line (first time for this track and car; cached afterwards)"));
        RestartSimulation();
        Vehicle.SetPaused(false);
        Loading = false;
        loader.Finish();
        Notify("Vehicle: " + VehicleProfiles.DisplayName(p.Profile));
        done?.Invoke();
    }

    public void SwitchVehicleAsync(string profileId, Action? done = null) => ChangeVehicleAsync(SetupFor(profileId), done);

    /// <summary>Collision footprint of the current car (asset-pipeline AABB).</summary>
    private void ConfigureCollisions()
    {
        Collisions.HalfLength = Vehicle.Visual.HalfLength;
        Collisions.HalfWidth = Vehicle.Visual.HalfWidth;
        Collisions.CentreBehindCg = Vehicle.Visual.CentreBehindCg;
    }

    private void OnVehicleChanged()
    {
        GameSession.VehicleProfile = Vehicle.Parameters.Profile;
        ConfigureCollisions();
        Telemetry.AddEvent("vehicle:" + Vehicle.Parameters.Profile);
    }

    public void ApplyParameters(VehicleParameters p)
    {
        if (p.Profile != Vehicle.Parameters.Profile)
        {
            // Another car: stop a recording (its metadata describes one car), swap, rebuild the line, back to the grid.
            if (Telemetry.Recording) Telemetry.Stop(Vehicle);
            Vehicle.ApplyParameters(p);
            BuildRacingLine();
            RestartSimulation();
            Notify("Vehicle: " + VehicleProfiles.DisplayName(p.Profile));
            return;
        }
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
        if (Loading) return;
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
