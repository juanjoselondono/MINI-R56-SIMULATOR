using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using Godot;
using Simulator.Race;
using Simulator.Simulation;
using Simulator.Track;
using Simulator.UI;
using Simulator.Vehicle;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;

namespace Simulator.Dev;

/// <summary>
/// In-game self-test (`-- --selftest`, run with --headless --fixed-fps 60 for determinism). Drives the real scene on
/// both tracks through the normal input path (scripted driver levels and shift requests), checks the results, prints a
/// PASS/FAIL table and quits with exit code 0 (all pass) or 1.
/// </summary>
public partial class SelfTest : Node
{
    private SimulationManager _sim = null!;
    private readonly List<(string Track, string Name, bool Pass, string Detail)> _results = new();
    private string _track = "";

    public override void _Ready()
    {
        _sim = GetParent<SimulationManager>();
        _ = Run();
    }

    private VehicleController V => _sim.Vehicle;
    private VehicleDynamics.Core.Simulation.VehicleDynamics D => _sim.Vehicle.Dynamics;

    private void Check(string name, bool pass, string detail)
    {
        _results.Add((_track, name, pass, detail));
        GD.Print($"[selftest] {_track,-8} {(pass ? "PASS" : "FAIL")}  {name}: {detail}");
    }

    private async Task Frames(int n)
    {
        for (int i = 0; i < n; i++) await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
    }

    /// <summary>Waits until <paramref name="cond"/> holds or <paramref name="simSeconds"/> of simulated time pass.</summary>
    private async Task<bool> Until(Func<bool> cond, double simSeconds)
    {
        double t0 = D.State.Time;
        int guard = 0;
        while (!cond())
        {
            if (D.State.Time - t0 > simSeconds || guard++ > 200_000) return false;
            await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
        }
        return true;
    }

    private static string F(double v, string fmt = "0.###") => v.ToString(fmt, CultureInfo.InvariantCulture);

    private async Task Run()
    {
        try
        {
            await Frames(2);
            // --selftest-only=Monaco runs just the converted-track suite (quick check after re-converting assets).
            bool monacoOnly = OS.GetCmdlineUserArgs().Contains("--selftest-only=Monaco"), boardOnly = false;
            if (OS.GetCmdlineUserArgs().Contains("--selftest-only=Nurburgring"))
            {
                _track = "Nurb";
                _sim.LoadTrack("Nurburgring");
                await Frames(2);
                await ConvertedTrackSuite();
                monacoOnly = true;
                boardOnly = true;
            }
            else if (OS.GetCmdlineUserArgs().Contains("--selftest-only=Leaderboard"))
            {
                // Best laps and per-vehicle setups only; add --with-webcam to take a real photo through the camera.
                _track = "board";
                await LeaderboardSuite();
                monacoOnly = true;
                boardOnly = true;
            }
            else if (OS.GetCmdlineUserArgs().Contains("--selftest-only=F1"))
            {
                _track = "F1";
                _sim.LoadTrack("Circuit");
                await Frames(2);
                await VehicleProfileSuite();
                monacoOnly = true;   // skip the Mini suites; Monaco below still runs only if requested
            }
            else if (!monacoOnly)
            {
                foreach (var track in new[] { "Oval", "Circuit" })
                {
                    _track = track;
                    _sim.LoadTrack(track);
                    await Frames(2);
                    await TrackSuite();
                }
                _track = "both";
                await UiAndComparison();
                _track = "board";
                await LeaderboardSuite();
                _track = "F1";
                _sim.LoadTrack("Circuit");
                await Frames(2);
                await VehicleProfileSuite();
            }
            bool fullRun = !OS.GetCmdlineUserArgs().Any(x => x.StartsWith("--selftest-only="));
            if (fullRun && TrackManager.Available.Contains("Nurburgring"))
            {
                _track = "Nurb";
                _sim.LoadTrack("Nurburgring");
                await Frames(2);
                await ConvertedTrackSuite();
            }
            if (boardOnly) { }
            else if (TrackManager.Available.Contains("Monaco"))
            {
                _track = "Monaco";
                _sim.LoadTrack("Monaco");
                await Frames(2);
                await MeshTrackSuite();
            }
            else GD.Print("[selftest] Monaco assets not converted (see docs/run_instructions.md) — Monaco suite skipped");
        }
        catch (Exception ex)
        {
            Check("exception", false, ex.GetType().Name + ": " + ex.Message);
        }

        int failed = _results.Count(r => !r.Pass);
        GD.Print("\n================ SELF-TEST SUMMARY ================");
        foreach (var r in _results) GD.Print($"{(r.Pass ? "PASS" : "FAIL")}  {r.Track,-8} {r.Name,-28} {r.Detail}");
        GD.Print($"==== {_results.Count - failed}/{_results.Count} passed ====");
        AppQuit.Quit(this, failed == 0 ? 0 : 1);
    }

    /// <summary>
    /// All-time best laps (own file in user://selftest, never the player's board) and the per-vehicle setup rules.
    /// With --with-webcam the last top lap takes a real podium photo through CameraServer.
    /// </summary>
    private async Task LeaderboardSuite()
    {
        // Test runs drive the profile defaults and never record laps, whatever the player saved.
        Check("test run: defaults, no board", !_sim.Leaderboard.Enabled
            && ConfigSerializer.ToJson(_sim.SetupFor(VehicleProfiles.F1)) == ConfigSerializer.ToJson(VehicleParameters.F1Provisional()),
            "saved setups and the leaderboard are left alone");

        // Setup panel rows follow the vehicle.
        var f1Rows = ParameterCatalog.Build(VehicleParameters.F1Provisional()).Select(e => e.Name).ToList();
        var miniRows = ParameterCatalog.Build(VehicleParameters.MiniCooperR56()).Select(e => e.Name).ToList();
        bool rows = f1Rows.Contains("Brake bias (front share)") && f1Rows.Contains("Lift coefficient C_l") && !f1Rows.Contains("Front spring K_s,f")
            && miniRows.Contains("Front spring K_s,f") && !miniRows.Contains("Brake bias (front share)") && !miniRows.Contains("Traction control (driver aid)");
        Check("setup rows per vehicle", rows, $"F1 {f1Rows.Count} rows, Mini {miniRows.Count} rows");

        bool webcam = OS.GetCmdlineUserArgs().Contains("--with-webcam");
        string dir = ProjectSettings.GlobalizePath($"user://selftest/board_{DateTime.Now:yyyyMMdd_HHmmss}");
        var service = new LeaderboardService
        {
            Name = "TestBoard", FilePath = Path.Combine(dir, "leaderboard.json"), PhotosDirectory = Path.Combine(dir, "photos"),
            Enabled = true, CameraAllowed = webcam,
        };
        AddChild(service);
        var mini = VehicleParameters.MiniCooperR56();
        for (int i = 0; i < 12; i++) service.Record("Circuit", mini, 90 + i, valid: true);
        bool invalidIgnored = service.Record("Circuit", mini, 50, valid: false) == null;
        var custom = mini.Clone();
        custom.Mass = 1000;
        var podium = service.Record("Circuit", custom, 89.5, valid: true);
        var reloaded = Leaderboard.Load(service.FilePath, out var err);
        var top = reloaded.Top("Circuit", VehicleProfiles.MiniR56);
        Check("top 10 saved as JSON", err == null && invalidIgnored && top.Count == 10 && top[0].Time == 89.5 && top[0].CustomSetup
            && top[^1].Time == 98 && podium?.Rank == 1 && reloaded.Top("Circuit", VehicleProfiles.F1).Count == 0,
            $"{top.Count} entries, best {F(top[0].Time)} s (custom setup {top[0].CustomSetup}), 10th {F(top[^1].Time)} s; {service.FilePath}");

        if (webcam && podium != null)
        {
            ulong t0 = Time.GetTicksMsec();
            while (!podium.Finished && Time.GetTicksMsec() - t0 < 12000) await Frames(1);
            var entry = Leaderboard.Load(service.FilePath, out _).Find(podium.Entry.Id);
            string? photo = entry?.Photo == null ? null : Path.Combine(service.PhotosDirectory, entry.Photo);
            var img = photo != null && File.Exists(photo) ? Image.LoadFromFile(photo) : null;
            Check("webcam podium photo", img != null && !img.IsEmpty(),
                img != null ? $"{photo} ({img.GetWidth()}×{img.GetHeight()})" : podium.PhotoNote ?? "no photo");
        }
        service.QueueFree();
        await Frames(1);
    }

    private async Task TrackSuite()
    {
        // 1. Grid: at rest on the spawn pose.
        var s0 = D.State;
        var (sx, sy, spsi) = _sim.Track.SpawnPose;
        Check("spawn at rest on the grid", s0.V == 0 && Math.Abs(s0.X - sx) < 1e-9 && Math.Abs(s0.Y - sy) < 1e-9 && Math.Abs(s0.Psi - spsi) < 1e-12,
            $"V={F(s0.V)} pose=({F(s0.X)}, {F(s0.Y)}, {F(s0.Psi)})");

        // 2. Fixed step: number of physics steps = simulated time / dt, whatever the frame pattern.
        long steps0 = V.Loop.Clock.StepsTaken;
        double t0 = D.State.Time;
        await Frames(37);
        long dSteps = V.Loop.Clock.StepsTaken - steps0;
        double dT = D.State.Time - t0;
        Check("fixed physics step", Math.Abs(dSteps * V.Parameters.Dt - dT) < 1e-9 && dSteps > 0, $"{dSteps} steps over {F(dT, "0.000")} s sim (dt={V.Parameters.Dt})");

        // 3. A full lap with the scripted centreline driver, recorded.
        _sim.RestartSimulation();
        var driver = new CentrelineDriver(_sim.Track.Line);
        V.AutoShift = true;
        V.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters);
        _sim.Telemetry.Start(V, _sim.Track.TrackName, $"selftest_{_sim.Track.TrackName}");
        int impulses0 = _sim.Collisions.ImpulseCount;
        bool lap = await Until(() => _sim.Laps.Laps.Count > 0, 300);
        var entry = _sim.Telemetry.Stop(V);
        var rec = lap ? _sim.Laps.Laps[0] : default;
        Check("scripted lap counted", lap && rec.Valid && _sim.Collisions.ImpulseCount == impulses0,
            lap ? $"lap {F(rec.Time, "0.00")} s, valid={rec.Valid}, impacts={_sim.Collisions.ImpulseCount - impulses0}" : "no lap within 300 s");
        _referenceLap = lap ? rec.Time : double.NaN;
        Check("speedometer = physics state", Math.Abs(_sim.Telemetry.Latest.VKmh - D.State.V * 3.6) < 1e-9, $"{F(D.State.V * 3.6, "0.00")} km/h");

        // 4. CSV export round trip.
        if (entry != null)
        {
            string path = TelemetryService.Export(entry, Path.Combine(SimulationManager.RunsDirectory, "selftest"));
            var lines = File.ReadAllLines(path);
            var header = lines.First(l => !l.StartsWith('#'));
            var rows = lines.SkipWhile(l => l.StartsWith('#')).Skip(1).Select(l => l.Split(',')).ToList();
            var times = rows.Select(r => double.Parse(r[0], CultureInfo.InvariantCulture)).ToList();
            double worst = 0;
            for (int i = 2; i < times.Count; i++) worst = Math.Max(worst, Math.Abs(times[i] - times[i - 1] - 0.02));
            bool headerOk = header == string.Join(",", SimulationRecorder.Columns);
            string massLine = "# Mass=" + V.Parameters.Mass.ToString("R", CultureInfo.InvariantCulture);
            bool meta = lines.Any(l => l == massLine) && lines.Any(l => l == $"# track={_sim.Track.TrackName}");
            bool cfg = File.Exists(Path.ChangeExtension(path, null) + ".config.json");
            Check("CSV export", headerOk && meta && cfg && worst < 1e-9 && rows.Count > 100,
                $"{rows.Count} rows, interval error {worst:E1} s, header={headerOk}, metadata={meta}, config={cfg} → {path}");
        }

        // 5. Barrier: from racing speed, steer into the right-hand wall and keep pushing for 12 s. Impulses only on
        //    contact entry or after a separation (never on consecutive steps), no speed gain, never through the wall.
        driver.TargetLateral = -40;
        driver.MaxSpeed = 25;
        var cm = _sim.Collisions;
        int imp0 = cm.ImpulseCount, ep0 = cm.ContactEpisodes, re0 = cm.Rearms, cons0 = cm.ConsecutiveImpulses;
        double worstBeyond = double.NegativeInfinity, firstImpactSpeed = -1, firstAfter = -1;
        double vLastStep = D.State.V;
        void OnImpact(ImpactEvent e)
        {
            if (firstImpactSpeed < 0) { firstImpactSpeed = vLastStep; firstAfter = D.State.V; }
        }
        cm.Impact += OnImpact;
        var prevAfter = V.AfterStep;
        V.AfterStep = (d, u) => { vLastStep = d.State.V; prevAfter?.Invoke(d, u); };
        double tEnd = D.State.Time + 12;
        while (D.State.Time < tEnd)
        {
            await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
            var g = _sim.Track.Geometry!;
            var pr = g.Project(D.State.X, D.State.Y);
            worstBeyond = Math.Max(worstBeyond, Math.Max(pr.Lateral - g.BarrierOffsetLeftAt(pr.S), -pr.Lateral - g.BarrierOffsetRightAt(pr.S)));
        }
        V.AfterStep = prevAfter;
        cm.Impact -= OnImpact;
        int imps = cm.ImpulseCount - imp0, eps = cm.ContactEpisodes - ep0, rearms = cm.Rearms - re0, cons = cm.ConsecutiveImpulses - cons0;
        Check("barrier impact response", imps >= 1 && imps <= eps + rearms && cons == 0 && worstBeyond < 0 && firstAfter < firstImpactSpeed,
            $"{imps} impulse(s), {eps} contact entries, {rearms} re-arms, {cons} consecutive; first impact {F(firstImpactSpeed, "0.0")}→{F(firstAfter, "0.0")} m/s; CG margin to wall {F(-worstBeyond, "0.00")} m");
        driver.TargetLateral = 0;
        driver.MaxSpeed = 60;
        _sim.RecoverToTrack();

        // 6. Pause: no physics steps while paused.
        V.SetPaused(true);
        long ps = V.Loop.Clock.StepsTaken;
        await Frames(20);
        Check("pause stops the clock", V.Loop.Clock.StepsTaken == ps, $"steps {ps} → {V.Loop.Clock.StepsTaken}");
        V.SetPaused(false);

        // 7. Parameter change reaches the equations.
        var p = V.Parameters.Clone();
        p.Mass = 1800;
        _sim.ApplyParameters(p);
        await Frames(3);
        double expected = TractionLimit.FxMax(V.Parameters, V.Parameters.Mu * V.Parameters.GripFactor(_sim.CurrentSurface));
        Check("parameters reach the model", D.Parameters.Mass == 1800 && Math.Abs(D.Outputs.FxMax - expected) < 1e-9,
            $"M={F(D.Parameters.Mass)} kg, F_x,max={F(D.Outputs.FxMax, "0.0")} N (expected {F(expected, "0.0")} N)");
        _sim.ApplyParameters(VehicleParameters.GameDefaults());

        // 8. Money shift: 2nd gear flat out, then a forced downshift to 1st above the limit destroys the engine.
        V.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters) with { Brake = 0, Throttle = 1 };
        _sim.RestartSimulation();
        V.AutoShift = false;
        await Frames(2);
        V.RequestShift(+1);
        await Until(() => D.State.V * 3.6 > 70, 30);
        V.RequestShift(-1);
        await Frames(3);
        Check("money shift destroys engine", D.State.EngineBlown, $"downshift to 1st at {F(D.State.V * 3.6, "0")} km/h, blown={D.State.EngineBlown}");

        // 9. Restart: back on the grid at rest, laps cleared, engine repaired.
        _sim.RestartSimulation();
        await Frames(1);
        var r = D.State;
        Check("restart", !r.EngineBlown && _sim.Laps.Lap == 0 && Math.Abs(r.X - sx) < 0.5 && Math.Abs(r.Y - sy) < 0.5,
            $"blown={r.EngineBlown}, lap={_sim.Laps.Lap}, V={F(r.V, "0.00")}");
        V.Autopilot = null;
        V.AutoShift = false;

        await GamepadChecks();
        await StandingStart();
        await RacingLineLap();
    }

    /// <summary>
    /// Logitech F710 in XInput mode = Godot's standard joypad layout. Events are injected through Godot's input pipeline
    /// exactly as the driver would produce them; the controller must switch to analog (unramped) inputs.
    /// </summary>
    private async Task GamepadChecks()
    {
        void Axis(JoyAxis a, float v) => Input.ParseInputEvent(new InputEventJoypadMotion { Device = 0, Axis = a, AxisValue = v });
        void Button(JoyButton b, bool down) => Input.ParseInputEvent(new InputEventJoypadButton { Device = 0, ButtonIndex = b, Pressed = down });

        _sim.RestartSimulation();
        V.AutoShift = false;
        Axis(JoyAxis.TriggerRight, 0.6f);
        Axis(JoyAxis.LeftX, -0.5f);
        await Frames(3);
        double thr = (0.6 - 0.05) / (1 - 0.05);
        double stick = Math.Pow((0.5 - 0.1) / (1 - 0.1), V.StickSteerExponent);
        double deltaMax = stick * V.Parameters.MaxSteerAngle;
        var u = V.LastInputs;
        Check("gamepad RT throttle + stick steer", V.UsingGamepad && Math.Abs(u.Throttle - thr) < 1e-6 && u.SteerAngle > 0.97 * deltaMax && u.SteerAngle <= deltaMax * (1 + 1e-6), // Godot action strengths are 32-bit floats
            $"RT 0.6 → throttle {F(u.Throttle, "0.0000")} (expected {F(thr, "0.0000")}); stick X −0.5 → δ {F(u.SteerAngle, "0.0000")} rad (≤ {F(deltaMax, "0.0000")}, speed assist only)");

        Axis(JoyAxis.TriggerRight, 0f);
        Axis(JoyAxis.TriggerLeft, 1f);
        await Frames(2);
        Check("gamepad LT brake", V.LastInputs.Brake == 1 && V.LastInputs.Throttle == 0, $"LT 1.0 → brake {F(V.LastInputs.Brake)}");
        Axis(JoyAxis.TriggerLeft, 0f);
        Axis(JoyAxis.LeftX, 0f);

        int g0 = D.State.Gear;
        Button(JoyButton.RightShoulder, true); await Frames(2); Button(JoyButton.RightShoulder, false); await Frames(1);
        int gUp = D.State.Gear;
        Button(JoyButton.LeftShoulder, true); await Frames(2); Button(JoyButton.LeftShoulder, false); await Frames(1);
        int gDown = D.State.Gear;
        Check("gamepad RB/LB shift", gUp == g0 + 1 && gDown == g0, $"gear {g0} → RB → {gUp} → LB → {gDown}");

        // A: racing line on/off.
        bool vis0 = _sim.RacingLine.Visible;
        Button(JoyButton.A, true); await Frames(2); Button(JoyButton.A, false); await Frames(1);
        bool vis1 = _sim.RacingLine.Visible;
        Button(JoyButton.A, true); await Frames(2); Button(JoyButton.A, false); await Frames(1);
        Check("gamepad A racing line toggle", vis1 == !vis0 && _sim.RacingLine.Visible == vis0, $"visible {vis0} → A → {vis1} → A → {_sim.RacingLine.Visible}");

        // X: reverse at rest, drive backwards with RT.
        _sim.RestartSimulation();
        await Frames(2);
        Button(JoyButton.X, true); await Frames(2); Button(JoyButton.X, false); await Frames(1);
        bool inR = D.State.InReverse;
        Axis(JoyAxis.TriggerRight, 1f);
        await Until(() => D.State.V < -2, 10);
        Axis(JoyAxis.TriggerRight, 0f);
        double vRev = D.State.V;
        Check("gamepad X reverse (GS6-55BG R = 3.143)", inR && vRev < -2 && RaceHud.GearLabel(D.State) == "R" && !D.State.TransmissionDamaged,
            $"gear {RaceHud.GearLabel(D.State)}, v_x {F(vRev * 3.6, "0.0")} km/h after RT");

        // X above 10 km/h: transmission destroyed, no torque; Back (Select) restarts and repairs.
        _sim.RestartSimulation();
        V.AutoShift = true;
        V.Autopilot = _ => (1, 0, 0);
        await Until(() => D.State.V * 3.6 > 20, 10);
        double vAt = D.State.V * 3.6;
        Button(JoyButton.X, true); await Frames(2); Button(JoyButton.X, false); await Frames(2);
        bool damaged = D.State.TransmissionDamaged;
        double torque = D.Outputs.EngineTorque, ftr = D.Outputs.FTractive;
        double v1 = D.State.V; await Frames(30); double v2 = D.State.V;
        V.Autopilot = null;
        V.AutoShift = false;
        Button(JoyButton.Back, true); await Frames(2); Button(JoyButton.Back, false); await Frames(1);
        Check("X above 10 km/h damages transmission", damaged && torque == 0 && ftr == 0 && v2 < v1 && !D.State.TransmissionDamaged && D.State.V == 0,
            $"X at {F(vAt, "0.0")} km/h → damaged={damaged}, engine torque {F(torque)} N·m, F_tractive {F(ftr)} N, full throttle {F(v1 * 3.6, "0.0")} → {F(v2 * 3.6, "0.0")} km/h; Back → repaired, at rest");
        V.Autopilot = null;
    }

    /// <summary>E5 physical launch clutch from rest at full throttle (driver inputs, auto-shift).</summary>
    private async Task StandingStart()
    {
        _sim.RestartSimulation();
        V.AutoShift = true;
        V.Autopilot = _ => (1, 0, 0);
        bool slippedAtStart = false;
        double minRpm = double.MaxValue, lockKmh = -1;
        var prevAfter = V.AfterStep;
        V.AfterStep = (d, u) =>
        {
            prevAfter?.Invoke(d, u);
            if (d.State.Time < 0.05 && !d.State.ClutchLocked) slippedAtStart = true;
            minRpm = Math.Min(minRpm, d.State.EngineRpm);
            if (lockKmh < 0 && d.State.ClutchLocked && d.State.V > 0.5) lockKmh = d.State.V * 3.6;
        };
        double t0 = D.State.Time;
        bool reached = await Until(() => D.State.V * 3.6 >= 50, 20);
        double t50 = D.State.Time - t0;
        V.AfterStep = prevAfter;
        V.Autopilot = null;
        V.AutoShift = false;
        Check("standing start (E5 clutch)", reached && slippedAtStart && lockKmh > 5 && minRpm >= V.Parameters.IdleRpm,
            $"0–50 km/h in {F(t50, "0.00")} s; clutch slips from rest, locks at {F(lockKmh, "0.0")} km/h; min engine speed {F(minRpm, "0")} rpm");
        _sim.RestartSimulation();
    }

    /// <summary>
    /// Converted Assetto Corsa track: grid spawn, ground following, mesh surfaces → μ, grade (E6), a scripted lap on the
    /// AC racing line with the Area3D gate cross-check, and wall contacts from the physics mesh without tunnelling.
    /// </summary>
    private async Task MeshTrackSuite()
    {
        var mesh = _sim.Track.Mesh!;
        var s0 = D.State;
        var (sx, sy, spsi) = _sim.Track.SpawnPose;
        Check("spawn on the AC grid (pole)", s0.V == 0 && Math.Abs(s0.X - sx) < 1e-9 && Math.Abs(s0.Y - sy) < 1e-9 && Math.Abs(s0.Psi - spsi) < 1e-12,
            $"pose=({F(s0.X, "0.00")}, {F(s0.Y, "0.00")}, {F(s0.Psi, "0.000")}) from models/start objects");
        await Frames(2);
        var g0 = mesh.Sample(s0.X, s0.Y, s0.Psi, V.Parameters);
        // AC grid dummies are placed a little above the road surface, hence the 1 m tolerance on the height.
        Check("ground under the grid", g0.Hit && Math.Abs(g0.Height - mesh.SpawnHeight) < 1.0 && g0.SurfaceKey == "TARMAC" && Math.Abs(V.GlobalPosition.Y - g0.Height) < 1e-3,
            $"hit={g0.Hit}, h={F(g0.Height, "0.00")} m (AC grid {F(mesh.SpawnHeight, "0.00")} m), car body y={F(V.GlobalPosition.Y, "0.00")}, surface {g0.SurfaceKey}, μ factor {F(g0.Grip)}");

        // Rendering distances (pop-in): visibility ranges by size with a fade, full-detail foliage, far clip, filtering.
        var meshes = mesh.FindChildren("*", nameof(MeshInstance3D), true, false).OfType<MeshInstance3D>().ToList();
        var ranged = meshes.Where(m => m.VisibilityRangeEnd > 0).ToList();
        int foliage = meshes.Count(m => m.LodBias >= MeshTrack.FoliageLodBias);
        var cam = GetViewport().GetCamera3D();
        var vp = GetViewport();
        bool rangesOk = ranged.All(m => m.VisibilityRangeEnd >= 400 && m.VisibilityRangeEndMargin >= 0.15f * m.VisibilityRangeEnd - 1e-3f
                                        && m.VisibilityRangeFadeMode == GeometryInstance3D.VisibilityRangeFadeModeEnum.Self);
        Check("render distances (no pop-in)", rangesOk && foliage > 0 && cam != null && cam.Far >= 4000
                && vp.AnisotropicFilteringLevel == Viewport.AnisotropicFiltering.Anisotropy16X && vp.MeshLodThreshold <= 0.5f,
            $"{meshes.Count} visual meshes, {ranged.Count} size-ranged ({(ranged.Count > 0 ? ranged.Min(m => m.VisibilityRangeEnd) : 0):0}–{(ranged.Count > 0 ? ranged.Max(m => m.VisibilityRangeEnd) : 0):0} m, 15 % fade); " +
            $"{foliage} alpha-tested meshes with LOD bias {MeshTrack.FoliageLodBias}; far {cam?.Far:0} m; AF {vp.AnisotropicFilteringLevel}, LOD threshold {vp.MeshLodThreshold} px, mip bias {vp.TextureMipmapBias}, scaling {vp.Scaling3DMode} {vp.Scaling3DScale:0.00}");

        // Kerbs: probe the edges of the track limits every 10 m (the racing line itself rarely puts the CG on one).
        int kerbHits = 0, probes = 0;
        var line = _sim.Track.Line;
        for (double sp = 0; sp < line.Length; sp += 10)
        {
            var pose = line.Pose(sp);
            var (left, right) = line.Limits(sp);
            foreach (double lat in new[] { left - 0.6, left - 0.2, -right + 0.2, -right + 0.6 })
            {
                var (px, py) = line.PointAt(sp, lat);
                probes++;
                if (mesh.Sample(px, py, pose.Psi, V.Parameters) is { Hit: true, SurfaceKey: "KERB" } k && k.Surface == VehicleDynamics.Core.Simulation.Surface.Curb) kerbHits++;
            }
        }
        Check("kerb layer at the track limits", kerbHits > 20,
            $"{kerbHits}/{probes} edge probes on KERB (μ factor {F(mesh.Friction.GetValueOrDefault("KERB", double.NaN))})");

        // Lap on the racing line: record surfaces, grade and heights seen, and the start/finish Area3D events.
        _sim.RestartSimulation();
        var driver = new CentrelineDriver(_sim.Track.Line) { LateralMargin = 0.6 };
        V.AutoShift = true;
        V.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters);
        int gateEvents = 0;
        void OnGate() => gateEvents++;
        mesh.StartFinishEntered += OnGate;
        var keys = new Dictionary<string, int>();
        var offAt = new List<string>();
        double minH = double.MaxValue, maxH = double.MinValue, maxGrade = 0, gripMismatch = 0;
        var prevAfter = V.AfterStep;
        V.AfterStep = (d, u) =>
        {
            prevAfter?.Invoke(d, u);
            if (_sim.CurrentGround is not { Hit: true } g) return;
            keys[g.SurfaceKey] = keys.GetValueOrDefault(g.SurfaceKey) + 1;
            if (g.SurfaceKey == "OFFTRACK" && offAt.Count < 400) offAt.Add(FormattableString.Invariant($"{_sim.Laps.Projection.S:0}/{_sim.Laps.Projection.Lateral:0.0}"));
            minH = Math.Min(minH, g.Height); maxH = Math.Max(maxH, g.Height);
            maxGrade = Math.Max(maxGrade, Math.Abs(u.RoadGrade));
            gripMismatch = Math.Max(gripMismatch, Math.Abs(d.Outputs.FxMax - TractionLimit.FxMax(d.Parameters, d.Parameters.Mu * g.Grip, d.Parameters.Theta + g.Grade)));
        };
        _sim.Telemetry.Start(V, "Monaco", "selftest_Monaco");
        int imp0 = _sim.Collisions.ImpulseCount;
        bool lap = await Until(() => _sim.Laps.Laps.Count > 0, 400);
        var entry = _sim.Telemetry.Stop(V);
        V.AfterStep = prevAfter;
        mesh.StartFinishEntered -= OnGate;
        var rec = lap ? _sim.Laps.Laps[0] : default;
        Check("scripted lap on the AC line", lap,
            lap ? $"lap {F(rec.Time, "0.00")} s over {F(_sim.Track.Line.Length, "0")} m, valid={rec.Valid}, wall impacts={_sim.Collisions.ImpulseCount - imp0}" : "no lap within 400 s");
        _referenceLap = lap ? rec.Time : double.NaN;
        Check("start/finish Area3D trigger", gateEvents >= 1 && entry != null && entry.Run.Samples.Any(r => r.Event?.Contains("sf_gate") == true),
            $"{gateEvents} gate event(s) logged with the lap timer's crossing");
        if (offAt.Count > 0) GD.Print("[selftest] OFFTRACK at s/lat: " + string.Join(" ", offAt.Where((_, i) => i % 20 == 0)));
        Check("mesh surfaces → μ (E2)", keys.ContainsKey("TARMAC") && gripMismatch < 1e-6,
            $"steps per surface: {string.Join(", ", keys.OrderBy(k => k.Key).Select(k => $"{k.Key} {k.Value}"))}; max |F_x,max − μ·factor·F_z| = {gripMismatch:E1} N");
        Check("elevation + grade (E6)", maxH - minH > 5 && maxGrade > 0.02 && maxGrade < 0.3,
            $"ground height {F(minH, "0.0")}…{F(maxH, "0.0")} m, max |grade| {F(maxGrade * 100, "0.0")} %");

        // Respawn regression: put the car at the highest point of the lap (Casino, ~40 m above the grid), then restart.
        // The ground search used to follow the last height, so the grid was out of reach and the car floated.
        double sHigh = 0; float hHigh = float.MinValue;
        for (double sp = 0; sp < line.Length; sp += 20)
        {
            var q = line.Pose(sp);
            if (mesh.ProbeGroundHeight(q.X, q.Y) is { } hp && hp > hHigh) { hHigh = hp; sHigh = sp; }
        }
        var top = line.Pose(sHigh);
        V.ResetVehicleAt(top.X, top.Y, top.Psi);
        await Frames(3);
        float yTop = V.GlobalPosition.Y;
        _sim.RestartSimulation();
        await Frames(1);
        float yGrid = V.GlobalPosition.Y;
        var gGrid = mesh.Sample(sx, sy, spsi, V.Parameters, updateHint: false);
        V.SetPaused(false);
        await Frames(3);
        bool physicsGround = _sim.CurrentGround is { Hit: true } cg && Math.Abs(cg.Height - gGrid.Height) < 0.3;
        Check("respawn snaps to the ground (RayCast3D)", Math.Abs(yTop - hHigh) < 0.3 && Math.Abs(yGrid - gGrid.Height) < 0.05 && physicsGround,
            $"high point s={F(sHigh, "0")} m: body {F(yTop, "0.00")} / ground {F(hHigh, "0.00")} m; after restart: body {F(yGrid, "0.00")} / ground {F(gGrid.Height, "0.00")} m; physics ground found={physicsGround}");

        // Racing line: reference lap with all three zones, rebuilt for this track.
        var rl = _sim.RacingLine;
        var (acc, coast, brk) = rl.ZoneShares();
        int switches = 0;
        for (int i = 0; i < rl.Points.Length; i++) if (rl.Points[i].Zone != rl.Points[(i + 1) % rl.Points.Length].Zone) switches++;
        if (OS.GetEnvironment("SELFTEST_DUMP_PROFILE") is { Length: > 0 } dump)
            File.WriteAllLines(dump, rl.Points.Select(x => FormattableString.Invariant($"{x.S:F1},{x.V:F2},{x.Throttle:F2},{x.Brake:F2},{x.Zone},{x.Curvature:F4}")));
        Check("racing line (reference lap zones)", rl.Points.Length > 100 && acc > 0.05 && coast > 0.02 && brk > 0.03 && rl.LapTime > 60 && rl.LapTime < 200
                && rl.Mesh != null && rl.Mesh.GetSurfaceCount() == 1,
            $"{rl.Points.Length} points, {switches} zone changes; accelerate {F(acc * 100, "0")} %, coast {F(coast * 100, "0")} %, brake {F(brk * 100, "0")} %; reference lap {F(rl.LapTime, "0.0")} s (quasi-steady estimate)");

        // Wall: steer into the right-hand side for 10 s; contacts from the physics mesh, never through a wall.
        _sim.RestartSimulation();
        V.AutoShift = true;
        V.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters);
        await Until(() => D.State.V > 15, 20);
        double maxYawKick = 0;
        void OnWallImpact(ImpactEvent _) => maxYawKick = Math.Max(maxYawKick, Math.Abs(_sim.Collisions.LastYawRateChange));
        driver.TargetLateral = -30;
        driver.MaxSpeed = 20;
        var cm = _sim.Collisions;
        cm.Impact += OnWallImpact;
        int i0 = cm.ImpulseCount, c0 = cm.ConsecutiveImpulses;
        bool tunnelled = false;
        var space = mesh.GetWorld3D().DirectSpaceState;
        double tEnd = D.State.Time + 10;
        while (D.State.Time < tEnd)
        {
            await ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
            var pr = _sim.Track.Line.Project(D.State.X, D.State.Y);
            float h = (_sim.CurrentGround?.Height ?? 0) + 0.5f;
            var from = Simulator.Dynamics.VehicleModelAdapter.ToGodotPosition(pr.CenterX, pr.CenterY, h);
            var to = Simulator.Dynamics.VehicleModelAdapter.ToGodotPosition(D.State.X, D.State.Y, h);
            if (space.IntersectRay(PhysicsRayQueryParameters3D.Create(from, to, TrackManager.BarrierCollisionLayer)).Count > 0) tunnelled = true;
        }
        cm.Impact -= OnWallImpact;
        int imps = cm.ImpulseCount - i0;
        Check("mesh wall contact (E3 rigid body)", imps >= 1 && cm.ConsecutiveImpulses == c0 && !tunnelled && maxYawKick > 0.05,
            $"{imps} impulse(s), consecutive {cm.ConsecutiveImpulses - c0}, CG crossed a wall: {tunnelled}; largest yaw-rate change from an impact {F(maxYawKick, "0.00")} rad/s");
        V.Autopilot = null;
        V.AutoShift = false;
        _sim.RestartSimulation();
        await RacingLineLap();
    }

    private double _referenceLap = double.NaN;
    private double _miniCircuitLineLap = double.NaN;

    /// <summary>
    /// Any converted track (written for the Nürburgring: many surface keys, some without a surfaces.ini friction, 25 km):
    /// grid and ground, surface classes on and beyond the track, μ actually applied per surface, racing line, and a
    /// scripted lap on the AC line with the Mini.
    /// </summary>
    private async Task ConvertedTrackSuite()
    {
        var mesh = _sim.Track.Mesh!;
        var line = _sim.Track.Line;
        var s0 = D.State;
        await Frames(2);
        var g0 = mesh.Sample(s0.X, s0.Y, s0.Psi, V.Parameters);
        // AC grid dummies stand up to ~1.5 m above the road (Nürburgring 1.3 m, Monaco 0.6 m).
        Check("grid: ground and road surface", g0.Hit && Math.Abs(g0.Height - mesh.SpawnHeight) < 2.0 && mesh.SurfaceClasses.GetValueOrDefault(g0.SurfaceKey) == "road"
                && Math.Abs(V.GlobalPosition.Y - g0.Height) < 1e-3,
            $"h={F(g0.Height, "0.00")} m (AC grid {F(mesh.SpawnHeight, "0.00")} m), surface {g0.SurfaceKey}, μ factor {F(g0.Grip)} (0 = E2 class factor); {mesh.SurfaceClasses.Count} surface keys, road μ {F(mesh.RoadFriction)}");

        // Surface classes: on the line (road or kerb) and 6 m beyond each AC edge (grass, sand, run-off...), every 25 m.
        int onRoad = 0, onTotal = 0;
        var outside = new Dictionary<string, int>();
        for (double sp = 0; sp < line.Length; sp += 25)
        {
            var pose = line.Pose(sp);
            mesh.SnapToGround(pose.X, pose.Y);
            var c = mesh.Sample(pose.X, pose.Y, pose.Psi, V.Parameters, updateHint: false);
            onTotal++;
            if (c.Hit && mesh.SurfaceClasses.GetValueOrDefault(c.SurfaceKey) is "road" or "kerb") onRoad++;
            var (left, right) = line.Limits(sp);
            foreach (double lat in new[] { left + 6, -right - 6 })
            {
                var (px, py) = line.PointAt(sp, lat);
                var o = mesh.Sample(px, py, pose.Psi, V.Parameters, updateHint: false);
                string cls = o.SurfaceKey == "OFFTRACK" ? "beyond mesh" : mesh.SurfaceClasses.GetValueOrDefault(o.SurfaceKey, "?");
                outside[cls] = outside.GetValueOrDefault(cls) + 1;
            }
        }
        _sim.RestartSimulation();
        await Frames(2);
        Check("surface classes on / beside the track", onRoad > 0.97 * onTotal && outside.GetValueOrDefault("grass") > 0,
            $"line: {onRoad}/{onTotal} road or kerb; 6 m beyond the edges: {string.Join(", ", outside.OrderByDescending(k => k.Value).Select(k => $"{k.Key} {k.Value}"))}");

        var rl = _sim.RacingLine;
        var (acc, coast, brk) = rl.ZoneShares();
        Check("racing line (reference lap)", rl.Points.Length > 1000 && acc > 0.05 && brk > 0.02 && rl.LapTime > 300 && rl.LapTime < 900,
            $"{_sim.RacingLineInfo}: {rl.Points.Length} points; accelerate {F(acc * 100, "0")} %, coast {F(coast * 100, "0")} %, brake {F(brk * 100, "0")} %; reference lap {SimulationManager.FormatTime(rl.LapTime)} (quasi-steady estimate)");

        // Scripted lap on the AC line: surfaces seen, μ applied (surfaces.ini value, or the E2 class factor), elevation.
        _sim.RestartSimulation();
        var driver = new CentrelineDriver(_sim.Track.Line) { LateralMargin = 0.6 };
        V.AutoShift = true;
        V.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters);
        var keys = new Dictionary<string, int>();
        var beyond = new SortedDictionary<int, double>();   // 100 m bucket → time beyond the AC limits [s]
        double minH = double.MaxValue, maxH = double.MinValue, maxGrade = 0, gripMismatch = 0;
        var prevAfter = V.AfterStep;
        V.AfterStep = (d, u) =>
        {
            prevAfter?.Invoke(d, u);
            var pr = _sim.Laps.Projection;
            var (lim, rim) = line.Limits(pr.S);
            if (pr.Lateral > lim || pr.Lateral < -rim) beyond[(int)(pr.S / 100)] = beyond.GetValueOrDefault((int)(pr.S / 100)) + d.Parameters.Dt;
            if (_sim.CurrentGround is not { Hit: true } g) return;
            keys[g.SurfaceKey] = keys.GetValueOrDefault(g.SurfaceKey) + 1;
            minH = Math.Min(minH, g.Height); maxH = Math.Max(maxH, g.Height);
            maxGrade = Math.Max(maxGrade, Math.Abs(u.RoadGrade));
            double factor = g.Grip > 0 ? g.Grip : d.Parameters.GripFactor(g.Surface);
            gripMismatch = Math.Max(gripMismatch, Math.Abs(d.Outputs.FxMax - TractionLimit.FxMax(d.Parameters, d.Parameters.Mu * factor, d.Parameters.Theta + g.Grade)));
        };
        int imp0 = _sim.Collisions.ImpulseCount;
        ulong wall0 = Time.GetTicksMsec();
        bool lap = await Until(() => _sim.Laps.Laps.Count > 0, 1500);
        V.AfterStep = prevAfter;
        V.Autopilot = null;
        V.AutoShift = false;
        var rec = lap ? _sim.Laps.Laps[0] : default;
        if (beyond.Count > 0)
            GD.Print("[selftest] beyond the AC limits (s bucket m: seconds): " + string.Join(", ", beyond.OrderByDescending(kv => kv.Value).Take(12).Select(kv => $"{kv.Key * 100}: {kv.Value:0.00}")));
        Check("scripted lap on the AC line", lap && rec.Valid,
            lap ? $"lap {SimulationManager.FormatTime(rec.Time)} over {F(line.Length, "0")} m, valid={rec.Valid}, wall impacts={_sim.Collisions.ImpulseCount - imp0} ({F((Time.GetTicksMsec() - wall0) / 1000.0, "0")} s wall time)"
                : $"no lap within 1500 s; reached s={F(_sim.Laps.Projection.S, "0")} m");
        Check("mesh surfaces → μ (E2)", keys.Count >= 2 && gripMismatch < 1e-6,
            $"steps per surface: {string.Join(", ", keys.OrderByDescending(k => k.Value).Select(k => $"{k.Key} {k.Value}"))}; max |F_x,max − μ·factor·F_z| = {gripMismatch:E1} N");
        Check("elevation + grade (E6)", maxH - minH > 100 && maxGrade > 0.05 && maxGrade < 0.3,
            $"ground height {F(minH, "0.0")}…{F(maxH, "0.0")} m, max |grade| {F(maxGrade * 100, "0.0")} %");
        _sim.RestartSimulation();
    }

    /// <summary>
    /// Vehicle profiles: swap to the provisional F1 (3D model, footprint, RWD, downforce), drive a lap on its racing line,
    /// swap back to the Mini.
    /// </summary>
    private async Task VehicleProfileSuite()
    {
        _sim.SwitchVehicle(VehicleProfiles.F1);
        await Frames(3);
        var vis = V.VisualsNode;
        var car = V.Visual;
        var p = V.Parameters;
        var meta = Json.ParseString(Godot.FileAccess.GetFileAsString(car.MetaPath)).AsGodotDictionary();
        double metaL = meta["wheelbase_m"].AsDouble();
        var hubs = meta["hubs"].AsGodotDictionary();
        double hubFrontZ = hubs["fl"].AsGodotArray()[2].AsDouble(), hubRearZ = hubs["rl"].AsGodotArray()[2].AsDouble();
        bool axlesMatch = Math.Abs(metaL - p.Wheelbase) < 1e-3 && Math.Abs(hubRearZ - p.CgToRear) < 5e-3
                          && Math.Abs(-hubFrontZ - (p.Wheelbase - p.CgToRear)) < 5e-3;
        Check("swap to F1 (model, footprint, physics)",
            vis != null && vis.SceneFilePath == car.VisualScene && vis.WheelCount == 4 && vis.HasSteeringWheel
            && p.Profile == VehicleProfiles.F1 && p.DriveLayout == DriveLayout.RearWheelDrive && p.Mass == 743
            && Math.Abs(_sim.Collisions.HalfWidth - 1.0) < 0.01 && axlesMatch && D.State.V == 0 && _sim.Laps.Lap == 0,
            $"scene {vis?.SceneFilePath}, wheels {vis?.WheelCount}, steering wheel {vis?.HasSteeringWheel}; footprint {F(2 * _sim.Collisions.HalfLength, "0.00")} × {F(2 * _sim.Collisions.HalfWidth, "0.00")} m; " +
            $"visual axles vs physics: L {F(metaL, "0.0000")} / {F(p.Wheelbase, "0.0000")} m, rear hub {F(hubRearZ, "0.000")} / c {F(p.CgToRear, "0.000")} m");

        GameSession.RacingLineSource = RacingLineSource.Optimised;
        _sim.BuildRacingLine();
        _sim.RestartSimulation();
        var driver = new CentrelineDriver(_sim.RacingLineTrack) { LookaheadTime = 0.45 };
        V.AutoShift = true;
        V.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters);
        double maxDown = 0, maxFrontDrive = 0, vMax = 0;
        string? firstInvalid = null;
        var prevAfter = V.AfterStep;
        V.AfterStep = (d, u) =>
        {
            prevAfter?.Invoke(d, u);
            maxDown = Math.Max(maxDown, d.Outputs.Downforce);
            if (firstInvalid == null && !_sim.Laps.LapValid)
            {
                var (l, r) = _sim.Track.Line.Limits(_sim.Laps.Projection.S);
                firstInvalid = FormattableString.Invariant($"lap invalid at s={_sim.Laps.Projection.S:0} lat={_sim.Laps.Projection.Lateral:0.00} (limits +{l:0.0}/-{r:0.0}) v={d.State.V * 3.6:0} km/h, line deviation {_sim.RacingLineTrack.Project(d.State.X, d.State.Y).Lateral:0.00} m, TC cut {d.Outputs.TractionControlCut:0.00}, brake {V.LastInputs.Brake:0.00}");
            }
            vMax = Math.Max(vMax, d.State.V);
            if (d.Outputs.Branch == DriveBranch.Drive) maxFrontDrive = Math.Max(maxFrontDrive, Math.Abs(d.Outputs.FxFront));
        };
        int imp0 = _sim.Collisions.ImpulseCount;
        double nextTrace = D.State.Time;
        var trace = new List<string>();
        bool lap = await Until(() =>
        {
            if (D.State.Time >= nextTrace)
            {
                bool fine = OS.GetEnvironment("SELFTEST_F1_TRACE") == "1" && D.State.Time > 12 && D.State.Time < 22;
                nextTrace += fine ? 0.25 : 5;
                if (fine)
                {
                    var q = D.Outputs;
                    trace.Add(FormattableString.Invariant($"t={D.State.Time:0.00} s={_sim.Laps.Projection.S:0} v={D.State.V * 3.6:0} vy={D.State.Vy:0.0} r={D.State.YawRate:0.00} {q.Branch} thr={q.ThrottleApplied:0.00} brk={V.LastInputs.Brake:0.00} δ={q.SteerAngle:0.000} κ={_sim.RacingLineTrack.Curvature(_sim.RacingLineTrack.Project(D.State.X, D.State.Y).S):0.000} F: fx={q.FxFront:0} fy={q.FyFront:0} g={q.GripFront:0} αf={q.AlphaFront:0.000} | R: fx={q.FxRear:0} fy={q.FyRear:0} g={q.GripRear:0} αr={q.AlphaRear:0.000} down={q.Downforce:0}"));
                    return _sim.Laps.Laps.Count > 0;
                }
                var o = D.Outputs;
                trace.Add(FormattableString.Invariant($"t={D.State.Time:0} s={_sim.Laps.Projection.S:0} lat={_sim.Laps.Projection.Lateral:0.0} v={D.State.V * 3.6:0} vy={D.State.Vy:0.0} r={D.State.YawRate:0.00} gear={D.State.Gear} branch={o.Branch} slip={o.SlipIndicator:0.0} grip={o.GripUsage:0.00} over={o.Oversteer} under={o.Understeer} contact={_sim.Collisions.InContact}"));
            }
            return _sim.Laps.Laps.Count > 0;
        }, 300);
        V.AfterStep = prevAfter;
        V.Autopilot = null;
        V.AutoShift = false;
        var rec = lap ? _sim.Laps.Laps[0] : default;
        int impacts = _sim.Collisions.ImpulseCount - imp0;
        if (!lap || !rec.Valid || impacts > 0) foreach (var t in trace) GD.Print("[selftest] F1 trace " + t);
        if (firstInvalid != null) GD.Print("[selftest] F1 " + firstInvalid);
        double w = p.Mass * p.G;
        Check("F1 lap on its racing line (RWD, downforce)", lap && rec.Valid && impacts == 0 && (double.IsNaN(_miniCircuitLineLap) || rec.Time < _miniCircuitLineLap) && maxDown > 0.3 * w && maxFrontDrive == 0,
            lap ? $"lap {F(rec.Time, "0.00")} s vs Mini {F(_miniCircuitLineLap, "0.00")} s, valid={rec.Valid}, impacts={impacts}; top {F(vMax * 3.6, "0")} km/h, peak downforce {F(maxDown, "0")} N ({F(maxDown / w, "0.00")} W); front drive force {F(maxFrontDrive)} N"
                : "no lap within 300 s");

        // In-session swap behind the loading screen (threaded model load, racing line on a worker thread).
        _sim.SwitchVehicle(VehicleProfiles.MiniR56);
        await Frames(2);
        var swapped = new TaskCompletionSource();
        _sim.SwitchVehicleAsync(VehicleProfiles.F1, () => swapped.TrySetResult());
        bool overlayShown = _sim.Loading && SceneManager.Instance is { IsLoading: true };
        int framesWaited = 0;
        while (!swapped.Task.IsCompleted && framesWaited++ < 3000) await Frames(1);
        Check("vehicle swap behind the loading screen", overlayShown && swapped.Task.IsCompleted && !_sim.Loading
                && SceneManager.Instance is { IsLoading: false } && V.Parameters.Profile == VehicleProfiles.F1
                && V.VisualsNode?.SceneFilePath == V.Visual.VisualScene && D.State.V == 0 && !V.Paused,
            $"overlay shown {overlayShown}, completed after {framesWaited} frames, profile {V.Parameters.Profile}, car at rest {D.State.V == 0}, running {!V.Paused}");

        _sim.SwitchVehicle(VehicleProfiles.MiniR56);
        await Frames(3);
        var mini = VehicleParameters.MiniCooperR56();
        bool same = SimulationRecorder.ParameterKeyValues(V.Parameters).SequenceEqual(SimulationRecorder.ParameterKeyValues(mini));
        Check("swap back to the Mini", same && V.VisualsNode?.SceneFilePath == V.Visual.VisualScene && V.Visual.Id == VehicleProfiles.MiniR56
                && Math.Abs(_sim.Collisions.HalfWidth - 0.841) < 0.002 && V.VisualsNode!.WheelCount == 4,
            $"parameters = Mini profile: {same}; scene {V.VisualsNode?.SceneFilePath}; footprint half width {F(_sim.Collisions.HalfWidth, "0.000")} m");
    }

    /// <summary>
    /// The same scripted driver, now following the optimised racing line instead of the track's reference line: the lap
    /// must be valid (track limits judged on the reference line), without wall impacts, and faster.
    /// </summary>
    private async Task RacingLineLap()
    {
        GameSession.RacingLineSource = RacingLineSource.Optimised;
        _sim.BuildRacingLine();
        _sim.RestartSimulation();
        var driver = new CentrelineDriver(_sim.RacingLineTrack) { LateralMargin = _sim.Track.Mesh != null ? 0.6 : 0.75 };
        V.AutoShift = true;
        V.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters);
        int imp0 = _sim.Collisions.ImpulseCount;
        var where = new List<string>();
        bool wasValid = true;
        void OnHit(ImpactEvent e) => where.Add(FormattableString.Invariant(
            $"impact s={_sim.Laps.Projection.S:0} lat={_sim.Laps.Projection.Lateral:0.0} line-dev={_sim.RacingLineTrack.Project(D.State.X, D.State.Y).Lateral:0.00} v={D.State.V * 3.6:0}"));
        _sim.Collisions.Impact += OnHit;
        var prevAfter = V.AfterStep;
        V.AfterStep = (d, u) =>
        {
            prevAfter?.Invoke(d, u);
            if (wasValid && !_sim.Laps.LapValid)
            {
                var (l, r) = _sim.Track.Line.Limits(_sim.Laps.Projection.S);
                where.Add(FormattableString.Invariant($"invalid at s={_sim.Laps.Projection.S:0} lat={_sim.Laps.Projection.Lateral:0.00} limits +{l:0.00}/-{r:0.00} line-dev={_sim.RacingLineTrack.Project(d.State.X, d.State.Y).Lateral:0.00}"));
                wasValid = false;
            }
        };
        bool lap = await Until(() => _sim.Laps.Laps.Count > 0, 400);
        V.AfterStep = prevAfter;
        _sim.Collisions.Impact -= OnHit;
        foreach (var w in where) GD.Print("[selftest] racing line: " + w);
        var rec = lap ? _sim.Laps.Laps[0] : default;
        int impacts = _sim.Collisions.ImpulseCount - imp0;
        V.Autopilot = null;
        V.AutoShift = false;
        if (lap && _sim.Track.TrackName == "Circuit" && _sim.Vehicle.Parameters.Profile == VehicleProfiles.MiniR56) _miniCircuitLineLap = rec.Time;
        Check("scripted lap on the racing line", lap && rec.Valid && impacts == 0 && rec.Time < _referenceLap,
            lap ? $"{_sim.RacingLineInfo}: lap {F(rec.Time, "0.00")} s vs {F(_referenceLap, "0.00")} s on the reference line, valid={rec.Valid}, wall impacts={impacts}; reference-lap estimate {F(_sim.RacingLine.LapTime, "0.0")} s" : "no lap within 400 s");
        _sim.RestartSimulation();
    }

    private async Task UiAndComparison()
    {
        var ui = GetTree().Root.FindChild("UI", true, false) as UiRoot;
        ui?.ShowPanels(true, true, true);
        await Frames(10);
        ui?.ShowPanels(false, false, false);
        int runs = _sim.Telemetry.Runs.Count;
        var a = _sim.Telemetry.Runs.FirstOrDefault();
        var b = _sim.Telemetry.Runs.Skip(1).FirstOrDefault();
        bool ok = ui != null && runs >= 2 && a != null && b != null;
        string detail = $"{runs} runs recorded, panels opened";
        if (ok)
        {
            var c = RunComparison.Compare(a!.Run, b!.Run, s => s.VKmh);
            detail += $"; Oval vs Circuit speed (time-aligned): n={c.Count}, max|Δ|={F(c.MaxAbs, "0.0")} km/h";
            ok = c.Count > 0;
        }
        Check("UI panels + run comparison", ok, detail);
    }
}
