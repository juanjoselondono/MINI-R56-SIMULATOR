using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using Godot;
using Simulator.Simulation;
using Simulator.UI;
using Simulator.Vehicle;
using VehicleDynamics.Core.Simulation;

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
            foreach (var track in new[] { "Oval", "Circuit" })
            {
                _track = track;
                _sim.LoadTrack(track);
                await Frames(2);
                await TrackSuite();
            }
            _track = "both";
            await UiAndComparison();
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
        var driver = new CentrelineDriver(_sim.Track.Geometry);
        V.AutoShift = true;
        V.Autopilot = vc => driver.Drive(vc.Dynamics.State, vc.Dynamics.Outputs, vc.Parameters);
        _sim.Telemetry.Start(V, _sim.Track.TrackName, $"selftest_{_sim.Track.TrackName}");
        int impulses0 = _sim.Collisions.ImpulseCount;
        bool lap = await Until(() => _sim.Laps.Laps.Count > 0, 300);
        var entry = _sim.Telemetry.Stop(V);
        var rec = lap ? _sim.Laps.Laps[0] : default;
        Check("scripted lap counted", lap && rec.Valid && _sim.Collisions.ImpulseCount == impulses0,
            lap ? $"lap {F(rec.Time, "0.00")} s, valid={rec.Valid}, impacts={_sim.Collisions.ImpulseCount - impulses0}" : "no lap within 300 s");
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
            var pr = _sim.Track.Geometry.Project(D.State.X, D.State.Y);
            worstBeyond = Math.Max(worstBeyond, Math.Max(pr.Lateral - _sim.Track.Geometry.BarrierOffsetLeftAt(pr.S), -pr.Lateral - _sim.Track.Geometry.BarrierOffsetRightAt(pr.S)));
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
