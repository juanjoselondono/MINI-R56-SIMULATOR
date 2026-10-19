using System;
using System.Linq;
using Godot;
using Simulator.Dynamics;
using VehicleDynamics.Core.Simulation;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace Simulator.Vehicle;

/// <summary>
/// Owns the physics core for one car. Each render frame: sample inputs once (as levels), shape them in
/// simulated time, advance the fixed-step <see cref="SimulationLoop"/>, then place this node at the
/// interpolated physical pose. Nothing here alters the physics state except the explicit reset.
/// </summary>
public partial class VehicleController : Node3D
{
    [Signal] public delegate void EngineBlownEventHandler();
    [Signal] public delegate void VehicleResetEventHandler();

    /// <summary>MATLAB §11 gear rule when on; manual E/Q shifting when off.</summary>
    [Export] public bool AutoShift { get; set; }

    /// <summary>
    /// Gameplay input assist (NOT physics): the steering command is divided by 1 + (V/ref)^2 so keyboard
    /// steering stays usable at speed. The physics still receives and clamps the resulting delta.
    /// </summary>
    [Export] public bool SpeedSensitiveSteering { get; set; } = true;
    [Export] public float SpeedSensitiveSteeringRefSpeed { get; set; } = 25f;

    public VehicleParameters Parameters { get; private set; } = new();
    public Dyn Dynamics { get; private set; } = null!;
    public SimulationLoop Loop { get; private set; } = null!;
    public InputShaper Shaper { get; } = new();
    public VehicleInputs LastInputs { get; private set; }
    public bool HeadlightsOn { get; private set; }

    /// <summary>Optional per-step hook (track queries, collisions, lap timing), re-attached on every reset.</summary>
    public Action<Dyn, VehicleInputs>? AfterStep { get; set; }

    private VehicleVisuals? _visuals;
    private (double X, double Y, double Psi) _spawn;
    private bool _wasBlown;
    private double _autodriveSeconds = -1;

    public override void _Ready()
    {
        InputSetup.EnsureActions();
        _spawn = VehicleModelAdapter.FromGodotTransform(GlobalTransform);
        _visuals = GetNodeOrNull<VehicleVisuals>("Visuals");
        if (_visuals != null) _visuals.WheelRadius = Parameters.WheelRadius;

        // `-- --autodrive=<seconds>`: full throttle with auto-shift, prints the state and quits (smoke test).
        var arg = OS.GetCmdlineUserArgs().FirstOrDefault(a => a.StartsWith("--autodrive"));
        if (arg != null)
        {
            _autodriveSeconds = arg.Contains('=') ? double.Parse(arg.Split('=')[1], System.Globalization.CultureInfo.InvariantCulture) : 10;
            AutoShift = true;
        }
        ResetVehicle();
    }

    /// <summary>Rebuilds the model at the spawn pose, at rest, with the current parameters.</summary>
    public void ResetVehicle()
    {
        Dynamics = new Dyn(Parameters, VehicleState.AtRest(_spawn.X, _spawn.Y, _spawn.Psi));
        Loop = new SimulationLoop(Dynamics) { AfterStep = (d, u) => AfterStep?.Invoke(d, u) };
        Shaper.Reset();
        _wasBlown = false;
        ApplyPose();
        EmitSignal(SignalName.VehicleReset);
    }

    /// <summary>Validates and applies a new parameter set between physics steps (state is kept).</summary>
    public void ApplyParameters(VehicleParameters p)
    {
        p.EnsureValid();
        Parameters = p.Clone();
        Dynamics.SetParameters(Parameters);
        if (_visuals != null) _visuals.WheelRadius = Parameters.WheelRadius;
    }

    public void SetPaused(bool paused) => Loop.Clock.Paused = paused;
    public bool Paused => Loop.Clock.Paused;

    public override void _Process(double delta)
    {
        if (Input.IsActionJustPressed(InputSetup.ResetVehicle)) ResetVehicle();
        if (Input.IsActionJustPressed(InputSetup.Pause)) SetPaused(!Paused);
        if (Input.IsActionJustPressed(InputSetup.ToggleAutoShift)) AutoShift = !AutoShift;
        if (Input.IsActionJustPressed(InputSetup.ToggleHeadlights)) HeadlightsOn = !HeadlightsOn;

        double throttle = Input.GetActionStrength(InputSetup.Throttle);
        double brake = Input.GetActionStrength(InputSetup.Brake);
        double steer = Input.GetActionStrength(InputSetup.SteerLeft) - Input.GetActionStrength(InputSetup.SteerRight);
        if (_autodriveSeconds > 0) { throttle = 1; brake = 0; steer = 0; }

        if (!Paused)
        {
            // Rate limits are expressed per second; frame time is clamped like the simulation clock.
            Shaper.Update(throttle, brake, steer, Math.Min(delta, Loop.Clock.MaxFrameTime));
        }

        double steerCmd = Shaper.Steer;
        if (SpeedSensitiveSteering)
        {
            double ratio = Dynamics.State.V / SpeedSensitiveSteeringRefSpeed;
            steerCmd /= 1 + ratio * ratio;
        }

        var u = VehicleInputs.FromNormalized(Shaper.Throttle, Shaper.Brake, steerCmd, Parameters, AutoShift);
        u.ShiftUp = Input.IsActionJustPressed(InputSetup.ShiftUp);
        u.ShiftDown = Input.IsActionJustPressed(InputSetup.ShiftDown);
        LastInputs = u;

        Loop.Advance(delta, u);
        ApplyPose();

        if (Dynamics.State.EngineBlown && !_wasBlown) EmitSignal(SignalName.EngineBlown);
        _wasBlown = Dynamics.State.EngineBlown;

        if (_autodriveSeconds > 0 && Dynamics.State.Time >= _autodriveSeconds)
        {
            var s = Dynamics.State;
            GD.Print(FormattableString.Invariant(
                $"AUTODRIVE t={s.Time:F3}s v={s.V * 3.6:F2}km/h dist={s.Distance:F1}m gear={s.Gear} rpm={s.EngineRpm:F0} x={s.X:F2} y={s.Y:F2} psi={s.Psi:F4} steps={Loop.Clock.StepsTaken}"));
            GetTree().Quit();
        }
    }

    private void ApplyPose()
    {
        var (x, y, psi) = Loop.InterpolatedPose();
        GlobalTransform = VehicleModelAdapter.ToGodotTransform(x, y, psi);

        var prev = Loop.PreviousState;
        var cur = Dynamics.State;
        double distance = prev.Distance + (cur.Distance - prev.Distance) * Loop.Alpha;
        _visuals?.UpdateVisuals(distance, Dynamics.Outputs.SteerAngle, LastInputs.Brake, cur.EngineBlown, HeadlightsOn);
    }
}
