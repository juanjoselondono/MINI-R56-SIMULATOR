using System;
using System.Linq;
using Godot;
using Simulator.Dynamics;
using Simulator.UI;
using VehicleDynamics.Core.Simulation;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace Simulator.Vehicle;

/// <summary>
/// Owns the physics core for one car. Each render frame: sample inputs once (as levels), shape them in
/// simulated time, advance the fixed-step <see cref="SimulationLoop"/>, then place this node at the
/// interpolated physical pose. Nothing here alters the physics state except the explicit resets.
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

    public VehicleParameters Parameters { get; private set; } = VehicleParameters.GameDefaults();

    /// <summary>
    /// Gamepad steering response: steer = sign(x)·|x|^exponent on the left-stick value (after the Input Map deadzone),
    /// for finer control near centre. Input mapping only; the model receives the resulting δ.
    /// </summary>
    [Export] public float StickSteerExponent { get; set; } = 1.5f;

    /// <summary>True while the last driving input came from a gamepad (analog: no keyboard ramping).</summary>
    public bool UsingGamepad { get; private set; }
    public Dyn Dynamics { get; private set; } = null!;
    public SimulationLoop Loop { get; private set; } = null!;
    public InputShaper Shaper { get; } = new();
    public VehicleInputs LastInputs { get; private set; }
    public bool HeadlightsOn { get; private set; }
    /// <summary>Wall time of the last frame's physics advance (all substeps + hooks) [ms], and its step count.</summary>
    public double LastAdvanceMs { get; private set; }
    public int LastStepsThisFrame { get; private set; }

    /// <summary>
    /// Scripted driver for tests (self-test, autodrive): returns (throttle, brake, steer[-1..1]) levels instead of the
    /// keyboard/gamepad. It goes through exactly the same input path as a human driver.
    /// </summary>
    public Func<VehicleController, (double Throttle, double Brake, double Steer)>? Autopilot { get; set; }

    // Hooks re-attached to every new SimulationLoop (reset, dt change).
    public Func<Dyn, VehicleInputs, VehicleInputs>? BeforeStep { get; set; }
    public Action<Dyn, VehicleInputs>? AfterStep { get; set; }
    private Func<Dyn, TrackSample>? _trackContext;
    public Func<Dyn, TrackSample>? TrackContext
    {
        get => _trackContext;
        set { _trackContext = value; if (Loop != null) Loop.TrackContext = value; }
    }
    private SimulationRecorder? _recorder;
    public SimulationRecorder? Recorder
    {
        get => _recorder;
        set { _recorder = value; if (Loop != null) Loop.Recorder = value; }
    }

    private int _requestedShift;
    private (double Throttle, double Brake, double Steer) _rawTargets;
    private VehicleState? _autodriveReport;

    /// <summary>Scripted shift request (+1 up, -1 down), delivered as an edge on the next frame like a key press.</summary>
    public void RequestShift(int direction) => _requestedShift = Math.Sign(direction);

    private VehicleVisuals? _visuals;
    private (double X, double Y, double Psi) _spawn;
    private bool _spawnSet;
    private bool _wasBlown;
    private double _autodriveSeconds = -1;

    public override void _Ready()
    {
        InputSetup.EnsureActions();
        if (!_spawnSet) _spawn = VehicleModelAdapter.FromGodotTransform(GlobalTransform);
        _visuals = GetNodeOrNull<VehicleVisuals>("Visuals");
        if (_visuals != null) _visuals.WheelRadius = Parameters.WheelRadius;

        // `-- --autodrive=<seconds>`: full throttle with auto-shift, prints the state and quits (smoke test).
        var arg = OS.GetCmdlineUserArgs().FirstOrDefault(a => a.StartsWith("--autodrive"));
        if (arg != null)
        {
            _autodriveSeconds = arg.Contains('=') ? double.Parse(arg.Split('=')[1], System.Globalization.CultureInfo.InvariantCulture) : 10;
            AutoShift = true;
            Autopilot = _ => (1, 0, 0);
        }
        if (Dynamics == null) ResetVehicle();
    }

    public void SetSpawn(double x, double y, double psi)
    {
        _spawn = (x, y, psi);
        _spawnSet = true;
    }

    /// <summary>Rebuilds the model at the spawn pose, at rest, with the current parameters.</summary>
    public void ResetVehicle() => ResetVehicleAt(_spawn.X, _spawn.Y, _spawn.Psi);

    /// <summary>
    /// Explicit reset to a pose at rest (spawn or recover-to-track). The only teleport in the game.
    /// <paramref name="keepEngineDamage"/>: recovering moves the car but does not repair a destroyed engine.
    /// </summary>
    public void ResetVehicleAt(double x, double y, double psi, bool keepEngineDamage = false)
    {
        bool paused = Loop?.Clock.Paused ?? false;
        var rest = VehicleState.AtRest(x, y, psi);
        if (keepEngineDamage && Dynamics != null) rest.EngineBlown = Dynamics.State.EngineBlown;
        Dynamics = new Dyn(Parameters, rest);
        CreateLoop(paused);
        Shaper.Reset();
        _wasBlown = Dynamics.State.EngineBlown;
        ApplyPose();
        EmitSignal(SignalName.VehicleReset);
    }

    private void CreateLoop(bool paused)
    {
        Loop = new SimulationLoop(Dynamics, new SimulationClock(Parameters.Dt))
        {
            BeforeStep = (d, u) =>
            {
                u = ShapeInputs(d, u, Loop.Clock.FixedDt);
                u = BeforeStep?.Invoke(d, u) ?? u;
                LastInputs = u;
                return u;
            },
            AfterStep = (d, u) =>
            {
                AfterStep?.Invoke(d, u);
                // Autodrive reports the state of the first step at or past the requested time (frame-rate independent).
                if (_autodriveSeconds > 0 && _autodriveReport == null && d.State.Time >= _autodriveSeconds - 1e-9) _autodriveReport = d.State;
            },
            TrackContext = _trackContext,
            Recorder = _recorder,
        };
        Loop.Clock.Paused = paused;
    }

    /// <summary>
    /// Validates and applies a new parameter set between physics steps; the state is kept. A new Dt takes effect by
    /// rebuilding the fixed-step clock (the state carries over; the recorder keeps its own sample interval).
    /// </summary>
    public void ApplyParameters(VehicleParameters p)
    {
        p.EnsureValid();
        bool dtChanged = p.Dt != Parameters.Dt;
        Parameters = p.Clone();
        Dynamics.SetParameters(Parameters);
        if (dtChanged) CreateLoop(Loop.Clock.Paused);
        if (_visuals != null) _visuals.WheelRadius = Parameters.WheelRadius;
    }

    public void SetPaused(bool paused) => Loop.Clock.Paused = paused;
    public bool Paused => Loop.Clock.Paused;

    /// <summary>The device of the most recent driving input decides between analog (gamepad) and ramped (keyboard) inputs.</summary>
    public override void _Input(InputEvent e)
    {
        if (e is InputEventJoypadMotion m && Math.Abs(m.AxisValue) > 0.2f) UsingGamepad = true;
        else if (e is InputEventJoypadButton { Pressed: true }) UsingGamepad = true;
        else if (e is InputEventKey { Pressed: true }) UsingGamepad = false;
    }

    public override void _Process(double delta)
    {
        bool typing = UiFocus.IsTyping(GetViewport());
        if (!typing)
        {
            if (Input.IsActionJustPressed(InputSetup.Pause)) SetPaused(!Paused);
            if (Input.IsActionJustPressed(InputSetup.ToggleAutoShift)) AutoShift = !AutoShift;
            if (Input.IsActionJustPressed(InputSetup.ToggleHeadlights)) HeadlightsOn = !HeadlightsOn;
        }

        double throttle = 0, brake = 0, steer = 0;
        if (Autopilot != null)
        {
            (throttle, brake, steer) = Autopilot(this);
        }
        else if (!typing)
        {
            throttle = Input.GetActionStrength(InputSetup.Throttle);
            brake = Input.GetActionStrength(InputSetup.Brake);
            steer = Input.GetActionStrength(InputSetup.SteerLeft) - Input.GetActionStrength(InputSetup.SteerRight);
            if (UsingGamepad) steer = Math.Sign(steer) * Math.Pow(Math.Abs(steer), StickSteerExponent);
        }

        // Raw targets are sampled once per frame; ramping and the steering assist run per physics step
        // (ShapeInputs), so the input history seen by the model does not depend on the frame rate.
        _rawTargets = (throttle, brake, steer);
        var u = new VehicleInputs { AutoShift = AutoShift, Surface = LastInputs.Surface };
        if (!Paused)
        {
            // Shift presses while paused are dropped, not queued for the resume.
            u.ShiftUp = (!typing && Input.IsActionJustPressed(InputSetup.ShiftUp)) || _requestedShift > 0;
            u.ShiftDown = (!typing && Input.IsActionJustPressed(InputSetup.ShiftDown)) || _requestedShift < 0;
            _requestedShift = 0;
        }

        long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
        LastStepsThisFrame = Loop.Advance(delta, u);
        LastAdvanceMs = (System.Diagnostics.Stopwatch.GetTimestamp() - t0) * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
        ApplyPose();

        if (Dynamics.State.EngineBlown && !_wasBlown) EmitSignal(SignalName.EngineBlown);
        _wasBlown = Dynamics.State.EngineBlown;

        if (_autodriveReport is { } s)
        {
            GD.Print(FormattableString.Invariant(
                $"AUTODRIVE t={s.Time:F3}s v={s.V * 3.6:F2}km/h dist={s.Distance:F1}m gear={s.Gear} rpm={s.EngineRpm:F0} x={s.X:F2} y={s.Y:F2} psi={s.Psi:F4} steps={Math.Round(s.Time / Parameters.Dt)} exact: V={s.V:R} X={s.X:R}"));
            _autodriveSeconds = -1;
            _autodriveReport = null;
            Simulation.AppQuit.Quit(this);
        }
    }

    /// <summary>
    /// Per physics step: rate-limit the frame's raw targets over dt (simulated time) and apply the speed-sensitive
    /// steering assist with the current speed, then convert to the model's input (delta = steer * MaxSteerAngle).
    /// </summary>
    private VehicleInputs ShapeInputs(Dyn d, VehicleInputs u, double dt)
    {
        if (UsingGamepad && Autopilot == null) Shaper.SetImmediate(_rawTargets.Throttle, _rawTargets.Brake, _rawTargets.Steer);
        else Shaper.Update(_rawTargets.Throttle, _rawTargets.Brake, _rawTargets.Steer, dt);
        double steerCmd = Shaper.Steer;
        if (SpeedSensitiveSteering && Autopilot == null)
        {
            double ratio = d.State.V / SpeedSensitiveSteeringRefSpeed;
            steerCmd /= 1 + ratio * ratio;
        }
        var shaped = VehicleInputs.FromNormalized(Shaper.Throttle, Shaper.Brake, steerCmd, Parameters, u.AutoShift);
        shaped.ShiftUp = u.ShiftUp;
        shaped.ShiftDown = u.ShiftDown;
        shaped.Surface = u.Surface;
        return shaped;
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
