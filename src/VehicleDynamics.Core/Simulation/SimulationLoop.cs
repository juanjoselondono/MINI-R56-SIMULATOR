namespace VehicleDynamics.Core.Simulation;

/// <summary>
/// Single-threaded fixed-step driver used by the game: inputs are sampled once per render frame as levels and
/// held for every substep; shift requests are latched edges consumed by exactly one physics step (if a frame
/// produces no step, the edge stays pending). Keeps the previous state for render interpolation.
/// </summary>
public sealed class SimulationLoop
{
    private bool _pendingUp;
    private bool _pendingDown;

    public VehicleDynamics Vehicle { get; }
    public SimulationClock Clock { get; }
    public SimulationRecorder? Recorder { get; set; }
    public VehicleState PreviousState { get; private set; }
    /// <summary>
    /// Optional per-step input adjustment from the current physical state (e.g. the surface under the car).
    /// It may only change fields that describe the environment; driver levels and shift edges are already set.
    /// </summary>
    public Func<VehicleDynamics, VehicleInputs, VehicleInputs>? BeforeStep { get; set; }
    /// <summary>Optional hook run after each physics step (track queries, collisions, lap timing).</summary>
    public Action<VehicleDynamics, VehicleInputs>? AfterStep { get; set; }
    /// <summary>Optional provider of the track context for the recorder.</summary>
    public Func<VehicleDynamics, TrackSample>? TrackContext { get; set; }

    public SimulationLoop(VehicleDynamics vehicle, SimulationClock? clock = null)
    {
        Vehicle = vehicle;
        Clock = clock ?? new SimulationClock(vehicle.Parameters.Dt);
        PreviousState = vehicle.State;
    }

    public double Alpha => Clock.Alpha;

    /// <summary>Feeds one render frame. Returns the number of physics steps executed.</summary>
    public int Advance(double frameTime, VehicleInputs levels)
    {
        _pendingUp |= levels.ShiftUp;
        _pendingDown |= levels.ShiftDown;
        int n = Clock.Advance(frameTime);
        for (int i = 0; i < n; i++)
        {
            var u = levels;
            u.ShiftUp = _pendingUp;
            u.ShiftDown = _pendingDown;
            _pendingUp = _pendingDown = false;
            if (BeforeStep != null) u = BeforeStep(Vehicle, u);
            PreviousState = Vehicle.State;
            Vehicle.Step(u, Clock.FixedDt);
            AfterStep?.Invoke(Vehicle, u);
            Recorder?.OnStep(Vehicle.State, Vehicle.Outputs, u, TrackContext?.Invoke(Vehicle));
        }
        return n;
    }

    /// <summary>Linear interpolation of position/heading between the last two physics states.</summary>
    public (double X, double Y, double Psi) InterpolatedPose()
    {
        var a = PreviousState;
        var b = Vehicle.State;
        double t = Clock.Alpha;
        double dpsi = FrameConversion.WrapAngle(b.Psi - a.Psi);
        return (a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Psi + dpsi * t);
    }
}

/// <summary>Keyboard ramping per simulated second (rate limits), applied per physics step, not per frame.</summary>
public sealed class InputShaper
{
    public double ThrottleRiseRate { get; set; } = 5;   // 1/s
    public double ThrottleFallRate { get; set; } = 8;
    public double BrakeRiseRate { get; set; } = 6;
    public double BrakeFallRate { get; set; } = 10;
    public double SteerRate { get; set; } = 2.5;        // normalized units per second
    public double SteerReturnRate { get; set; } = 4;

    public double Throttle { get; private set; }
    public double Brake { get; private set; }
    public double Steer { get; private set; }

    public void Update(double targetThrottle, double targetBrake, double targetSteer, double dt)
    {
        Throttle = Ramp(Throttle, Math.Clamp(targetThrottle, 0, 1), ThrottleRiseRate, ThrottleFallRate, dt);
        Brake = Ramp(Brake, Math.Clamp(targetBrake, 0, 1), BrakeRiseRate, BrakeFallRate, dt);
        double ts = Math.Clamp(targetSteer, -1, 1);
        bool towardCentre = Math.Abs(ts) < Math.Abs(Steer) || (Steer != 0 && Math.Sign(ts) != Math.Sign(Steer));
        double rate = towardCentre ? SteerReturnRate : SteerRate;
        Steer = Ramp(Steer, ts, rate, rate, dt);
    }

    public void Reset() => Throttle = Brake = Steer = 0;

    /// <summary>Analog devices (gamepad triggers / stick) bypass the keyboard rate limits.</summary>
    public void SetImmediate(double throttle, double brake, double steer)
    {
        Throttle = Math.Clamp(throttle, 0, 1);
        Brake = Math.Clamp(brake, 0, 1);
        Steer = Math.Clamp(steer, -1, 1);
    }

    private static double Ramp(double cur, double target, double up, double down, double dt)
    {
        if (target > cur) return Math.Min(target, cur + up * dt);
        return Math.Max(target, cur - down * dt);
    }
}
