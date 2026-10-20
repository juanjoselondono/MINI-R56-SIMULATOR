using VehicleDynamics.Core.Reference;

namespace VehicleDynamics.Core.Simulation;

/// <summary>
/// Engine torque evaluated directly with PCHIP on the model.m nodes (not through the 50-rpm table).
/// Idle clamp, fuel cut, clutch launch and money shift thresholds are seed-game rules (monaco.html).
/// </summary>
public sealed class EngineModel
{
    private readonly Pchip _curve;
    private readonly double _rpmMin;
    private readonly double _rpmMax;
    private readonly VehicleParameters _p;

    public EngineModel(VehicleParameters p)
    {
        _p = p;
        _curve = new Pchip(p.RpmRaw, p.TorqueRaw);
        _rpmMin = p.RpmRaw[0];
        _rpmMax = p.RpmRaw[^1];
    }

    /// <summary>Full-load torque [Nm]: rpm below the table/idle uses the idle value; rpm >= fuel cut gives 0.</summary>
    public double Torque(double rpm)
    {
        if (rpm >= _p.FuelCutRpm) return 0;
        double r = Math.Max(rpm, Math.Max(_p.IdleRpm, _rpmMin));
        r = Math.Min(r, _rpmMax);
        return _curve.Evaluate(r);
    }

    /// <summary>rpm = V / R_w * i_g * i_fd * 60 / (2 pi).</summary>
    public static double WheelRpm(double v, double rw, double totalRatio) => (v / rw) * totalRatio * (60 / (2 * Math.PI));

    /// <summary>
    /// Engine speed from wheel speed. Clutch launch rule taken from the JS seed (not in MATLAB):
    /// in 1st gear below the clutch target with throttle above the threshold,
    /// rpm = max(wheel_rpm, idle + throttle * (target - idle)); otherwise max(idle, wheel_rpm).
    /// </summary>
    public double EngineRpm(double wheelRpm, int gear, double throttle)
    {
        if (_p.ClutchLaunch && gear == 1 && wheelRpm < _p.ClutchTargetRpm && throttle > _p.ClutchThrottleThreshold)
            return Math.Max(_p.IdleRpm + throttle * (_p.ClutchTargetRpm - _p.IdleRpm), wheelRpm);
        return Math.Max(_p.IdleRpm, wheelRpm);
    }
}
