namespace VehicleDynamics.Core.Simulation;

/// <summary>
/// Fixed-step accumulator. Frame time is clamped to <see cref="MaxFrameTime"/>; the number of steps is derived
/// from the accumulated (clamped) time so that the total step count does not depend on how that time was split
/// into frames. <see cref="Alpha"/> is the interpolation factor between the previous and current state.
/// </summary>
public sealed class SimulationClock
{
    private double _accumulated;   // total clamped time fed so far [s]
    private long _stepsTaken;

    public double FixedDt { get; }
    public double MaxFrameTime { get; set; }
    public bool Paused { get; set; }
    public long StepsTaken => _stepsTaken;
    public double SimulatedTime => _stepsTaken * FixedDt;

    /// <summary>Interpolation factor in [0, 1).</summary>
    public double Alpha { get; private set; }

    public SimulationClock(double fixedDt = 0.005, double maxFrameTime = 0.25)
    {
        if (!(fixedDt > 0)) throw new ArgumentOutOfRangeException(nameof(fixedDt));
        if (!(maxFrameTime >= fixedDt)) throw new ArgumentOutOfRangeException(nameof(maxFrameTime));
        FixedDt = fixedDt;
        MaxFrameTime = maxFrameTime;
    }

    /// <summary>Feeds one render frame; returns how many fixed steps to run now.</summary>
    public int Advance(double frameTime)
    {
        if (Paused || !(frameTime > 0) || double.IsNaN(frameTime)) return 0;
        _accumulated += Math.Min(frameTime, MaxFrameTime);
        // Tolerance absorbs summation round-off (e.g. 0.005 fed as 0.002 + 0.003).
        long target = (long)Math.Floor(_accumulated / FixedDt + 1e-7);
        int n = (int)Math.Max(0, target - _stepsTaken);
        _stepsTaken += n;
        Alpha = Math.Clamp(_accumulated / FixedDt - _stepsTaken, 0, 1 - 1e-12);
        return n;
    }

    public void Reset()
    {
        _accumulated = 0;
        _stepsTaken = 0;
        Alpha = 0;
    }
}
