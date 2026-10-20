namespace VehicleDynamics.Core.Simulation;

/// <summary>Integrated state (SI, model frame: X east, Y north, Psi CCW from +X).</summary>
public struct VehicleState
{
    public double X;
    public double Y;
    public double Psi;
    /// <summary>Speed along the path [m/s], always >= 0 (MATLAB has no reverse).</summary>
    public double V;
    /// <summary>1-based gear.</summary>
    public int Gear;
    /// <summary>Algebraic engine speed of the last step (telemetry).</summary>
    public double EngineRpm;
    public double Distance;
    public double Time;
    public bool EngineBlown;

    public static VehicleState AtRest(double x = 0, double y = 0, double psi = 0) => new()
    {
        X = x, Y = y, Psi = psi, V = 0, Gear = 1, EngineRpm = 1000, Distance = 0, Time = 0, EngineBlown = false,
    };
}
