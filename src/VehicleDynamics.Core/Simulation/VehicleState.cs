namespace VehicleDynamics.Core.Simulation;

/// <summary>Integrated state (SI, model frame: X east, Y north, Psi CCW from +X).</summary>
public struct VehicleState
{
    public double X;
    public double Y;
    public double Psi;
    /// <summary>
    /// Longitudinal speed v_x [m/s] in the body frame (E4), or the speed along the path (E1). Always >= 0: model.m has
    /// no reverse. This is the speed the longitudinal model.m equations act on and the speedometer shows.
    /// </summary>
    public double V;
    /// <summary>E4 lateral velocity of the CG in the body frame [m/s], + = left. 0 in the E1 kinematic model.</summary>
    public double Vy;
    /// <summary>E4 yaw rate r [rad/s], + = CCW. 0 in the E1 kinematic model (its yaw rate is algebraic).</summary>
    public double YawRate;
    /// <summary>1-based gear.</summary>
    public int Gear;
    /// <summary>Engine speed of the last step [rpm] (telemetry; equals the wheel-driven speed while the clutch is locked).</summary>
    public double EngineRpm;
    /// <summary>E5 engine speed state [rad/s] while the launch clutch slips (0 = not yet set: idle).</summary>
    public double EngineOmega;
    /// <summary>E5 launch clutch locked (engine speed tied to wheel speed). Always true without the physical clutch.</summary>
    public bool ClutchLocked;
    /// <summary>Longitudinal acceleration of the previous step [m/s²], used for the E4 axle load transfer.</summary>
    public double AxPrev;
    public double Distance;
    public double Time;
    public bool EngineBlown;

    public static VehicleState AtRest(double x = 0, double y = 0, double psi = 0) => new()
    {
        X = x, Y = y, Psi = psi, V = 0, Gear = 1, EngineRpm = 1000, Distance = 0, Time = 0, EngineBlown = false,
    };
}
