namespace VehicleDynamics.Core.Simulation;

public enum DriveBranch { Drive, Brake, Coast, Blown }

/// <summary>Derived quantities of the last step (telemetry). Forces in N, accelerations in m/s^2.</summary>
public struct VehicleOutputs
{
    public DriveBranch Branch;
    public LateralModel LateralModel;
    public double WheelRpm;
    public double EngineRpm;
    public double EngineTorque;
    public bool RevLimiter;
    /// <summary>Driveline force at the wheels from engine (clutch) torque, before any limit.</summary>
    public double FTractive;
    /// <summary>Drive force that reaches the chassis (after the traction limit / friction circle).</summary>
    public double FTractiveEff;
    public double FxMax;
    /// <summary>Total lateral grip capacity [N] (E1: friction-circle radius; E4: front + rear axle capacity).</summary>
    public double FGrip;
    public double FDrag;
    public double FRolling;
    public double FGrade;
    public double FBrake;
    public double FEngineBrake;
    public double MassEquivalent;
    /// <summary>Longitudinal acceleration of the chassis dv_x/dt [m/s²].</summary>
    public double Ax;
    /// <summary>Lateral (centripetal) acceleration [m/s²]: E4 (F_yf cos δ + F_yr)/M, E1 V·r.</summary>
    public double Ay;
    public double YawRate;
    public double Beta;
    public double SteerAngle;
    public double Vx;
    public double Vy;
    public double FyDemand;
    public double Fy;
    public double GripUsage;
    public double SlipIndicator;
    public bool Understeer;
    /// <summary>E4: the rear axle saturates before the front.</summary>
    public bool Oversteer;
    public bool EngineBlownThisStep;

    // E4 dynamic single-track
    public double AlphaFront;
    public double AlphaRear;
    public double FyFront;
    public double FyRear;
    public double FzFront;
    public double FzRear;
    /// <summary>dv_y/dt and dr/dt of the dynamic model (before the low-speed blend).</summary>
    public double VyDot;
    public double YawAccel;

    // E5 launch clutch
    public bool ClutchLocked;
    public double ClutchSlipRpm;
    public double ClutchTorque;
    public double EngineOmegaDot;
}
