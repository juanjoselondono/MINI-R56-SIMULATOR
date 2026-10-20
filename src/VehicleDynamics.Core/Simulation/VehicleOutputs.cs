namespace VehicleDynamics.Core.Simulation;

public enum DriveBranch { Drive, Brake, Coast, Blown }

/// <summary>Derived quantities of the last step (telemetry). Forces in N, accelerations in m/s^2.</summary>
public struct VehicleOutputs
{
    public DriveBranch Branch;
    public double WheelRpm;
    public double EngineRpm;
    public double EngineTorque;
    public bool RevLimiter;
    public double FTractive;
    public double FTractiveEff;
    public double FxMax;
    public double FGrip;
    public double FDrag;
    public double FRolling;
    public double FGrade;
    public double FBrake;
    public double FEngineBrake;
    public double MassEquivalent;
    public double Ax;
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
    public bool EngineBlownThisStep;
}
