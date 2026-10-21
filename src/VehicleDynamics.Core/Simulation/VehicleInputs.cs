namespace VehicleDynamics.Core.Simulation;

/// <summary>Driver inputs, held constant over a physics step.</summary>
public struct VehicleInputs
{
    /// <summary>[0, 1]</summary>
    public double Throttle;
    /// <summary>[0, 1]</summary>
    public double Brake;
    /// <summary>Front wheel angle delta [rad], + = left. Clamped to +-MaxSteerAngle by the model.</summary>
    public double SteerAngle;
    /// <summary>Edge request; consumed by exactly one physics step (see SimulationLoop).</summary>
    public bool ShiftUp;
    public bool ShiftDown;
    /// <summary>
    /// EXTENSION R1, edge request: engage reverse (from a forward gear) or 1st (from reverse). Engaging against the
    /// direction of travel above <see cref="VehicleParameters.ReverseEngageMaxSpeed"/> damages the transmission.
    /// </summary>
    public bool ToggleReverse;
    /// <summary>MATLAB §11 gear rule: lowest gear whose 6500-rpm speed is >= V.</summary>
    public bool AutoShift;
    /// <summary>Surface under the car (E2). Asphalt when no track is used.</summary>
    public Surface Surface;
    /// <summary>
    /// Surface friction multiplier on μ from the track data (e.g. Assetto Corsa surfaces.ini FRICTION); 0 = use the E2
    /// factor of <see cref="Surface"/>.
    /// </summary>
    public double SurfaceGrip;
    /// <summary>E6: local road grade along the heading [rad], + = uphill, added to model.m's global slope. 0 on flat tracks.</summary>
    public double RoadGrade;

    /// <summary>Builds inputs from a normalized steer command s in [-1, 1]: delta = s * MaxSteerAngle (E1).</summary>
    public static VehicleInputs FromNormalized(double throttle, double brake, double steer, VehicleParameters p, bool autoShift = false)
        => new()
        {
            Throttle = throttle, Brake = brake, SteerAngle = Math.Clamp(steer, -1, 1) * p.MaxSteerAngle, AutoShift = autoShift,
        };
}
