namespace VehicleDynamics.Core.Simulation;

/// <summary>EXTENSION A1: aerodynamic downforce (not in model.m; zero unless <see cref="VehicleParameters.LiftCoefficient"/> &gt; 0).</summary>
public static class Aerodynamics
{
    /// <summary>F_down = ½ ρ v² C_l A [N], v = longitudinal speed (sign irrelevant).</summary>
    public static double Downforce(VehicleParameters p, double v) => 0.5 * p.RhoAir * p.LiftCoefficient * p.FrontalArea * (v * v);

    /// <summary>Downforce per unit mass and squared speed, k = ½ ρ C_l A / M [1/m]: a_z,aero = k v².</summary>
    public static double SpecificCoefficient(VehicleParameters p) => 0.5 * p.RhoAir * p.LiftCoefficient * p.FrontalArea / p.Mass;
}
