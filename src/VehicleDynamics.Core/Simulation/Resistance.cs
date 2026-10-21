namespace VehicleDynamics.Core.Simulation;

/// <summary>model.m §4 resistances (same formulas; rolling factor is the E2 surface extension, 1 on asphalt).</summary>
public static class Resistance
{
    public static double Drag(VehicleParameters p, double v) => 0.5 * p.RhoAir * p.FrontalArea * p.Cd * (v * v);

    public static double Rolling(VehicleParameters p, double v, double rollingFactor = 1.0) => Rolling(p, v, rollingFactor, p.Theta);

    public static double Rolling(VehicleParameters p, double v, double rollingFactor, double theta)
    {
        double vMph = (v * 3.6) / 1.60934;
        return (0.01 * (1 + (vMph / 100))) * rollingFactor * (p.Mass * p.G * Math.Cos(theta));
    }

    public static double Grade(VehicleParameters p) => Grade(p, p.Theta);

    public static double Grade(VehicleParameters p, double theta) => p.Mass * p.G * Math.Sin(theta);
}
