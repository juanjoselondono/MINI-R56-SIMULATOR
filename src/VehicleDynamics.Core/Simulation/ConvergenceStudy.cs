namespace VehicleDynamics.Core.Simulation;

public readonly record struct ConvergenceRow(
    string Case, Integrator Integrator, double Dt, double VEnd, double DEnd, double ErrV, double ErrD);

/// <summary>
/// Timestep convergence of the interactive model: speed and distance at a fixed end time for several dt,
/// semi-implicit Euler (model.m scheme) vs RK4, against an RK4 solution with dt = <see cref="ReferenceDt"/>.
/// Cases: "r1_launch" (full throttle, auto-shift, 1st gear at 1000 rpm, clutch rule off = MATLAB R1 conditions;
/// gear changes make a(V) discontinuous), "wot_3rd" (full throttle held in 3rd from 60 km/h: no shift, but
/// PCHIP torque is only C1 at its nodes) and "coast_4th" (smooth: coasting from 100 km/h in 4th).
/// </summary>
public static class ConvergenceStudy
{
    public static readonly double[] DefaultDts = { 0.001, 0.002, 0.005, 0.01, 0.02 };
    public static readonly string[] Cases = { "r1_launch", "wot_3rd", "coast_4th" };
    public const double ReferenceDt = 1e-5;

    public static List<ConvergenceRow> Run(double tEnd = 10, double[]? dts = null)
    {
        dts ??= DefaultDts;
        var rows = new List<ConvergenceRow>();
        foreach (var c in Cases)
        {
            var (vRef, dRef) = Simulate(c, Integrator.RungeKutta4, ReferenceDt, tEnd);
            foreach (var integ in new[] { Integrator.SemiImplicitEuler, Integrator.RungeKutta4 })
                foreach (var dt in dts)
                {
                    var (v, d) = Simulate(c, integ, dt, tEnd);
                    rows.Add(new ConvergenceRow(c, integ, dt, v, d, Math.Abs(v - vRef), Math.Abs(d - dRef)));
                }
        }
        return rows;
    }

    public static (double V, double D) Simulate(string caseName, Integrator integ, double dt, double tEnd)
    {
        var p = new VehicleParameters { ClutchLaunch = false };
        var dyn = new VehicleDynamics(p) { Integrator = integ };
        var s = VehicleState.AtRest();
        VehicleInputs u;
        if (caseName == "r1_launch")
        {
            s.V = (p.RpmRaw[0] * (2 * Math.PI / 60) / dyn.TotalRatio(1)) * p.WheelRadius;
            s.Gear = 1;
            u = new VehicleInputs { Throttle = 1, AutoShift = true };
        }
        else if (caseName == "wot_3rd")
        {
            s.V = 60 / 3.6;
            s.Gear = 3;
            u = new VehicleInputs { Throttle = 1 };
        }
        else
        {
            s.V = 100 / 3.6;
            s.Gear = 4;
            u = new VehicleInputs();
        }
        dyn.State = s;
        int n = (int)Math.Round(tEnd / dt);
        for (int i = 0; i < n; i++) dyn.Step(u, dt);
        return (dyn.State.V, dyn.State.Distance);
    }
}
