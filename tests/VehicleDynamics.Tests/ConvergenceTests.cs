using VehicleDynamics.Core.Simulation;
using Xunit.Abstractions;

namespace VehicleDynamics.Tests;

public class ConvergenceTests
{
    private readonly ITestOutputHelper _out;
    public ConvergenceTests(ITestOutputHelper output) => _out = output;

    [Fact]
    public void TimestepConvergence_EulerVsRk4()
    {
        var rows = ConvergenceStudy.Run(10);
        _out.WriteLine("case,integrator,dt,v_end_mps,d_end_m,err_v_mps,err_d_m");
        foreach (var r in rows)
            _out.WriteLine($"{r.Case},{r.Integrator},{r.Dt},{r.VEnd:R},{r.DEnd:R},{r.ErrV:E3},{r.ErrD:E3}");
        ConvergenceRow Row(string c, Integrator i, double dt) => rows.Single(r => r.Case == c && r.Integrator == i && r.Dt == dt);

        // Euler (model.m scheme) is first order in distance in every case, including the launch with gear shifts:
        // halving dt halves the error (observed ratio between dt = 0.01 and 0.005).
        foreach (var c in ConvergenceStudy.Cases)
        {
            double order = Math.Log(Row(c, Integrator.SemiImplicitEuler, 0.01).ErrD / Row(c, Integrator.SemiImplicitEuler, 0.005).ErrD) / Math.Log(2);
            _out.WriteLine($"{c} Euler observed order (d) = {order:F3}");
            Assert.InRange(order, 0.9, 1.1);
        }

        // At the game/model.m step (0.005 s) Euler stays within 0.1 % of the reference distance after 10 s.
        var eu = Row("r1_launch", Integrator.SemiImplicitEuler, 0.005);
        Assert.True(eu.ErrD / eu.DEnd < 1e-3, $"{eu.ErrD / eu.DEnd}");

        // Smooth cases (no shift): RK4 is at round-off / 1e-8 level for every dt in the study.
        foreach (var c in new[] { "wot_3rd", "coast_4th" })
            foreach (var dt in ConvergenceStudy.DefaultDts)
                Assert.True(Row(c, Integrator.RungeKutta4, dt).ErrD < 1e-8, $"{c} RK4 dt={dt}");

        // Launch case: shifts make a(V) jump, RK4 loses its order but still beats Euler at every dt.
        foreach (var dt in ConvergenceStudy.DefaultDts)
            Assert.True(Row("r1_launch", Integrator.RungeKutta4, dt).ErrD < Row("r1_launch", Integrator.SemiImplicitEuler, dt).ErrD);
    }
}
