using VehicleDynamics.Core.Reference;
using VehicleDynamics.Core.Simulation;
using Xunit.Abstractions;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace VehicleDynamics.Tests;

public class InteractiveModelTests
{
    private readonly ITestOutputHelper _out;
    public InteractiveModelTests(ITestOutputHelper output) => _out = output;

    private static Dyn Car(VehicleParameters? p = null, double v = 0, int gear = 1)
    {
        var d = new Dyn(p ?? new VehicleParameters());
        var s = VehicleState.AtRest();
        s.V = v;
        s.Gear = gear;
        d.State = s;
        return d;
    }

    private static void Run(Dyn d, VehicleInputs u, double seconds)
    {
        int n = (int)Math.Round(seconds / d.Parameters.Dt);
        for (int i = 0; i < n; i++) d.Step(u, d.Parameters.Dt);
    }

    [Fact]
    public void Defaults_EqualModelM()
    {
        var p = new VehicleParameters();
        var r = ReferenceParameters.Default;
        Assert.Equal(r.M, p.Mass);
        Assert.Equal(r.Rw, p.WheelRadius);
        Assert.Equal(r.Mu, p.Mu);
        Assert.Equal(r.GearRatios, p.GearRatios);
        Assert.Equal(r.TorqueRaw, p.TorqueRaw);
        Assert.Equal(0.005, p.Dt);
        Assert.Empty(p.Validate());
        Assert.Equal(ReferenceModel.Build(r).FxMax, TractionLimit.FxMax(p, p.Mu));
    }

    [Fact]
    public void ZeroInput_FlatGround_StaysAtRest()
    {
        var d = Car();
        Run(d, default, 10);
        Assert.Equal(0, d.State.V);
        Assert.Equal(0, d.State.Distance);
        Assert.Equal(0, d.State.X);
        Assert.Equal(0, d.State.Y);
    }

    /// <summary>
    /// MATLAB has no static friction / hold model and no reverse. Documented behaviour of the interactive layer:
    /// uphill with no input the car stays at V = 0 only because V is clamped at 0 (it does not roll back);
    /// downhill, the seed-game engine brake (30 Nm through the gearbox, applied even at rest) plus rolling
    /// resistance hold the car at -5 % and -10 % in 1st gear, while at -15 % gravity wins and it rolls forward.
    /// With the brake applied it holds on -15 %.
    /// </summary>
    [Theory]
    [InlineData(10, 0, false)]
    [InlineData(-5, 0, false)]
    [InlineData(-10, 0, false)]
    [InlineData(-15, 0, true)]
    [InlineData(-15, 1, false)]
    public void ZeroThrottle_OnSlope_DocumentedBehaviour(double slopePct, double brake, bool expectRollForward)
    {
        var p = new VehicleParameters { SlopeValue = slopePct };
        var d = Car(p);
        Run(d, new VehicleInputs { Brake = brake }, 5);
        _out.WriteLine($"slope {slopePct}% brake {brake}: V after 5 s = {d.State.V:F4} m/s, distance {d.State.Distance:F3} m");
        if (expectRollForward) Assert.True(d.State.V > 0.5);
        else
        {
            Assert.Equal(0, d.State.V);
            Assert.Equal(0, d.State.Distance);
        }
        Assert.True(d.State.V >= 0);
    }

    [Fact]
    public void StraightAcceleration_IsMonotonic_WithAutoShift()
    {
        var d = Car();
        var u = new VehicleInputs { Throttle = 1, AutoShift = true };
        double prevV = d.State.V, prevD = d.State.Distance;
        int gears = 1;
        for (int i = 0; i < 12000; i++)
        {
            d.Step(u, 0.005);
            Assert.True(d.State.V >= prevV, $"V decreased at t={d.State.Time}");
            Assert.True(d.State.Distance >= prevD);
            gears = Math.Max(gears, d.State.Gear);
            prevV = d.State.V;
            prevD = d.State.Distance;
        }
        _out.WriteLine($"After 60 s: V = {d.State.V * 3.6:F2} km/h, highest gear {gears}");
        Assert.True(gears >= 4);
        Assert.Equal(0, d.State.Y, 12);
    }

    [Fact]
    public void Braking_DecelerationMatchesMatlabFormula()
    {
        var p = new VehicleParameters();
        var d = Car(p, 100 / 3.6, 3);
        d.Step(new VehicleInputs { Brake = 1 }, 0.005);
        double v = 100 / 3.6;
        double vMph = (v * 3.6) / 1.60934;
        double fRes = 0.5 * 1.225 * 1.99 * 0.378 * v * v + 0.01 * (1 + vMph / 100) * 1420 * 9.81;
        double expected = -(0.90 * 0.90 * 1420 * 9.81 + fRes) / (1420 * 1.05);
        _out.WriteLine($"Braking decel at 100 km/h = {d.Outputs.Ax:F6} m/s^2 (expected {expected:F6})");
        Assert.Equal(expected, d.Outputs.Ax, 9);
        Assert.Equal(DriveBranch.Brake, d.Outputs.Branch);
        Run(d, new VehicleInputs { Brake = 1 }, 10);
        Assert.Equal(0, d.State.V);
    }

    [Fact]
    public void GearRpmMapping_MatchesKinematics()
    {
        var p = new VehicleParameters();
        var reference = ReferenceModel.Build(ReferenceParameters.Default);
        var d = Car(p);
        for (int g = 1; g <= 6; g++)
        {
            Assert.Equal(reference.VGearKmh[g - 1].Max(), d.GearMaxSpeedKmh(g));
            double v = 3000 * (2 * Math.PI / 60) / (p.GearRatios[g - 1] * p.FinalDrive) * p.WheelRadius;
            var s = VehicleState.AtRest();
            s.V = v;
            s.Gear = g;
            d.State = s;
            d.Step(default, 0.005);
            Assert.Equal(3000, d.Outputs.EngineRpm, 6);
        }
        // Auto-shift rule = MATLAB: lowest gear whose 6500-rpm speed >= V
        Assert.Equal(1, d.AutoGear(d.GearMaxSpeedKmh(1) / 3.6));
        Assert.Equal(2, d.AutoGear(d.GearMaxSpeedKmh(1) / 3.6 + 1e-6));
    }

    [Fact]
    public void Engine_IdleClampFuelCutAndClutch()
    {
        var p = new VehicleParameters();
        var e = new EngineModel(p);
        Assert.Equal(75, e.Torque(500), 12);
        Assert.Equal(75, e.Torque(1000), 12);
        Assert.Equal(0, e.Torque(6500));
        Assert.Equal(0, e.Torque(7000));
        Assert.True(e.Torque(6499.99) > 117.9);
        Assert.Equal(160, e.Torque(4250), 12);
        // Clutch launch rule (JS seed): 1st gear, wheel rpm 1200, throttle 1 -> 3500 rpm
        Assert.Equal(3500, e.EngineRpm(1200, 1, 1.0));
        Assert.Equal(1000 + 0.5 * 2500, e.EngineRpm(1200, 1, 0.5));
        Assert.Equal(4000, e.EngineRpm(4000, 1, 1.0));
        Assert.Equal(1200, e.EngineRpm(1200, 2, 1.0));
        Assert.Equal(1000, e.EngineRpm(10, 1, 0.05));
    }

    [Fact]
    public void FuelCut_NoTractionAboveLimit()
    {
        var p = new VehicleParameters { MechanicalDamage = false };
        double v = 6800 * (2 * Math.PI / 60) / (p.GearRatios[1] * p.FinalDrive) * p.WheelRadius;
        var d = Car(p, v, 2);
        d.Step(new VehicleInputs { Throttle = 1 }, 0.005);
        Assert.True(d.Outputs.RevLimiter);
        Assert.Equal(0, d.Outputs.FTractive);
        Assert.True(d.Outputs.Ax < 0);
    }

    [Fact]
    public void MoneyShift_BlowsEngine_AndToggleDisablesIt()
    {
        var p = new VehicleParameters();
        var d = Car(p, 100 / 3.6, 2);
        d.Step(new VehicleInputs { Throttle = 1 }, 0.005);
        Assert.False(d.State.EngineBlown); // 2nd gear at 100 km/h ~ 6.8k rpm
        d.Step(new VehicleInputs { Throttle = 1, ShiftDown = true }, 0.005);
        Assert.True(d.State.EngineBlown);
        Assert.True(d.Outputs.EngineBlownThisStep);
        Assert.Equal(DriveBranch.Blown, d.Outputs.Branch);
        double v = d.State.V;
        double expected = -(1420 * 9.81 * 0.9 + 0.5 * 1.225 * 1.99 * 0.378 * v * v) / 1420;
        d.Step(new VehicleInputs { Throttle = 1 }, 0.005);
        Assert.Equal(expected, d.Outputs.Ax, 2);
        Assert.Equal(0, d.Outputs.FTractive);

        var p2 = new VehicleParameters { MechanicalDamage = false };
        var d2 = Car(p2, 100 / 3.6, 2);
        d2.Step(new VehicleInputs { Throttle = 1, ShiftDown = true }, 0.005);
        Assert.False(d2.State.EngineBlown);
        Assert.Equal(1, d2.State.Gear);
    }

    [Fact]
    public void KinematicBicycle_ConstantRadiusBelowGrip()
    {
        var p = new VehicleParameters();
        double delta = 0.1;
        var d = Car(p, 8, 1);
        var pts = new List<(double X, double Y)>();
        var u = new VehicleInputs { SteerAngle = delta };
        for (int i = 0; i < 400; i++)
        {
            d.Step(u, 0.005);
            Assert.False(d.Outputs.Understeer);
            if (i % 100 == 0) pts.Add((d.State.X, d.State.Y));
        }
        double rCg = CircleRadius(pts[0], pts[1], pts[3]);
        double beta = Math.Atan(p.CgToRear / p.Wheelbase * Math.Tan(delta));
        double expectedCg = p.Wheelbase / (Math.Tan(delta) * Math.Cos(beta));
        double rRear = Math.Sqrt(rCg * rCg - p.CgToRear * p.CgToRear);
        _out.WriteLine($"R_cg fitted {rCg:F6} m (expected {expectedCg:F6}); rear-axle radius {rRear:F6} vs L/tan(delta) {p.Wheelbase / Math.Tan(delta):F6}");
        Assert.Equal(expectedCg, rCg, 3);
        Assert.Equal(p.Wheelbase / Math.Tan(delta), rRear, 3);
        Assert.True(d.Outputs.YawRate > 0);
    }

    [Fact]
    public void Understeer_SaturatesLateralForce()
    {
        var p = new VehicleParameters();
        var d = Car(p, 25, 3);
        d.Step(new VehicleInputs { SteerAngle = 0.1, Throttle = 0.3 }, 0.005);
        var o = d.Outputs;
        Assert.True(o.Understeer);
        Assert.True(o.SlipIndicator > 1);
        Assert.Equal(o.FGrip / p.Mass, o.Ay, 1);
        double pathRadius = 25 / o.YawRate;
        Assert.True(pathRadius > p.Wheelbase / Math.Tan(0.1));
        // Friction circle fully used laterally: no longitudinal force left
        Assert.Equal(0, o.FTractiveEff, 6);
        var d2 = Car(p, 25, 3);
        d2.Step(new VehicleInputs { SteerAngle = -0.1 }, 0.005);
        Assert.True(d2.Outputs.YawRate < 0);
    }

    [Fact]
    public void FrictionCircle_CouplesLongitudinalAndLateral()
    {
        var p = new VehicleParameters();
        double fGrip = TractionLimit.FxMax(p, p.Mu);
        double target = 0.6 * fGrip; // lateral demand
        // M v^2 cos(beta) tan(delta) / L = target  with v = 8 m/s
        double v = 8;
        double delta = Math.Atan(target * p.Wheelbase / (p.Mass * v * v));
        for (int it = 0; it < 50; it++)
        {
            double beta = Math.Atan(p.CgToRear / p.Wheelbase * Math.Tan(delta));
            delta = Math.Atan(target * p.Wheelbase / (p.Mass * v * v * Math.Cos(beta)));
        }
        var d = Car(p, v, 1);
        d.Step(new VehicleInputs { Throttle = 1, SteerAngle = delta }, 0.005);
        var o = d.Outputs;
        Assert.Equal(target, o.Fy, 6);
        Assert.Equal(0.8 * fGrip, o.FTractiveEff, 6); // sqrt(1 - 0.6^2) = 0.8
        Assert.True(o.FTractive > o.FTractiveEff);
        Assert.Equal(1.0, o.GripUsage, 9);
        // Straight: full traction limit is available
        var d2 = Car(p, v, 1);
        d2.Step(new VehicleInputs { Throttle = 1 }, 0.005);
        Assert.Equal(fGrip, d2.Outputs.FTractiveEff, 9);
    }

    [Fact]
    public void Surfaces_ScaleGripAndRolling()
    {
        var p = new VehicleParameters();
        var d = Car(p, 10, 2);
        d.Step(new VehicleInputs { Throttle = 1, Surface = Surface.Grass }, 0.005);
        Assert.Equal(TractionLimit.FxMax(p, 0.9 * 0.6), d.Outputs.FxMax, 9);
        Assert.Equal(4 * Resistance.Rolling(p, 10), d.Outputs.FRolling, 9);
    }

    [Fact]
    public void ParameterChangeBetweenSteps_RejectsInvalid()
    {
        var d = Car();
        Assert.Throws<ArgumentException>(() => d.SetParameters(new VehicleParameters { Mass = -5 }));
        d.SetParameters(new VehicleParameters { Mass = 1800 });
        Assert.Equal(1800, d.Parameters.Mass);
    }

    [Fact]
    public void InteractiveR1_AgreesWithMatlabHistory()
    {
        var mat = TestData.ReadCsv(Path.Combine(TestData.Dir, "matlab_S01", "r1_history.csv"));
        var p = new VehicleParameters { ClutchLaunch = false };
        var reference = ReferenceModel.Build(ReferenceParameters.Default);
        var d = Car(p, reference.VGearKmh[0].Min() / 3.6, 1);
        var u = new VehicleInputs { Throttle = 1, AutoShift = true };
        double maxDv = 0, maxDd = 0;
        int n = mat["t_s"].Length;
        for (int i = 0; i < n; i++)
        {
            maxDv = Math.Max(maxDv, Math.Abs(d.State.V * 3.6 - mat["v_kmh"][i]));
            maxDd = Math.Max(maxDd, Math.Abs(d.State.Distance - mat["d_m"][i]));
            d.Step(u, 0.005);
        }
        _out.WriteLine($"interactive vs MATLAB R1 ({n} samples): max |dv| = {maxDv:E3} km/h, max |dd| = {maxDd:E3} m");
        Assert.True(maxDv < 0.01, $"{maxDv}");
        Assert.True(maxDd < 0.05, $"{maxDd}");
    }

    private static double CircleRadius((double X, double Y) a, (double X, double Y) b, (double X, double Y) c)
    {
        double ab = Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
        double bc = Math.Sqrt((b.X - c.X) * (b.X - c.X) + (b.Y - c.Y) * (b.Y - c.Y));
        double ca = Math.Sqrt((c.X - a.X) * (c.X - a.X) + (c.Y - a.Y) * (c.Y - a.Y));
        double area2 = Math.Abs((b.X - a.X) * (c.Y - a.Y) - (c.X - a.X) * (b.Y - a.Y));
        return ab * bc * ca / (2 * area2);
    }
}
