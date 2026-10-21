using VehicleDynamics.Core.Collision;
using VehicleDynamics.Core.Simulation;
using Xunit.Abstractions;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace VehicleDynamics.Tests;

/// <summary>E4 dynamic single-track, E5 launch clutch and correction C1.</summary>
public class DynamicModelTests
{
    private readonly ITestOutputHelper _out;
    public DynamicModelTests(ITestOutputHelper output) => _out = output;

    private static Dyn Car(VehicleParameters p, double v, int gear)
    {
        var s = VehicleState.AtRest();
        s.V = v;
        s.Gear = gear;
        return new Dyn(p, s);
    }

    /// <summary>Holds v_x near <paramref name="target"/> with a proportional throttle/brake.</summary>
    private static VehicleInputs Cruise(Dyn d, double target, double delta)
    {
        double e = target - d.State.V;
        return new VehicleInputs
        {
            SteerAngle = delta, AutoShift = true,
            Throttle = e > 0 ? Math.Clamp(0.3 + 0.5 * e, 0, 1) : 0.3 + 0.5 * e > 0 ? 0.3 + 0.5 * e : 0,
        };
    }

    [Fact]
    public void Dynamic_StraightLine_IsBitIdenticalToKinematic()
    {
        var pk = new VehicleParameters { LateralModel = LateralModel.Kinematic, ClutchLaunch = false, GripLimitOnTyreForce = false };
        var pd = pk.Clone();
        pd.LateralModel = LateralModel.Dynamic;
        var a = Car(pk, 2.3, 1);
        var b = Car(pd, 2.3, 1);
        var u = new VehicleInputs { Throttle = 1, AutoShift = true };
        for (int i = 0; i < 4000; i++)
        {
            a.Step(u, 0.005);
            b.Step(u, 0.005);
            Assert.Equal(a.State.V, b.State.V);
            Assert.Equal(a.State.Distance, b.State.Distance);
        }
        Assert.Equal(0, b.State.Vy);
        Assert.Equal(0, b.State.YawRate);
    }

    [Fact]
    public void Dynamic_SteadyStateYawRate_MatchesLinearBicycle()
    {
        var p = new VehicleParameters { ClutchLaunch = false };
        double v = 20, delta = 0.01;
        var d = Car(p, v, 3);
        for (int i = 0; i < 2400; i++) d.Step(Cruise(d, v, delta), 0.005);
        double vx = d.State.V;
        double lr = p.CgToRear, lf = p.Wheelbase - lr;
        double k = p.Mass * (lr / p.CorneringStiffnessFront - lf / p.CorneringStiffnessRear) / p.Wheelbase;
        double expected = vx * delta / (p.Wheelbase + k * vx * vx);
        _out.WriteLine($"v_x {vx:F3} m/s: r = {d.State.YawRate:F6} rad/s, linear bicycle {expected:F6} (K = {k:E3} s²/m, understeer)");
        Assert.True(k > 0);
        Assert.Equal(expected, d.State.YawRate, expected * 0.01);
        Assert.Equal(d.State.V * d.State.YawRate, d.Outputs.Ay, 0.02);
    }

    [Fact]
    public void Dynamic_SignsAndSymmetry()
    {
        var p = new VehicleParameters { ClutchLaunch = false };
        var left = Car(p, 15, 2);
        var right = Car(p, 15, 2);
        for (int i = 0; i < 400; i++)
        {
            left.Step(new VehicleInputs { SteerAngle = 0.05 }, 0.005);
            right.Step(new VehicleInputs { SteerAngle = -0.05 }, 0.005);
        }
        Assert.True(left.State.YawRate > 0 && left.Outputs.Ay > 0 && left.State.Psi > 0);
        Assert.Equal(-left.State.YawRate, right.State.YawRate, 12);
        Assert.Equal(-left.State.Vy, right.State.Vy, 12);
        Assert.Equal(left.State.V, right.State.V, 12);
    }

    [Fact]
    public void Dynamic_StepSteer_IsStableAndSettles()
    {
        var p = new VehicleParameters { ClutchLaunch = false };
        var d = Car(p, 25, 4);
        double peak = 0, rAt3 = 0;
        for (int i = 0; i < 1600; i++)
        {
            d.Step(Cruise(d, 25, 0.02), 0.005);
            peak = Math.Max(peak, d.State.YawRate);
            if (i == 599) rAt3 = d.State.YawRate;
        }
        _out.WriteLine($"step steer 0.02 rad at 25 m/s: peak r {peak:F4}, r(3 s) {rAt3:F4}, r(8 s) {d.State.YawRate:F4} rad/s");
        Assert.True(double.IsFinite(d.State.YawRate));
        Assert.True(peak < 1.5 * d.State.YawRate);
        Assert.Equal(rAt3, d.State.YawRate, 0.01);
    }

    [Fact]
    public void Dynamic_Saturation_BoundsLateralAccelerationByFriction()
    {
        var p = new VehicleParameters { ClutchLaunch = false };
        var d = Car(p, 25, 4);
        double maxAy = 0;
        bool understeer = false, slip = false;
        for (int i = 0; i < 600; i++)
        {
            d.Step(new VehicleInputs { SteerAngle = 0.3 }, 0.005);
            maxAy = Math.Max(maxAy, Math.Abs(d.Outputs.Ay));
            understeer |= d.Outputs.Understeer;
            slip |= d.Outputs.SlipIndicator > 1;
        }
        _out.WriteLine($"full lock at 25 m/s: max |a_y| {maxAy:F3} m/s² vs μg {p.Mu * p.G:F3}");
        Assert.True(maxAy <= p.Mu * p.G * 1.001);
        Assert.True(maxAy > 0.7 * p.Mu * p.G);
        Assert.True(understeer && slip);
    }

    [Fact]
    public void Dynamic_LowSpeed_FollowsKinematicModel()
    {
        var p = new VehicleParameters { ClutchLaunch = false };
        var d = Car(p, 1.0, 1);
        d.Step(new VehicleInputs { SteerAngle = 0.2, Throttle = 0.02 }, 0.005);
        Assert.Equal(d.State.V * Math.Tan(0.2) / p.Wheelbase, d.State.YawRate, 12);
    }

    [Fact]
    public void Dynamic_RandomInputs_StayFinite()
    {
        var p = VehicleParameters.GameDefaults();
        var d = new Dyn(p);
        var rng = new Random(11);
        var u = new VehicleInputs { AutoShift = true };
        for (int i = 0; i < 12000; i++)
        {
            if (i % 200 == 0)
                u = new VehicleInputs
                {
                    Throttle = rng.NextDouble(), Brake = rng.NextDouble() < 0.2 ? rng.NextDouble() : 0,
                    SteerAngle = (rng.NextDouble() * 2 - 1) * p.MaxSteerAngle, AutoShift = true,
                    Surface = rng.NextDouble() < 0.2 ? Surface.Grass : Surface.Asphalt,
                };
            d.Step(u, 0.005);
            var s = d.State;
            Assert.True(double.IsFinite(s.V) && double.IsFinite(s.Vy) && double.IsFinite(s.YawRate) && double.IsFinite(s.X));
            Assert.True(s.V >= 0);
            Assert.True(s.EngineRpm >= p.IdleRpm - 1e-9 || s.EngineBlown);
        }
    }

    [Fact]
    public void Clutch_FullThrottleLaunch_SlipsGripLimitedThenLocksWithoutRpmJump()
    {
        var p = VehicleParameters.GameDefaults();
        var d = new Dyn(p);
        var u = new VehicleInputs { Throttle = 1 };
        double lockTime = -1, lockKmh = 0, rpmBefore = 0, rpmAfter = 0, minRpm = double.MaxValue, axSlip = double.NaN, axExpected = 0;
        for (int i = 0; i < 1000 && lockTime < 0; i++)
        {
            double rpm0 = d.Outputs.EngineRpm;
            d.Step(u, 0.005);
            minRpm = Math.Min(minRpm, d.State.EngineRpm);
            var o = d.Outputs;
            if (!o.ClutchLocked && o.FTractiveEff == o.FxMax && double.IsNaN(axSlip))
            {
                // First grip-limited slipping step: the tyre force F_x,max accelerates the bare mass (C1).
                axSlip = o.Ax;
                axExpected = (o.FxMax - (o.FDrag + o.FRolling + o.FGrade)) / p.Mass;
            }
            if (d.State.ClutchLocked)
            {
                lockTime = d.State.Time; lockKmh = d.State.V * 3.6;
                rpmBefore = rpm0;
                d.Step(u, 0.005);
                rpmAfter = d.Outputs.EngineRpm;
            }
        }
        _out.WriteLine($"lock at {lockTime:F3} s, {lockKmh:F1} km/h; rpm {rpmBefore:F0} -> {rpmAfter:F0}; min rpm {minRpm:F0}; grip-limited a_x while slipping {axSlip:F3} (expected {axExpected:F3})");
        Assert.True(lockTime > 0.3 && lockTime < 3);
        Assert.True(Math.Abs(rpmAfter - rpmBefore) < 80);
        Assert.True(minRpm >= p.IdleRpm);
        Assert.False(double.IsNaN(axSlip), "the slipping launch never became grip-limited");
        Assert.Equal(axExpected, axSlip, 12);
    }

    [Fact]
    public void Clutch_PartialThrottle_IsGentleAndDoesNotStall()
    {
        var p = VehicleParameters.GameDefaults();
        var d = new Dyn(p);
        double maxAx = 0;
        for (int i = 0; i < 2000; i++)
        {
            d.Step(new VehicleInputs { Throttle = 0.3 }, 0.005);
            maxAx = Math.Max(maxAx, d.Outputs.Ax);
            Assert.True(d.State.EngineRpm >= p.IdleRpm);
        }
        _out.WriteLine($"throttle 0.3: max a_x {maxAx:F3} m/s², locked {d.State.ClutchLocked}, {d.State.V * 3.6:F1} km/h after 10 s");
        Assert.True(maxAx < 2.0);
        Assert.True(d.State.ClutchLocked);
        Assert.True(d.State.V > 3);
    }

    [Fact]
    public void Clutch_AtRest_NoCreepOnFlat_RollsDownhillThenEngineBrakes()
    {
        var p = VehicleParameters.GameDefaults();
        var flat = new Dyn(p);
        for (int i = 0; i < 1000; i++) flat.Step(new VehicleInputs(), 0.005);
        Assert.Equal(0, flat.State.V);

        var pd = VehicleParameters.GameDefaults();
        pd.SlopeValue = -10;
        var hill = new Dyn(pd);
        for (int i = 0; i < 4000; i++) hill.Step(new VehicleInputs(), 0.005);
        _out.WriteLine($"-10 % slope, no input: {hill.State.V * 3.6:F2} km/h after 20 s, clutch locked {hill.State.ClutchLocked}, rpm {hill.State.EngineRpm:F0}");
        Assert.True(hill.State.ClutchLocked);
        Assert.True(hill.State.V > 0);
    }

    [Fact]
    public void C1_GripLimitedTyreForceAcceleratesBareMass()
    {
        // Low friction so 1st gear is grip-limited even after the rotating-inertia share.
        var on = new VehicleParameters { Mu = 0.3, ClutchLaunch = false, LateralModel = LateralModel.Kinematic };
        var off = on.Clone();
        off.GripLimitOnTyreForce = false;
        var a = Car(on, 8, 1);
        var b = Car(off, 8, 1);
        a.Step(new VehicleInputs { Throttle = 1 }, 0.005);
        b.Step(new VehicleInputs { Throttle = 1 }, 0.005);
        double fRes = a.Outputs.FDrag + a.Outputs.FRolling + a.Outputs.FGrade;
        double cap = a.Outputs.FxMax;
        Assert.Equal((cap - fRes) / on.Mass, a.Outputs.Ax, 12);
        Assert.Equal((cap - fRes) / (off.Mass * (1 + off.MassFactors[0])), b.Outputs.Ax, 12);

        // Not grip-limited (dry, 3rd gear): both formulations agree.
        var c = Car(new VehicleParameters { ClutchLaunch = false }, 15, 3);
        var e = Car(new VehicleParameters { ClutchLaunch = false, GripLimitOnTyreForce = false }, 15, 3);
        c.Step(new VehicleInputs { Throttle = 1 }, 0.005);
        e.Step(new VehicleInputs { Throttle = 1 }, 0.005);
        Assert.Equal(e.Outputs.Ax, c.Outputs.Ax, 12);
    }

    [Fact]
    public void Collision_Dynamic_ExpressesVelocityInNewBodyFrame()
    {
        var d = new Dyn(new VehicleParameters(), new VehicleState { V = 20, Vy = 2, YawRate = 0.4, Gear = 3, ClutchLocked = true });
        double speed = d.Speed;
        var r = new CollisionResult(true, 1, 2, 0.3, 12, 5, -5, VelocityHeading: 0.5);
        d.ApplyCollision(r);
        Assert.Equal(12 * Math.Cos(0.2), d.State.V, 12);
        Assert.Equal(12 * Math.Sin(0.2), d.State.Vy, 12);
        Assert.Equal(0.4 * 12 / speed, d.State.YawRate, 12);
        Assert.True(d.Speed <= 12 + 1e-12);
    }

    [Fact]
    public void GameDefaults_OnlyChangeTheMass()
    {
        var g = VehicleParameters.GameDefaults();
        Assert.Equal(1150, g.Mass);
        g.Mass = 1420;
        Assert.Equal(ConfigSerializer.ToJson(new VehicleParameters()), ConfigSerializer.ToJson(g));
        var back = ConfigSerializer.FromJson(ConfigSerializer.ToJson(VehicleParameters.GameDefaults()));
        Assert.Equal(LateralModel.Dynamic, back.LateralModel);
        Assert.Equal(LaunchModel.PhysicalClutch, back.LaunchModel);
    }
}
