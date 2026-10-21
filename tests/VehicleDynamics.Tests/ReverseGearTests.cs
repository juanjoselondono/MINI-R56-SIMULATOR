using VehicleDynamics.Core.Simulation;
using Xunit.Abstractions;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace VehicleDynamics.Tests;

/// <summary>EXTENSION R1: reverse gear (Getrag G255 / GS6-55BG, 3.143) and transmission damage.</summary>
public class ReverseGearTests
{
    private readonly ITestOutputHelper _out;
    public ReverseGearTests(ITestOutputHelper output) => _out = output;

    private static Dyn Car(double v = 0, VehicleParameters? p = null)
    {
        var d = new Dyn(p ?? VehicleParameters.GameDefaults());
        var s = VehicleState.AtRest();
        s.V = v;
        d.State = s;
        return d;
    }

    private static void Run(Dyn d, VehicleInputs u, double seconds)
    {
        for (int i = 0, n = (int)Math.Round(seconds / d.Parameters.Dt); i < n; i++) d.Step(u, d.Parameters.Dt);
    }

    private static void Toggle(Dyn d) => d.Step(new VehicleInputs { ToggleReverse = true }, d.Parameters.Dt);

    [Fact]
    public void ReverseRatio_IsTheGetragG255Value()
    {
        var p = new VehicleParameters();
        Assert.Equal(3.143, p.ReverseRatio);
        Assert.Equal(3.143 * p.FinalDrive, new Dyn(p).TotalRatio(VehicleState.ReverseGear), 12);
    }

    [Fact]
    public void FromRest_EngagesReverse_AndDrivesBackwards()
    {
        var d = Car();
        Toggle(d);
        Assert.Equal(VehicleState.ReverseGear, d.State.Gear);
        Assert.False(d.State.TransmissionDamaged);
        Run(d, new VehicleInputs { Throttle = 1 }, 4);
        _out.WriteLine($"4 s full throttle in R: v_x = {d.State.V * 3.6:F1} km/h, engine {d.State.EngineRpm:F0} rpm, x = {d.State.X:F2} m");
        Assert.True(d.State.V < -3);
        Assert.True(d.State.X < -5);                // heading +x, so the car moved towards −x
        Assert.True(d.State.EngineRpm > 1000);      // the engine turns forwards while the car goes backwards
        // Throttle released, brake: the car stops and stays stopped (brakes never drive it forwards).
        Run(d, new VehicleInputs { Brake = 1 }, 4);
        Assert.Equal(0, d.State.V);
        Run(d, new VehicleInputs(), 5);
        Assert.Equal(0, d.State.V);
    }

    [Fact]
    public void Reverse_TorqueMultiplication_UsesTheReverseRatio()
    {
        // Same engine speed in 1st (forwards) and in reverse (backwards): tractive force scales with the total ratio.
        var p = VehicleParameters.GameDefaults();
        var d = new Dyn(p);
        double tr1 = d.TotalRatio(1), trR = d.TotalRatio(VehicleState.ReverseGear);
        var s1 = VehicleState.AtRest(); s1.V = 5; s1.Gear = 1; s1.ClutchLocked = true;
        var sR = VehicleState.AtRest(); sR.V = -5 * tr1 / trR; sR.Gear = VehicleState.ReverseGear; sR.ClutchLocked = true;
        var u = new VehicleInputs { Throttle = 1 };
        var o1 = d.Evaluate(s1, u);
        var oR = d.Evaluate(sR, u);
        _out.WriteLine($"1st: {o1.EngineRpm:F1} rpm, F = {o1.FTractive:F1} N, a = {o1.Ax:F3}; R: {oR.EngineRpm:F1} rpm, F = {oR.FTractive:F1} N, a = {oR.Ax:F3}");
        Assert.Equal(o1.EngineRpm, oR.EngineRpm, 9);
        Assert.Equal(trR / tr1, oR.FTractive / o1.FTractive, 12);
        Assert.True(oR.Ax < 0);
    }

    [Fact]
    public void AtRestInReverse_DoesNotCreep()
    {
        var d = Car();
        Toggle(d);
        Run(d, new VehicleInputs(), 10);
        Assert.Equal(0, d.State.V);
        Assert.Equal(0, d.State.X, 12);
    }

    [Fact]
    public void Steering_InReverse_TurnsTheOtherWay()
    {
        var d = Car();
        Toggle(d);
        Run(d, new VehicleInputs { Throttle = 0.5 }, 2);
        Run(d, new VehicleInputs { Throttle = 0.5, SteerAngle = 0.2 }, 2);
        _out.WriteLine($"reversing with left lock: r = {d.State.YawRate:F3} rad/s, v_x = {d.State.V:F2}");
        Assert.True(d.State.V < 0);
        Assert.True(d.State.YawRate < 0);   // backing up with left lock swings the nose right (clockwise)
    }

    [Theory]
    [InlineData(15, true)]
    [InlineData(10.5, true)]
    [InlineData(9, false)]
    [InlineData(0, false)]
    public void ToggleAboveTenKmh_DestroysTheTransmission(double kmh, bool damaged)
    {
        var d = Car(kmh / 3.6);
        d.State = d.State with { ClutchLocked = true };
        Toggle(d);
        Assert.Equal(damaged, d.State.TransmissionDamaged);
        Assert.Equal(damaged, d.Outputs.TransmissionDamagedThisStep);
        Assert.Equal(damaged ? 1 : VehicleState.ReverseGear, d.State.Gear);
    }

    [Fact]
    public void DamagedTransmission_NoTorque_CoastsAndBrakes()
    {
        var d = Car(15 / 3.6);
        Toggle(d);
        Assert.True(d.State.TransmissionDamaged);
        double v0 = d.State.V;
        d.Step(new VehicleInputs { Throttle = 1 }, d.Parameters.Dt);
        Assert.Equal(0, d.Outputs.EngineTorque);
        Assert.Equal(0, d.Outputs.FTractive);
        Assert.Equal(0, d.Outputs.FEngineBrake);
        Assert.True(d.State.V < v0);                // throttle does nothing; resistances slow the car
        // Shifting and the reverse toggle are dead until a restart.
        int g = d.State.Gear;
        d.Step(new VehicleInputs { ShiftUp = true }, d.Parameters.Dt);
        Toggle(d);
        Assert.Equal(g, d.State.Gear);
        Run(d, new VehicleInputs { Brake = 1 }, 3);
        Assert.Equal(0, d.State.V);
        // A restart is a new state: damage cleared.
        Assert.False(VehicleState.AtRest().TransmissionDamaged);
    }

    [Fact]
    public void FromReverse_EngagingFirstWhileRollingBackFast_AlsoDamages()
    {
        var d = Car();
        Toggle(d);
        Run(d, new VehicleInputs { Throttle = 1 }, 6);
        Assert.True(d.State.V < -10 / 3.6);
        Toggle(d);
        Assert.True(d.State.TransmissionDamaged);

        var e = Car();
        Toggle(e);
        Run(e, new VehicleInputs { Throttle = 0.3 }, 1);
        Assert.True(e.State.V > -10 / 3.6 && e.State.V < 0);
        Toggle(e);
        Assert.False(e.State.TransmissionDamaged);
        Assert.Equal(1, e.State.Gear);
    }

    [Fact]
    public void NoMechanicalDamage_ToggleIgnoredAboveLimit()
    {
        var p = VehicleParameters.GameDefaults();
        p.MechanicalDamage = false;
        var d = Car(20 / 3.6, p);
        Toggle(d);
        Assert.False(d.State.TransmissionDamaged);
        Assert.NotEqual(VehicleState.ReverseGear, d.State.Gear);
    }

    [Fact]
    public void KinematicModel_HasNoReverse()
    {
        var p = VehicleParameters.GameDefaults();
        p.LateralModel = LateralModel.Kinematic;
        var d = Car(0, p);
        Toggle(d);
        Assert.Equal(1, d.State.Gear);
    }

    [Fact]
    public void Loop_ConsumesTheToggleInOneStep()
    {
        var d = Car();
        var loop = new SimulationLoop(d, new SimulationClock(d.Parameters.Dt));
        loop.Advance(0.05, new VehicleInputs { ToggleReverse = true });   // 10 steps, one toggle
        Assert.Equal(VehicleState.ReverseGear, d.State.Gear);
        loop.Advance(0.05, new VehicleInputs());
        Assert.Equal(VehicleState.ReverseGear, d.State.Gear);
    }
}
