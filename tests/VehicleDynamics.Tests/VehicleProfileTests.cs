using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;
using Xunit.Abstractions;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace VehicleDynamics.Tests;

/// <summary>Vehicle profiles, RWD and the A1 aerodynamic downforce.</summary>
public class VehicleProfileTests
{
    private readonly ITestOutputHelper _out;
    public VehicleProfileTests(ITestOutputHelper output) => _out = output;

    private static VehicleState Moving(double v, int gear)
    {
        var s = VehicleState.AtRest();
        s.V = v; s.Gear = gear; s.ClutchLocked = true;
        return s;
    }

    [Fact]
    public void Profiles_AreValid_AndMiniIsTheGameDefault()
    {
        foreach (var (id, _) in VehicleProfiles.All)
        {
            var p = VehicleProfiles.Create(id);
            Assert.Empty(p.Validate());
            Assert.Equal(id, p.Profile);
        }
        var mini = VehicleProfiles.Create(VehicleProfiles.MiniR56);
        var game = VehicleParameters.GameDefaults();
        Assert.Equal(SimulationRecorder.ParameterKeyValues(game), SimulationRecorder.ParameterKeyValues(mini));
        Assert.Equal(DriveLayout.FrontWheelDrive, mini.DriveLayout);
        Assert.Equal(0, mini.LiftCoefficient);
        // The class defaults stay model.m (validation runs use them): FWD, no downforce.
        var m = new VehicleParameters();
        Assert.Equal(DriveLayout.FrontWheelDrive, m.DriveLayout);
        Assert.Equal(0, m.LiftCoefficient);
        Assert.Throws<ArgumentException>(() => VehicleProfiles.Create("nope"));
    }

    [Fact]
    public void F1_ProvisionalValues()
    {
        var f1 = VehicleParameters.F1Provisional();
        var mini = VehicleParameters.MiniCooperR56();
        Assert.Equal(743, f1.Mass);
        Assert.Equal(DriveLayout.RearWheelDrive, f1.DriveLayout);
        Assert.True(f1.CorneringStiffnessFront > 2 * mini.CorneringStiffnessFront);
        Assert.True(f1.CorneringStiffnessRear > 2 * mini.CorneringStiffnessRear);
        Assert.Equal(0.330, f1.WheelRadius, 3);
        Assert.Equal(f1.GearRatios.Length, f1.MassFactors.Length);
        double peakKw = f1.RpmRaw.Zip(f1.TorqueRaw, (r, t) => r * Math.PI / 30 * t / 1000).Max();
        _out.WriteLine($"F1 peak power {peakKw:F0} kW; mass factors {string.Join(" ", f1.MassFactors.Select(x => x.ToString("F3")))}");
        Assert.InRange(peakKw, 650, 750);
    }

    [Fact]
    public void Downforce_FollowsHalfRhoV2ClA_AndLoadsTheTyres()
    {
        var p = VehicleParameters.F1Provisional();
        var d = new Dyn(p);
        var u = new VehicleInputs();
        var o = d.Evaluate(Moving(50, 5), u);
        Assert.Equal(0.5 * p.RhoAir * p.LiftCoefficient * p.FrontalArea * 2500, o.Downforce, 9);
        // Axle loads: weight plus downforce split by the aero balance (no longitudinal transfer when AxPrev = 0).
        double w = p.Mass * p.G;
        Assert.Equal(w * p.CgToRear / p.Wheelbase + o.Downforce * p.AeroBalanceFront, o.FzFront, 6);
        Assert.Equal(w + o.Downforce, o.FzFront + o.FzRear, 6);
        // Traction limit and brakes grow with it; rolling resistance too.
        Assert.Equal(TractionLimit.FxMax(p, p.Mu, p.Theta, o.Downforce), o.FxMax, 9);
        var slow = d.Evaluate(Moving(5, 1), u);
        Assert.True(o.FxMax > slow.FxMax * 1.5);
        var noAero = p.Clone(); noAero.LiftCoefficient = 0;
        var o0 = new Dyn(noAero).Evaluate(Moving(50, 5), u);
        Assert.Equal(0, o0.Downforce);
        Assert.True(o.FGrip > 1.5 * o0.FGrip);
        _out.WriteLine($"50 m/s: downforce {o.Downforce:F0} N = {o.Downforce / w:F2} W; axle grip {o.FGrip / w:F2} g vs {o0.FGrip / w:F2} g without aero");
    }

    [Fact]
    public void RwdTractionLimit_IsTheRearAxleForm()
    {
        var p = VehicleParameters.F1Provisional();
        double w = p.Mass * p.G;
        double expected = p.Mu * (w * (p.Wheelbase - p.CgToRear) / p.Wheelbase) / (1 - p.CgHeight / p.Wheelbase * p.Mu);
        Assert.Equal(expected, TractionLimit.FxMax(p, p.Mu, 0), 9);
        double df = 5000;
        double withAero = p.Mu * (w * (p.Wheelbase - p.CgToRear) / p.Wheelbase + df * (1 - p.AeroBalanceFront)) / (1 - p.CgHeight / p.Wheelbase * p.Mu);
        Assert.Equal(withAero, TractionLimit.FxMax(p, p.Mu, 0, df), 9);
    }

    [Fact]
    public void DriveForce_OnTheDrivenAxle()
    {
        var u = new VehicleInputs { Throttle = 1 };
        var f1 = new Dyn(VehicleParameters.F1Provisional()).Evaluate(Moving(20, 2), u);
        var mini = new Dyn(VehicleParameters.MiniCooperR56()).Evaluate(Moving(20, 2), u);
        Assert.Equal(0, f1.FxFront);
        Assert.True(f1.FxRear > 1000);
        Assert.Equal(0, mini.FxRear);
        Assert.True(mini.FxFront > 1000);
    }

    [Fact]
    public void F1_StraightLine_IsPlausible()
    {
        var p = VehicleParameters.F1Provisional();
        var d = new Dyn(p);
        var go = new VehicleInputs { Throttle = 1, AutoShift = true };
        double t100 = double.NaN, t200 = double.NaN;
        for (int i = 0; i < 60 / p.Dt; i++)
        {
            d.Step(go, p.Dt);
            if (double.IsNaN(t100) && d.State.V * 3.6 >= 100) t100 = d.State.Time;
            if (double.IsNaN(t200) && d.State.V * 3.6 >= 200) t200 = d.State.Time;
        }
        double vTop = d.State.V * 3.6;
        // Braking from top speed: peak deceleration with downforce.
        var stop = new VehicleInputs { Brake = 1 };
        d.Step(stop, p.Dt);
        double decel = -d.Outputs.Ax / p.G;
        _out.WriteLine($"F1 0-100 {t100:F2} s, 0-200 {t200:F2} s, speed after 60 s {vTop:F0} km/h, braking at top speed {decel:F1} g");
        Assert.InRange(t100, 2.0, 4.0);
        Assert.InRange(t200, 4.0, 8.0);
        Assert.InRange(vTop, 300, 370);
        Assert.InRange(decel, 3.5, 6.5);
    }

    [Fact]
    public void TractionControl_PreventsPowerOnSpin_AndTheLaunchStaysWithinItsCost()
    {
        (double vy, double r, double v) PowerOnCorner(bool tc)
        {
            var p = VehicleParameters.F1Provisional();
            p.TractionControl = tc;
            var d = new Dyn(p);
            d.State = Moving(20, 2);
            var u = new VehicleInputs { Throttle = 1, SteerAngle = 0.08, AutoShift = true };
            double maxVy = 0, maxR = 0;
            for (int i = 0; i < 3 / p.Dt; i++)
            {
                d.Step(u, p.Dt);
                maxVy = Math.Max(maxVy, Math.Abs(d.State.Vy));
                maxR = Math.Max(maxR, Math.Abs(d.State.YawRate));
            }
            return (maxVy, maxR, d.State.V);
        }
        var off = PowerOnCorner(false);
        var on = PowerOnCorner(true);
        _out.WriteLine($"full throttle, 0.08 rad lock from 72 km/h, 3 s: TC off max |v_y| {off.vy:F1} m/s, |r| {off.r:F2} rad/s; TC on {on.vy:F1} m/s, {on.r:F2} rad/s, v {on.v * 3.6:F0} km/h");
        Assert.True(off.vy > 5);              // power oversteer: the rear lets go
        Assert.True(on.vy < 2 && on.v > 20);  // stays on its line and still accelerates

        double Launch(bool tc)
        {
            var p = VehicleParameters.F1Provisional();
            p.TractionControl = tc;
            var d = new Dyn(p);
            var go = new VehicleInputs { Throttle = 1, AutoShift = true };
            while (d.State.V * 3.6 < 100 && d.State.Time < 10) d.Step(go, p.Dt);
            return d.State.Time;
        }
        double tOff = Launch(false), tOn = Launch(true);
        _out.WriteLine($"0-100 km/h: TC off {tOff:F2} s, TC on {tOn:F2} s");
        Assert.InRange(tOff, 2.2, 3.2);
        Assert.InRange(tOn, tOff, 4.0);
    }

    [Fact]
    public void ReferenceLap_F1CornersFasterWithDownforce()
    {
        var circuit = TrackLibrary.Circuit();
        var f1 = VehicleParameters.F1Provisional();
        var noAero = f1.Clone(); noAero.LiftCoefficient = 0;
        double lapF1 = ReferenceLap.LapTime(ReferenceLap.Compute(circuit, f1), circuit.Length);
        double lapNoAero = ReferenceLap.LapTime(ReferenceLap.Compute(circuit, noAero), circuit.Length);
        double lapMini = ReferenceLap.LapTime(ReferenceLap.Compute(circuit, VehicleParameters.MiniCooperR56()), circuit.Length);
        _out.WriteLine($"Circuit reference lap: F1 {lapF1:F1} s, F1 without downforce {lapNoAero:F1} s, Mini {lapMini:F1} s");
        Assert.True(lapF1 < lapNoAero && lapNoAero < lapMini);
    }
}
