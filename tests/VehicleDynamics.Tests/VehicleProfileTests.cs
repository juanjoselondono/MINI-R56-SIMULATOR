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
        double muR = p.Mu * p.RearGripFactor;   // wider rear tyres
        double expected = muR * (w * (p.Wheelbase - p.CgToRear) / p.Wheelbase) / (1 - p.CgHeight / p.Wheelbase * muR);
        Assert.Equal(expected, TractionLimit.FxMax(p, p.Mu, 0), 9);
        double df = 5000;
        double withAero = muR * (w * (p.Wheelbase - p.CgToRear) / p.Wheelbase + df * (1 - p.AeroBalanceFront)) / (1 - p.CgHeight / p.Wheelbase * muR);
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
        Assert.InRange(tOff, 2.0, 3.2);
        Assert.InRange(tOn, tOff, 4.0);
    }

    /// <summary>
    /// Corner exit: steady cornering on radius R at a share of μg (speed-holding throttle, yaw-rate steering
    /// controller), then instant full throttle for 2.5 s with the steering held. Returns the peak yaw rate over the
    /// neutral-steer yaw rate v tan δ / L (&gt; 1 = oversteer) and the steady-state rear-minus-front slip angle before
    /// the throttle (&lt; 0 = understeer).
    /// </summary>
    private static (double YawRatio, double PreGapDeg) CornerExit(VehicleParameters p, double radius, double ayShare)
    {
        var d = new Dyn(p);
        double v0 = Math.Sqrt(ayShare * p.Mu * p.G * radius);
        var s = Moving(v0, d.AutoGear(v0));
        s.YawRate = v0 / radius;
        d.State = s;
        double delta = Math.Atan(p.Wheelbase / radius);
        var u = new VehicleInputs { AutoShift = true };
        for (int i = 0; i < 4 / p.Dt; i++)
        {
            delta += 2 * (d.State.V / radius - d.State.YawRate) * p.Dt;
            u.SteerAngle = Math.Clamp(delta, -p.MaxSteerAngle, p.MaxSteerAngle);
            u.Throttle = Math.Clamp(0.3 + 0.5 * (v0 - d.State.V), 0, 1);
            d.Step(u, p.Dt);
        }
        double pre = (Math.Abs(d.Outputs.AlphaRear) - Math.Abs(d.Outputs.AlphaFront)) * 180 / Math.PI;
        double peak = 0;
        u.Throttle = 1;
        for (int i = 0; i < 2.5 / p.Dt; i++)
        {
            d.Step(u, p.Dt);
            double rNeutral = d.State.V * Math.Tan(Math.Abs(d.Outputs.SteerAngle)) / p.Wheelbase;
            if (rNeutral > 0.05) peak = Math.Max(peak, Math.Abs(d.State.YawRate) / rNeutral);
        }
        return (peak, pre);
    }

    /// <summary>
    /// Corner entry (trail braking): steady cornering on radius R at a share of μg as in <see cref="CornerExit"/>, then
    /// the brake at <paramref name="brake"/> for 0.8 s with the steering held (no correction), then the brake released
    /// to a speed-holding throttle for 2 s, steering still held. Returns the peak yaw rate over the neutral-steer yaw rate
    /// while braking (&lt; 1 = the car runs wide, "plows"; &gt; 1 = it rotates), the peak body slip angle |β| over both
    /// phases, and |β| at the end (recovered when small).
    /// </summary>
    private static (double YawRatio, double PeakBetaDeg, double EndBetaDeg, double SatRatio) CornerEntry(VehicleParameters p, double radius, double ayShare, double brake)
    {
        var d = new Dyn(p);
        // ayShare of the grip including downforce: v²/R = f μ (g + k v²).
        double k = Aerodynamics.SpecificCoefficient(p);
        double v0 = Math.Sqrt(ayShare * p.Mu * p.G / (1 / radius - ayShare * p.Mu * k));
        var s = Moving(v0, d.AutoGear(v0));
        s.YawRate = v0 / radius;
        d.State = s;
        double delta = Math.Atan(p.Wheelbase / radius);
        var u = new VehicleInputs { AutoShift = true };
        for (int i = 0; i < 4 / p.Dt; i++)
        {
            delta += 2 * (d.State.V / radius - d.State.YawRate) * p.Dt;
            u.SteerAngle = Math.Clamp(delta, -p.MaxSteerAngle, p.MaxSteerAngle);
            u.Throttle = Math.Clamp(0.3 + 0.5 * (v0 - d.State.V), 0, 1);
            d.Step(u, p.Dt);
        }
        double peak = 0, beta = 0, sat = 0;
        u.Throttle = 0;
        u.Brake = brake;
        for (int i = 0; i < 0.8 / p.Dt; i++)
        {
            d.Step(u, p.Dt);
            if (i == (int)(0.3 / p.Dt)) sat = SaturationSlip(p, d.Outputs, rear: true) / SaturationSlip(p, d.Outputs, rear: false);
            double rNeutral = d.State.V * Math.Tan(Math.Abs(d.Outputs.SteerAngle)) / p.Wheelbase;
            if (rNeutral > 0.05) peak = Math.Max(peak, Math.Abs(d.State.YawRate) / rNeutral);
            beta = Math.Max(beta, Math.Abs(d.Outputs.Beta));
        }
        u.Brake = 0;
        double v1 = d.State.V;
        for (int i = 0; i < 2 / p.Dt; i++)
        {
            u.Throttle = Math.Clamp(0.3 + 0.5 * (v1 - d.State.V), 0, 1);
            d.Step(u, p.Dt);
            beta = Math.Max(beta, Math.Abs(d.Outputs.Beta));
        }
        return (peak, beta * 180 / Math.PI, Math.Abs(d.Outputs.Beta) * 180 / Math.PI, sat);
    }

    /// <summary>Slip angle where an axle's linear force C_α α reaches what its friction circle leaves laterally, F_y,max / C_α.</summary>
    private static double SaturationSlip(VehicleParameters p, VehicleOutputs o, bool rear)
    {
        double g = rear ? o.GripRear : o.GripFront, fx = rear ? o.FxRear : o.FxFront;
        return Math.Sqrt(Math.Max(0, g * g - fx * fx)) / (rear ? p.CorneringStiffnessRear : p.CorneringStiffnessFront);
    }

    [Fact]
    public void F1_CornerEntry_RotatesSlightlyUnderTrailBraking_AndRecovers()
    {
        foreach (double radius in new[] { 50.0, 150.0 })
        {
            var p = VehicleParameters.F1Provisional();
            foreach (double b in new[] { 0.3, 0.6 })
            {
                var (ratio, beta, end, sat) = CornerEntry(p, radius, 0.85, b);
                _out.WriteLine($"R {radius} m at 0.85 grip, brake {b}: peak r / r_neutral {ratio:F2}, α*_r/α*_f {sat:F2}, peak β {beta:F1}°, β after release {end:F1}°");
                Assert.InRange(ratio, 1.0, 1.4);   // rotates a little on entry (no plowing) ...
                Assert.True(sat < 1);               // ... because the rear reaches its saturation slip angle first
                Assert.True(beta < 4);              // a small slide, not a spin
                Assert.True(end < 1.5);             // and it settles once the brake is released
            }
            var (full, betaFull, _, _) = CornerEntry(p, radius, 0.85, 1.0);
            _out.WriteLine($"R {radius} m, full brake: peak r / r_neutral {full:F2}, peak β {betaFull:F1}°");
            Assert.InRange(full, 0.9, 1.1);         // a stamped (keyboard) brake neither plows nor spins
            Assert.True(betaFull < 4);

            // The previous setup (C_α 180k/320k, 38 % front downforce, load-proportional brakes) plowed at speed.
            var old = VehicleParameters.F1Provisional();
            old.CorneringStiffnessFront = 180000; old.CorneringStiffnessRear = 320000;
            old.AeroBalanceFront = 0.38; old.BrakeBiasFront = 0;
            var (oldRatio, _, _, _) = CornerEntry(old, radius, 0.85, 0.6);
            _out.WriteLine($"R {radius} m, previous setup: peak r / r_neutral {oldRatio:F2}");
            Assert.True(oldRatio < 0.9);
        }
    }

    [Fact]
    public void F1_CornerExit_StaysInUndersteerWithTractionControl()
    {
        foreach (double radius in new[] { 12.0, 40.0, 100.0 })
        {
            var (ratio, pre) = CornerExit(VehicleParameters.F1Provisional(), radius, 0.7);
            var raw = VehicleParameters.F1Provisional();
            raw.TractionControl = false;
            var (ratioOff, _) = CornerExit(raw, radius, 0.7);
            _out.WriteLine($"R {radius} m at 0.7 μg: before throttle α_r − α_f = {pre:F2}°; full-throttle exit peak r / r_neutral = {ratio:F2} (TC off: {ratioOff:F1})");
            Assert.True(pre < 0);            // steady state: understeer margin
            Assert.True(ratio <= 1.05);      // power exit with TC: at most marginally faster than neutral steer
            Assert.True(ratioOff > 5);       // the raw chassis still spins under full power (no TC on real 2019 cars)
        }
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
