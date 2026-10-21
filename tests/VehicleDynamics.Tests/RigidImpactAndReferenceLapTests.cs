using VehicleDynamics.Core.Collision;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;
using Xunit.Abstractions;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace VehicleDynamics.Tests;

public class RigidImpactAndReferenceLapTests
{
    private readonly ITestOutputHelper _out;
    public RigidImpactAndReferenceLapTests(ITestOutputHelper output) => _out = output;

    private const double M = 1150, Izz = 1100;

    private static double Energy(double vx, double vy, double w) => 0.5 * M * (vx * vx + vy * vy) + 0.5 * Izz * w * w;

    /// <summary>Car heading +x at 15 m/s into a wall ahead (normal −x), hitting at a point (px, py) relative to the CG.</summary>
    private static RigidContactInput Ahead(double px, double py, double e = 0.25, double muW = 0.4, double vx = 15, double vy = 0, double w = 0)
        => new(0, 0, 0, vx, vy, w, M, Izz, px, py, -1, 0, 0.01, e, muW);

    [Fact]
    public void CentredHeadOn_ReversesNormalSpeed_NoYaw()
    {
        var r = RigidBodyImpact.Resolve(Ahead(1.6, 0), bounce: true);
        Assert.True(r.ImpulseApplied);
        Assert.Equal(0, r.YawRate, 12);
        Assert.Equal(0.25 * 15, r.V, 9);               // v' = −e v along the normal
        Assert.Equal(Math.PI, Math.Abs(r.VelocityHeading), 9);
        Assert.Equal(15, r.Severity, 9);
    }

    [Fact]
    public void FrontLeftCorner_InducesYaw_AndLosesLessSpeed()
    {
        var centred = RigidBodyImpact.Resolve(Ahead(1.6, 0), bounce: true);
        var corner = RigidBodyImpact.Resolve(Ahead(1.6, 0.8), bounce: true);
        var cornerRight = RigidBodyImpact.Resolve(Ahead(1.6, -0.8), bounce: true);
        _out.WriteLine($"front-left: r' = {corner.YawRate:F3} rad/s, V' = {corner.V:F2} m/s (centred {centred.V:F2}); front-right r' = {cornerRight.YawRate:F3}");
        // Left side stopped, right side carries on: the car yaws left (CCW, +r); mirrored for the right corner.
        Assert.True(corner.YawRate > 0.5);
        Assert.Equal(-corner.YawRate, cornerRight.YawRate, 9);
        // Part of the impulse goes into rotation, so the normal impulse is smaller than for a centred hit.
        Assert.True(Energy(Math.Cos(corner.VelocityHeading) * corner.V, Math.Sin(corner.VelocityHeading) * corner.V, corner.YawRate) <= Energy(15, 0, 0));
        // Contact-point normal velocity after the impulse = −e · before.
        var after = new RigidContactInput(0, 0, 0, corner.V * Math.Cos(corner.VelocityHeading), corner.V * Math.Sin(corner.VelocityHeading),
            corner.YawRate, M, Izz, 1.6, 0.8, -1, 0, 0, 0.25, 0.4);
        Assert.Equal(0.25 * 15, RigidBodyImpact.ContactNormalSpeed(after, out _, out _), 6);
    }

    [Fact]
    public void GlancingScrape_WallFrictionYawsTheCar()
    {
        // Moving along +x at 25 m/s, drifting 2 m/s into a wall on the left (normal −y); front-left corner touches.
        var c = new RigidContactInput(0, 0, 0, 25, 2, 0, M, Izz, 1.6, 0.8, 0, -1, 0.01, 0.25, 0.4);
        var noFriction = RigidBodyImpact.Resolve(c with { WallFriction = 0 }, bounce: true);
        var friction = RigidBodyImpact.Resolve(c, bounce: true);
        _out.WriteLine($"scrape: r' = {friction.YawRate:F3} rad/s (no friction {noFriction.YawRate:F3}), V' = {friction.V:F2} (no friction {noFriction.V:F2})");
        Assert.True(friction.V < noFriction.V);
        Assert.True(Math.Abs(friction.YawRate - noFriction.YawRate) > 1e-3);
    }

    [Fact]
    public void Energy_NeverIncreases_RandomContacts()
    {
        var rng = new Random(7);
        double worst = double.NegativeInfinity;
        for (int k = 0; k < 20000; k++)
        {
            double a = rng.NextDouble() * 2 * Math.PI;
            var c = new RigidContactInput(0, 0, rng.NextDouble() * 6, rng.NextDouble() * 60 - 10, rng.NextDouble() * 20 - 10,
                rng.NextDouble() * 6 - 3, M, Izz, rng.NextDouble() * 4 - 2, rng.NextDouble() * 2 - 1, Math.Cos(a), Math.Sin(a),
                0.02, rng.NextDouble(), rng.NextDouble() * 1.5);
            var r = RigidBodyImpact.Resolve(c, bounce: rng.Next(2) == 0);
            double before = Energy(c.VelX, c.VelY, c.YawRate);
            double after = Energy(r.V * Math.Cos(r.VelocityHeading is var h && double.IsFinite(h) ? h : 0),
                r.V * Math.Sin(double.IsFinite(r.VelocityHeading) ? r.VelocityHeading : 0), r.YawRate);
            worst = Math.Max(worst, (after - before) / Math.Max(before, 1));
            Assert.True(r.NormalSpeed >= 0 || RigidBodyImpact.ContactNormalSpeed(c with
            {
                VelX = r.V * Math.Cos(double.IsFinite(r.VelocityHeading) ? r.VelocityHeading : 0),
                VelY = r.V * Math.Sin(double.IsFinite(r.VelocityHeading) ? r.VelocityHeading : 0),
                YawRate = r.YawRate,
            }, out _, out _) >= -1e-9, "contact point must not keep approaching");
        }
        _out.WriteLine($"worst relative energy change {worst:E2}");
        Assert.True(worst <= 1e-12);
    }

    [Fact]
    public void ApplyCollision_AcceptsRotationalToTranslationalTransfer()
    {
        var p = VehicleParameters.GameDefaults();
        var d = new Dyn(p);
        var s = VehicleState.AtRest();
        s.V = 2; s.YawRate = 3; s.ClutchLocked = true;
        d.State = s;
        // Spinning car whose rear corner hits a wall: CG speed may rise, total energy may not.
        var c = new RigidContactInput(0, 0, 0, 2, 0, 3, p.Mass, p.YawInertia, -2.1, 0.8, 0, -1, 0, 0.25, 0.4);
        var r = RigidBodyImpact.Resolve(c, bounce: true);
        d.ApplyCollision(r);
        Assert.Equal(r.YawRate, d.State.YawRate, 12);
        Assert.Throws<ArgumentException>(() => d.ApplyCollision(r with { V = 50 }));
    }

    [Fact]
    public void ReferenceLap_Oval_BrakesBeforeCornersAndHoldsCornerSpeed()
    {
        var oval = TrackLibrary.Oval();
        var p = VehicleParameters.GameDefaults();
        var pts = ReferenceLap.Compute(oval, p);
        double vArc = Math.Sqrt(0.85 * p.Mu * p.G / (1.0 / 80));
        var mid = pts.First(x => x.S > 400 + 40 * Math.PI);
        double lap = ReferenceLap.LapTime(pts, oval.Length);
        var brakeStart = pts.First(x => x.Zone == DriveZone.Brake);
        _out.WriteLine($"arc v {mid.V:F2} m/s (limit {vArc:F2}); lap {lap:F2} s; first brake at s = {brakeStart.S:F0} m; " +
            $"zones: {pts.Count(x => x.Zone == DriveZone.Accelerate)} acc / {pts.Count(x => x.Zone == DriveZone.Coast)} coast / {pts.Count(x => x.Zone == DriveZone.Brake)} brake");
        Assert.Equal(vArc, mid.V, 6);
        Assert.Equal(DriveZone.Coast, mid.Zone);
        // Exit of a turn is throttle, the straight ends under braking just before the next turn.
        Assert.Equal(DriveZone.Accelerate, pts.First(x => x.S > 50).Zone);
        Assert.True(brakeStart.S > 150 && brakeStart.S < 400);
        Assert.Equal(DriveZone.Brake, pts.Last(x => x.S < 399).Zone);
        Assert.True(lap > 40 && lap < 80);
        // A heavier car with less grip brakes earlier.
        var heavy = p.Clone(); heavy.Mass = 1800; heavy.Mu = 0.8;
        var brakeHeavy = ReferenceLap.Compute(oval, heavy).First(x => x.Zone == DriveZone.Brake);
        Assert.True(brakeHeavy.S < brakeStart.S + 1e-9);
    }
}
