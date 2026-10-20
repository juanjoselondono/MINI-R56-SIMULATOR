using VehicleDynamics.Core.Collision;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;
using Xunit.Abstractions;

namespace VehicleDynamics.Tests;

public class TrackAndCollisionTests
{
    private readonly ITestOutputHelper _out;
    public TrackAndCollisionTests(ITestOutputHelper output) => _out = output;

    [Fact]
    public void Oval_MatchesModelGeometryAndCloses()
    {
        var t = TrackLibrary.Oval();
        Assert.Equal(2 * 400 + 2 * Math.PI * 80, t.Length, 9);
        var (p, h) = t.ClosureError();
        _out.WriteLine($"Oval closure: {p:E2} m, {h:E2} rad");
        Assert.True(p < 1e-6 && h < 1e-9);
        var a = t.Pose(400);
        Assert.Equal(400, a.X, 9); Assert.Equal(0, a.Y, 9); Assert.Equal(0, a.Psi, 12);
        var b = t.Pose(400 + 80 * Math.PI);
        Assert.Equal(400, b.X, 9); Assert.Equal(160, b.Y, 9); Assert.Equal(Math.PI, b.Psi, 12);
        Assert.Equal(1.0 / 80, t.Curvature(500), 15);
        Assert.Equal(0, t.Curvature(100));
        Assert.Equal(0, t.Pose(0).X);
    }

    [Fact]
    public void Circuit_RequirementsAndClosure()
    {
        var t = TrackLibrary.Circuit();
        var (p, h) = t.ClosureError();
        _out.WriteLine($"Circuit length {t.Length:F3} m, closure {p:E2} m / {h:E2} rad");
        foreach (var s in t.Segments)
            _out.WriteLine($"  {s.Name,-16} {s.Kind,-8} L={s.Length,9:F3}  R={(s.Kind == SegmentKind.Straight ? "-" : s.Radius.ToString("F1"))}  turn={s.TurnAngle * 180 / Math.PI:F1} deg");
        Assert.True(p < 1e-6 && h < 1e-9);
        Assert.True(t.Length >= 2500);
        Assert.Contains(t.Segments, s => s.Kind == SegmentKind.Arc && s.Radius >= 25 && s.Radius <= 30 && Math.Abs(s.TurnAngle) >= Math.PI * 0.9);
        Assert.True(t.Segments.Count(s => s.Kind == SegmentKind.Arc && s.Radius >= 120) >= 2);
        Assert.True(t.Segments.Count(s => s.Kind == SegmentKind.Straight && s.Length >= 500) >= 2);
        // left-right chicane: a left arc followed (directly or via a short link) by a right arc
        bool chicane = false;
        for (int i = 0; i + 2 < t.Segments.Count; i++)
        {
            var a = t.Segments[i];
            var mid = t.Segments[i + 1];
            var b = t.Segments[i + 2];
            if (a.Kind == SegmentKind.Arc && a.K0 > 0 && mid.Kind == SegmentKind.Straight && mid.Length < 30 && b.Kind == SegmentKind.Arc && b.K0 < 0)
                chicane = true;
        }
        Assert.True(chicane);
    }

    [Theory]
    [InlineData("Oval")]
    [InlineData("Circuit")]
    public void Projection_RoundTripInsideBarriers(string name)
    {
        var t = name == "Oval" ? TrackLibrary.Oval() : TrackLibrary.Circuit();
        var rng = new Random(7);
        double worstS = 0, worstLat = 0;
        for (int i = 0; i < 3000; i++)
        {
            double s = rng.NextDouble() * t.Length;
            int seg = t.SegmentIndexAt(s);
            double left = t.BarrierOffsetLeft(seg), right = t.BarrierOffsetRight(seg);
            double lat = -right + rng.NextDouble() * (left + right);
            var (x, y) = t.PointAt(s, lat);
            var pr = t.Project(x, y);
            double ds = Math.Abs(pr.S - s);
            ds = Math.Min(ds, t.Length - ds);
            worstS = Math.Max(worstS, ds);
            worstLat = Math.Max(worstLat, Math.Abs(pr.Lateral - lat));
        }
        _out.WriteLine($"{name}: projection round trip worst |ds| = {worstS:E2} m, |dlat| = {worstLat:E2} m");
        Assert.True(worstS < 1e-6);
        Assert.True(worstLat < 1e-6);
    }

    [Fact]
    public void Track_PoseContinuousAtJoints()
    {
        foreach (var t in new[] { TrackLibrary.Oval(), TrackLibrary.Circuit() })
        {
            for (int i = 1; i < t.Segments.Count; i++)
            {
                var prev = t.Segments[i - 1];
                var end = prev.PoseAt(prev.Length);
                var st = t.Segments[i].Start;
                Assert.Equal(st.X, end.X, 9);
                Assert.Equal(st.Y, end.Y, 9);
                Assert.Equal(st.Psi, end.Psi, 12);
            }
        }
    }

    [Fact]
    public void Clothoid_HeadingClosedFormAndPositionAccurate()
    {
        var seg = TrackSegment.Clothoid(60, 0, 1.0 / 30);
        var t = new TrackGeometry("cl", new Pose2(1, 2, 0.3), new[] { seg }, closed: false);
        Assert.Equal(0.3 + 0.5 * (1.0 / 30) * 60, t.EndPose.Psi, 12);
        Assert.Equal(1.0 / 60, t.Curvature(30), 15);
        // brute-force midpoint integration with 2e6 steps
        int n = 2_000_000;
        double h = 60.0 / n, x = 1, y = 2;
        for (int i = 0; i < n; i++)
        {
            double u = (i + 0.5) * h;
            double th = 0.3 + (1.0 / 30) * u * u / 120;
            x += h * Math.Cos(th);
            y += h * Math.Sin(th);
        }
        Assert.Equal(x, t.EndPose.X, 7);
        Assert.Equal(y, t.EndPose.Y, 7);
        var (px, py) = t.PointAt(25, 3);
        var pr = t.Project(px, py);
        Assert.Equal(25, pr.S, 6);
        Assert.Equal(3, pr.Lateral, 6);
    }

    [Fact]
    public void CrossSection_SurfaceLookup()
    {
        var t = TrackLibrary.Oval();
        Assert.Equal(Surface.Asphalt, t.SurfaceAt(0, 5.9));
        Assert.Equal(Surface.Curb, t.SurfaceAt(0, -6.5));
        Assert.Equal(Surface.Grass, t.SurfaceAt(0, 12));
        Assert.Equal(Surface.Barrier, t.SurfaceAt(0, -16.5));
        var (x, y) = t.PointAt(200, 8);
        Assert.Equal(Surface.Grass, t.SurfaceAt(x, y));
    }

    [Fact]
    public void BarrierContact_ReportsPenetrationAndInwardNormal()
    {
        var t = TrackLibrary.Oval();
        // On straight 1 (heading +X), left barrier at y = 16. Car centred at y = 15.5, half width 0.85.
        var c = BarrierContact.Check(t, 200, 15.5, 0, 1.9, 0.85);
        Assert.NotNull(c);
        Assert.True(c!.Value.LeftSide);
        Assert.Equal(0.35, c.Value.Penetration, 9);
        Assert.Equal(0, c.Value.NormalX, 12);
        Assert.Equal(-1, c.Value.NormalY, 12);
        Assert.Null(BarrierContact.Check(t, 200, 0, 0, 1.9, 0.85));
        // Right barrier in the middle of curve 1 (outside of the left turn)
        var p = t.Pose(400 + 40 * Math.PI);
        var (x, y) = t.PointAt(400 + 40 * Math.PI, -16.2);
        var c2 = BarrierContact.Check(t, x, y, p.Psi, 1.9, 0.85);
        Assert.NotNull(c2);
        Assert.False(c2!.Value.LeftSide);
        // deepest corner: outer front/rear corner at radius sqrt((80 + 16.2 + 0.85)^2 + 1.9^2)
        Assert.Equal(Math.Sqrt(97.05 * 97.05 + 1.9 * 1.9) - 96, c2.Value.Penetration, 6);
    }

    [Fact]
    public void Collision_EnergyNeverIncreases()
    {
        var rng = new Random(3);
        for (int i = 0; i < 20000; i++)
        {
            double v = rng.NextDouble() * 70;
            var c = new CollisionInput(0, 0, rng.NextDouble() * 6.3 - 3.15, rng.NextDouble() * 0.2 - 0.1, v,
                0, -1, rng.NextDouble() * 0.5, rng.NextDouble(), rng.NextDouble(), rng.NextDouble() * Math.PI);
            var r = CollisionResponse.Resolve(c);
            Assert.True(r.V <= v + 1e-12, $"V grew {v} -> {r.V}");
            Assert.True(r.V >= 0);
        }
    }

    [Fact]
    public void Collision_HeadOnAndGlancing()
    {
        // Head-on into a wall whose normal points -X (car heading +X): V' = e V, severity = V
        var r = CollisionResponse.Resolve(new CollisionInput(0, 0, 0, 0, 20, -1, 0, 0.3));
        Assert.True(r.ImpulseApplied);
        Assert.Equal(5, r.V, 9);
        Assert.Equal(20, r.Severity, 9);
        Assert.Equal(-0.3, r.X, 12);
        Assert.Equal(0.6, Math.Abs(r.Psi), 12); // heading kick limited to maxYawKick
        // Glancing 10 deg: small severity, speed mostly preserved, heading turned away from the wall
        double a = 10 * Math.PI / 180;
        var g = CollisionResponse.Resolve(new CollisionInput(0, 0, a, 0, 30, 0, -1, 0.1));
        Assert.Equal(30 * Math.Sin(a), g.Severity, 9);
        Assert.True(g.V > 0.9 * 30);
        Assert.True(g.Psi < a);
        // Moving away: no impulse, position still pushed out
        var away = CollisionResponse.Resolve(new CollisionInput(0, 0, -a, 0, 30, 0, -1, 0.1));
        Assert.False(away.ImpulseApplied);
        Assert.Equal(-0.1, away.Y, 12);
        Assert.Equal(30, away.V);
    }

    [Fact]
    public void Collision_LatchPreventsRepeatedImpulses()
    {
        var latch = new ContactLatch();
        double v = 20, psi = 0.3, y = 15.0;
        int impulses = 0;
        // Car pressing into a wall (normal -Y) for 50 frames: only the entry produces an impulse.
        for (int f = 0; f < 50; f++)
        {
            var input = new CollisionInput(0, y, psi, 0, v, 0, -1, 0.05);
            double un = CollisionResponse.NormalSpeed(input);
            bool apply = latch.Update(true, un);
            var r = CollisionResponse.Resolve(input, apply);
            if (r.ImpulseApplied) impulses++;
            v = r.V;
            psi = r.Psi;
            // driver keeps steering into the wall
            psi = Math.Max(psi, 0.05);
        }
        Assert.Equal(1, impulses);
        // Separation then a new approach: second impulse allowed
        latch.Update(false, 0);
        Assert.True(latch.Update(true, -1));
        Assert.False(latch.Update(true, -1));
        Assert.False(latch.Update(true, 0.5));   // moving away -> unlatched
        Assert.True(latch.Update(true, -0.5));
    }
}
