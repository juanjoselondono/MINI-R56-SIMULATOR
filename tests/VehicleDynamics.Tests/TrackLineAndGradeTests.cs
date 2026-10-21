using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;
using Xunit.Abstractions;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace VehicleDynamics.Tests;

/// <summary>Converted-track support: polyline reference line, per-step road grade (E6), surface grip override (E2).</summary>
public class TrackLineAndGradeTests
{
    private readonly ITestOutputHelper _out;
    public TrackLineAndGradeTests(ITestOutputHelper output) => _out = output;

    /// <summary>The model.m oval sampled every 1 m, as a converted racing line would be.</summary>
    private static PolylineTrack SampledOval(out TrackGeometry oval, double[]? left = null, double[]? right = null)
    {
        oval = TrackLibrary.Oval();
        var pts = oval.Sample(1.0).SkipLast(1).ToArray();
        return new PolylineTrack("oval-sampled", pts.Select(p => p.Pose.X).ToArray(), pts.Select(p => p.Pose.Y).ToArray(), left, right);
    }

    [Fact]
    public void Polyline_SampledOval_MatchesAnalyticGeometry()
    {
        var line = SampledOval(out var oval);
        double relLen = Math.Abs(line.Length - oval.Length) / oval.Length;
        double kArc = line.Curvature(400 + 40 * Math.PI), kStraight = line.Curvature(200);
        _out.WriteLine($"length {line.Length:F3} vs {oval.Length:F3} m (rel {relLen:E1}); κ arc {kArc:F5} (1/80 = {1.0 / 80:F5}), straight {kStraight:E1}");
        Assert.True(relLen < 1e-4);
        Assert.Equal(1.0 / 80, kArc, 4);
        Assert.True(Math.Abs(kStraight) < 1e-9);

        // Project(PointAt(s, lat)) returns (s, lat) on straights and arcs, including across the closing seam.
        foreach (double s in new[] { 0.5, 150.0, 400 + 30 * Math.PI, 700.0, line.Length - 0.25 })
            foreach (double lat in new[] { -6.0, 0.0, 4.5 })
            {
                var (x, y) = line.PointAt(s, lat);
                var pr = line.Project(x, y);
                Assert.Equal(lat, pr.Lateral, 2);
                Assert.True(Math.Abs(FrameConversion.WrapAngle((pr.S - s) / line.Length * 2 * Math.PI)) * line.Length / (2 * Math.PI) < 0.05,
                    $"s={s} lat={lat}: projected s={pr.S}");
            }

        // Far off the line the brute-force fallback still finds the nearest point.
        var far = line.Project(200, -300);
        Assert.Equal(200, far.S, 1);
        Assert.Equal(-300, far.Lateral, 1);
    }

    [Fact]
    public void Polyline_Limits_InterpolateTrackEdges()
    {
        var o = TrackLibrary.Oval();
        int n = o.Sample(1.0).Count() - 1;
        var left = Enumerable.Range(0, n).Select(i => 5.0 + (i % 2)).ToArray();
        var right = Enumerable.Repeat(7.0, n).ToArray();
        var line = SampledOval(out _, left, right);
        var (l, r) = line.Limits(10.5);
        Assert.Equal(5.5, l, 6);
        Assert.Equal(7.0, r, 9);
    }

    /// <summary>E6: a per-step RoadGrade on flat parameters is the same physics as model.m's global slope.</summary>
    [Fact]
    public void RoadGrade_EqualsGlobalSlope()
    {
        var sloped = new VehicleParameters { SlopeMode = SlopeMode.Pct, SlopeValue = 6 };
        var flat = new VehicleParameters();
        var a = new Dyn(sloped);
        var b = new Dyn(flat);
        var u = new VehicleInputs { Throttle = 1 };
        var ug = u;
        ug.RoadGrade = sloped.Theta;
        for (int i = 0; i < 2000; i++) { a.Step(u, sloped.Dt); b.Step(ug, flat.Dt); }
        _out.WriteLine($"10 s up 6 %: V {a.State.V:R} vs {b.State.V:R} m/s");
        Assert.Equal(a.State.V, b.State.V);
        Assert.Equal(a.State.Distance, b.State.Distance);
        Assert.Equal(a.Outputs.FGrade, b.Outputs.FGrade);
        Assert.True(a.Outputs.FGrade > 0);
    }

    /// <summary>E2: track-supplied friction (surfaces.ini) replaces the generic surface factor on μ.</summary>
    [Fact]
    public void SurfaceGrip_OverridesSurfaceFactor()
    {
        var p = VehicleParameters.GameDefaults();
        var d = new Dyn(p);
        var u = new VehicleInputs { Surface = Surface.Curb, SurfaceGrip = 0.95 };
        Assert.Equal(TractionLimit.FxMax(p, p.Mu * 0.95), d.Evaluate(d.State, u).FxMax);
        u.SurfaceGrip = 0;
        Assert.Equal(TractionLimit.FxMax(p, p.Mu * p.GripFactor(Surface.Curb)), d.Evaluate(d.State, u).FxMax);
    }
}
