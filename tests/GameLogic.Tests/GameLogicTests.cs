using System.Diagnostics;
using Simulator.Audio;
using Simulator.Dev;
using Simulator.Race;
using Simulator.Vehicle;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;
using Xunit.Abstractions;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace GameLogic.Tests;

public class EngineSynthTests
{
    private readonly ITestOutputHelper _out;
    public EngineSynthTests(ITestOutputHelper o) => _out = o;

    private static float[] Render(EngineSynth s, SynthControl c, double seconds)
    {
        int n = (int)(seconds * s.SampleRate);
        var l = new float[n];
        var r = new float[n];
        for (int i = 0; i < n; i += 256)
        {
            int m = Math.Min(256, n - i);
            s.Render(c, l.AsSpan(i, m), r.AsSpan(i, m));
        }
        return l;
    }

    /// <summary>Fundamental by autocorrelation over plausible firing periods (15–300 Hz).</summary>
    private static double EstimateF0(float[] x, int fs)
    {
        int start = fs / 2; // skip the 20 ms smoothing transient generously
        int len = fs;
        int minLag = fs / 300, maxLag = fs / 15;
        double best = double.NegativeInfinity;
        int bestLag = minLag;
        var corr = new double[maxLag + 2];
        for (int lag = minLag; lag <= maxLag + 1; lag++)
        {
            double c = 0;
            for (int i = start; i < start + len; i++) c += x[i] * x[i + lag];
            corr[lag] = c;
        }
        for (int lag = minLag + 1; lag <= maxLag; lag++)
        {
            // First strong peak: the true period, not a multiple of it.
            if (corr[lag] > corr[lag - 1] && corr[lag] >= corr[lag + 1] && corr[lag] > 0.6 * corr.Max())
            {
                bestLag = lag;
                break;
            }
            if (corr[lag] > best) { best = corr[lag]; bestLag = lag; }
        }
        // Parabolic interpolation around the peak.
        double a = corr[bestLag - 1], b = corr[bestLag], c2 = corr[bestLag + 1];
        double offset = 0.5 * (a - c2) / (a - 2 * b + c2);
        return fs / (bestLag + offset);
    }

    [Theory]
    [InlineData(1000)]
    [InlineData(3000)]
    [InlineData(6000)]
    public void Fundamental_TracksEngineRpm(double rpm)
    {
        var s = new EngineSynth(44100);
        var x = Render(s, new SynthControl { Rpm = rpm, Throttle = 0.6 }, 2.0);
        double f0 = EstimateF0(x, 44100);
        double expected = EngineSynth.FundamentalHz(rpm);
        _out.WriteLine($"rpm {rpm}: expected {expected:F2} Hz, measured {f0:F2} Hz ({100 * (f0 - expected) / expected:+0.00;-0.00} %)");
        Assert.InRange(f0, expected * 0.98, expected * 1.02);
    }

    [Fact]
    public void Squeal_SilentAtOrBelowSlipOne_AudibleAbove()
    {
        var quiet = Render(new EngineSynth(44100), new SynthControl { EngineOff = true, SpeedMps = 0.0, SlipIndicator = 1.0 }, 0.5);
        Assert.All(quiet, v => Assert.Equal(0f, v));

        // At 10 m/s squeal is enabled but slip 1.0 must add nothing on top of the wind noise.
        var atOne = Render(new EngineSynth(44100), new SynthControl { EngineOff = true, SpeedMps = 10, SlipIndicator = 1.0 }, 1.0);
        var above = Render(new EngineSynth(44100), new SynthControl { EngineOff = true, SpeedMps = 10, SlipIndicator = 1.6 }, 1.0);
        double Rms(float[] v) => Math.Sqrt(v.Skip(v.Length / 2).Average(z => (double)z * z));
        _out.WriteLine($"RMS slip=1.0: {Rms(atOne):E3}, slip=1.6: {Rms(above):E3}");
        Assert.True(Rms(above) > 5 * Rms(atOne));
    }

    [Fact]
    public void Render_DoesNotAllocate_AndIsCheap()
    {
        var s = new EngineSynth(44100);
        var l = new float[256];
        var r = new float[256];
        var c = new SynthControl { Rpm = 5000, Throttle = 1, SpeedMps = 30, SlipIndicator = 1.3, Coasting = false, RevLimiter = true };
        s.Render(c, l, r);
        var sw = new Stopwatch();
        long before = GC.GetAllocatedBytesForCurrentThread();
        sw.Start();
        const int chunks = 2000;
        for (int i = 0; i < chunks; i++) s.Render(c, l, r);
        sw.Stop();
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;  // Stopwatch created outside the window
        double usPerChunk = sw.Elapsed.TotalMilliseconds * 1000 / chunks;
        _out.WriteLine($"{usPerChunk:F1} µs per 256-frame chunk (real time budget {256.0 / 44100 * 1e6:F0} µs), allocated {allocated} B");
        Assert.Equal(0, allocated);
        Assert.True(usPerChunk < 256.0 / 44100 * 1e6 / 4);
    }
}

public class LapTimerTests
{
    private static VehicleState At(TrackGeometry t, double s, double time, double v = 20)
    {
        var p = t.Pose(s);
        var st = VehicleState.AtRest(p.X, p.Y, p.Psi);
        st.V = v;
        st.Time = time;
        return st;
    }

    [Fact]
    public void ForwardLap_CountsOnce_WithInterpolatedCrossing()
    {
        var t = TrackLibrary.Oval();
        var lt = new LapTimer(t);
        double dt = 0.05, L = t.Length;
        // Standing start at s = 0, constant 20 m/s for one lap plus 10 m.
        double time = 0;
        lt.AfterStep(At(t, 0, 0, 0), dt);
        for (double s = 1; s <= L + 10; s += 1)
        {
            time = s / 20;
            lt.AfterStep(At(t, t.NormalizeS(s), time), dt);
        }
        Assert.Single(lt.Laps);
        // Lap started at t = 0 (first moving step's previous time) and ended at s = L, t = L / 20.
        Assert.Equal(L / 20, lt.Laps[0].Time, 2);
        Assert.Equal(2, lt.Lap);
    }

    [Fact]
    public void BackwardsCrossingAndCutting_DoNotCompleteALap()
    {
        var t = TrackLibrary.Oval();
        var lt = new LapTimer(t);
        double L = t.Length;
        lt.AfterStep(At(t, 0, 0, 0), 0.05);
        lt.AfterStep(At(t, 5, 0.25), 0.05);         // lap 1 starts
        lt.AfterStep(At(t, L - 5, 0.5), 0.05);      // wrong way back across the line
        lt.AfterStep(At(t, 5, 0.75), 0.05);         // and forward again: no checkpoints passed
        Assert.Empty(lt.Laps);
        lt.AfterStep(At(t, L / 2, 10), 0.05);       // jump to half distance: checkpoint crossing needs < 50 m steps
        lt.AfterStep(At(t, L - 5, 20), 0.05);
        lt.AfterStep(At(t, 5, 20.5), 0.05);
        Assert.Empty(lt.Laps);
    }
}

public class CollisionAndDrivingTests
{
    private readonly ITestOutputHelper _out;
    public CollisionAndDrivingTests(ITestOutputHelper o) => _out = o;

    private static double Energy(Dyn d, VehicleParameters p)
        => 0.5 * p.Mass * (d.State.V * d.State.V + d.State.Vy * d.State.Vy) + 0.5 * p.YawInertia * d.State.YawRate * d.State.YawRate;

    [Fact]
    public void AngledImpact_EnergyNeverIncreases_NoTunnelling()
    {
        var t = TrackLibrary.Oval();
        var p = new VehicleParameters();
        // Mid straight 1, heading 30 degrees to the right towards the right-hand barrier at 30 m/s.
        var start = t.Pose(150);
        var dyn = new Dyn(p, new VehicleState { X = start.X, Y = start.Y, Psi = start.Psi - Math.PI / 6, V = 30, Gear = 4, EngineRpm = 3000 });
        var cm = new CollisionManager(t);
        int impacts = 0;
        cm.Impact += _ => impacts++;
        double maxLatBeyond = double.NegativeInfinity;
        double vBefore = 0, vAfter = 0;
        var u = new VehicleInputs { Throttle = 1, AutoShift = true };
        for (int i = 0; i < 2000; i++) // 10 s, full throttle, wheel straight: the car keeps pressing to the right
        {
            dyn.Step(u, p.Dt);
            double vStep = dyn.State.V;
            double eStep = Energy(dyn, p);
            int before = impacts;
            cm.AfterStep(dyn);
            Assert.True(Energy(dyn, p) <= eStep * (1 + 1e-9) + 1e-9, "collision response increased kinetic energy");
            if (impacts > before) { vBefore = vStep; vAfter = dyn.State.V; }
            var pr = t.Project(dyn.State.X, dyn.State.Y);
            maxLatBeyond = Math.Max(maxLatBeyond, -pr.Lateral - t.BarrierOffsetRightAt(pr.S));
        }
        _out.WriteLine($"impacts {impacts}, impact speed {vBefore:F2} -> {vAfter:F2} m/s, worst CG beyond wall {maxLatBeyond:F3} m, final V {dyn.State.V:F2} m/s, consecutive {cm.ConsecutiveImpulses}");
        // E4 rigid-body response: the first hit yaws the car, so the other end may swing into the wall too (a second,
        // separate impact). Never an impulse on consecutive steps.
        Assert.InRange(impacts, 1, 4);
        Assert.Equal(0, cm.ConsecutiveImpulses);
        Assert.True(vAfter < vBefore);
        Assert.True(maxLatBeyond < -0.5, "the CG must stay on the track side of the wall");
    }

    [Fact]
    public void PressingIntoWall_CannotBuildSpeed()
    {
        var t = TrackLibrary.Oval();
        var p = new VehicleParameters();
        var pose = t.Pose(150);
        // Perpendicular to the right wall, nose 1 m from it, at rest, full throttle.
        double off = -(t.BarrierOffsetRightAt(150) - 1.756 - 1.0);
        var (x, y) = t.PointAt(150, off);
        var dyn = new Dyn(p, VehicleState.AtRest(x, y, pose.Psi - Math.PI / 2));
        var cm = new CollisionManager(t);
        double vmax = 0;
        for (int i = 0; i < 1000; i++)
        {
            dyn.Step(new VehicleInputs { Throttle = 1 }, p.Dt);
            cm.AfterStep(dyn);
            if (i > 400) vmax = Math.Max(vmax, dyn.State.V);
        }
        _out.WriteLine($"max speed while pressed against the wall (last 3 s): {vmax:F4} m/s");
        Assert.True(vmax < 0.1);
    }

    [Theory]
    [InlineData("Oval")]
    [InlineData("Circuit")]
    public void CentrelineDriver_CompletesALap(string name)
    {
        var t = name == "Oval" ? TrackLibrary.Oval() : TrackLibrary.Circuit();
        var p = new VehicleParameters();
        var start = t.Pose(0);
        var dyn = new Dyn(p, VehicleState.AtRest(start.X, start.Y, start.Psi));
        var driver = new CentrelineDriver(t);
        var lt = new LapTimer(t);
        var cm = new CollisionManager(t);
        int impacts = 0;
        cm.Impact += _ => impacts++;
        double maxLat = 0;
        for (int i = 0; i < 200 * 300 && lt.Laps.Count == 0; i++)
        {
            var (th, br, st) = driver.Drive(dyn.State, dyn.Outputs, p);
            var u = VehicleInputs.FromNormalized(th, br, st, p, autoShift: true);
            u.Surface = t.SurfaceAt(dyn.State.X, dyn.State.Y);
            dyn.Step(u, p.Dt);
            cm.AfterStep(dyn);
            lt.AfterStep(dyn.State, p.Dt);
            maxLat = Math.Max(maxLat, Math.Abs(lt.Projection.Lateral));
        }
        _out.WriteLine($"{name}: lap {(lt.Laps.Count > 0 ? lt.Laps[0].Time : double.NaN):F2} s, valid {lt.Laps.FirstOrDefault().Valid}, max |lateral| {maxLat:F2} m, impacts {impacts}");
        Assert.Single(lt.Laps);
        Assert.True(lt.Laps[0].Valid);
        Assert.Equal(0, impacts);
        Assert.True(maxLat < t.Section.RoadHalfWidth);
    }
}
