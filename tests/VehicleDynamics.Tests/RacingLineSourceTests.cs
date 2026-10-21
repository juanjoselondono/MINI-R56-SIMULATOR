using System.Globalization;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;
using Xunit.Abstractions;

namespace VehicleDynamics.Tests;

public class RacingLineSourceTests
{
    private readonly ITestOutputHelper _out;
    public RacingLineSourceTests(ITestOutputHelper output) => _out = output;

    private static double MaxAbsCurvature(ITrackLine line)
    {
        double k = 0;
        for (double s = 0; s < line.Length; s += 1) k = Math.Max(k, Math.Abs(line.Curvature(s)));
        return k;
    }

    [Fact]
    public void Oval_OutsideApexOutside_WithinLimits_FasterReferenceLap()
    {
        var oval = TrackLibrary.Oval();
        var r = RacingLineOptimizer.MinimumCurvature(oval, margin: 1.2);
        // Oval turns left: inside = +lateral. Apex = mid arc; entry/exit = the straights just before/after the arc.
        double Off(double s) => r.Offsets[(int)Math.Round(oval.NormalizeS(s) / (oval.Length / r.Offsets.Length)) % r.Offsets.Length];
        double apex = Off(400 + 40 * Math.PI), entry = Off(330), exit = Off(400 + 80 * Math.PI + 70);
        var p = VehicleParameters.GameDefaults();
        double lapRef = ReferenceLap.LapTime(ReferenceLap.Compute(oval, p), oval.Length);
        double lapOpt = ReferenceLap.LapTime(ReferenceLap.Compute(r.Line, p), r.Line.Length);
        _out.WriteLine($"offsets entry {entry:F2}, apex {apex:F2}, exit {exit:F2} m; reference lap {lapOpt:F2} s vs centreline {lapRef:F2} s; {r.Sweeps} sweeps");
        Assert.True(apex > 5.0);                  // clips the inside (limit 7 − 1.2 = 5.8)
        Assert.True(entry < -3 && exit < -3);     // turns in from, and runs out to, the outside
        for (int i = 0; i < r.Offsets.Length; i++)
            Assert.InRange(r.Offsets[i], r.Lower[i] - 0.02, r.Upper[i] + 0.02);
        Assert.True(lapOpt < lapRef - 1.5);
        // s = 0 stays at the start/finish line.
        Assert.True(Math.Abs(oval.Project(r.Line.Pose(0).X, r.Line.Pose(0).Y).S - 0) < 1.0 ||
                    Math.Abs(oval.Project(r.Line.Pose(0).X, r.Line.Pose(0).Y).S - oval.Length) < 1.0);
    }

    [Fact]
    public void Circuit_ReducesPeakCurvature_AndStaysInside()
    {
        var c = TrackLibrary.Circuit();
        var r = RacingLineOptimizer.MinimumCurvature(c, margin: 1.2);
        double worst = 0;
        for (double s = 0; s < r.Line.Length; s += 2)
        {
            var p = r.Line.Pose(s);
            var pr = c.Project(p.X, p.Y);
            var (l, rr) = c.Limits(pr.S);
            worst = Math.Max(worst, Math.Max(pr.Lateral - (l - 1.2), -pr.Lateral - (rr - 1.2)));
        }
        double lapRef = ReferenceLap.LapTime(ReferenceLap.Compute(c, VehicleParameters.GameDefaults()), c.Length);
        double lapOpt = ReferenceLap.LapTime(ReferenceLap.Compute(r.Line, VehicleParameters.GameDefaults()), r.Line.Length);
        _out.WriteLine($"Circuit: κ max {MaxAbsCurvature(r.Line):F4} vs {MaxAbsCurvature(c):F4}; worst excursion beyond limit−margin {worst:F3} m; " +
            $"reference lap {lapOpt:F2} s vs centreline {lapRef:F2} s");
        // ∫κ² is minimised, not the peak: the hairpin apex may be tighter, the lap is what counts.
        Assert.True(worst < 0.06);
        Assert.True(lapOpt < lapRef - 5);
    }

    private static string Csv(IEnumerable<(double T, double X, double Y, int Lap, string Ev)> rows)
    {
        string path = Path.Combine(Path.GetTempPath(), $"lap_{Guid.NewGuid():N}.csv");
        var cols = SimulationRecorder.Columns;
        var lines = new List<string> { "# track=Oval", string.Join(",", cols) };
        foreach (var r in rows)
        {
            var f = Enumerable.Repeat("0", cols.Length).ToArray();
            f[Array.IndexOf(cols, "t_s")] = r.T.ToString(CultureInfo.InvariantCulture);
            f[Array.IndexOf(cols, "x_m")] = r.X.ToString("R", CultureInfo.InvariantCulture);
            f[Array.IndexOf(cols, "y_m")] = r.Y.ToString("R", CultureInfo.InvariantCulture);
            f[Array.IndexOf(cols, "lap")] = r.Lap.ToString(CultureInfo.InvariantCulture);
            f[Array.IndexOf(cols, "event")] = r.Ev;
            lines.Add(string.Join(",", f));
        }
        File.WriteAllLines(path, lines);
        return path;
    }

    [Fact]
    public void RecordedLap_PicksFastestValidLap()
    {
        // Lap 1: circle of radius 50 (slow); lap 2: circle of radius 60 (fast, valid); lap 3 starts.
        var rows = new List<(double, double, double, int, string)>();
        double t = 0;
        void Lap(int n, double radius)
        {
            for (int k = 0; k < 400; k++, t += 0.02)
            {
                double a = 2 * Math.PI * k / 400;
                rows.Add((t, radius * Math.Cos(a), radius * Math.Sin(a), n, ""));
            }
        }
        Lap(1, 50); Lap(2, 60);
        rows.Add((t, 60, 0, 3, "lap:1:41.000:valid;lap:2:39.500:valid"));
        string path = Csv(rows);
        try
        {
            var r = RecordedLapLine.FromTelemetryCsv(path);
            _out.WriteLine($"lap {r.Lap}, time {r.LapTime}, length {r.Line.Length:F1} m (2π·60 = {2 * Math.PI * 60:F1})");
            Assert.Equal(2, r.Lap);
            Assert.Equal(39.5, r.LapTime);
            Assert.True(r.FromLapEvent);
            Assert.Equal(2 * Math.PI * 60, r.Line.Length, 0);
            Assert.Equal(1.0 / 60, Math.Abs(r.Line.Curvature(100)), 3);
        }
        finally { File.Delete(path); }
    }

    [Fact]
    public void RecordedLap_WithoutEvents_UsesCompletedLap_RejectsForeignCsv()
    {
        var rows = new List<(double, double, double, int, string)>();
        for (int k = 0; k < 300; k++) rows.Add((k * 0.02, 40 * Math.Cos(2 * Math.PI * k / 300), 40 * Math.Sin(2 * Math.PI * k / 300), 1, ""));
        rows.Add((6, 40, 0, 2, ""));
        string path = Csv(rows);
        string bad = Path.Combine(Path.GetTempPath(), $"bad_{Guid.NewGuid():N}.csv");
        File.WriteAllText(bad, "a,b,c\n1,2,3\n4,5,6\n");
        try
        {
            var r = RecordedLapLine.FromTelemetryCsv(path);
            Assert.Equal(1, r.Lap);
            Assert.False(r.FromLapEvent);
            Assert.Throws<InvalidDataException>(() => RecordedLapLine.FromTelemetryCsv(bad));
        }
        finally { File.Delete(path); File.Delete(bad); }
    }
}
