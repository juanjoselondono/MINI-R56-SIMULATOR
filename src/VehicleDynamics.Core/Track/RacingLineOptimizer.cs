using System.Globalization;

namespace VehicleDynamics.Core.Track;

/// <summary>
/// EXTENSION (driver aid): geometric racing line inside the track limits (outside → apex → outside through every
/// corner), by curvature minimisation.
///
/// The line is a reference line moved sideways: P_i = C_i + n_i N_i (N_i = left normal at equally spaced s_i), with the
/// box constraints −(right_i − margin) ≤ n_i ≤ left_i − margin (the margin keeps the body, half width ≈ 0.84 m, inside).
/// Energy E = Σ |P_{i−1} − 2 P_i + P_{i+1}|² (discrete ∫κ²), a convex quadratic in n, solved by projected Gauss–Seidel
/// with over-relaxation (exact 1-D minimiser per coordinate, clamped to the box), coarse-to-fine (8Δ, 4Δ, 2Δ, Δ).
///
/// Outer iterations re-solve around the previous result, resampled by its own arc length: a single solve is biased
/// towards the inside of corners, because points there sit closer together and the second difference hides curvature.
/// Measured with the reference lap (ReferenceLap, game parameters): Oval 40.40 s vs 42.82 s on the centreline, Circuit
/// 73.48 s vs 84.17 s, converged after 2–4 outer passes. A reweighted variant aiming at the peak curvature (IRLS,
/// |κ|^(p−2), p = 4–8) was tried and was slower on both tracks (kinks), so the plain ∫κ² line is used. It spreads a turn
/// over long entry/exit transitions and accepts a slightly tighter apex.
/// </summary>
public static class RacingLineOptimizer
{
    public sealed record Result(PolylineTrack Line, double[] ReferenceS, double[] Offsets, double[] Lower, double[] Upper, int Sweeps);

    public static Result MinimumCurvature(ITrackLine reference, double margin = 1.2, double spacing = 2.0, double omega = 1.7,
        int outerIterations = 3)
    {
        ITrackLine around = reference;
        PolylineTrack line = null!;
        int sweeps = 0;
        for (int k = 0; k < Math.Max(1, outerIterations); k++)
        {
            (line, int sw) = SolveAround(around, margin, spacing, omega);
            sweeps += sw;
            around = line;
        }

        // Diagnostics against the original reference: lateral position of the line at each reference station.
        int n = Math.Max(8, (int)Math.Round(reference.Length / spacing));
        var s = new double[n]; var off = new double[n]; var lo = new double[n]; var hi = new double[n];
        for (int i = 0; i < n; i++)
        {
            s[i] = reference.Length * i / n;
            var p = reference.Pose(s[i]);
            off[i] = -line.Project(p.X, p.Y).Lateral;
            var (l, r) = reference.Limits(s[i]);
            lo[i] = -(r - margin);
            hi[i] = l - margin;
            if (lo[i] > hi[i]) lo[i] = hi[i] = 0.5 * (lo[i] + hi[i]);
        }
        return new Result(line, s, off, lo, hi, sweeps);
    }

    private const int Coarsest = 8;
    private static int Stations(ITrackLine line, double spacing)
        => Math.Max(Coarsest * 4, (int)Math.Ceiling(line.Length / spacing / Coarsest) * Coarsest);

    private static (PolylineTrack Line, int Sweeps) SolveAround(ITrackLine reference, double margin, double spacing, double omega)
    {
        int n = Stations(reference, spacing);
        double ds = reference.Length / n;
        var cx = new double[n]; var cy = new double[n]; var nx = new double[n]; var ny = new double[n];
        var lo = new double[n]; var hi = new double[n];
        var left = new double[n]; var right = new double[n];
        for (int i = 0; i < n; i++)
        {
            var p = reference.Pose(i * ds);
            cx[i] = p.X; cy[i] = p.Y;
            nx[i] = -Math.Sin(p.Psi); ny[i] = Math.Cos(p.Psi);
            (left[i], right[i]) = reference.Limits(i * ds);
            lo[i] = -(right[i] - margin);
            hi[i] = left[i] - margin;
            if (lo[i] > hi[i]) lo[i] = hi[i] = 0.5 * (lo[i] + hi[i]);
        }

        var off = new double[n];
        for (int i = 0; i < n; i++) off[i] = Math.Clamp(0, lo[i], hi[i]);
        int sweeps = 0;
        foreach (int step in new[] { Coarsest, 4, 2, 1 })
        {
            int m = n / step;
            int maxSweeps = step == Coarsest ? 6000 : 2500;
            for (int sweep = 0; sweep < maxSweeps; sweep++)
            {
                double maxChange = 0;
                for (int j = 0; j < m; j++)
                {
                    int i = j * step;
                    int Idx(int k) => ((j + k) % m + m) % m * step;
                    double Px(int k) { int q = Idx(k); return cx[q] + off[q] * nx[q]; }
                    double Py(int k) { int q = Idx(k); return cy[q] + off[q] * ny[q]; }
                    // Second differences T(k) = P(k−1) − 2P(k) + P(k+1) for k = −1, 0, +1 around station i.
                    double tmx = Px(-2) - 2 * Px(-1) + Px(0), tmy = Py(-2) - 2 * Py(-1) + Py(0);
                    double t0x = Px(-1) - 2 * Px(0) + Px(1), t0y = Py(-1) - 2 * Py(0) + Py(1);
                    double tpx = Px(0) - 2 * Px(1) + Px(2), tpy = Py(0) - 2 * Py(1) + Py(2);
                    // ∂E/∂n_i = 2 N_i·(T(−1) − 2 T(0) + T(+1)); ∂²E/∂n_i² = 12.
                    double grad = 2 * ((tmx + tpx - 2 * t0x) * nx[i] + (tmy + tpy - 2 * t0y) * ny[i]);
                    double next = Math.Clamp(off[i] - omega * grad / 12, lo[i], hi[i]);
                    maxChange = Math.Max(maxChange, Math.Abs(next - off[i]));
                    off[i] = next;
                }
                sweeps++;
                if (maxChange < 1e-5) break;
            }
            if (step > 1)
            {
                // Prolong to the next finer level: linear between solved stations, clamped to the local limits.
                int half = step / 2;
                for (int j = 0; j < m; j++)
                {
                    int a = j * step, b = ((j + 1) % m) * step, mid = a + half;
                    off[mid] = Math.Clamp(0.5 * (off[a] + off[b]), lo[mid], hi[mid]);
                }
            }
        }

        var x = new double[n]; var y = new double[n]; var l = new double[n]; var r = new double[n];
        for (int i = 0; i < n; i++)
        {
            x[i] = cx[i] + off[i] * nx[i];
            y[i] = cy[i] + off[i] * ny[i];
            l[i] = left[i] - off[i];
            r[i] = right[i] + off[i];
        }
        string name = reference.Name.EndsWith(" (racing line)") ? reference.Name : reference.Name + " (racing line)";
        return (new PolylineTrack(name, x, y, l, r), sweeps);
    }
}

/// <summary>
/// Racing line from a recorded lap: reads a telemetry CSV exported by the game (columns x_m, y_m, lap, event) and
/// takes the fastest valid lap (its "lap:N:time:valid" event). Without lap events (recording stopped at the line), it
/// falls back to the longest lap that was completed (followed by a later lap). The positions are de-duplicated,
/// resampled every 1 m and lightly smoothed (±2 m) to remove sampling noise. The line closes on itself.
/// </summary>
public static class RecordedLapLine
{
    public sealed record Result(PolylineTrack Line, int Lap, double? LapTime, bool FromLapEvent, int Samples);

    public static Result FromTelemetryCsv(string path)
    {
        var rows = File.ReadLines(path).Where(l => l.Length > 0 && l[0] != '#').ToList();
        if (rows.Count < 3) throw new InvalidDataException("No telemetry rows.");
        var header = rows[0].Split(',');
        int Col(string name)
        {
            int i = Array.IndexOf(header, name);
            if (i < 0) throw new InvalidDataException($"Column '{name}' missing: not a telemetry export of this game.");
            return i;
        }
        int ix = Col("x_m"), iy = Col("y_m"), ilap = Col("lap"), iev = Col("event");
        var data = new List<(int Lap, double X, double Y)>();
        var lapTimes = new Dictionary<int, double>();
        foreach (var row in rows.Skip(1))
        {
            var f = row.Split(',');
            if (f.Length <= Math.Max(Math.Max(ix, iy), Math.Max(ilap, iev))) continue;
            int lap = (int)double.Parse(f[ilap], CultureInfo.InvariantCulture);
            data.Add((lap, double.Parse(f[ix], CultureInfo.InvariantCulture), double.Parse(f[iy], CultureInfo.InvariantCulture)));
            // Events are ';'-joined (commas are replaced on export): "lap:N:T:valid".
            foreach (var e in f[iev].Split(';'))
            {
                var parts = e.Split(':');
                if (parts.Length == 4 && parts[0] == "lap" && parts[3] == "valid"
                    && int.TryParse(parts[1], out int lapNo) && double.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out double t))
                    lapTimes[lapNo] = t;
            }
        }

        int best;
        double? time = null;
        bool fromEvent = lapTimes.Count > 0 && lapTimes.Keys.Any(k => data.Count(d => d.Lap == k) > 10);
        if (fromEvent)
        {
            var pick = lapTimes.Where(kv => data.Count(d => d.Lap == kv.Key) > 10).OrderBy(kv => kv.Value).First();
            best = pick.Key;
            time = pick.Value;
        }
        else
        {
            int maxLap = data.Max(d => d.Lap);
            var completed = data.Select(d => d.Lap).Where(k => k > 0 && k < maxLap).Distinct().ToList();
            if (completed.Count == 0) completed = data.Select(d => d.Lap).Where(k => k > 0).Distinct().ToList();
            if (completed.Count == 0) throw new InvalidDataException("No lap in the recording (lap column is 0 throughout).");
            best = completed.OrderByDescending(k => data.Count(d => d.Lap == k)).First();
        }

        var pts = data.Where(d => d.Lap == best).Select(d => (d.X, d.Y)).ToList();
        var clean = new List<(double X, double Y)>();
        foreach (var p in pts)
            if (clean.Count == 0 || Math.Abs(p.X - clean[^1].X) + Math.Abs(p.Y - clean[^1].Y) > 0.3) clean.Add(p);
        if (clean.Count < 20) throw new InvalidDataException($"Lap {best} has too few distinct positions ({clean.Count}).");

        // Resample every 1 m along the polyline (closed back to the first point), then a ±2 m moving average.
        var cum = new double[clean.Count + 1];
        for (int i = 0; i < clean.Count; i++)
        {
            var a = clean[i]; var b = clean[(i + 1) % clean.Count];
            cum[i + 1] = cum[i] + Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
        }
        int n = Math.Max(20, (int)Math.Round(cum[^1]));
        var rx = new double[n]; var ry = new double[n];
        int seg = 0;
        for (int k = 0; k < n; k++)
        {
            double sk = cum[^1] * k / n;
            while (seg < clean.Count - 1 && cum[seg + 1] < sk) seg++;
            double len = cum[seg + 1] - cum[seg];
            double t = len > 0 ? (sk - cum[seg]) / len : 0;
            var a = clean[seg]; var b = clean[(seg + 1) % clean.Count];
            rx[k] = a.X + t * (b.X - a.X);
            ry[k] = a.Y + t * (b.Y - a.Y);
        }
        var sx = new double[n]; var sy = new double[n];
        for (int k = 0; k < n; k++)
        {
            double ax = 0, ay = 0;
            for (int o = -2; o <= 2; o++) { ax += rx[((k + o) % n + n) % n]; ay += ry[((k + o) % n + n) % n]; }
            sx[k] = ax / 5; sy[k] = ay / 5;
        }
        return new Result(new PolylineTrack($"recorded lap {best}", sx, sy), best, time, fromEvent, pts.Count);
    }
}
