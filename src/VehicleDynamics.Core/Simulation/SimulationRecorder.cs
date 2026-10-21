using System.Globalization;
using System.Text;

namespace VehicleDynamics.Core.Simulation;

/// <summary>Track context supplied by the game layer for telemetry (optional).</summary>
public struct TrackSample
{
    public double S;
    public double Lateral;
    public Surface Surface;
    public int Lap;
    public double LapTime;
    public double CollisionSeverity;
}

/// <summary>One telemetry row (DESIGN_CONTRACT §6).</summary>
public struct TelemetrySample
{
    public double T, X, Y, Psi, V, VKmh, Vx, Vy, YawRate, Beta, Ax, Ay;
    public int Gear;
    public double EngineRpm, Throttle, Brake, Steer;
    public double FTractive, FTractiveEff, FxMax, FDrag, FRolling, FGrade, FBrake, GripUsage, SlipIndicator;
    public double TrackS, TrackLateral;
    public Surface Surface;
    public int Lap;
    public double LapTime, CollisionSeverity;
    public string Event;
    // Appended columns (E4 dynamic single-track, E5 launch clutch)
    public double AlphaFront, AlphaRear, FyFront, FyRear, FzFront, FzRear, ClutchSlipRpm;
    public bool ClutchLocked;
    /// <summary>Odometer [m] (kept for comparisons; not a §6 column).</summary>
    public double Distance;
}

public sealed class RecordedRun
{
    public string Name { get; init; } = "";
    public List<KeyValuePair<string, string>> Metadata { get; } = new();
    public List<TelemetrySample> Samples { get; } = new();
}

public readonly record struct ComparisonResult(int Count, double MaxAbs, double Rmse, double MaxRel);

public static class RunComparison
{
    /// <summary>Compares b against a at a's sample times (b linearly interpolated in time; samples outside b's range skipped).</summary>
    public static ComparisonResult Compare(RecordedRun a, RecordedRun b, Func<TelemetrySample, double> sel)
    {
        var bt = b.Samples.Select(s => s.T).ToArray();
        var bv = b.Samples.Select(sel).ToArray();
        int n = 0;
        double maxAbs = 0, sumSq = 0, maxRel = 0;
        foreach (var s in a.Samples)
        {
            if (bt.Length == 0 || s.T < bt[0] || s.T > bt[^1]) continue;
            int k = Array.BinarySearch(bt, s.T);
            double vb;
            if (k >= 0) vb = bv[k];
            else
            {
                k = ~k;
                double t = (s.T - bt[k - 1]) / (bt[k] - bt[k - 1]);
                vb = (1 - t) * bv[k - 1] + t * bv[k];
            }
            double va = sel(s);
            double e = Math.Abs(va - vb);
            maxAbs = Math.Max(maxAbs, e);
            sumSq += e * e;
            double den = Math.Max(Math.Abs(va), Math.Abs(vb));
            if (den > 1e-9) maxRel = Math.Max(maxRel, e / den);
            n++;
        }
        return new ComparisonResult(n, maxAbs, n > 0 ? Math.Sqrt(sumSq / n) : 0, maxRel);
    }
}

/// <summary>
/// Records telemetry at a fixed simulation interval (default 0.02 s = every 4 steps at dt = 0.005 s), never at
/// render rate. Events and the maximum collision severity between samples are carried into the next sample.
/// </summary>
public sealed class SimulationRecorder
{
    public static readonly string[] Columns =
    {
        "t_s", "x_m", "y_m", "psi_rad", "v_mps", "v_kmh", "vx_mps", "vy_mps", "yaw_rate_rps", "beta_rad", "ax_mps2",
        "ay_mps2", "gear", "engine_rpm", "throttle", "brake", "steer_rad", "f_tractive_n", "f_tractive_eff_n",
        "f_x_max_n", "f_drag_n", "f_rolling_n", "f_grade_n", "f_brake_n", "grip_usage", "slip_indicator",
        "track_s_m", "track_lateral_m", "surface", "lap", "lap_time_s", "collision_severity_mps", "event",
        "alpha_f_rad", "alpha_r_rad", "f_yf_n", "f_yr_n", "f_zf_n", "f_zr_n", "clutch_locked", "clutch_slip_rpm",
    };

    private readonly int _stepsPerSample;
    private long _step;
    private readonly List<string> _pendingEvents = new();
    private double _pendingSeverity;

    public double Dt { get; }
    public double SampleInterval { get; }
    public RecordedRun Run { get; private set; }

    public SimulationRecorder(double dt, double sampleInterval = 0.02, string runName = "run")
    {
        if (!(dt > 0) || !(sampleInterval > 0)) throw new ArgumentOutOfRangeException(nameof(sampleInterval));
        Dt = dt;
        SampleInterval = sampleInterval;
        _stepsPerSample = Math.Max(1, (int)Math.Round(sampleInterval / dt));
        Run = new RecordedRun { Name = runName };
    }

    public int StepsPerSample => _stepsPerSample;

    public void Reset(string? runName = null)
    {
        Run = new RecordedRun { Name = runName ?? Run.Name };
        _step = 0;
        _pendingEvents.Clear();
        _pendingSeverity = 0;
    }

    public void AddMetadata(string key, string value) => Run.Metadata.Add(new(key, value));

    public void AddParameters(VehicleParameters p)
    {
        foreach (var kv in ParameterKeyValues(p)) AddMetadata(kv.Key, kv.Value);
    }

    public void AddEvent(string evt)
    {
        if (!string.IsNullOrEmpty(evt)) _pendingEvents.Add(evt);
    }

    /// <summary>Records the initial state (step 0).</summary>
    public void RecordInitial(in VehicleState s, in VehicleOutputs o, in VehicleInputs u, TrackSample? track = null)
    {
        _step = 0;
        Append(s, o, u, track);
    }

    /// <summary>Call once after every physics step; returns true when a row was written.</summary>
    public bool OnStep(in VehicleState s, in VehicleOutputs o, in VehicleInputs u, TrackSample? track = null)
    {
        _step++;
        if (track is { } tr) _pendingSeverity = Math.Max(_pendingSeverity, tr.CollisionSeverity);
        if (_step % _stepsPerSample != 0) return false;
        Append(s, o, u, track);
        return true;
    }

    private void Append(in VehicleState s, in VehicleOutputs o, in VehicleInputs u, TrackSample? track)
    {
        var row = MakeSample(s, o, u, track, string.Join(";", _pendingEvents));
        row.CollisionSeverity = Math.Max(_pendingSeverity, row.CollisionSeverity);
        Run.Samples.Add(row);
        _pendingEvents.Clear();
        _pendingSeverity = 0;
    }

    /// <summary>Builds one row. Without track context, track_s_m is the odometer and lateral is 0.</summary>
    public static TelemetrySample MakeSample(in VehicleState s, in VehicleOutputs o, in VehicleInputs u, TrackSample? track = null, string evt = "")
    {
        var t = track ?? new TrackSample { S = s.Distance, Surface = u.Surface };
        return new TelemetrySample
        {
            T = s.Time, X = s.X, Y = s.Y, Psi = s.Psi, V = s.V, VKmh = s.V * 3.6, Vx = o.Vx, Vy = o.Vy,
            YawRate = o.YawRate, Beta = o.Beta, Ax = o.Ax, Ay = o.Ay, Gear = s.Gear, EngineRpm = s.EngineRpm,
            Throttle = u.Throttle, Brake = u.Brake, Steer = o.SteerAngle, FTractive = o.FTractive,
            FTractiveEff = o.FTractiveEff, FxMax = o.FxMax, FDrag = o.FDrag, FRolling = o.FRolling, FGrade = o.FGrade,
            FBrake = o.FBrake, GripUsage = o.GripUsage, SlipIndicator = o.SlipIndicator, TrackS = t.S,
            TrackLateral = t.Lateral, Surface = t.Surface, Lap = t.Lap, LapTime = t.LapTime,
            CollisionSeverity = t.CollisionSeverity, Event = evt, Distance = s.Distance,
            AlphaFront = o.AlphaFront, AlphaRear = o.AlphaRear, FyFront = o.FyFront, FyRear = o.FyRear,
            FzFront = o.FzFront, FzRear = o.FzRear, ClutchLocked = o.ClutchLocked, ClutchSlipRpm = o.ClutchSlipRpm,
        };
    }

    public void WriteCsv(string path)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path))!);
        using var w = new StreamWriter(path, false, new UTF8Encoding(false)) { NewLine = "\n" };
        WriteCsv(w, Run);
    }

    public static void WriteCsv(TextWriter w, RecordedRun run)
    {
        w.Write($"# run_id={run.Name}\n");
        foreach (var kv in run.Metadata) w.Write($"# {kv.Key}={kv.Value}\n");
        w.Write(string.Join(",", Columns));
        w.Write('\n');
        var sb = new StringBuilder();
        foreach (var s in run.Samples)
        {
            sb.Clear();
            void F(double v) { sb.Append(v.ToString("R", CultureInfo.InvariantCulture)).Append(','); }
            F(s.T); F(s.X); F(s.Y); F(s.Psi); F(s.V); F(s.VKmh); F(s.Vx); F(s.Vy); F(s.YawRate); F(s.Beta);
            F(s.Ax); F(s.Ay); sb.Append(s.Gear).Append(','); F(s.EngineRpm); F(s.Throttle); F(s.Brake); F(s.Steer);
            F(s.FTractive); F(s.FTractiveEff); F(s.FxMax); F(s.FDrag); F(s.FRolling); F(s.FGrade); F(s.FBrake);
            F(s.GripUsage); F(s.SlipIndicator); F(s.TrackS); F(s.TrackLateral);
            sb.Append(s.Surface.ToString().ToLowerInvariant()).Append(',');
            sb.Append(s.Lap).Append(','); F(s.LapTime); F(s.CollisionSeverity);
            sb.Append((s.Event ?? "").Replace(',', ';')).Append(',');
            F(s.AlphaFront); F(s.AlphaRear); F(s.FyFront); F(s.FyRear); F(s.FzFront); F(s.FzRear);
            sb.Append(s.ClutchLocked ? 1 : 0).Append(',');
            sb.Append(s.ClutchSlipRpm.ToString("R", CultureInfo.InvariantCulture));
            sb.Append('\n');
            w.Write(sb.ToString());
        }
    }

    /// <summary>Flat key=value list of every public parameter (arrays as space-separated lists in brackets).</summary>
    public static List<KeyValuePair<string, string>> ParameterKeyValues(VehicleParameters p)
    {
        var r = new List<KeyValuePair<string, string>>();
        foreach (var prop in typeof(VehicleParameters).GetProperties())
        {
            if (!prop.CanRead || prop.GetIndexParameters().Length > 0) continue;
            object? v = prop.GetValue(p);
            string s = v switch
            {
                double d => d.ToString("R", CultureInfo.InvariantCulture),
                double[] a => "[" + string.Join(" ", a.Select(x => x.ToString("R", CultureInfo.InvariantCulture))) + "]",
                IFormattable f => f.ToString(null, CultureInfo.InvariantCulture),
                null => "",
                _ => v.ToString() ?? "",
            };
            r.Add(new(prop.Name, s));
        }
        return r;
    }
}
