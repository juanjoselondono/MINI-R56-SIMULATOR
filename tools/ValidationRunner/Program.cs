using System.Globalization;
using System.Text;
using System.Text.Json;
using VehicleDynamics.Core.Reference;
using VehicleDynamics.Core.Simulation;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

// C# side of the validation (DESIGN_CONTRACT §7, validation/FILE_CONTRACT.md).
// Usage: ValidationRunner [repoRoot]
var root = args.Length > 0 ? Path.GetFullPath(args[0]) : FindRepoRoot();
var scenariosPath = Path.Combine(root, "validation", "scenarios.json");
var outRoot = Path.Combine(root, "validation", "out", "csharp");
var goldenRoot = Path.Combine(root, "validation", "golden");
Directory.CreateDirectory(outRoot);
var inv = CultureInfo.InvariantCulture;
string F(double v) => v.ToString("R", inv);

using var doc = JsonDocument.Parse(File.ReadAllText(scenariosPath));
var goldenRows = new List<string> { "id,file,column,n,max_abs_err,max_rel_err" };
var interRows = new List<string> { "id,n,max_abs_dv_kmh,rmse_dv_kmh,max_abs_dd_m,t_reach_matlab_s,t_reach_interactive_s" };
string gitSha = GitSha(root);
var srcRows = new List<string> { "id,n,max_abs_da_total,max_abs_da_torque,max_abs_da_residual,int_abs_da_total_kmh,int_abs_da_torque_kmh,int_abs_da_residual_kmh,min_engine_rpm,max_engine_rpm,frac_traction_limited,max_abs_dv_fuelcut7000_kmh,clutch_max_abs_dv_kmh,clutch_t_reach_s,t_reach_matlab_s" };

foreach (var sc in doc.RootElement.GetProperty("scenarios").EnumerateArray())
{
    string id = sc.GetProperty("id").GetString()!;
    var overrides = new List<KeyValuePair<string, double>>();
    foreach (var o in sc.GetProperty("overrides").EnumerateObject()) overrides.Add(new(o.Name, o.Value.GetDouble()));
    var p = ReferenceParameters.Default.WithOverrides(overrides);
    var t = ReferenceModel.Build(p);
    var oval = ReferenceOval.Run(p, t);
    var dir = Path.Combine(outRoot, id);
    Directory.CreateDirectory(dir);

    for (int g = 0; g < t.NumGears; g++)
    {
        var rows = Enumerable.Range(0, t.RpmAxis.Length).Select(i => new[]
        {
            t.RpmAxis[i], t.VGearKmh[g][i], t.TEngine[i], t.TWheelNm[g][i], t.FTractiveN[g][i], t.AxMs2[g][i],
            t.FTractiveEffective[g][i], t.AxReal[g][i],
        });
        WriteCsv(Path.Combine(dir, $"tables_gear{g + 1}.csv"),
            "rpm,v_kmh,t_engine_nm,t_wheel_nm,f_tractive_n,a_x_ms2,f_tractive_eff_n,a_x_real", rows);
    }
    WriteCsv(Path.Combine(dir, "resist.csv"), "v_kmh,f_drag_n,f_rolling_n,f_pend_n,f_resistive_n,p_demanded_kw,a_x_envelope",
        Enumerable.Range(0, t.VKmh.Length).Select(i => new[]
        {
            t.VKmh[i], t.FDrag[i], t.FRolling[i], t.FPend[i], t.FResistive[i], t.PDemandedKW[i], t.AxEnvelope[i],
        }));
    WriteCsv(Path.Combine(dir, "r1_history.csv"), "t_s,d_m,v_kmh",
        Enumerable.Range(0, oval.R1.HistoryT.Length).Select(i => new[] { oval.R1.HistoryT[i], oval.R1.HistoryD[i], oval.R1.HistoryVKmh[i] }));
    WriteCsv(Path.Combine(dir, "r2_history.csv"), "t_s,d_m,v_kmh",
        Enumerable.Range(0, oval.R2.HistoryT.Length).Select(i => new[] { oval.R2.HistoryT[i], oval.R2.HistoryD[i], oval.R2.HistoryVKmh[i] }));
    WriteCsv(Path.Combine(dir, "engine_torque.csv"), "rpm,t_engine_nm",
        Enumerable.Range(0, t.RpmAxis.Length).Select(i => new[] { t.RpmAxis[i], t.TEngine[i] }));
    var probe = MatlabColon.Range(1000, 7, 6500).Append(6500).Distinct().OrderBy(x => x).ToArray();
    var pc = new Pchip(p.RpmRaw, p.TorqueRaw);
    WriteCsv(Path.Combine(dir, "pchip_probe.csv"), "rpm,t_engine_nm", probe.Select(r => new[] { r, pc.Evaluate(r) }));
    WriteSummary(Path.Combine(dir, "summary.json"), id, t, oval);

    // Interactive layer under R1 conditions
    var (runOff, tReachOff) = InteractiveR1(p, t, clutch: false, id);
    WriteRun(Path.Combine(dir, "interactive_r1.csv"), runOff);
    var (runOn, _) = InteractiveR1(p, t, clutch: true, id);
    WriteRun(Path.Combine(dir, "interactive_r1_clutch.csv"), runOn);

    // Interactive vs MATLAB (golden when present, else the C# reference port which equals it)
    string gR1 = Path.Combine(goldenRoot, id, "r1_history.csv");
    var mat = File.Exists(gR1) ? ReadCsv(gR1) : new Dictionary<string, double[]>
    {
        ["t_s"] = oval.R1.HistoryT, ["d_m"] = oval.R1.HistoryD, ["v_kmh"] = oval.R1.HistoryVKmh,
    };
    int n = Math.Min(mat["t_s"].Length, runOff.Samples.Count);
    double maxDv = 0, sq = 0, maxDd = 0;
    for (int i = 0; i < n; i++)
    {
        double dv = Math.Abs(runOff.Samples[i].VKmh - mat["v_kmh"][i]);
        maxDv = Math.Max(maxDv, dv);
        sq += dv * dv;
        maxDd = Math.Max(maxDd, Math.Abs(runOff.Samples[i].Distance - mat["d_m"][i]));
    }
    double tReachMat = mat["t_s"][^1] + p.Dt;
    srcRows.Add(ErrorSources(id, p, t, mat, runOn, tReachMat));
    interRows.Add(string.Join(",", id, mat["t_s"].Length.ToString(inv), F(maxDv), F(Math.Sqrt(sq / Math.Max(1, n))), F(maxDd), F(tReachMat), F(tReachOff)));

    string gDir = Path.Combine(goldenRoot, id);
    string goldenNote = "golden: absent";
    if (Directory.Exists(gDir))
    {
        double worst = 0;
        foreach (var gf in Directory.GetFiles(gDir, "*.csv"))
        {
            string name = Path.GetFileName(gf);
            string cf = Path.Combine(dir, name);
            if (!File.Exists(cf)) continue;
            var a = ReadCsv(gf);
            var b = ReadCsv(cf);
            foreach (var col in a.Keys)
            {
                if (!b.ContainsKey(col)) { goldenRows.Add($"{id},{name},{col},MISSING,,"); worst = double.PositiveInfinity; continue; }
                var (cnt, mAbs, mRel) = Errors(a[col], b[col]);
                goldenRows.Add(string.Join(",", id, name, col, cnt.ToString(inv), F(mAbs), F(mRel)));
                worst = Math.Max(worst, mRel);
            }
        }
        string gs = Path.Combine(gDir, "summary.json");
        if (File.Exists(gs))
        {
            var a = ReadSummary(gs);
            var b = ReadSummary(Path.Combine(dir, "summary.json"));
            foreach (var kv in a)
            {
                if (!b.TryGetValue(kv.Key, out var bv)) continue;
                var (cnt, mAbs, mRel) = Errors(new[] { kv.Value }, new[] { bv });
                goldenRows.Add(string.Join(",", id, "summary.json", kv.Key, cnt.ToString(inv), F(mAbs), F(mRel)));
                worst = Math.Max(worst, mRel);
            }
        }
        goldenNote = $"golden max rel err {worst:E3}";
    }
    Console.WriteLine($"{id}: F_x_max={F(t.FxMax)} v_curva_final_kmh={F(oval.VCurvaFinalKmh)} t_vuelta={F(oval.TVueltaCompleta)} | " +
                      $"{goldenNote} | interactive R1 max|dv|={maxDv:E3} km/h, t_reach {tReachOff:F3} s vs {tReachMat:F3} s");
}

File.WriteAllLines(Path.Combine(outRoot, "interactive_vs_matlab.csv"), interRows);
File.WriteAllLines(Path.Combine(outRoot, "interactive_error_sources.csv"), srcRows);
if (goldenRows.Count > 1) File.WriteAllLines(Path.Combine(outRoot, "golden_comparison.csv"), goldenRows);

var conv = ConvergenceStudy.Run(10);
var convLines = new List<string> { "case,integrator,dt,v_end_mps,d_end_m,err_v_mps,err_d_m" };
convLines.AddRange(conv.Select(r => string.Join(",", r.Case, r.Integrator, F(r.Dt), F(r.VEnd), F(r.DEnd), F(r.ErrV), F(r.ErrD))));
File.WriteAllLines(Path.Combine(outRoot, "convergence_study.csv"), convLines);
Console.WriteLine($"Wrote {outRoot}");
return 0;

(RecordedRun Run, double TReach) InteractiveR1(ReferenceParameters rp, ReferenceTables rt, bool clutch, string id)
{
    var vp = VehicleParameters.FromReference(rp);
    vp.ClutchLaunch = clutch;
    vp.LaunchModel = LaunchModel.SeedRule; // this study quantifies the seed clutch rule
    var dyn = new Dyn(vp);
    var s = VehicleState.AtRest();
    s.V = rt.VGearKmh[0].Min() / 3.6;
    s.Gear = 1;
    s.EngineRpm = rp.RpmAxisStart;
    dyn.State = s;
    var u = new VehicleInputs { Throttle = 1, AutoShift = true };
    var run = new RecordedRun { Name = $"{id}_interactive_r1{(clutch ? "_clutch" : "")}" };
    run.Metadata.Add(new("scenario", id));
    run.Metadata.Add(new("layer", "interactive (VehicleDynamics.Core.Simulation)"));
    run.Metadata.Add(new("conditions", "R1: throttle=1 brake=0 steer=0 autoshift=on start=1st gear at 1000 rpm; stop when d>=L_recta or t>=60"));
    run.Metadata.Add(new("L_recta", F(rp.LRecta)));
    run.Metadata.Add(new("sample_interval_s", F(rp.Dt)));
    run.Metadata.Add(new("integrator", dyn.Integrator.ToString()));
    run.Metadata.Add(new("git_sha", gitSha));
    run.Metadata.AddRange(SimulationRecorder.ParameterKeyValues(vp));
    double tReach = double.NaN;
    while (dyn.State.Distance < rp.LRecta && dyn.State.Time < rp.TimeCap)
    {
        var before = dyn.State;
        dyn.Step(u, vp.Dt);
        var row = SimulationRecorder.MakeSample(before, dyn.Outputs, u);
        row.Gear = dyn.State.Gear;
        row.EngineRpm = dyn.Outputs.EngineRpm;
        run.Samples.Add(row);
        tReach = dyn.State.Time;
    }
    return (run, tReach);
}

// Splits the interactive-vs-MATLAB acceleration gap along the MATLAB R1 trajectory into
// (a) torque: direct PCHIP vs linear interpolation of the 50-rpm torque table (same traction cap), and
// (b) residual: linear interpolation of the quadratic resistances and across the traction-limit kink.
// Also checks the fuel cut (6500 vs 7000 rpm) and the seed clutch rule effect.
string ErrorSources(string id, ReferenceParameters rp, ReferenceTables rt, Dictionary<string, double[]> mat, RecordedRun clutchRun, double tReachMat)
{
    var vp = VehicleParameters.FromReference(rp);
    vp.ClutchLaunch = false;
    var dyn = new Dyn(vp);
    var u = new VehicleInputs { Throttle = 1, AutoShift = true };
    double mT = 0, mTq = 0, mR = 0, iT = 0, iTq = 0, iR = 0, rMin = double.MaxValue, rMax = 0;
    int lim = 0, n = mat["v_kmh"].Length;
    foreach (var vk in mat["v_kmh"])
    {
        double v = vk / 3.6;
        int g = dyn.AutoGear(v);
        double aM = Math.Max(0, MatlabInterp.LinearExtrap(rt.VGearKmh[g - 1], rt.AxReal[g - 1], vk));
        var st = VehicleState.AtRest(); st.Gear = g; st.V = v;
        var o = dyn.Evaluate(st, u, v);
        double meq = vp.Mass * (1 + vp.MassFactors[g - 1]);
        double tLin = MatlabInterp.LinearExtrap(rt.VGearKmh[g - 1], rt.TEngine, vk);
        double fLin = Math.Min(tLin * dyn.TotalRatio(g) * vp.EtaT / vp.WheelRadius, rt.FxMax);
        if (o.FTractiveEff >= rt.FxMax) lim++;
        double daTq = (o.FTractiveEff - fLin) / meq, daT = o.Ax - aM, daR = daT - daTq;
        mT = Math.Max(mT, Math.Abs(daT)); mTq = Math.Max(mTq, Math.Abs(daTq)); mR = Math.Max(mR, Math.Abs(daR));
        iT += Math.Abs(daT) * rp.Dt; iTq += Math.Abs(daTq) * rp.Dt; iR += Math.Abs(daR) * rp.Dt;
        rMin = Math.Min(rMin, o.EngineRpm); rMax = Math.Max(rMax, o.EngineRpm);
    }
    List<double> RunV(VehicleParameters q)
    {
        var d = new Dyn(q);
        var st = VehicleState.AtRest(); st.V = rt.VGearKmh[0].Min() / 3.6; st.Gear = 1; d.State = st;
        var vs = new List<double>();
        while (d.State.Distance < rp.LRecta && d.State.Time < rp.TimeCap) { vs.Add(d.State.V * 3.6); d.Step(u, q.Dt); }
        return vs;
    }
    var vq = vp.Clone(); vq.FuelCutRpm = 7000;
    var a = RunV(vp); var b = RunV(vq);
    double dFc = a.Count == b.Count ? a.Zip(b, (x, y) => Math.Abs(x - y)).Max() : double.PositiveInfinity;
    double cdv = 0;
    for (int i = 0; i < Math.Min(n, clutchRun.Samples.Count); i++) cdv = Math.Max(cdv, Math.Abs(clutchRun.Samples[i].VKmh - mat["v_kmh"][i]));
    double cReach = clutchRun.Samples[^1].T + rp.Dt;
    return string.Join(",", id, n.ToString(inv), F(mT), F(mTq), F(mR), F(iT * 3.6), F(iTq * 3.6), F(iR * 3.6), F(rMin), F(rMax),
        F((double)lim / n), F(dFc), F(cdv), F(cReach), F(tReachMat));
}

void WriteRun(string path, RecordedRun run)
{
    using var w = new StreamWriter(path, false, new UTF8Encoding(false)) { NewLine = "\n" };
    SimulationRecorder.WriteCsv(w, run);
}

void WriteCsv(string path, string header, IEnumerable<double[]> rows)
{
    var sb = new StringBuilder();
    sb.Append(header).Append('\n');
    foreach (var r in rows) sb.Append(string.Join(",", r.Select(F))).Append('\n');
    File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
}

void WriteSummary(string path, string id, ReferenceTables t, OvalResult o)
{
    using var fs = File.Create(path);
    using var w = new Utf8JsonWriter(fs, new JsonWriterOptions { Indented = true });
    w.WriteStartObject();
    w.WriteString("scenario_id", id);
    w.WriteNumber("F_x_max", t.FxMax);
    w.WriteNumber("v_curva_adherencia_kmh", o.VCurvaAdherenciaKmh);
    w.WriteNumber("optimal_gear_curva", o.OptimalGearCurva);
    w.WriteNumber("v_max_gear_motriz_kmh", o.VMaxGearMotrizKmh);
    w.WriteNumber("v_curva_final_kmh", o.VCurvaFinalKmh);
    w.WriteNumber("d_frenado_r1", o.R1.DFrenado);
    w.WriteNumber("t_frenado_r1", o.R1.TFrenado);
    w.WriteNumber("d_acc_efectiva_r1", o.R1.DAccEfectiva);
    w.WriteNumber("t_acc_r1", o.R1.TAcc);
    w.WriteNumber("v_max_recta1", o.R1.VMaxKmh);
    w.WriteNumber("t_recta1_total", o.R1.TTotal);
    w.WriteNumber("d_frenado_r2", o.R2.DFrenado);
    w.WriteNumber("t_frenado_r2", o.R2.TFrenado);
    w.WriteNumber("d_acc_efectiva_r2", o.R2.DAccEfectiva);
    w.WriteNumber("t_acc_r2", o.R2.TAcc);
    w.WriteNumber("v_max_recta2", o.R2.VMaxKmh);
    w.WriteNumber("t_recta2_total", o.R2.TTotal);
    w.WriteNumber("t_curva1_total", o.TCurva1Total);
    w.WriteNumber("t_curva2_total", o.TCurva2Total);
    w.WriteNumber("t_vuelta_completa", o.TVueltaCompleta);
    w.WriteNumber("distancia_lap", o.DistanciaLap);
    w.WriteEndObject();
}

static (int, double, double) Errors(double[] a, double[] b)
{
    if (a.Length != b.Length) return (-1, double.PositiveInfinity, double.PositiveInfinity);
    double mAbs = 0, mRel = 0;
    for (int i = 0; i < a.Length; i++)
    {
        double e = Math.Abs(a[i] - b[i]);
        mAbs = Math.Max(mAbs, e);
        mRel = Math.Max(mRel, e / Math.Max(1e-6, Math.Max(Math.Abs(a[i]), Math.Abs(b[i]))));
    }
    return (a.Length, mAbs, mRel);
}

static Dictionary<string, double[]> ReadCsv(string path)
{
    var lines = File.ReadAllLines(path).Where(l => l.Length > 0 && !l.StartsWith('#')).ToArray();
    var header = lines[0].Split(',');
    var cols = header.Select(_ => new List<double>()).ToArray();
    for (int i = 1; i < lines.Length; i++)
    {
        var parts = lines[i].Split(',');
        for (int j = 0; j < header.Length; j++)
            cols[j].Add(double.TryParse(parts[j], NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : double.NaN);
    }
    return header.Select((h, j) => (h, j)).ToDictionary(x => x.h, x => cols[x.j].ToArray());
}

static Dictionary<string, double> ReadSummary(string path)
{
    using var d = JsonDocument.Parse(File.ReadAllText(path));
    var r = new Dictionary<string, double>();
    foreach (var p in d.RootElement.EnumerateObject())
        if (p.Value.ValueKind == JsonValueKind.Number) r[p.Name] = p.Value.GetDouble();
    return r;
}

static string FindRepoRoot()
{
    var d = new DirectoryInfo(Directory.GetCurrentDirectory());
    while (d != null && !File.Exists(Path.Combine(d.FullName, "VehicleDynamics.sln"))) d = d.Parent;
    if (d != null) return d.FullName;
    d = new DirectoryInfo(AppContext.BaseDirectory);
    while (d != null && !File.Exists(Path.Combine(d.FullName, "VehicleDynamics.sln"))) d = d.Parent;
    return d?.FullName ?? throw new InvalidOperationException("Repository root (VehicleDynamics.sln) not found; pass it as argument.");
}

static string GitSha(string root)
{
    try
    {
        string head = Path.Combine(root, ".git", "HEAD");
        if (!File.Exists(head)) return "unavailable";
        string h = File.ReadAllText(head).Trim();
        if (!h.StartsWith("ref: ")) return h;
        string refPath = Path.Combine(root, ".git", h[5..]);
        return File.Exists(refPath) ? File.ReadAllText(refPath).Trim() : "unavailable";
    }
    catch (IOException) { return "unavailable"; }
}
