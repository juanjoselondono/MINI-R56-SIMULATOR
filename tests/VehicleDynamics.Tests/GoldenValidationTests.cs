using System.Text.Json;
using VehicleDynamics.Core.Reference;
using VehicleDynamics.Core.Simulation;
using Xunit.Abstractions;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace VehicleDynamics.Tests;

/// <summary>
/// C# vs the committed MATLAB golden data (validation/golden/&lt;id&gt;), so `dotnet test` validates without MATLAB.
/// Tolerances come from validation/matlab/compare_config.json (the same file compare_results.m uses).
/// The golden folder is required: a missing scenario fails instead of being skipped.
/// </summary>
public class GoldenValidationTests
{
    private readonly ITestOutputHelper _out;
    public GoldenValidationTests(ITestOutputHelper output) => _out = output;

    private static string Root => TestData.RepoRoot;

    public static IEnumerable<object[]> ScenarioIds()
    {
        using var doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(Root, "validation", "scenarios.json")));
        foreach (var s in doc.RootElement.GetProperty("scenarios").EnumerateArray())
            yield return new object[] { s.GetProperty("id").GetString()! };
    }

    private static ReferenceParameters ParamsFor(string id)
    {
        using var doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(Root, "validation", "scenarios.json")));
        var s = doc.RootElement.GetProperty("scenarios").EnumerateArray().Single(e => e.GetProperty("id").GetString() == id);
        return ReferenceParameters.Default.WithOverrides(
            s.GetProperty("overrides").EnumerateObject().Select(o => new KeyValuePair<string, double>(o.Name, o.Value.GetDouble())));
    }

    private static JsonElement Config(string section)
    {
        var doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(Root, "validation", "matlab", "compare_config.json")));
        return doc.RootElement.GetProperty(section).Clone();
    }

    private static string GoldenDir(string id)
    {
        string d = Path.Combine(Root, "validation", "golden", id);
        Assert.True(File.Exists(Path.Combine(d, "summary.json")), $"golden data missing for {id}: run validation/matlab/run_reference.m");
        return d;
    }

    [Theory]
    [MemberData(nameof(ScenarioIds))]
    public void ReferenceLayer_MatchesGolden(string id)
    {
        var cfg = Config("reference");
        double rel = cfg.GetProperty("rel_tol").GetDouble(), abs = cfg.GetProperty("abs_tol").GetDouble();
        string g = GoldenDir(id);
        var p = ParamsFor(id);
        var t = ReferenceModel.Build(p);
        var o = ReferenceOval.Run(p, t);

        int n = 0, fails = 0;
        double worstAbs = 0, worstRel = 0;
        string worst = "", firstFail = "";
        void Col(string file, string col, IReadOnlyList<double> mine)
        {
            var gold = TestData.ReadCsv(Path.Combine(g, file))[col];
            Assert.True(gold.Length == mine.Count, $"{id} {file}:{col} length golden {gold.Length} vs C# {mine.Count}");
            for (int i = 0; i < gold.Length; i++)
            {
                double e = Math.Abs(mine[i] - gold[i]);
                n++;
                if (e > worstAbs) { worstAbs = e; worst = $"{file}:{col}[{i}]"; }
                if (gold[i] != 0) worstRel = Math.Max(worstRel, e / Math.Abs(gold[i]));
                if (!(e <= abs + rel * Math.Abs(gold[i]))) { fails++; if (firstFail == "") firstFail = $"{file}:{col}[{i}] {mine[i]:R} vs {gold[i]:R}"; }
            }
        }

        for (int k = 0; k < t.NumGears; k++)
        {
            string f = $"tables_gear{k + 1}.csv";
            Col(f, "rpm", t.RpmAxis); Col(f, "v_kmh", t.VGearKmh[k]); Col(f, "t_engine_nm", t.TEngine);
            Col(f, "t_wheel_nm", t.TWheelNm[k]); Col(f, "f_tractive_n", t.FTractiveN[k]); Col(f, "a_x_ms2", t.AxMs2[k]);
            Col(f, "f_tractive_eff_n", t.FTractiveEffective[k]); Col(f, "a_x_real", t.AxReal[k]);
        }
        Col("resist.csv", "v_kmh", t.VKmh); Col("resist.csv", "f_drag_n", t.FDrag); Col("resist.csv", "f_rolling_n", t.FRolling);
        Col("resist.csv", "f_pend_n", t.FPend); Col("resist.csv", "f_resistive_n", t.FResistive);
        Col("resist.csv", "p_demanded_kw", t.PDemandedKW); Col("resist.csv", "a_x_envelope", t.AxEnvelope);
        foreach (var (name, r) in new[] { ("r1", o.R1), ("r2", o.R2) })
        {
            Col($"{name}_history.csv", "t_s", r.HistoryT); Col($"{name}_history.csv", "d_m", r.HistoryD);
            Col($"{name}_history.csv", "v_kmh", r.HistoryVKmh);
        }
        Col("engine_torque.csv", "rpm", t.RpmAxis); Col("engine_torque.csv", "t_engine_nm", t.TEngine);
        var probeRpm = TestData.ReadCsv(Path.Combine(g, "pchip_probe.csv"))["rpm"];
        var expectedRpm = MatlabColon.Range(1000, 7, 6500).Append(6500).Distinct().OrderBy(x => x).ToArray();
        Col("pchip_probe.csv", "rpm", expectedRpm);
        var pc = new Pchip(p.RpmRaw, p.TorqueRaw);
        Col("pchip_probe.csv", "t_engine_nm", probeRpm.Select(pc.Evaluate).ToArray());

        var gs = TestData.ReadSummary(Path.Combine(g, "summary.json"));
        var mineS = ReferenceLayerTests.SummaryValues(t, o);
        foreach (var key in mineS.Keys) Assert.True(gs.ContainsKey(key), $"{id}: golden summary lacks {key}");
        foreach (var (key, gv) in gs)
        {
            Assert.True(mineS.ContainsKey(key), $"{id}: C# summary lacks golden key {key}");
            double e = Math.Abs(mineS[key] - gv);
            n++;
            if (e > worstAbs) { worstAbs = e; worst = "summary:" + key; }
            if (gv != 0) worstRel = Math.Max(worstRel, e / Math.Abs(gv));
            if (!(e <= abs + rel * Math.Abs(gv))) { fails++; if (firstFail == "") firstFail = $"summary:{key} {mineS[key]:R} vs {gv:R}"; }
        }

        _out.WriteLine($"{id}: {n} values, max abs err {worstAbs:E3} ({worst}), max rel err {worstRel:E3}, fails {fails}");
        Assert.True(fails == 0, $"{id}: {fails} values outside |e| <= {abs} + {rel}|g|; first: {firstFail}");
    }

    /// <summary>
    /// Interactive layer (VehicleDynamics.Core.Simulation, clutch launch rule off) under R1 conditions vs MATLAB R1.
    /// Tolerance: speed from compare_config.json (justified in docs/validation_report.md), distance = v_tol/3.6 * t
    /// (integral of the speed bound), same number of steps and identical step count to reach L_recta / the 60 s cap.
    /// </summary>
    [Theory]
    [MemberData(nameof(ScenarioIds))]
    public void InteractiveLayer_R1_WithinJustifiedTolerance(string id)
    {
        var cfg = Config("interactive");
        double vTol = cfg.GetProperty("v_kmh_abs_tol").GetDouble();
        var h = TestData.ReadCsv(Path.Combine(GoldenDir(id), "r1_history.csv"));
        var run = RunInteractiveR1(ParamsFor(id));

        Assert.Equal(h["t_s"].Length, run.T.Count);
        double maxDv = 0, maxDd = 0, maxDt = 0, sq = 0;
        for (int i = 0; i < run.T.Count; i++)
        {
            double dv = Math.Abs(run.VKmh[i] - h["v_kmh"][i]);
            double dd = Math.Abs(run.D[i] - h["d_m"][i]);
            maxDv = Math.Max(maxDv, dv); maxDd = Math.Max(maxDd, dd); sq += dv * dv;
            maxDt = Math.Max(maxDt, Math.Abs(run.T[i] - h["t_s"][i]));
            Assert.True(dv <= vTol, $"{id} step {i}: |dv| = {dv} km/h > {vTol}");
            Assert.True(dd <= vTol / 3.6 * h["t_s"][i] + 1e-9, $"{id} step {i}: |dd| = {dd} m");
        }
        _out.WriteLine($"{id}: n={run.T.Count} max|dv|={maxDv:E3} km/h RMSE={Math.Sqrt(sq / run.T.Count):E3} km/h max|dd|={maxDd:E3} m max|dt|={maxDt:E1} s");
        Assert.True(maxDt < 1e-9);
    }

    internal sealed record R1Run(List<double> T, List<double> D, List<double> VKmh);

    /// <summary>R1 conditions as in tools/ValidationRunner (FILE_CONTRACT.md, interactive_r1.csv): rows = states before each step.</summary>
    internal static R1Run RunInteractiveR1(ReferenceParameters rp, bool clutch = false)
    {
        var t = ReferenceModel.Build(rp);
        var vp = VehicleParameters.FromReference(rp);
        vp.ClutchLaunch = clutch;
        vp.LaunchModel = LaunchModel.SeedRule; // this study quantifies the seed clutch rule
        var dyn = new Dyn(vp);
        var s = VehicleState.AtRest();
        s.V = t.VGearKmh[0].Min() / 3.6;
        s.Gear = 1;
        dyn.State = s;
        var u = new VehicleInputs { Throttle = 1, AutoShift = true };
        var run = new R1Run(new(), new(), new());
        while (dyn.State.Distance < rp.LRecta && dyn.State.Time < rp.TimeCap)
        {
            run.T.Add(dyn.State.Time); run.D.Add(dyn.State.Distance); run.VKmh.Add(dyn.State.V * 3.6);
            dyn.Step(u, vp.Dt);
        }
        return run;
    }
}
