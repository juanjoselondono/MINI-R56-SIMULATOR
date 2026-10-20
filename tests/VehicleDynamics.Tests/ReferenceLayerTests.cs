using System.Globalization;
using System.Text.Json;
using VehicleDynamics.Core.Reference;
using Xunit.Abstractions;

namespace VehicleDynamics.Tests;

public class ReferenceLayerTests
{
    private readonly ITestOutputHelper _out;
    public ReferenceLayerTests(ITestOutputHelper output) => _out = output;

    [Fact]
    public void Pchip_ReproducesMatlabTorqueTable()
    {
        var ml = TestData.MlValues()["Tengine"];
        var p = ReferenceParameters.Default;
        var rpm = MatlabColon.Range(1000, 50, 6500);
        var t = new Pchip(p.RpmRaw, p.TorqueRaw).Evaluate(rpm);
        Assert.Equal(ml.Length, t.Length);
        double maxRel = 0;
        for (int i = 0; i < t.Length; i++) maxRel = Math.Max(maxRel, TestData.RelErr(t[i], ml[i]));
        _out.WriteLine($"T_engine max rel err vs MATLAB = {maxRel:E3}");
        Assert.True(maxRel <= 1e-12, $"max rel err {maxRel}");
    }

    [Fact]
    public void Pchip_OffGridAndExtrapolationMatchMatlab()
    {
        var v = TestData.MlValues();
        var p = ReferenceParameters.Default;
        var pc = new Pchip(p.RpmRaw, p.TorqueRaw);
        var x = v["pchipx"];
        var y = v["pchipy"];
        double maxRel = 0;
        for (int i = 0; i < x.Length; i++) maxRel = Math.Max(maxRel, TestData.RelErr(pc.Evaluate(x[i]), y[i]));
        maxRel = Math.Max(maxRel, TestData.RelErr(pc.Evaluate(900), v["pchipext"][0]));
        maxRel = Math.Max(maxRel, TestData.RelErr(pc.Evaluate(6600), v["pchipext"][1]));
        _out.WriteLine($"pchip off-grid/extrap max rel err = {maxRel:E3}");
        Assert.True(maxRel <= 1e-12, $"max rel err {maxRel}");
    }

    [Fact]
    public void Pchip_InterpolatesNodesAndIsShapePreserving()
    {
        var pc = new Pchip(new double[] { 0, 1, 2, 3 }, new double[] { 0, 1, 1, 0 });
        Assert.Equal(1.0, pc.Evaluate(1.0), 15);
        Assert.Equal(1.0, pc.Evaluate(1.5), 12); // flat interval stays flat (zero slopes at sign change)
        for (double xq = 0; xq <= 3; xq += 0.01) Assert.InRange(pc.Evaluate(xq), -1e-12, 1 + 1e-12);
    }

    [Fact]
    public void MatlabColon_MatchesMatlabExactly()
    {
        int lines = 0;
        foreach (var line in File.ReadAllLines(Path.Combine(TestData.Dir, "ml_colon.txt")))
        {
            var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts[0] == "up")
            {
                double a = Parse(parts[1]), d = Parse(parts[2]), b = Parse(parts[3]);
                var r = MatlabColon.Range(a, d, b);
                Assert.Equal(int.Parse(parts[4]), r.Length);
                Assert.Equal(Parse(parts[5]), r[^1]);
            }
            else
            {
                double a = Parse(parts[0]), step = Parse(parts[1]), b = Parse(parts[2]);
                int n = int.Parse(parts[3]);
                var r = MatlabColon.Range(a, -step, b);
                Assert.Equal(n, r.Length);
                for (int i = 0; i < n; i++) Assert.Equal(Parse(parts[4 + i]), r[i]); // bit-exact
            }
            lines++;
        }
        Assert.True(lines >= 10);
        // up 1 0.1 220 v(1235) and v(607) checks
        var vs = MatlabColon.Range(1, 0.1, 220);
        Assert.Equal(124.39999999999999, vs[1234]);
        Assert.Equal(61.600000000000001, vs[606]);
    }

    /// <summary>1200 random ranges (steps 0.1, -0.05, 0.05, 1, 0.3, -0.1, 7, 0.01) from Data/matlab_scripts/colons_random.m.</summary>
    [Fact]
    public void MatlabColon_RandomRangesBitExact()
    {
        int ranges = 0, elems = 0, bad = 0;
        string first = "";
        foreach (var line in File.ReadAllLines(Path.Combine(TestData.Dir, "ml_colon_random.txt")))
        {
            var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            double a = Parse(parts[0]), d = Parse(parts[1]), b = Parse(parts[2]);
            int n = int.Parse(parts[3]);
            var r = MatlabColon.Range(a, d, b);
            ranges++;
            if (r.Length != n) { bad++; if (first == "") first = $"{line[..60]}: length {r.Length} vs {n}"; continue; }
            for (int i = 0; i < n; i++)
            {
                elems++;
                if (r[i] != Parse(parts[4 + i])) { bad++; if (first == "") first = $"{parts[0]}:{parts[1]}:{parts[2]} element {i}"; break; }
            }
        }
        _out.WriteLine($"colon: {ranges} ranges, {elems} elements compared, {bad} ranges differ {first}");
        Assert.Equal(1200, ranges);
        Assert.True(bad == 0, first);
    }

    [Fact]
    public void MatlabColon_EdgeCases()
    {
        Assert.Empty(MatlabColon.Range(1, -0.1, 2));
        Assert.Empty(MatlabColon.Range(2, 0.1, 1));
        Assert.Single(MatlabColon.Range(1, -0.05, 1));
        Assert.Equal(111, MatlabColon.Range(1000, 50, 6500).Length);
        Assert.Equal(220, MatlabColon.Range(0.1, 1, 220).Length);
        Assert.Equal(2191, MatlabColon.Range(1, 0.1, 220).Length);
    }

    [Fact]
    public void FxMax_AtDefaults()
    {
        var t = ReferenceModel.Build(ReferenceParameters.Default);
        Assert.Equal(6302.146956, t.FxMax, 6);
        Assert.Equal(TestData.MlValues()["Fxmax"][0], t.FxMax); // bit-exact vs MATLAB
        _out.WriteLine($"F_x_max = {t.FxMax:R}");
    }

    [Theory]
    [InlineData("matlab_S01", 0.0)]
    [InlineData("matlab_S02", 10.0)]
    public void TablesHistoriesAndSummary_MatchMatlab(string dir, double slope)
    {
        var p = ReferenceParameters.Default with { SlopeValue = slope };
        var root = Path.Combine(TestData.Dir, dir);
        double maxRel = CompareAll(p, root, out string worst);
        _out.WriteLine($"{dir}: max rel err over every exported quantity = {maxRel:E3} ({worst})");
        Assert.True(maxRel <= 1e-9, $"{dir}: {maxRel} at {worst}");
    }

    // Per-scenario golden comparison (required, compare_config.json tolerances): GoldenValidationTests.

    /// <summary>787-point off-node PCHIP probe written by the MATLAB harness (validation/golden/S01/pchip_probe.csv).</summary>
    [Fact]
    public void Pchip_GoldenProbeMatches()
    {
        string f = Path.Combine(TestData.RepoRoot, "validation", "golden", "S01", "pchip_probe.csv");
        Assert.True(File.Exists(f), "golden pchip_probe.csv missing: run validation/matlab/run_reference.m");
        var c = TestData.ReadCsv(f);
        var p = ReferenceParameters.Default;
        var pc = new Pchip(p.RpmRaw, p.TorqueRaw);
        double maxRel = 0;
        for (int i = 0; i < c["rpm"].Length; i++) maxRel = Math.Max(maxRel, TestData.RelErr(pc.Evaluate(c["rpm"][i]), c["t_engine_nm"][i]));
        _out.WriteLine($"pchip probe ({c["rpm"].Length} points) max rel err = {maxRel:E3}");
        Assert.True(maxRel <= 1e-12);
    }

    internal static double CompareAll(ReferenceParameters p, string root, out string worst)
    {
        var t = ReferenceModel.Build(p);
        var o = ReferenceOval.Run(p, t);
        double maxRel = 0;
        string w = "";
        void Cmp(string name, double a, double b)
        {
            // relative error with an absolute floor so that quantities that are ~0 compare sensibly
            double e = Math.Abs(a - b) / Math.Max(1e-6, Math.Max(Math.Abs(a), Math.Abs(b)));
            if (e > maxRel || double.IsNaN(e)) { maxRel = double.IsNaN(e) ? double.PositiveInfinity : e; w = name; }
        }
        for (int g = 0; g < t.NumGears; g++)
        {
            var c = TestData.ReadCsv(Path.Combine(root, $"tables_gear{g + 1}.csv"));
            Assert.Equal(t.RpmAxis.Length, c["rpm"].Length);
            for (int i = 0; i < t.RpmAxis.Length; i++)
            {
                Cmp($"g{g + 1}.rpm", t.RpmAxis[i], c["rpm"][i]);
                Cmp($"g{g + 1}.v_kmh", t.VGearKmh[g][i], c["v_kmh"][i]);
                Cmp($"g{g + 1}.t_engine_nm", t.TEngine[i], c["t_engine_nm"][i]);
                Cmp($"g{g + 1}.t_wheel_nm", t.TWheelNm[g][i], c["t_wheel_nm"][i]);
                Cmp($"g{g + 1}.f_tractive_n", t.FTractiveN[g][i], c["f_tractive_n"][i]);
                Cmp($"g{g + 1}.a_x_ms2", t.AxMs2[g][i], c["a_x_ms2"][i]);
                Cmp($"g{g + 1}.f_tractive_eff_n", t.FTractiveEffective[g][i], c["f_tractive_eff_n"][i]);
                Cmp($"g{g + 1}.a_x_real", t.AxReal[g][i], c["a_x_real"][i]);
            }
        }
        var r = TestData.ReadCsv(Path.Combine(root, "resist.csv"));
        Assert.Equal(t.VKmh.Length, r["v_kmh"].Length);
        for (int i = 0; i < t.VKmh.Length; i++)
        {
            Cmp("resist.v_kmh", t.VKmh[i], r["v_kmh"][i]);
            Cmp("resist.f_drag_n", t.FDrag[i], r["f_drag_n"][i]);
            Cmp("resist.f_rolling_n", t.FRolling[i], r["f_rolling_n"][i]);
            Cmp("resist.f_pend_n", t.FPend[i], r["f_pend_n"][i]);
            Cmp("resist.f_resistive_n", t.FResistive[i], r["f_resistive_n"][i]);
            Cmp("resist.p_demanded_kw", t.PDemandedKW[i], r["p_demanded_kw"][i]);
            Cmp("resist.a_x_envelope", t.AxEnvelope[i], r["a_x_envelope"][i]);
        }
        foreach (var (name, sr) in new[] { ("r1", o.R1), ("r2", o.R2) })
        {
            var h = TestData.ReadCsv(Path.Combine(root, $"{name}_history.csv"));
            Assert.Equal(h["t_s"].Length, sr.HistoryT.Length);
            for (int i = 0; i < sr.HistoryT.Length; i++)
            {
                Cmp($"{name}.t_s", sr.HistoryT[i], h["t_s"][i]);
                Cmp($"{name}.d_m", sr.HistoryD[i], h["d_m"][i]);
                Cmp($"{name}.v_kmh", sr.HistoryVKmh[i], h["v_kmh"][i]);
            }
        }
        var s = TestData.ReadSummary(Path.Combine(root, "summary.json"));
        foreach (var kv in SummaryValues(t, o)) Cmp("summary." + kv.Key, kv.Value, s[kv.Key]);
        worst = w;
        return maxRel;
    }

    internal static Dictionary<string, double> SummaryValues(ReferenceTables t, OvalResult o) => new()
    {
        ["F_x_max"] = t.FxMax, ["v_curva_adherencia_kmh"] = o.VCurvaAdherenciaKmh, ["optimal_gear_curva"] = o.OptimalGearCurva,
        ["v_max_gear_motriz_kmh"] = o.VMaxGearMotrizKmh, ["v_curva_final_kmh"] = o.VCurvaFinalKmh,
        ["d_frenado_r1"] = o.R1.DFrenado, ["t_frenado_r1"] = o.R1.TFrenado, ["d_acc_efectiva_r1"] = o.R1.DAccEfectiva,
        ["t_acc_r1"] = o.R1.TAcc, ["v_max_recta1"] = o.R1.VMaxKmh, ["t_recta1_total"] = o.R1.TTotal,
        ["d_frenado_r2"] = o.R2.DFrenado, ["t_frenado_r2"] = o.R2.TFrenado, ["d_acc_efectiva_r2"] = o.R2.DAccEfectiva,
        ["t_acc_r2"] = o.R2.TAcc, ["v_max_recta2"] = o.R2.VMaxKmh, ["t_recta2_total"] = o.R2.TTotal,
        ["t_curva1_total"] = o.TCurva1Total, ["t_curva2_total"] = o.TCurva2Total,
        ["t_vuelta_completa"] = o.TVueltaCompleta, ["distancia_lap"] = o.DistanciaLap,
    };

    [Fact]
    public void Baseline_ReportedValues()
    {
        var t = ReferenceModel.Build(ReferenceParameters.Default);
        var o = ReferenceOval.Run(ReferenceParameters.Default, t);
        _out.WriteLine($"F_x_max={t.FxMax:R} v_curva_final_kmh={o.VCurvaFinalKmh:R} t_vuelta={o.TVueltaCompleta:R}");
        Assert.Equal(60.6, o.VCurvaFinalKmh, 9);
        Assert.Equal(63.928121, o.TVueltaCompleta, 6);
        Assert.Equal(117.62, o.R1.VMaxKmh, 2);
        Assert.Equal(122.66, o.R2.VMaxKmh, 2);
        Assert.Equal(56.1, o.R1.DFrenado, 1);
        Assert.Equal(2, o.OptimalGearCurva);
    }

    [Fact]
    public void Overrides_UnknownNameRejected()
    {
        Assert.Throws<ArgumentException>(() => ReferenceParameters.Default.WithOverride("car.unknown", 1));
        Assert.Equal(1800, ReferenceParameters.Default.WithOverride("car.M", 1800).M);
    }

    private static double Parse(string s) => double.Parse(s, CultureInfo.InvariantCulture);
}
