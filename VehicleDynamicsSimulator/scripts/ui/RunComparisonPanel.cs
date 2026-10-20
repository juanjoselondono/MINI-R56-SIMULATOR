using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Godot;
using Simulator.Simulation;
using VehicleDynamics.Core.Simulation;

namespace Simulator.UI;

/// <summary>
/// Compares recorded runs: overlays speed against time or distance (each run rebased to its own start) and reports
/// <see cref="RunComparison"/> metrics against the first selected run, plus the parameters that differ.
/// </summary>
public partial class RunComparisonPanel : PanelContainer
{
    private static readonly Color[] Palette =
    {
        new(1.0f, 0.62f, 0.2f), new(0.4f, 0.8f, 1f), new(0.7f, 1f, 0.5f), new(1f, 0.45f, 0.75f), new(0.85f, 0.85f, 0.4f), new(0.7f, 0.6f, 1f),
    };

    private SimulationManager _sim = null!;
    private ItemList _list = null!;
    private TelemetryPlot _plot = null!;
    private Label _stats = null!;
    private bool _byDistance = true;
    private int _knownRuns = -1;

    public void Init(SimulationManager sim)
    {
        _sim = sim;
        Theme = UiTheme.Get();
        CustomMinimumSize = new Vector2(720, 0);
        var outer = new VBoxContainer();
        AddChild(outer);
        outer.AddChild(UiTheme.Label("RUN COMPARISON", 18, UiTheme.Accent));
        outer.AddChild(UiTheme.Label("Select two or more recorded runs (Ctrl/Shift-click). Metrics compare each run with the first selected one.", 11, UiTheme.Muted));

        var bar = new HFlowContainer();
        var axis = new OptionButton { FocusMode = FocusModeEnum.None };
        axis.AddItem("align by distance from run start");
        axis.AddItem("align by time from run start");
        axis.ItemSelected += i => { _byDistance = i == 0; Recompute(); };
        bar.AddChild(axis);
        bar.AddChild(UiTheme.Button("Export Selected", ExportSelected, "CSV + parameter JSON for each selected run"));
        outer.AddChild(bar);

        _list = new ItemList { SelectMode = ItemList.SelectModeEnum.Multi, CustomMinimumSize = new Vector2(0, 120), FocusMode = FocusModeEnum.None };
        _list.MultiSelected += (_, _) => Recompute();
        outer.AddChild(_list);
        _plot = new TelemetryPlot { Title = "Speed [km/h]", CustomMinimumSize = new Vector2(600, 220), SizeFlagsHorizontal = SizeFlags.ExpandFill };
        outer.AddChild(_plot);
        var scroll = new ScrollContainer { SizeFlagsVertical = SizeFlags.ExpandFill, CustomMinimumSize = new Vector2(0, 160) };
        _stats = UiTheme.Label("", 12, UiTheme.Text);
        _stats.AutowrapMode = TextServer.AutowrapMode.WordSmart;
        _stats.SizeFlagsHorizontal = SizeFlags.ExpandFill;
        scroll.AddChild(_stats);
        outer.AddChild(scroll);
    }

    public override void _Process(double delta)
    {
        if (!Visible || _sim == null) return;
        if (_sim.Telemetry.Runs.Count != _knownRuns) RefreshList();
    }

    private void RefreshList()
    {
        _knownRuns = _sim.Telemetry.Runs.Count;
        _list.Clear();
        foreach (var r in _sim.Telemetry.Runs)
        {
            var smp = r.Run.Samples;
            double dur = smp.Count > 1 ? smp[^1].T - smp[0].T : 0;
            double dist = smp.Count > 1 ? smp[^1].Distance - smp[0].Distance : 0;
            _list.AddItem(string.Create(CultureInfo.InvariantCulture, $"{r.Run.Name}  ·  {r.Track}  ·  {smp.Count} samples  ·  {dur:0.0} s  ·  {dist:0} m"));
        }
        Recompute();
    }

    private List<RunEntry> Selected() => _list.GetSelectedItems().Select(i => _sim.Telemetry.Runs[i]).ToList();

    /// <summary>Copy of a run whose T is time (or distance) since its first sample, so runs started at different times align.</summary>
    private RecordedRun Rebase(RecordedRun r)
    {
        var copy = new RecordedRun { Name = r.Name };
        if (r.Samples.Count == 0) return copy;
        double t0 = r.Samples[0].T, d0 = r.Samples[0].Distance;
        double last = double.NegativeInfinity;
        foreach (var s in r.Samples)
        {
            var c = s;
            c.T = _byDistance ? s.Distance - d0 : s.T - t0;
            if (c.T <= last) continue; // standing still: keep the axis strictly increasing for interpolation
            last = c.T;
            copy.Samples.Add(c);
        }
        return copy;
    }

    private void Recompute()
    {
        var sel = Selected();
        _plot.Series.Clear();
        _plot.XLabel = _byDistance ? "m from run start" : "s from run start";
        var rebased = sel.Select(e => Rebase(e.Run)).ToList();
        for (int i = 0; i < rebased.Count; i++)
            _plot.Series.Add(new PlotSeries(sel[i].Run.Name, Palette[i % Palette.Length],
                rebased[i].Samples.Select(s => new Vector2((float)s.T, (float)s.VKmh)).ToList()));
        _plot.QueueRedraw();

        if (sel.Count < 2) { _stats.Text = sel.Count == 0 ? "No run selected." : "Select a second run to compare."; return; }
        var lines = new List<string>();
        var basePar = SimulationRecorder.ParameterKeyValues(sel[0].Parameters).ToDictionary(k => k.Key, k => k.Value);
        for (int i = 1; i < sel.Count; i++)
        {
            string M(string what, Func<TelemetrySample, double> f, string unit)
            {
                var c = RunComparison.Compare(rebased[0], rebased[i], f);
                return string.Create(CultureInfo.InvariantCulture, $"{what}: max|Δ| {c.MaxAbs:0.###} {unit}, RMSE {c.Rmse:0.###} {unit} (n = {c.Count})");
            }
            lines.Add($"{sel[i].Run.Name}  vs  {sel[0].Run.Name}");
            lines.Add("  " + M("speed", s => s.VKmh, "km/h"));
            lines.Add("  " + M("engine speed", s => s.EngineRpm, "rpm"));
            lines.Add("  " + M("a_x", s => s.Ax, "m/s²"));
            var diffs = SimulationRecorder.ParameterKeyValues(sel[i].Parameters)
                .Where(kv => !basePar.TryGetValue(kv.Key, out var v) || v != kv.Value)
                .Select(kv => $"{kv.Key}: {basePar.GetValueOrDefault(kv.Key, "?")} → {kv.Value}").ToList();
            lines.Add("  parameters: " + (diffs.Count == 0 ? "identical" : string.Join("; ", diffs)));
            if (sel[i].Track != sel[0].Track) lines.Add($"  note: different tracks ({sel[0].Track} vs {sel[i].Track})");
        }
        _stats.Text = string.Join("\n", lines);
    }

    private void ExportSelected()
    {
        var paths = Selected().Select(e => TelemetryService.Export(e, SimulationManager.RunsDirectory)).ToList();
        _stats.Text = paths.Count == 0 ? "No run selected." : "Exported:\n" + string.Join("\n", paths);
    }
}
