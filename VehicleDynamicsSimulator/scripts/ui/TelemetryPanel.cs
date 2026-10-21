using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Godot;
using Simulator.Simulation;
using VehicleDynamics.Core.Simulation;

namespace Simulator.UI;

/// <summary>
/// Engineering telemetry: every quantity the model provides, with units and source tags, live plots from the
/// fixed-interval sample buffer (not render frames), recording controls and CSV export.
/// </summary>
public partial class TelemetryPanel : PanelContainer
{
    private SimulationManager _sim = null!;
    private readonly List<(Label Value, Func<string> Get)> _values = new();
    private TelemetryPlot _speed = null!, _rpm = null!, _accel = null!, _inputs = null!, _grip = null!, _gg = null!;
    private Button _record = null!;
    private Label _status = null!;
    private bool _byDistance;
    private double _plotTimer;

    public void Init(SimulationManager sim)
    {
        _sim = sim;
        Theme = UiTheme.Get();
        CustomMinimumSize = new Vector2(640, 0);
        var outer = new VBoxContainer();
        AddChild(outer);
        outer.AddChild(UiTheme.Label("ENGINEERING TELEMETRY", 18, UiTheme.Accent));

        var bar = new HFlowContainer();
        _record = UiTheme.Button("● Record", () => _sim.ToggleRecording(), "F5");
        bar.AddChild(_record);
        bar.AddChild(UiTheme.Button("Discard Recording", () =>
        {
            if (_sim.Telemetry.Recording) { _sim.Telemetry.Stop(_sim.Vehicle); _sim.Telemetry.Runs.RemoveAt(_sim.Telemetry.Runs.Count - 1); }
            _sim.Notify("Recording discarded");
        }));
        bar.AddChild(UiTheme.Button("Export CSV", () => { var p = _sim.ExportLatestRun(); if (p != null) _status.Text = "Exported " + p + " (+ .config.json)"; },
            "Stops the recording and writes CSV + parameter JSON to " + SimulationManager.RunsDirectory));
        var axis = new OptionButton { FocusMode = FocusModeEnum.None };
        axis.AddItem("x: time [s]");
        axis.AddItem("x: distance [m]");
        axis.ItemSelected += i => _byDistance = i == 1;
        bar.AddChild(axis);
        outer.AddChild(bar);
        _status = UiTheme.Label("", 11, UiTheme.Muted);
        _status.AutowrapMode = TextServer.AutowrapMode.WordSmart;
        outer.AddChild(_status);

        var scroll = new ScrollContainer { SizeFlagsVertical = SizeFlags.ExpandFill, HorizontalScrollMode = ScrollContainer.ScrollMode.Disabled };
        var content = new VBoxContainer { SizeFlagsHorizontal = SizeFlags.ExpandFill };
        scroll.AddChild(content);
        outer.AddChild(scroll);

        var grid = new GridContainer { Columns = 6 };
        grid.AddThemeConstantOverride("h_separation", 10);
        content.AddChild(grid);
        BuildValues(grid);

        content.AddChild(Wrap(UiTheme.Label(
            "Not available (not in the model): per-wheel forces and longitudinal slip ratios (the E4 single-track model works per axle; " +
            "model.m has an axle-level traction limit only), suspension travel, roll and pitch, lateral load transfer, tyre temperatures.",
            10, UiTheme.Muted)));

        _speed = Plot("Speed [km/h]");
        _rpm = Plot("Engine speed [rpm]");
        _accel = Plot("Acceleration [m/s²]");
        _inputs = Plot("Driver inputs");
        _inputs.FixedY = new Vector2(-1.05f, 1.05f);
        _grip = Plot("Grip usage / slip indicator");
        _gg = Plot("g-g diagram: a_y vs a_x [m/s²], circle = F_grip / M");
        _gg.Scatter = true;
        _gg.CustomMinimumSize = new Vector2(300, 240);
        foreach (var p in new[] { _speed, _rpm, _accel, _inputs, _grip, _gg }) content.AddChild(p);
    }

    private static Label Wrap(Label l) { l.AutowrapMode = TextServer.AutowrapMode.WordSmart; return l; }

    private static TelemetryPlot Plot(string title) => new() { Title = title, SizeFlagsHorizontal = SizeFlags.ExpandFill };

    private void BuildValues(GridContainer grid)
    {
        string F(double v, string fmt) => v.ToString(fmt, CultureInfo.InvariantCulture);
        var d = () => _sim.Vehicle.Dynamics;
        void V(string name, string unit, string tag, Func<string> get)
        {
            grid.AddChild(UiTheme.Label(name, 12, UiTheme.Muted));
            var value = UiTheme.Label("", 12, UiTheme.Text);
            value.CustomMinimumSize = new Vector2(70, 0);
            value.HorizontalAlignment = HorizontalAlignment.Right;
            grid.AddChild(value);
            grid.AddChild(UiTheme.Label(unit + (tag.Length > 0 ? "  " + tag : ""), 10, tag.Length > 0 ? UiTheme.Ext : UiTheme.Muted));
            _values.Add((value, get));
        }
        V("time t", "s", "", () => F(d().State.Time, "0.00"));
        V("speed V", "km/h", "", () => F(d().State.V * 3.6, "0.0"));
        V("V", "m/s", "", () => F(d().State.V, "0.00"));
        string Lat() => d().Parameters.LateralModel == VehicleDynamics.Core.Simulation.LateralModel.Dynamic ? "[E4]" : "[E1]";
        V("v_x", "m/s", "", () => F(d().Outputs.Vx, "0.00"));
        V("v_y", "m/s", "[E4/E1]", () => F(d().Outputs.Vy, "0.000") + " " + Lat());
        V("yaw rate r", "rad/s", "[E4/E1]", () => F(d().Outputs.YawRate, "0.000"));
        V("sideslip β", "rad", "[E4/E1]", () => F(d().Outputs.Beta, "0.0000"));
        V("steer δ", "rad", "", () => F(d().Outputs.SteerAngle, "0.000"));
        V("slip angle α_f", "rad", "[E4]", () => F(d().Outputs.AlphaFront, "0.0000"));
        V("slip angle α_r", "rad", "[E4]", () => F(d().Outputs.AlphaRear, "0.0000"));
        V("F_y front", "N", "[E4]", () => F(d().Outputs.FyFront, "0"));
        V("F_y rear", "N", "[E4]", () => F(d().Outputs.FyRear, "0"));
        V("F_z front", "N", "[E4]", () => F(d().Outputs.FzFront, "0"));
        V("F_z rear", "N", "[E4]", () => F(d().Outputs.FzRear, "0"));
        V("clutch", "rpm slip", "[E5]", () => d().Outputs.ClutchLocked ? "locked" : F(d().Outputs.ClutchSlipRpm, "0"));
        V("clutch torque", "N·m", "[E5]", () => F(d().Outputs.ClutchTorque, "0"));
        V("a_x", "m/s²", "", () => F(d().Outputs.Ax, "0.00"));
        V("a_y", "m/s²", "[E4/E1]", () => F(d().Outputs.Ay, "0.00"));
        V("gear", "", "", () => RaceHud.GearLabel(d().State));
        V("engine speed", "rpm", "", () => F(d().State.EngineRpm, "0"));
        V("engine torque", "N·m", "", () => F(d().Outputs.EngineTorque, "0.0"));
        V("throttle", "–", "", () => F(_sim.Vehicle.LastInputs.Throttle, "0.00"));
        V("brake", "–", "", () => F(_sim.Vehicle.LastInputs.Brake, "0.00"));
        V("drive branch", "", "", () => d().Outputs.Branch.ToString());
        V("F_tractive", "N", "", () => F(d().Outputs.FTractive, "0"));
        V("F_tractive,eff", "N", "", () => F(d().Outputs.FTractiveEff, "0"));
        V("F_x,max", "N", "", () => F(d().Outputs.FxMax, "0"));
        V("lateral grip capacity", "N", "[E4/E1]", () => F(d().Outputs.FGrip, "0"));
        V("F_y total", "N", "[E4/E1]", () => F(d().Outputs.Fy, "0"));
        V("F_drag", "N", "", () => F(d().Outputs.FDrag, "0.0"));
        V("F_rolling", "N", "", () => F(d().Outputs.FRolling, "0.0"));
        V("F_grade", "N", "", () => F(d().Outputs.FGrade, "0.0"));
        V("F_brake", "N", "", () => F(d().Outputs.FBrake, "0"));
        V("F_engine brake", "N", "JS seed, coast only", () => d().Outputs.Branch == VehicleDynamics.Core.Simulation.DriveBranch.Coast ? F(d().Outputs.FEngineBrake, "0") : "0");
        V("M_eq", "kg", "", () => F(d().Outputs.MassEquivalent, "0"));
        V("grip usage", "–", "", () => F(d().Outputs.GripUsage, "0.00"));
        V("slip indicator", "–", "", () => F(d().Outputs.SlipIndicator, "0.00"));
        V("x, y", "m", "", () => F(d().State.X, "0.0") + ", " + F(d().State.Y, "0.0"));
        V("heading ψ", "rad", "", () => F(d().State.Psi, "0.000"));
        V("track s", "m", "", () => F(_sim.Laps.Projection.S, "0.0"));
        V("lateral offset", "m", "", () => F(_sim.Laps.Projection.Lateral, "0.00"));
        V("surface", "", "[E2]", () => _sim.CurrentSurface.ToString());
        V("lap / time", "s", "", () => $"{_sim.Laps.Lap} / {F(_sim.Laps.CurrentLapTime, "0.00")}");
        V("last impact", "m/s", "[E3]", () => F(_sim.LastImpactSeverity, "0.00"));
    }

    public override void _Process(double delta)
    {
        if (!Visible || _sim == null) return;
        foreach (var (label, get) in _values) label.Text = get();
        var tel = _sim.Telemetry;
        _record.Text = tel.Recording ? "■ Stop Recording" : "● Record";
        if (tel.Recording)
            _status.Text = $"Recording '{tel.Active!.Run.Name}': {tel.Active.Run.Samples.Count} samples every {tel.SampleInterval:0.###} s (sim time)";
        else if (_status.Text.Length == 0 || _status.Text.StartsWith("Recording"))
            _status.Text = $"{tel.Runs.Count} recorded run(s). Exports go to {SimulationManager.RunsDirectory}";

        _plotTimer -= delta;
        if (_plotTimer > 0) return;
        _plotTimer = 0.1;
        UpdatePlots();
    }

    private void UpdatePlots()
    {
        var live = _sim.Telemetry.Live;
        float X(TelemetrySample s) => _byDistance ? (float)s.Distance : (float)s.T;
        string xl = _byDistance ? "m" : "s";
        List<Vector2> Series(Func<TelemetrySample, double> f) => live.Select(s => new Vector2(X(s), (float)f(s))).ToList();

        Set(_speed, xl, new PlotSeries("V", UiTheme.Accent, Series(s => s.VKmh)));
        Set(_rpm, xl, new PlotSeries("rpm", new Color(1, 0.4f, 0.4f), Series(s => s.EngineRpm)));
        Set(_accel, xl, new PlotSeries("a_x", new Color(0.4f, 0.8f, 1), Series(s => s.Ax)), new PlotSeries("a_y [E1]", new Color(0.7f, 1, 0.5f), Series(s => s.Ay)));
        double dmax = Math.Max(1e-6, _sim.Vehicle.Parameters.MaxSteerAngle);
        Set(_inputs, xl, new PlotSeries("throttle", UiTheme.Good, Series(s => s.Throttle)), new PlotSeries("brake", UiTheme.Bad, Series(s => s.Brake)),
            new PlotSeries("δ/δmax", new Color(0.6f, 0.7f, 1), Series(s => s.Steer / dmax)));
        Set(_grip, xl, new PlotSeries("grip usage", UiTheme.Accent, Series(s => s.GripUsage)), new PlotSeries("slip", new Color(1, 0.4f, 0.8f), Series(s => s.SlipIndicator)));

        _gg.Series.Clear();
        _gg.Series.Add(new PlotSeries("samples", UiTheme.Accent, live.Select(s => new Vector2((float)s.Ay, (float)s.Ax)).ToList()));
        var o = _sim.Vehicle.Dynamics.Outputs;
        _gg.Circle = (float)(o.FGrip / _sim.Vehicle.Parameters.Mass);
        _gg.QueueRedraw();
    }

    private static void Set(TelemetryPlot p, string xl, params PlotSeries[] s)
    {
        p.XLabel = xl;
        p.Series.Clear();
        p.Series.AddRange(s);
        p.QueueRedraw();
    }
}
