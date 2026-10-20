using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Godot;
using Simulator.Simulation;
using VehicleDynamics.Core.Simulation;

namespace Simulator.UI;

/// <summary>
/// Vehicle setup: edits a working copy of <see cref="VehicleParameters"/>; nothing reaches the physics until Apply,
/// which goes through <see cref="SimulationManager.ApplyParameters"/> (validated, between physics steps).
/// </summary>
public partial class VehicleSetupPanel : PanelContainer
{
    private SimulationManager _sim = null!;
    private VehicleParameters _edit = new();
    private readonly VehicleParameters _defaults = new();
    private readonly List<(ParamEntry Entry, HSlider? Slider, SpinBox? Spin, CheckBox? Check)> _rows = new();
    private Label _status = null!;
    private Label _derived = null!;
    private Button _apply = null!;
    private Button _pause = null!;
    private LineEdit _saveName = null!;
    private OptionButton _configs = null!;
    private bool _syncing;

    public void Init(SimulationManager sim)
    {
        _sim = sim;
        Theme = UiTheme.Get();
        CustomMinimumSize = new Vector2(500, 0);
        var outer = new VBoxContainer();
        outer.AddThemeConstantOverride("separation", 6);
        AddChild(outer);

        outer.AddChild(UiTheme.Label("VEHICLE SETUP", 18, UiTheme.Accent));
        outer.AddChild(UiTheme.Label("Edits apply on [Apply Setup]. Tags: model.m = original MATLAB model; JS seed = seed game rule; Ext. = extension, provisional value.", 11, UiTheme.Muted));

        var buttons = new HFlowContainer();
        _apply = UiTheme.Button("Apply Setup", Apply);
        buttons.AddChild(_apply);
        buttons.AddChild(UiTheme.Button("Reset Parameters", () => { _edit = new VehicleParameters(); SyncFromEdit(); }, "Back to model.m / seed defaults (not applied yet)"));
        buttons.AddChild(UiTheme.Button("Revert Edits", () => { _edit = _sim.Vehicle.Parameters.Clone(); SyncFromEdit(); }, "Discard edits not yet applied"));
        buttons.AddChild(UiTheme.Button("Reset Vehicle", () => _sim.RecoverToTrack(), "Recover to the track centreline at rest"));
        _pause = UiTheme.Button("Pause", () => _sim.Vehicle.SetPaused(!_sim.Vehicle.Paused));
        buttons.AddChild(_pause);
        buttons.AddChild(UiTheme.Button("Restart Simulation", () => _sim.RestartSimulation(), "Back to the grid at rest; laps cleared"));
        outer.AddChild(buttons);

        var cfgRow = new HBoxContainer();
        _saveName = new LineEdit { PlaceholderText = "config name", SizeFlagsHorizontal = SizeFlags.ExpandFill };
        _saveName.TextSubmitted += _ => { SaveConfig(); _saveName.ReleaseFocus(); };
        cfgRow.AddChild(_saveName);
        cfgRow.AddChild(UiTheme.Button("Save Configuration", SaveConfig, "Saves the APPLIED parameters as JSON"));
        outer.AddChild(cfgRow);
        var loadRow = new HBoxContainer();
        _configs = new OptionButton { SizeFlagsHorizontal = SizeFlags.ExpandFill, FocusMode = FocusModeEnum.None };
        loadRow.AddChild(_configs);
        loadRow.AddChild(UiTheme.Button("Load Configuration", LoadConfig, "Loads into the editor; press Apply to use it"));
        loadRow.AddChild(UiTheme.Button("⟳", RefreshConfigList, "Rescan the configuration folder"));
        outer.AddChild(loadRow);

        _status = UiTheme.Label("", 12, UiTheme.Muted);
        _status.AutowrapMode = TextServer.AutowrapMode.WordSmart;
        outer.AddChild(_status);
        _derived = UiTheme.Label("", 12, UiTheme.Text);
        outer.AddChild(_derived);

        var scroll = new ScrollContainer { SizeFlagsVertical = SizeFlags.ExpandFill, HorizontalScrollMode = ScrollContainer.ScrollMode.Disabled };
        var list = new VBoxContainer { SizeFlagsHorizontal = SizeFlags.ExpandFill };
        scroll.AddChild(list);
        outer.AddChild(scroll);

        string? group = null;
        foreach (var e in ParameterCatalog.Build())
        {
            if (e.Group != group)
            {
                group = e.Group;
                var h = UiTheme.Label(group.ToUpperInvariant(), 13, UiTheme.Accent);
                list.AddChild(new HSeparator());
                list.AddChild(h);
            }
            list.AddChild(BuildRow(e));
        }

        _edit = _sim.Vehicle.Parameters.Clone();
        SyncFromEdit();
        RefreshConfigList();
    }

    private Control BuildRow(ParamEntry e)
    {
        var box = new VBoxContainer { TooltipText = e.Help };
        box.AddThemeConstantOverride("separation", 0);
        var line = new HBoxContainer();
        var name = UiTheme.Label(e.Name, 13);
        name.CustomMinimumSize = new Vector2(190, 0);
        name.TooltipText = e.Help;
        name.MouseFilter = MouseFilterEnum.Pass;
        line.AddChild(name);
        var tagColor = e.Source == ParamSource.ModelM ? UiTheme.Muted : e.Source == ParamSource.JsSeed ? UiTheme.Accent : UiTheme.Ext;

        if (e.IsToggle)
        {
            var cb = new CheckBox { FocusMode = FocusModeEnum.None, SizeFlagsHorizontal = SizeFlags.ExpandFill };
            cb.Toggled += on => { if (_syncing) return; e.Set(_edit, on ? 1 : 0); Changed(); };
            line.AddChild(cb);
            line.AddChild(UiTheme.Label(ParameterCatalog.Tag(e.Source), 10, tagColor));
            line.AddChild(ResetButton(e));
            box.AddChild(line);
            _rows.Add((e, null, null, cb));
            return box;
        }

        var slider = new HSlider
        {
            MinValue = e.Min, MaxValue = e.Max, Step = e.Step, SizeFlagsHorizontal = SizeFlags.ExpandFill,
            FocusMode = FocusModeEnum.None, CustomMinimumSize = new Vector2(110, 0),
        };
        var spin = new SpinBox
        {
            MinValue = e.Min, MaxValue = e.Max, Step = e.Step, CustomMinimumSize = new Vector2(124, 0),
            Suffix = e.Unit == "–" ? "" : e.Unit, SelectAllOnFocus = true,
        };
        spin.GetLineEdit().TextSubmitted += _ => spin.GetLineEdit().ReleaseFocus();
        slider.ValueChanged += v => { if (_syncing) return; _syncing = true; spin.Value = v; _syncing = false; e.Set(_edit, v); Changed(); };
        spin.ValueChanged += v => { if (_syncing) return; _syncing = true; slider.Value = v; _syncing = false; e.Set(_edit, v); Changed(); };
        line.AddChild(slider);
        line.AddChild(spin);
        line.AddChild(ResetButton(e));
        box.AddChild(line);
        var meta = UiTheme.Label($"{ParameterCatalog.Tag(e.Source)} · range {Fmt(e.Min)}…{Fmt(e.Max)} {e.Unit} · default {Fmt(e.Get(_defaults))}", 10, tagColor);
        box.AddChild(meta);
        _rows.Add((e, slider, spin, null));
        return box;
    }

    private Button ResetButton(ParamEntry e)
        => UiTheme.Button("↺", () => { e.Set(_edit, e.Get(_defaults)); SyncFromEdit(); }, "Reset to default " + Fmt(e.Get(_defaults)));

    private static string Fmt(double v) => v.ToString("0.#####", CultureInfo.InvariantCulture);

    private void SyncFromEdit()
    {
        _syncing = true;
        foreach (var (e, slider, spin, check) in _rows)
        {
            double v = e.Get(_edit);
            if (check != null) check.ButtonPressed = v >= 0.5;
            if (slider != null) slider.Value = v;
            if (spin != null) spin.Value = v;
        }
        _syncing = false;
        Changed();
    }

    private void Changed()
    {
        var errors = _edit.Validate();
        _apply.Disabled = errors.Count > 0;
        bool dirty = ConfigSerializer.ToJson(_edit) != ConfigSerializer.ToJson(_sim.Vehicle.Parameters);
        if (errors.Count > 0)
        {
            _status.Text = "Invalid: " + string.Join("; ", errors);
            _status.AddThemeColorOverride("font_color", UiTheme.Bad);
        }
        else
        {
            _status.Text = dirty ? "Edited — press Apply Setup to use these values." : "Editor matches the running parameters.";
            _status.AddThemeColorOverride("font_color", dirty ? UiTheme.Accent : UiTheme.Good);
        }
        UpdateDerived();
    }

    private void UpdateDerived()
    {
        if (_edit.Validate().Count > 0) { _derived.Text = ""; return; }
        var p = _edit;
        var dyn = new VehicleDynamics.Core.Simulation.VehicleDynamics(p);
        string gears = string.Join("  ", Enumerable.Range(1, p.NumGears).Select(g => $"{g}:{dyn.GearMaxSpeedKmh(g):0}"));
        _derived.Text = string.Create(CultureInfo.InvariantCulture,
            $"Derived (edited values): R_w = {p.WheelRadius:0.0000} m · K_φf = {p.KPhiFront:0} · K_φr = {p.KPhiRear:0} N·m/rad\n" +
            $"F_x,max (asphalt) = {TractionLimit.FxMax(p, p.Mu):0.0} N · θ = {p.Theta:0.0000} rad\n" +
            $"Gear speed at {p.RpmRaw[^1]:0} rpm [km/h]: {gears}");
    }

    private void Apply()
    {
        try
        {
            _sim.ApplyParameters(_edit.Clone());
            Changed();
        }
        catch (ArgumentException ex)
        {
            _status.Text = ex.Message;
            _status.AddThemeColorOverride("font_color", UiTheme.Bad);
        }
    }

    private void SaveConfig()
    {
        string name = string.IsNullOrWhiteSpace(_saveName.Text) ? $"setup_{DateTime.Now:yyyyMMdd_HHmmss}" : _saveName.Text.Trim();
        foreach (char c in Path.GetInvalidFileNameChars()) name = name.Replace(c, '_');
        string path = Path.Combine(SimulationManager.ConfigsDirectory, name + ".json");
        try
        {
            ConfigSerializer.Save(_sim.Vehicle.Parameters, path);
            _status.Text = "Saved applied parameters to " + path;
            _status.AddThemeColorOverride("font_color", UiTheme.Good);
            RefreshConfigList();
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or ArgumentException)
        {
            _status.Text = "Save failed: " + ex.Message;
            _status.AddThemeColorOverride("font_color", UiTheme.Bad);
        }
    }

    private void RefreshConfigList()
    {
        _configs.Clear();
        if (!Directory.Exists(SimulationManager.ConfigsDirectory)) return;
        foreach (var f in Directory.GetFiles(SimulationManager.ConfigsDirectory, "*.json").OrderBy(f => f))
            _configs.AddItem(Path.GetFileName(f));
    }

    private void LoadConfig()
    {
        if (_configs.Selected < 0) { _status.Text = "No configuration selected."; return; }
        string path = Path.Combine(SimulationManager.ConfigsDirectory, _configs.GetItemText(_configs.Selected));
        if (ConfigSerializer.TryLoad(path, out var p, out var err))
        {
            _edit = p!;
            SyncFromEdit();
            _status.Text = $"Loaded {path} into the editor — press Apply Setup to use it.";
            _status.AddThemeColorOverride("font_color", UiTheme.Accent);
        }
        else
        {
            _status.Text = "Load failed: " + err;
            _status.AddThemeColorOverride("font_color", UiTheme.Bad);
        }
    }

    public override void _Process(double delta)
    {
        if (!Visible || _sim == null) return;
        _pause.Text = _sim.Vehicle.Paused ? "Resume" : "Pause";
    }

    /// <summary>Re-reads the running parameters into the editor (when the panel is opened).</summary>
    public void Refresh()
    {
        _edit = _sim.Vehicle.Parameters.Clone();
        SyncFromEdit();
        RefreshConfigList();
    }
}
