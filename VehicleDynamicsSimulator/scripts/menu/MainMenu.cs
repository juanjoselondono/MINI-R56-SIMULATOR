using System.IO;
using System.Linq;
using Godot;
using Simulator.Simulation;
using Simulator.UI;
using VehicleDynamics.Core.Simulation;

namespace Simulator.Menu;

/// <summary>Track and configuration choice. Command-line test modes skip straight to the simulation.</summary>
public partial class MainMenu : Control
{
    private OptionButton _config = null!;
    private Label _info = null!;

    public override void _Ready()
    {
        var args = OS.GetCmdlineUserArgs();
        if (args.Any(a => a.StartsWith("--autodrive") || a == "--selftest" || a == "--demo" || a.StartsWith("--track=") || a.StartsWith("--shots=") || a.StartsWith("--perf=")))
        {
            CallDeferred(MethodName.Start);
            return;
        }

        Theme = UiTheme.Get();
        SetAnchorsPreset(LayoutPreset.FullRect);
        var bg = new ColorRect { Color = new Color(0.04f, 0.045f, 0.06f) };
        bg.SetAnchorsPreset(LayoutPreset.FullRect);
        AddChild(bg);

        var v = new VBoxContainer();
        v.AddThemeConstantOverride("separation", 14);
        v.AddChild(UiTheme.Label("MINI COOPER R56 (N16)", 40, UiTheme.Accent));
        v.AddChild(UiTheme.Label("Vehicle dynamics simulator — model.m longitudinal model, validated against MATLAB", 16, UiTheme.Muted));
        v.AddChild(new HSeparator());

        v.AddChild(UiTheme.Label("TRACK", 14, UiTheme.Accent));
        foreach (var name in Track.TrackManager.Available)
        {
            var g = Track.TrackManager.Create(name);
            string desc = name == "Oval"
                ? $"model.m oval: 2 × 400 m straights, R80 semicircles — {g.Length:0.0} m"
                : $"Road circuit: hairpin R28, sweepers R120–150, chicane — {g.Length:0} m";
            var b = UiTheme.Button($"{name}   ·   {desc}", () => { GameSession.TrackName = name; Start(); });
            b.Alignment = HorizontalAlignment.Left;
            b.CustomMinimumSize = new Vector2(620, 46);
            b.AddThemeFontSizeOverride("font_size", 18);
            v.AddChild(b);
        }

        v.AddChild(UiTheme.Label("VEHICLE CONFIGURATION", 14, UiTheme.Accent));
        _config = new OptionButton { CustomMinimumSize = new Vector2(620, 36), FocusMode = FocusModeEnum.None };
        _config.AddItem("model.m defaults");
        string dir = SimulationManager.ConfigsDirectory;
        if (Directory.Exists(dir))
            foreach (var f in Directory.GetFiles(dir, "*.json").OrderBy(f => f)) _config.AddItem(Path.GetFileName(f));
        _config.ItemSelected += _ => Validate();
        v.AddChild(_config);
        _info = UiTheme.Label("", 12, UiTheme.Muted);
        v.AddChild(_info);
        v.AddChild(UiTheme.Label($"Configurations: {dir}\nTelemetry exports: {SimulationManager.RunsDirectory}", 12, UiTheme.Muted));
        v.AddChild(UiTheme.Button("Quit", () => GetTree().Quit()));

        var centre = new CenterContainer();
        centre.SetAnchorsPreset(LayoutPreset.FullRect);
        centre.AddChild(v);
        AddChild(centre);
        Validate();
    }

    private void Validate()
    {
        GameSession.ConfigPath = null;
        if (_config.Selected <= 0) { _info.Text = "Default parameters (identical to model.m)."; return; }
        string path = Path.Combine(SimulationManager.ConfigsDirectory, _config.GetItemText(_config.Selected));
        if (ConfigSerializer.TryLoad(path, out _, out var err))
        {
            GameSession.ConfigPath = path;
            _info.Text = "Will load " + path;
        }
        else _info.Text = "Invalid configuration: " + err;
    }

    private void Start() => GetTree().ChangeSceneToFile("res://scenes/Main.tscn");
}
