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
        v.AddChild(UiTheme.Label("VEHICLE DYNAMICS SIMULATOR", 40, UiTheme.Accent));
        v.AddChild(UiTheme.Label("Mini Cooper R56: model.m longitudinal model, validated against MATLAB · F1: provisional profile with downforce", 16, UiTheme.Muted));
        v.AddChild(new HSeparator());

        // Vehicle first: the track buttons start the session.
        v.AddChild(UiTheme.Label("VEHICLE", 14, UiTheme.Accent));
        var vehicles = new HBoxContainer();
        vehicles.AddThemeConstantOverride("separation", 10);
        var group = new ButtonGroup();
        foreach (var (id, displayName) in VehicleProfiles.All)
        {
            var b = new Button
            {
                Text = displayName, ToggleMode = true, ButtonGroup = group, FocusMode = FocusModeEnum.None,
                ButtonPressed = id == GameSession.VehicleProfile, CustomMinimumSize = new Vector2(305, 40),
            };
            b.Toggled += on => { if (on) { GameSession.VehicleProfile = id; Validate(); } };
            vehicles.AddChild(b);
        }
        v.AddChild(vehicles);

        v.AddChild(UiTheme.Label("TRACK", 14, UiTheme.Accent));
        foreach (var name in Track.TrackManager.Available)
        {
            string desc = name switch
            {
                "Oval" => $"model.m oval: 2 × 400 m straights, R80 semicircles — {Track.TrackManager.Create(name).Length:0.0} m",
                "Circuit" => $"Road circuit: hairpin R28, sweepers R120–150, chicane — {Track.TrackManager.Create(name).Length:0} m",
                _ => "Monaco street circuit — converted Assetto Corsa track (mesh surfaces, walls, elevation)",
            };
            var b = UiTheme.Button($"{name}   ·   {desc}", () => { GameSession.TrackName = name; Start(); });
            b.Alignment = HorizontalAlignment.Left;
            b.CustomMinimumSize = new Vector2(620, 46);
            b.AddThemeFontSizeOverride("font_size", 18);
            v.AddChild(b);
        }

        v.AddChild(UiTheme.Label("VEHICLE CONFIGURATION", 14, UiTheme.Accent));
        _config = new OptionButton { CustomMinimumSize = new Vector2(620, 36), FocusMode = FocusModeEnum.None };
        _config.AddItem("Vehicle profile defaults");
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
        string vehicle = VehicleProfiles.DisplayName(GameSession.VehicleProfile);
        if (_config.Selected <= 0)
        {
            _info.Text = GameSession.VehicleProfile == VehicleProfiles.MiniR56
                ? "Mini: model.m parameters with the game's mass 1150 kg and μ 1.05."
                : $"{vehicle}: provisional parameters (no MATLAB model for this car).";
            return;
        }
        string path = Path.Combine(SimulationManager.ConfigsDirectory, _config.GetItemText(_config.Selected));
        if (ConfigSerializer.TryLoad(path, out var cfg, out var err))
        {
            GameSession.ConfigPath = path;
            _info.Text = cfg!.Profile == GameSession.VehicleProfile
                ? "Will load " + path
                : $"Will load {path} — it is a {VehicleProfiles.DisplayName(cfg.Profile)} configuration, so that car is used.";
        }
        else _info.Text = "Invalid configuration: " + err;
    }

    private void Start() => GetTree().ChangeSceneToFile("res://scenes/Main.tscn");
}
