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
        if (args.Contains("--via-loading-screen"))
        {
            // Dev: the interactive start path (loading screen, background loading) without clicking.
            if (args.FirstOrDefault(a => a.StartsWith("--track=")) is { } trackArg) GameSession.TrackName = trackArg["--track=".Length..];
            if (args.FirstOrDefault(a => a.StartsWith("--vehicle=")) is { } vehicleArg) GameSession.VehicleProfile = vehicleArg["--vehicle=".Length..];
            Callable.From(() => SceneManager.Instance!.StartSession()).CallDeferred();
            return;
        }
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
            b.Toggled += on => { if (on) { GameSession.VehicleProfile = id; RefreshConfigs(); } };
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

        v.AddChild(UiTheme.Label("VEHICLE SETUP", 14, UiTheme.Accent));
        _config = new OptionButton { CustomMinimumSize = new Vector2(620, 36), FocusMode = FocusModeEnum.None };
        _config.ItemSelected += _ => Validate();
        string dir = SimulationManager.ConfigsDirectory;
        v.AddChild(_config);
        _info = UiTheme.Label("", 12, UiTheme.Muted);
        v.AddChild(_info);
        v.AddChild(UiTheme.Label($"Setups (one file and folder per car): {dir}\nTelemetry exports: {SimulationManager.RunsDirectory}", 12, UiTheme.Muted));
        v.AddChild(UiTheme.Button("Quit", () => GetTree().Quit()));

        var centre = new CenterContainer();
        centre.SetAnchorsPreset(LayoutPreset.FullRect);
        centre.AddChild(v);
        AddChild(centre);
        var credit = UiTheme.Credit();
        AddChild(credit);
        credit.SetAnchorsAndOffsetsPreset(LayoutPreset.BottomRight, LayoutPresetMode.Minsize, 32);
        credit.GrowHorizontal = GrowDirection.Begin;
        credit.GrowVertical = GrowDirection.Begin;
        RefreshConfigs();

        // Dev: `-- --menu-shot=PATH` saves the menu as PNG and quits (docs screenshots).
        if (args.FirstOrDefault(a => a.StartsWith("--menu-shot=")) is { } shot)
            GetTree().CreateTimer(1.0).Timeout += () =>
            {
                GetViewport().GetTexture().GetImage().SavePng(shot["--menu-shot=".Length..]);
                GetTree().Quit();
            };
    }

    /// <summary>
    /// The selected car's setups only: its current setup (setup_&lt;key&gt;.json), its factory defaults and the named
    /// setups in its own folder. Another car's files are never listed.
    /// </summary>
    private void RefreshConfigs()
    {
        var store = SimulationManager.Setups;
        string profile = GameSession.VehicleProfile;
        _config.Clear();
        _config.AddItem(File.Exists(store.ActivePath(profile))
            ? $"Current setup ({Path.GetFileName(store.ActivePath(profile))})"
            : "Current setup (none saved yet: defaults)");
        _config.AddItem("Factory defaults");
        foreach (var name in store.ListNamed(profile)) _config.AddItem(name);
        _config.Select(0);
        Validate();
    }

    private void Validate()
    {
        GameSession.ConfigPath = null;
        GameSession.FactoryDefaults = false;
        var store = SimulationManager.Setups;
        string profile = GameSession.VehicleProfile;
        string vehicle = VehicleProfiles.DisplayName(profile);
        string defaults = profile == VehicleProfiles.MiniR56
            ? "Mini: model.m parameters with the game's mass 1150 kg and μ 1.05."
            : $"{vehicle}: provisional parameters (no MATLAB model for this car).";
        if (_config.Selected <= 0)
        {
            string active = store.ActivePath(profile);
            if (!File.Exists(active)) { _info.Text = defaults + " Apply Setup in the setup panel (Tab) saves this car's setup."; return; }
            store.LoadActive(profile, out var err);
            _info.Text = err == null ? "Will load " + active : $"{err} — the defaults will be used.";
            return;
        }
        if (_config.Selected == 1)
        {
            GameSession.FactoryDefaults = true;
            _info.Text = defaults + " (The saved setup is kept until you apply another one.)";
            return;
        }
        string path = store.NamedPath(profile, _config.GetItemText(_config.Selected));
        try
        {
            VehicleSetupStore.LoadFor(profile, path);
            GameSession.ConfigPath = path;
            _info.Text = "Will load " + path;
        }
        catch (System.Exception ex) when (ex is System.ArgumentException or IOException or System.UnauthorizedAccessException)
        {
            _info.Text = "Invalid setup: " + ex.Message;
        }
    }

    /// <summary>
    /// Menu: loading screen + background loading (SceneManager). Command-line test modes (deferred call from _Ready) load
    /// synchronously, so their timing and results stay deterministic.
    /// </summary>
    private void Start()
    {
        if (_config != null && SceneManager.Instance != null) SceneManager.Instance.StartSession();
        else GetTree().ChangeSceneToFile("res://scenes/Main.tscn");
    }
}
