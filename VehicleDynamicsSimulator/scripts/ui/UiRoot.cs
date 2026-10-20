using System.Linq;
using Godot;
using Simulator.Simulation;
using Simulator.Vehicle;

namespace Simulator.UI;

/// <summary>Hosts the HUD and the panels; toggles them with Tab / F2 / F3 / F1.</summary>
public partial class UiRoot : CanvasLayer
{
    [Export] public NodePath SimulationPath { get; set; } = new("../SimulationManager");

    private SimulationManager _sim = null!;
    private VehicleSetupPanel _setup = null!;
    private TelemetryPanel _telemetry = null!;
    private RunComparisonPanel _comparison = null!;
    private PanelContainer _help = null!;

    public override void _Ready()
    {
        _sim = GetNode<SimulationManager>(SimulationPath);
        // SimulationManager loads the track in its _Ready, which runs before ours (it is earlier in the tree).
        var hud = new RaceHud();
        AddChild(hud);
        hud.Init(_sim);

        _setup = new VehicleSetupPanel();
        AddChild(_setup);
        _setup.Init(_sim);
        Dock(_setup, left: true);

        _telemetry = new TelemetryPanel();
        AddChild(_telemetry);
        _telemetry.Init(_sim);
        Dock(_telemetry, left: false);

        _comparison = new RunComparisonPanel();
        AddChild(_comparison);
        _comparison.Init(_sim);
        _comparison.SetAnchorsPreset(Control.LayoutPreset.Center);
        _comparison.SetAnchorsAndOffsetsPreset(Control.LayoutPreset.Center, Control.LayoutPresetMode.KeepSize);
        _comparison.OffsetLeft = -380; _comparison.OffsetRight = 380; _comparison.OffsetTop = -330; _comparison.OffsetBottom = 330;

        _help = BuildHelp();
        AddChild(_help);

        _setup.Visible = _telemetry.Visible = _comparison.Visible = _help.Visible = false;

        // `-- --show-panels=setup,telemetry,compare,help` (screenshots / self-test).
        foreach (var a in OS.GetCmdlineUserArgs())
        {
            if (!a.StartsWith("--show-panels=")) continue;
            var set = a["--show-panels=".Length..].Split(',');
            ShowPanels(set.Contains("setup"), set.Contains("telemetry"), set.Contains("compare"));
            _help.Visible = set.Contains("help");
        }
    }

    private static void Dock(Control c, bool left)
    {
        c.AnchorTop = 0; c.AnchorBottom = 1;
        c.AnchorLeft = left ? 0 : 1; c.AnchorRight = left ? 0 : 1;
        float w = c.CustomMinimumSize.X;
        c.OffsetTop = 12; c.OffsetBottom = -12;
        c.OffsetLeft = left ? 12 : -w - 12;
        c.OffsetRight = left ? w + 12 : -12;
    }

    private PanelContainer BuildHelp()
    {
        var v = new VBoxContainer();
        v.AddChild(UiTheme.Label("CONTROLS", 18, UiTheme.Accent));
        string[] lines =
        {
            "W / ↑ / RT          throttle",
            "S / ↓ / LT          brake",
            "A D / ← → / stick   steer (speed-sensitive input assist)",
            "E / RB, Q / LB      shift up / down (manual)",
            "T                   toggle auto-shift (model.m gear rule)",
            "R / Back            restart: grid, at rest, laps cleared",
            "Backspace / B       recover to track centreline (lap invalid)",
            "P / Start           pause / resume the simulation clock",
            "C / Y               camera: chase / bonnet / cockpit",
            "H                   headlights",
            "F5 / X              start / stop telemetry recording",
            "Tab                 vehicle setup panel",
            "F2                  telemetry dashboard",
            "F3                  run comparison",
            "Esc                 main menu",
            "",
            "No reverse gear exists in the model: use Backspace after a head-on stop.",
            "Downshifting at speed above the money-shift limit destroys the engine (seed-game rule).",
        };
        foreach (var l in lines)
        {
            var lab = UiTheme.Label(l, 14);
            lab.AddThemeFontOverride("font", new SystemFont { FontNames = new[] { "monospace" } });
            v.AddChild(lab);
        }
        var p = UiTheme.Panel(v);
        p.SetAnchorsAndOffsetsPreset(Control.LayoutPreset.Center, Control.LayoutPresetMode.Minsize);
        return p;
    }

    public override void _Process(double delta)
    {
        if (UiFocus.IsTyping(GetViewport())) return;
        if (Input.IsActionJustPressed(InputSetup.ToggleSetup))
        {
            _setup.Visible = !_setup.Visible;
            if (_setup.Visible) _setup.Refresh();
        }
        if (Input.IsActionJustPressed(InputSetup.ToggleTelemetry)) _telemetry.Visible = !_telemetry.Visible;
        if (Input.IsActionJustPressed(InputSetup.ToggleComparison)) _comparison.Visible = !_comparison.Visible;
        if (Input.IsActionJustPressed(InputSetup.ToggleHelp)) _help.Visible = !_help.Visible;
    }

    /// <summary>For the self-test and screenshots.</summary>
    public void ShowPanels(bool setup, bool telemetry, bool comparison)
    {
        _setup.Visible = setup;
        if (setup) _setup.Refresh();
        _telemetry.Visible = telemetry;
        _comparison.Visible = comparison;
    }
}
