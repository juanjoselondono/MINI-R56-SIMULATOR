using Godot;
using Simulator.Race;
using Simulator.Simulation;

namespace Simulator.UI;

/// <summary>
/// Top-10 lap banner (top centre): rank and time, the live webcam preview (mirrored, like a selfie camera) with the
/// photo countdown, then the photo next to the time for a few seconds. Driving continues underneath.
/// </summary>
public partial class PodiumOverlay : PanelContainer
{
    private const double HoldSeconds = 6.0;
    private static readonly Vector2 PhotoSize = new(256, 144);

    private SimulationManager _sim = null!;
    private Label _headline = null!;
    private Label _time = null!;
    private Label _status = null!;
    private Control _frame = null!;
    private HBoxContainer _row = null!;
    private PodiumMoment? _shown;
    private object? _shownTexture;
    private double _finishedAt = -1;

    public void Init(SimulationManager sim)
    {
        _sim = sim;
        Theme = UiTheme.Get();
        MouseFilter = MouseFilterEnum.Ignore;
        _row = new HBoxContainer();
        _row.AddThemeConstantOverride("separation", 16);
        AddChild(_row);
        _frame = LeaderboardPanel.PhotoFrame(null, PhotoSize);
        _row.AddChild(_frame);
        var text = new VBoxContainer { Alignment = BoxContainer.AlignmentMode.Center };
        _headline = UiTheme.Label("", 26, UiTheme.Accent);
        _time = UiTheme.Label("", 34, UiTheme.Text);
        _time.AddThemeFontOverride("font", new SystemFont { FontNames = new[] { "monospace" } });
        _status = UiTheme.Label("", 14, UiTheme.Muted);
        text.AddChild(_headline);
        text.AddChild(_time);
        text.AddChild(_status);
        _row.AddChild(text);
        Visible = false;
    }

    public override void _Process(double delta)
    {
        if (_sim == null) return;
        var m = _sim.Leaderboard.Current;
        double now = Time.GetTicksMsec() / 1000.0;
        if (m != _shown)
        {
            _shown = m;
            _finishedAt = -1;
            _shownTexture = null;
            if (m != null)
            {
                _headline.Text = $"TOP 10 · P{m.Rank} ALL TIME";
                _time.Text = SimulationManager.FormatTime(m.Entry.Time);
            }
        }
        if (m == null) { Visible = false; return; }
        if (m.Finished && _finishedAt < 0) _finishedAt = now;
        if (_finishedAt >= 0 && now - _finishedAt > HoldSeconds) { Visible = false; return; }
        Visible = true;

        // Preview while the camera is open, then the photo.
        Texture2D? tex = m.Photo ?? (Texture2D?)_sim.Leaderboard.Webcam?.Preview;
        if (tex == null && !m.Finished) tex = _shownTexture as Texture2D;   // camera closed, photo not yet decoded
        bool live = m.Photo == null && tex != null;
        if (!ReferenceEquals(tex, _shownTexture))
        {
            _shownTexture = tex;
            var frame = LeaderboardPanel.PhotoFrame(tex, PhotoSize, mirror: live);
            _row.RemoveChild(_frame);
            _frame.QueueFree();
            _frame = frame;
            _row.AddChild(_frame);
            _row.MoveChild(_frame, 0);
        }
        _frame.Visible = tex != null || m.Countdown != null || m.Photo != null;
        _status.Text = m.Countdown is { } c ? $"Podium photo in {System.Math.Ceiling(c):0}…  smile!"
            : !m.Finished ? "Taking the photo…"
            : m.Photo != null ? "Saved to your Best Laps (F4)"
            : m.PhotoNote ?? "Saved to your Best Laps (F4)";
        _status.AddThemeColorOverride("font_color", m.Countdown != null ? UiTheme.Accent : UiTheme.Muted);
    }
}
