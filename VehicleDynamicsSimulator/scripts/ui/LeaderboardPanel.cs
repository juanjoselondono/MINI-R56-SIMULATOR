using System.Collections.Generic;
using System.IO;
using Godot;
using Simulator.Race;
using Simulator.Simulation;
using VehicleDynamics.Core.Simulation;

namespace Simulator.UI;

/// <summary>
/// Best Laps (F4): the all-time top 10 of the current track and vehicle, each time with its podium photo (or an empty
/// frame when none was taken), the date and a mark for laps driven with a modified setup. Also holds the podium-photo
/// setting (webcam on/off).
/// </summary>
public partial class LeaderboardPanel : PanelContainer
{
    public static readonly Vector2 Thumb = new(96, 54);

    private SimulationManager _sim = null!;
    private VBoxContainer _rows = null!;
    private ScrollContainer _scroll = null!;
    private Label _title = null!;
    private Label _note = null!;
    private CheckBox _photos = null!;
    private string _shownKey = "";
    /// <summary>Thumbnails by file; released in _ExitTree (on the main thread: GC finalizers at exit run elsewhere).</summary>
    private readonly Dictionary<string, Texture2D?> _cache = new();

    public void Init(SimulationManager sim)
    {
        _sim = sim;
        Theme = UiTheme.Get();
        CustomMinimumSize = new Vector2(520, 0);
        var v = new VBoxContainer();
        v.AddThemeConstantOverride("separation", 6);
        AddChild(v);
        _title = UiTheme.Label("", 18, UiTheme.Accent);
        v.AddChild(_title);
        _photos = new CheckBox
        {
            Text = "Podium photo with the webcam for a top-10 lap", FocusMode = FocusModeEnum.None,
            ButtonPressed = LeaderboardService.PhotosEnabled,
            TooltipText = "After a 3-second countdown with a live preview. The camera is opened only for the photo; photos stay on this computer.",
        };
        _photos.Toggled += on => LeaderboardService.PhotosEnabled = on;
        foreach (var style in new[] { "normal", "pressed", "hover", "hover_pressed", "focus" })
            _photos.AddThemeStyleboxOverride(style, new StyleBoxEmpty());
        v.AddChild(_photos);
        _note = UiTheme.Label("", 11, UiTheme.Muted);
        _note.AutowrapMode = TextServer.AutowrapMode.WordSmart;
        v.AddChild(_note);
        v.AddChild(new HSeparator());
        _rows = new VBoxContainer { SizeFlagsHorizontal = SizeFlags.ExpandFill };
        _rows.AddThemeConstantOverride("separation", 4);
        _scroll = new ScrollContainer { HorizontalScrollMode = ScrollContainer.ScrollMode.Disabled };
        _scroll.AddChild(_rows);
        v.AddChild(_scroll);
        sim.Leaderboard.Changed += () => _shownKey = "";
    }

    public override void _Process(double delta)
    {
        if (!Visible || _sim == null) return;
        string key = $"{_sim.Track.TrackName}|{_sim.Vehicle.Parameters.Profile}|{_sim.Leaderboard.Board.All.Count}|{_sim.Leaderboard.LatestId}";
        if (key != _shownKey) Rebuild(key);
        // As tall as the list, but never taller than the screen: scroll beyond that.
        float room = GetViewportRect().Size.Y - 70 - 40 - (Size.Y - _scroll.Size.Y);
        _scroll.CustomMinimumSize = new Vector2(0, Mathf.Max(80, Mathf.Min(_rows.GetCombinedMinimumSize().Y, room)));
    }

    private void Rebuild(string key)
    {
        _shownKey = key;
        var service = _sim.Leaderboard;
        string track = _sim.Track.TrackName, vehicle = _sim.Vehicle.Parameters.Profile;
        _title.Text = $"BEST LAPS · {track} · {VehicleProfiles.DisplayName(vehicle)}";
        _note.Text = !service.Enabled ? "Test run: laps are not recorded."
            : $"All-time top {Leaderboard.Size} valid laps of this track and car · {service.FilePath}";
        foreach (var c in _rows.GetChildren()) { _rows.RemoveChild(c); c.QueueFree(); }
        var top = service.Board.Top(track, vehicle);
        if (top.Count == 0) _rows.AddChild(UiTheme.Label("No laps yet: complete a valid lap to set the first time.", 14, UiTheme.Muted));
        for (int i = 0; i < top.Count; i++) _rows.AddChild(Row(i + 1, top[i], top[i].Id == service.LatestId));
    }

    private Control Row(int rank, LeaderboardEntry e, bool latest)
    {
        var h = new HBoxContainer();
        h.AddThemeConstantOverride("separation", 12);
        var pos = UiTheme.Label($"P{rank}", 18, rank <= 3 ? UiTheme.Accent : UiTheme.Text);
        pos.CustomMinimumSize = new Vector2(44, 0);
        pos.VerticalAlignment = VerticalAlignment.Center;
        h.AddChild(pos);
        h.AddChild(PhotoFrame(PhotoTexture(e), Thumb));
        var text = new VBoxContainer { SizeFlagsHorizontal = SizeFlags.ExpandFill, Alignment = BoxContainer.AlignmentMode.Center };
        var time = UiTheme.Label(SimulationManager.FormatTime(e.Time), 22, latest ? UiTheme.Good : UiTheme.Text);
        time.AddThemeFontOverride("font", new SystemFont { FontNames = new[] { "monospace" } });
        text.AddChild(time);
        string tags = (latest ? "new · " : "") + e.Date.ToString("yyyy-MM-dd HH:mm") + (e.CustomSetup ? " · custom setup" : "");
        text.AddChild(UiTheme.Label(tags, 11, UiTheme.Muted));
        h.AddChild(text);
        return h;
    }

    /// <summary>A fixed-size frame with the photo (kept aspect, centred) or an empty placeholder.</summary>
    public static Control PhotoFrame(Texture2D? texture, Vector2 size, bool mirror = false)
    {
        var frame = new PanelContainer { CustomMinimumSize = size };
        var style = new StyleBoxFlat { BgColor = new Color(0.12f, 0.13f, 0.16f) };
        style.SetCornerRadiusAll(4);
        frame.AddThemeStyleboxOverride("panel", style);
        if (texture != null)
            frame.AddChild(new TextureRect
            {
                Texture = texture, ExpandMode = TextureRect.ExpandModeEnum.IgnoreSize, FlipH = mirror,
                StretchMode = TextureRect.StretchModeEnum.KeepAspectCovered, CustomMinimumSize = size, ClipContents = true,
            });
        else
        {
            var l = UiTheme.Label("no photo", 11, UiTheme.Muted);
            l.HorizontalAlignment = HorizontalAlignment.Center;
            l.VerticalAlignment = VerticalAlignment.Center;
            frame.AddChild(l);
        }
        return frame;
    }

    public override void _ExitTree()
    {
        foreach (var t in _cache.Values) t?.Dispose();
        _cache.Clear();
    }

    private Texture2D? PhotoTexture(LeaderboardEntry e)
    {
        if (e.Photo == null) return null;
        string path = _sim.Leaderboard.PhotoPath(e);
        if (_cache.TryGetValue(path, out var t)) return t;
        Texture2D? tex = null;
        if (File.Exists(path))
        {
            var img = Image.LoadFromFile(path);
            if (img != null && !img.IsEmpty()) tex = ImageTexture.CreateFromImage(img);
        }
        _cache[path] = tex;
        return tex;
    }
}
