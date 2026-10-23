using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Godot;
using Simulator.Track;
using Simulator.UI;
using Simulator.Vehicle;
using VehicleDynamics.Core.Simulation;

namespace Simulator.Simulation;

/// <summary>
/// Autoload (project.godot): asynchronous session loading behind <see cref="LoadingScreen"/>.
/// 1. The loading screen is shown at once (on its own CanvasLayer, so it survives the scene change).
/// 2. The main scene, the selected track scene (+ its physics mesh) and the selected vehicle's visuals load on Godot's
///    loader threads (ResourceLoader.LoadThreadedRequest / LoadThreadedGetStatus): progress 0–65 %.
/// 3. The main scene is switched in (from the loaded PackedScene) with <see cref="GameSession.StagedLoad"/> set;
///    SimulationManager then builds the session in stages and reports 65–100 % (the racing-line optimisation runs on a
///    worker thread), and calls <see cref="Finish"/>.
/// Measured on Monaco + F1 before this existed: a 9.9 s frozen frame (2.3 s F1 model, 0.7 s track scene, 0.5 s
/// collision, 6.4 s racing-line optimisation).
/// </summary>
public partial class SceneManager : Node
{
    public static SceneManager? Instance { get; private set; }
    public const string MainScene = "res://scenes/Main.tscn";

    public bool IsLoading { get; private set; }
    private CanvasLayer _layer = null!;
    private LoadingScreen _screen = null!;
    private readonly List<Resource> _held = new();
    // Progress reported from a worker thread, applied on the main thread.
    private double _workerProgress = -1;
    private string _workerStatus = "";
    private readonly object _lock = new();

    public override void _Ready()
    {
        Instance = this;
        ProcessMode = ProcessModeEnum.Always;
        _layer = new CanvasLayer { Layer = 100, Visible = false };
        AddChild(_layer);
        _screen = GD.Load<PackedScene>("res://scenes/ui/LoadingScreen.tscn").Instantiate<LoadingScreen>();
        _layer.AddChild(_screen);
    }

    private double _longestFrame, _loadStart;
    private readonly Queue<double> _shotAt = new();
    private string? _shotDir;

    public void Show(string title)
    {
        IsLoading = true;
        _longestFrame = 0;
        _loadStart = Time.GetTicksMsec() / 1000.0;
        // Dev: `--loading-shots=DIR` saves the loading screen at 30 % and 85 %.
        if (OS.GetCmdlineUserArgs().FirstOrDefault(a => a.StartsWith("--loading-shots=")) is { } a)
        {
            _shotDir = a["--loading-shots=".Length..];
            _shotAt.Clear(); _shotAt.Enqueue(0.30); _shotAt.Enqueue(0.85);
        }
        _screen.Begin(title);
        _layer.Visible = true;
    }

    public void Report(double progress, string status) => _screen.SetProgress(progress, status);

    /// <summary>Thread-safe progress report (e.g. from the racing-line optimiser's worker thread).</summary>
    public void ReportFromWorker(double progress, string status)
    {
        lock (_lock) { _workerProgress = progress; _workerStatus = status; }
    }

    public override void _Process(double delta)
    {
        lock (_lock)
        {
            if (_workerProgress >= 0) _screen.SetProgress(_workerProgress, _workerStatus);
            _workerProgress = -1;
        }
        if (!IsLoading) return;
        _longestFrame = System.Math.Max(_longestFrame, delta);
        if (_shotDir != null && _shotAt.Count > 0 && _screen.Progress >= _shotAt.Peek())
        {
            double at = _shotAt.Dequeue();
            System.IO.Directory.CreateDirectory(_shotDir);
            GetViewport().GetTexture().GetImage().SavePng(System.IO.Path.Combine(_shotDir, $"loading_{at * 100:0}.png"));
        }
    }

    public void Finish()
    {
        GD.Print(System.FormattableString.Invariant(
            $"[load] loading screen: {Time.GetTicksMsec() / 1000.0 - _loadStart:F1} s total, longest frame {_longestFrame * 1000:F0} ms"));
        _screen.SetProgress(1, "Ready");
        _layer.Visible = false;
        IsLoading = false;
        _held.Clear();   // the session's nodes now hold what they use
    }

    public SignalAwaiter NextFrame() => ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);

    /// <summary>Menu → session: loading screen, threaded asset loading, then the main scene builds itself in stages.</summary>
    public async void StartSession()
    {
        if (IsLoading) return;
        string track = GameSession.TrackName;
        string vehicle = GameSession.VehicleProfile;
        if (GameSession.ConfigPath is { } cfg && ConfigSerializer.TryLoad(cfg, out var p, out _)) vehicle = p!.Profile;
        Show($"{track}  ·  {VehicleProfiles.DisplayName(vehicle)}");
        await NextFrame();   // draw the loading screen before any work

        var paths = new List<string> { MainScene, VehicleCatalog.Get(vehicle).VisualScene };
        paths.AddRange(TrackManager.ResourcesFor(track));
        if (!await LoadThreaded(paths, 0.0, 0.65, "Loading track and vehicle models")) return;

        GameSession.StagedLoad = true;
        Report(0.66, "Building the session");
        await NextFrame();
        GetTree().ChangeSceneToPacked(_held.OfType<PackedScene>().First(r => r.ResourcePath == MainScene));
    }

    /// <summary>
    /// Requests every path on Godot's loader threads and waits (frame by frame, never blocking) until all are loaded,
    /// mapping their mean progress onto [from, to]. The loaded resources are held until <see cref="Finish"/>.
    /// </summary>
    public async Task<bool> LoadThreaded(IReadOnlyList<string> paths, double from, double to, string status)
    {
        foreach (var path in paths)
        {
            // No sub-threads: with them, Godot's loader threads raced on the Monaco GLB's sub-resources ("another
            // resource is loaded from path …::ArrayMesh"). One loader thread per request is enough here.
            var err = ResourceLoader.LoadThreadedRequest(path, "", false);
            if (err != Error.Ok) return Fail($"Cannot load {path} ({err}).");
        }
        var progress = new Godot.Collections.Array();
        var done = new HashSet<string>();
        while (done.Count < paths.Count)
        {
            double sum = 0;
            foreach (var path in paths)
            {
                if (done.Contains(path)) { sum += 1; continue; }
                var st = ResourceLoader.LoadThreadedGetStatus(path, progress);
                switch (st)
                {
                    case ResourceLoader.ThreadLoadStatus.Loaded:
                        _held.Add(ResourceLoader.LoadThreadedGet(path));
                        done.Add(path);
                        sum += 1;
                        break;
                    case ResourceLoader.ThreadLoadStatus.InProgress:
                        sum += progress.Count > 0 ? (double)progress[0] : 0;
                        break;
                    default:
                        return Fail($"Loading {path} failed ({st}).");
                }
            }
            Report(from + (to - from) * sum / paths.Count, status);
            if (done.Count < paths.Count) await NextFrame();
        }
        return true;
    }

    private bool Fail(string message)
    {
        GD.PushError(message);
        Report(1, message);
        IsLoading = false;
        GetTree().CreateTimer(3).Timeout += () =>
        {
            _layer.Visible = false;
            GetTree().ChangeSceneToFile("res://scenes/MainMenu.tscn");
        };
        return false;
    }
}
