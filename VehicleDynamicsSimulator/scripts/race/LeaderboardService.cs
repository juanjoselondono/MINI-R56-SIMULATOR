using System;
using System.IO;
using System.Threading.Tasks;
using Godot;
using VehicleDynamics.Core.Simulation;

namespace Simulator.Race;

/// <summary>A top-10 lap that just happened: shown by the podium overlay while its photo is taken.</summary>
public sealed class PodiumMoment
{
    public required LeaderboardEntry Entry { get; init; }
    public required int Rank { get; init; }
    public double StartedAt { get; init; } = Time.GetTicksMsec() / 1000.0;
    /// <summary>Seconds until the photo while the countdown runs; null otherwise.</summary>
    public double? Countdown { get; set; }
    public ImageTexture? Photo { get; set; }
    /// <summary>Why there is no photo (webcam off, none found, no frame), or null.</summary>
    public string? PhotoNote { get; set; }
    public bool Finished { get; set; }
}

/// <summary>
/// All-time best laps of the game (<see cref="Leaderboard"/>, user://leaderboard.json): every valid lap is offered;
/// one that makes its track + vehicle top 10 is stored and, with podium photos on, the webcam takes a photo after a
/// 3-second countdown (user://leaderboard_photos/&lt;entry id&gt;.png), shown next to the time. Command-line test runs
/// never touch the player's board (the self-test uses its own file, without the camera).
/// </summary>
public partial class LeaderboardService : Node
{
    public static string DefaultPath => ProjectSettings.GlobalizePath("user://leaderboard.json");
    public static string DefaultPhotosDirectory => ProjectSettings.GlobalizePath("user://leaderboard_photos");
    private const string SettingsFile = "user://settings.cfg";
    public const double CountdownSeconds = 3.0;

    public string FilePath { get; init; } = DefaultPath;
    public string PhotosDirectory { get; init; } = DefaultPhotosDirectory;
    /// <summary>False in command-line test runs: laps are not recorded.</summary>
    public bool Enabled { get; init; } = true;
    /// <summary>The camera may be used (false in test runs); the player's choice is <see cref="PhotosEnabled"/>.</summary>
    public bool CameraAllowed { get; init; } = true;

    public Leaderboard Board { get; private set; } = new();
    public WebcamCapture? Webcam { get; private set; }
    public PodiumMoment? Current { get; private set; }
    /// <summary>Id of the most recent entry (highlighted in the list).</summary>
    public string? LatestId { get; private set; }
    public event Action? Changed;

    private static bool? _photosEnabled;

    /// <summary>Player setting (privacy): take a webcam photo for top-10 laps. Stored in user://settings.cfg; default on.</summary>
    public static bool PhotosEnabled
    {
        get
        {
            if (_photosEnabled is { } on) return on;
            var cfg = new ConfigFile();
            _photosEnabled = cfg.Load(SettingsFile) != Error.Ok || cfg.GetValue("leaderboard", "webcam_photos", true).AsBool();
            return _photosEnabled.Value;
        }
        set
        {
            _photosEnabled = value;
            var cfg = new ConfigFile();
            cfg.Load(SettingsFile);
            cfg.SetValue("leaderboard", "webcam_photos", value);
            cfg.Save(SettingsFile);
        }
    }

    public override void _Ready()
    {
        Board = Leaderboard.Load(FilePath, out var err);
        if (err != null) GD.PushWarning($"Leaderboard {FilePath} could not be read ({err}); it was kept aside and a new board started.");
        if (Enabled && CameraAllowed)
        {
            Webcam = new WebcamCapture { Name = "Webcam" };
            AddChild(Webcam);
        }
    }

    public override void _ExitTree()
    {
        // Release GPU-backed textures here, on the main thread (not from a GC finalizer at exit).
        Current?.Photo?.Dispose();
        Current = null;
    }

    public string PhotoPath(LeaderboardEntry e) => Path.Combine(PhotosDirectory, e.Photo ?? "");

    /// <summary>
    /// Offers a completed lap. Invalid laps and laps outside the top 10 are ignored. Returns the moment (rank, photo
    /// state) when the lap made the board.
    /// </summary>
    public PodiumMoment? Record(string track, VehicleParameters vehicle, double time, bool valid)
    {
        if (!Enabled || !valid) return null;
        bool custom = ConfigSerializer.ToJson(vehicle) != ConfigSerializer.ToJson(VehicleProfiles.Create(vehicle.Profile));
        if (Board.Submit(track, vehicle.Profile, time, DateTime.Now, custom) is not { } r) return null;
        foreach (var d in r.Dropped) DeletePhoto(d);
        Save();
        LatestId = r.Entry.Id;
        var moment = new PodiumMoment { Entry = r.Entry, Rank = r.Rank };
        Current = moment;
        Changed?.Invoke();
        if (Webcam != null && PhotosEnabled) _ = TakePhotoAsync(moment);
        else
        {
            moment.PhotoNote = Webcam == null ? null : "podium photos are off (setting in the Best Laps panel, F4)";
            moment.Finished = true;
        }
        return moment;
    }

    private PodiumMoment? _capturing;

    /// <summary>
    /// Countdown, then the first usable frame. One camera, one photo at a time: a newer top lap takes over a capture in
    /// progress (the older lap keeps its place on the board, without a photo).
    /// </summary>
    private async Task TakePhotoAsync(PodiumMoment m)
    {
        var cam = Webcam!;
        if (_capturing is { } older) older.PhotoNote = "no photo: a newer top lap took the camera";
        _capturing = m;
        if (cam.Preview == null && !cam.Begin())
        {
            m.PhotoNote = "no photo: " + cam.LastError;
            m.Finished = true;
            _capturing = null;
            return;
        }
        string feed = cam.FeedName ?? "webcam";
        double t0 = Now;
        while (Now - t0 < CountdownSeconds)
        {
            m.Countdown = CountdownSeconds - (Now - t0);
            await NextFrame();
            if (_capturing != m) { m.Countdown = null; m.Finished = true; return; }
        }
        m.Countdown = null;
        Image? img = null;
        for (double t1 = Now; img == null && Now - t1 < 2.0; await NextFrame())
        {
            if (_capturing != m) { m.Finished = true; return; }
            img = cam.Grab();
        }
        cam.Stop();
        _capturing = null;
        if (img == null)
        {
            m.PhotoNote = "no photo: the webcam delivered no image";
            m.Finished = true;
            return;
        }
        try
        {
            Directory.CreateDirectory(PhotosDirectory);
            string file = m.Entry.Id + ".png";
            var err = img.SavePng(Path.Combine(PhotosDirectory, file));
            if (err != Error.Ok) throw new IOException(err.ToString());
            // The lap may have been pushed off the board meanwhile (several top laps within seconds).
            if (Board.Find(m.Entry.Id) is { } entry)
            {
                entry.Photo = file;
                Save();
            }
            else DeletePhoto(new LeaderboardEntry { Photo = file });
            m.Photo = ImageTexture.CreateFromImage(img);
            GD.Print($"Podium photo saved: {Path.Combine(PhotosDirectory, file)} ({img.GetWidth()}×{img.GetHeight()}, {feed})");
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            m.PhotoNote = "photo not saved: " + ex.Message;
        }
        m.Finished = true;
        Changed?.Invoke();
    }

    private void Save()
    {
        try
        {
            Board.Save(FilePath);
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            GD.PushWarning("Leaderboard not saved: " + ex.Message);
        }
    }

    private void DeletePhoto(LeaderboardEntry e)
    {
        if (e.Photo == null) return;
        try { File.Delete(Path.Combine(PhotosDirectory, e.Photo)); }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException) { }
    }

    private static double Now => Time.GetTicksMsec() / 1000.0;

    private SignalAwaiter NextFrame() => ToSignal(GetTree(), SceneTree.SignalName.ProcessFrame);
}
