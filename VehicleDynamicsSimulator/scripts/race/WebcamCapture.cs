using System.Linq;
using Godot;
using Godot.Collections;

namespace Simulator.Race;

/// <summary>
/// Webcam access for the podium photo, through Godot's CameraServer: a <see cref="CameraFeed"/> per device, shown and
/// read back through a <see cref="CameraTexture"/>. (VideoStreamPlayer cannot be used: it only plays video files and
/// streams, it has no camera input.) The camera is opened only for a photo (<see cref="Begin"/> … <see cref="Stop"/>):
/// device monitoring alone lists the cameras without streaming frames, so the camera light stays off while driving.
/// </summary>
public partial class WebcamCapture : Node
{
    /// <summary>Live image while open (for a preview); null when closed.</summary>
    public CameraTexture? Preview { get; private set; }
    public string? FeedName => _feed?.GetName();
    public string? LastError { get; private set; }

    private CameraFeed? _feed;

    public override void _Ready() => CameraServer.MonitoringFeeds = true;   // device list fills in asynchronously

    public override void _ExitTree()
    {
        Stop();
        CameraServer.MonitoringFeeds = false;
    }

    public static bool HasCamera => CameraServer.GetFeedCount() > 0;

    /// <summary>Opens the first camera (front-facing preferred) at up to 1280 px wide. False when there is none.</summary>
    public bool Begin()
    {
        Stop();
        LastError = null;
        var feeds = CameraServer.Feeds();
        var feed = feeds.FirstOrDefault(f => f.GetPosition() == CameraFeed.FeedPosition.Front) ?? feeds.FirstOrDefault();
        if (feed == null)
        {
            LastError = "no webcam found";
            return false;
        }
        SelectFormat(feed);
        feed.FeedIsActive = true;
        if (!feed.FeedIsActive)
        {
            LastError = $"could not open '{feed.GetName()}'";
            return false;
        }
        _feed = feed;
        Preview = new CameraTexture { CameraFeedId = feed.GetId(), WhichFeed = CameraServer.FeedImage.RgbaImage };
        return true;
    }

    /// <summary>
    /// Current frame as an RGB image at most 720 px high, or null while no frame has arrived yet (the first frames of a
    /// freshly opened camera are empty or black).
    /// </summary>
    public Image? Grab()
    {
        var img = Preview?.GetImage();
        if (img == null || img.IsEmpty()) return null;
        if (img.GetFormat() != Image.Format.Rgb8) img.Convert(Image.Format.Rgb8);
        if (MeanBrightness(img) < 0.01) return null;
        if (img.GetHeight() > 720) img.Resize(img.GetWidth() * 720 / img.GetHeight(), 720, Image.Interpolation.Bilinear);
        return img;
    }

    public void Stop()
    {
        if (_feed != null) _feed.FeedIsActive = false;
        _feed = null;
        Preview?.Dispose();
        Preview = null;
    }

    /// <summary>The largest format up to 1280 px wide (enough for a photo, light to decode); the driver default otherwise.</summary>
    private static void SelectFormat(CameraFeed feed)
    {
        var formats = feed.GetFormats();
        int best = -1, bestWidth = 0;
        for (int i = 0; i < formats.Count; i++)
        {
            var f = formats[i].AsGodotDictionary();
            int w = f.TryGetValue("width", out var wv) ? wv.AsInt32() : 0;
            if (w <= 1280 && w > bestWidth) { best = i; bestWidth = w; }
        }
        if (best >= 0) feed.SetFormat(best, new Dictionary());
    }

    private static double MeanBrightness(Image img)
    {
        double sum = 0;
        int n = 0;
        for (int y = 0; y < 8; y++)
            for (int x = 0; x < 8; x++, n++)
                sum += img.GetPixel((2 * x + 1) * img.GetWidth() / 16, (2 * y + 1) * img.GetHeight() / 16).V;
        return sum / n;
    }
}
