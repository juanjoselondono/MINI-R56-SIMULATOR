using Godot;

namespace Simulator.UI;

/// <summary>
/// Full-screen loading view (scenes/ui/LoadingScreen.tscn): background image, title, status line and progress bar.
/// The background is assets/ui/loading_bg.png when present (imported, or read directly if not imported yet);
/// otherwise the dark fallback colour. The bar eases towards the reported progress so stages do not jump.
/// </summary>
public partial class LoadingScreen : Control
{
    public const string BackgroundPath = "res://assets/ui/loading_bg.png";

    private TextureRect _background = null!;
    private Label _title = null!;
    private Label _status = null!;
    private ProgressBar _bar = null!;
    private double _target;
    /// <summary>Displayed progress [0, 1].</summary>
    public double Progress => _bar.Value / 100.0;

    public override void _Ready()
    {
        _background = GetNode<TextureRect>("Background");
        _title = GetNode<Label>("Bottom/Title");
        _status = GetNode<Label>("Bottom/Status");
        _bar = GetNode<ProgressBar>("Bottom/Progress");
        _background.Texture = LoadBackground();
    }

    private static Texture2D? LoadBackground()
    {
        if (ResourceLoader.Exists(BackgroundPath)) return GD.Load<Texture2D>(BackgroundPath);
        if (!FileAccess.FileExists(BackgroundPath)) return null;
        var image = Image.LoadFromFile(ProjectSettings.GlobalizePath(BackgroundPath));
        return image == null || image.IsEmpty() ? null : ImageTexture.CreateFromImage(image);
    }

    public void Begin(string title)
    {
        _title.Text = title;
        _status.Text = "";
        _target = 0;
        _bar.Value = 0;
    }

    /// <summary>Progress in [0, 1] and the stage being worked on.</summary>
    public void SetProgress(double progress, string status)
    {
        _target = Mathf.Clamp(progress, _target, 1);
        if (!string.IsNullOrEmpty(status)) _status.Text = status;
    }

    public override void _Process(double delta)
    {
        // Ease towards the target (about 90 % of the gap per 0.25 s), never backwards.
        double k = 1 - System.Math.Exp(-delta * 9);
        _bar.Value = Mathf.Lerp((float)_bar.Value, (float)(_target * 100), (float)k);
        if (_target >= 1) _bar.Value = 100;
    }
}
