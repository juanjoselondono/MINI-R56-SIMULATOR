using Godot;

namespace Simulator.UI;

/// <summary>Dark, semi-transparent dashboard theme shared by all panels.</summary>
public static class UiTheme
{
    public static readonly Color Accent = new(1.0f, 0.62f, 0.2f);
    public static readonly Color Text = new(0.92f, 0.93f, 0.95f);
    public static readonly Color Muted = new(0.62f, 0.65f, 0.7f);
    public static readonly Color Good = new(0.45f, 0.85f, 0.5f);
    public static readonly Color Bad = new(1.0f, 0.38f, 0.35f);
    public static readonly Color Ext = new(0.55f, 0.75f, 1.0f);

    private static Theme? _theme;

    public static Theme Get()
    {
        if (_theme != null) return _theme;
        var t = new Theme { DefaultFontSize = 14 };
        var panel = new StyleBoxFlat
        {
            BgColor = new Color(0.06f, 0.07f, 0.09f, 0.88f),
            BorderColor = new Color(1, 1, 1, 0.08f),
            CornerRadiusTopLeft = 8, CornerRadiusTopRight = 8, CornerRadiusBottomLeft = 8, CornerRadiusBottomRight = 8,
            ContentMarginLeft = 14, ContentMarginRight = 14, ContentMarginTop = 12, ContentMarginBottom = 12,
        };
        panel.SetBorderWidthAll(1);
        t.SetStylebox("panel", "PanelContainer", panel);

        var btn = new StyleBoxFlat
        {
            BgColor = new Color(0.16f, 0.17f, 0.2f),
            CornerRadiusTopLeft = 5, CornerRadiusTopRight = 5, CornerRadiusBottomLeft = 5, CornerRadiusBottomRight = 5,
            ContentMarginLeft = 10, ContentMarginRight = 10, ContentMarginTop = 5, ContentMarginBottom = 5,
        };
        var btnHover = (StyleBoxFlat)btn.Duplicate();
        btnHover.BgColor = new Color(0.24f, 0.25f, 0.3f);
        var btnPressed = (StyleBoxFlat)btn.Duplicate();
        btnPressed.BgColor = new Color(0.55f, 0.33f, 0.1f);
        var btnDisabled = (StyleBoxFlat)btn.Duplicate();
        btnDisabled.BgColor = new Color(0.1f, 0.1f, 0.12f);
        t.SetStylebox("normal", "Button", btn);
        t.SetStylebox("hover", "Button", btnHover);
        t.SetStylebox("pressed", "Button", btnPressed);
        t.SetStylebox("disabled", "Button", btnDisabled);
        t.SetStylebox("focus", "Button", new StyleBoxEmpty());
        t.SetColor("font_color", "Button", Text);
        t.SetColor("font_disabled_color", "Button", new Color(0.4f, 0.4f, 0.45f));
        t.SetColor("font_color", "Label", Text);
        t.SetColor("font_color", "CheckBox", Text);
        _theme = t;
        return t;
    }

    public static Label Label(string text, int size = 14, Color? color = null)
    {
        var l = new Label { Text = text };
        l.AddThemeFontSizeOverride("font_size", size);
        if (color is { } c) l.AddThemeColorOverride("font_color", c);
        return l;
    }

    public static Button Button(string text, System.Action onPressed, string? tooltip = null)
    {
        var b = new Button { Text = text, FocusMode = Control.FocusModeEnum.None, TooltipText = tooltip ?? "" };
        b.Pressed += onPressed;
        return b;
    }

    public static PanelContainer Panel(Control content)
    {
        var p = new PanelContainer { Theme = Get() };
        p.AddChild(content);
        return p;
    }
}
