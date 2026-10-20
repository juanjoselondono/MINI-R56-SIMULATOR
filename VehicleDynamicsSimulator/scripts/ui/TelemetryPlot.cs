using System;
using System.Collections.Generic;
using System.Globalization;
using Godot;

namespace Simulator.UI;

public sealed record PlotSeries(string Name, Color Color, IReadOnlyList<Vector2> Points);

/// <summary>Minimal line plot (custom drawing): auto-scaled axes, legend, optional fixed y-range.</summary>
public partial class TelemetryPlot : Control
{
    public string Title { get; set; } = "";
    public string XLabel { get; set; } = "";
    public List<PlotSeries> Series { get; } = new();
    public Vector2? FixedY { get; set; }
    /// <summary>Equal axis scaling and a circle of this radius (used by the g-g diagram).</summary>
    public float? Circle { get; set; }
    public bool Scatter { get; set; }

    public TelemetryPlot()
    {
        CustomMinimumSize = new Vector2(300, 130);
        MouseFilter = MouseFilterEnum.Ignore;
    }

    public override void _Draw()
    {
        var font = ThemeDB.FallbackFont;
        var r = new Rect2(Vector2.Zero, Size);
        DrawRect(r, new Color(0, 0, 0, 0.35f));
        var plot = new Rect2(44, 18, Size.X - 52, Size.Y - 36);
        DrawString(font, new Vector2(6, 13), Title, HorizontalAlignment.Left, -1, 12, UiTheme.Accent);

        float xmin = float.MaxValue, xmax = float.MinValue, ymin = float.MaxValue, ymax = float.MinValue;
        foreach (var s in Series)
            foreach (var p in s.Points)
            {
                xmin = Math.Min(xmin, p.X); xmax = Math.Max(xmax, p.X);
                ymin = Math.Min(ymin, p.Y); ymax = Math.Max(ymax, p.Y);
            }
        if (xmin > xmax) { DrawString(font, plot.Position + new Vector2(8, plot.Size.Y / 2), "no samples yet", HorizontalAlignment.Left, -1, 12, UiTheme.Muted); return; }
        if (FixedY is { } fy) { ymin = fy.X; ymax = fy.Y; }
        if (Circle is { } c)
        {
            float m = Math.Max(c * 1.15f, Math.Max(Math.Max(Math.Abs(xmin), Math.Abs(xmax)), Math.Max(Math.Abs(ymin), Math.Abs(ymax))));
            xmin = ymin = -m; xmax = ymax = m;
            float side = Math.Min(plot.Size.X, plot.Size.Y);
            plot = new Rect2(plot.Position + new Vector2((plot.Size.X - side) / 2, 0), new Vector2(side, side));
        }
        if (xmax - xmin < 1e-6f) xmax = xmin + 1;
        if (ymax - ymin < 1e-6f) { ymax += 0.5f; ymin -= 0.5f; }

        Vector2 Map(Vector2 p) => new(
            plot.Position.X + (p.X - xmin) / (xmax - xmin) * plot.Size.X,
            plot.End.Y - (p.Y - ymin) / (ymax - ymin) * plot.Size.Y);

        var grid = new Color(1, 1, 1, 0.08f);
        for (int i = 0; i <= 4; i++)
        {
            float y = plot.Position.Y + plot.Size.Y * i / 4f;
            DrawLine(new Vector2(plot.Position.X, y), new Vector2(plot.End.X, y), grid);
            float v = ymax - (ymax - ymin) * i / 4f;
            DrawString(font, new Vector2(2, y + 4), v.ToString("0.##", CultureInfo.InvariantCulture), HorizontalAlignment.Left, 40, 10, UiTheme.Muted);
        }
        DrawString(font, new Vector2(plot.Position.X, Size.Y - 3),
            $"{xmin.ToString("0.#", CultureInfo.InvariantCulture)} … {xmax.ToString("0.#", CultureInfo.InvariantCulture)} {XLabel}",
            HorizontalAlignment.Left, -1, 10, UiTheme.Muted);
        if (Circle is { } rad)
        {
            var centre = Map(Vector2.Zero);
            float pr = rad / (xmax - xmin) * plot.Size.X;
            DrawArc(centre, pr, 0, Mathf.Tau, 64, new Color(1, 1, 1, 0.35f), 1.5f);
            DrawLine(Map(new Vector2(xmin, 0)), Map(new Vector2(xmax, 0)), grid);
            DrawLine(Map(new Vector2(0, ymin)), Map(new Vector2(0, ymax)), grid);
        }

        float lx = plot.End.X;
        foreach (var s in Series)
        {
            if (s.Points.Count > 1)
            {
                if (Scatter)
                    foreach (var p in s.Points) DrawCircle(Map(p), 1.5f, s.Color);
                else
                {
                    var pts = new Vector2[s.Points.Count];
                    for (int i = 0; i < pts.Length; i++) pts[i] = Map(s.Points[i]);
                    DrawPolyline(pts, s.Color, 1.5f, true);
                }
            }
            var w = font.GetStringSize(s.Name, HorizontalAlignment.Left, -1, 11).X;
            lx -= w + 14;
            DrawRect(new Rect2(lx, 6, 8, 8), s.Color);
            DrawString(font, new Vector2(lx + 10, 14), s.Name, HorizontalAlignment.Left, -1, 11, UiTheme.Text);
        }
    }
}
