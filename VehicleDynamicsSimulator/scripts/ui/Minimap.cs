using System;
using Godot;
using Simulator.Simulation;
using Simulator.Vehicle;
using VehicleDynamics.Core.Track;

namespace Simulator.UI;

/// <summary>
/// Track minimap (HUD, top left; F6 toggles it). The map is drawn in 2D from the track's reference line
/// (<see cref="ITrackLine"/>, model frame), north up, scaled to fit the panel; the car is a triangle at its physical
/// position and heading. No second camera: an orthographic Camera3D in a SubViewport would render the whole scene again
/// every frame (7.9 M triangles at the Nürburgring) and a 25 km track seen from above is a hairline under trees and
/// roofs. The line is sampled once per track; each frame only the car marker moves.
/// </summary>
public partial class Minimap : Control
{
    public const float MapSize = 240f;
    private const float Padding = 14f;

    private SimulationManager _sim = null!;
    private ITrackLine? _line;
    private Vector2[] _points = Array.Empty<Vector2>();
    private Vector2 _startA, _startB;
    private double _minX, _maxY, _scale;
    private Vector2 _offset;
    private readonly StyleBoxFlat _panel = new()
    {
        BgColor = new Color(0.06f, 0.07f, 0.09f, 0.72f), BorderColor = new Color(1, 1, 1, 0.08f),
        CornerRadiusTopLeft = 8, CornerRadiusTopRight = 8, CornerRadiusBottomLeft = 8, CornerRadiusBottomRight = 8,
    };

    public void Init(SimulationManager sim)
    {
        _sim = sim;
        _panel.SetBorderWidthAll(1);
        CustomMinimumSize = new Vector2(MapSize, MapSize);
        MouseFilter = MouseFilterEnum.Ignore;
        Visible = GameSession.MinimapVisible;
    }

    /// <summary>F6 (action <see cref="InputSetup.ToggleMinimap"/>, remappable in the Input Map): show / hide the map.</summary>
    public override void _UnhandledInput(InputEvent @event)
    {
        if (!@event.IsActionPressed(InputSetup.ToggleMinimap) || @event.IsEcho()) return;
        Toggle();
        GetViewport().SetInputAsHandled();
    }

    /// <summary>Points of the drawn track outline (0 until the map has been built for a track).</summary>
    public int PointCount => _points.Length;

    public void Toggle()
    {
        Visible = !Visible;
        GameSession.MinimapVisible = Visible;   // kept across restarts and track changes, like the racing line
    }

    public override void _Process(double delta)
    {
        if (!Visible || _sim?.Track?.Line == null) return;
        if (!ReferenceEquals(_sim.Track.Line, _line)) Rebuild(_sim.Track.Line);
        QueueRedraw();   // the car marker moves every frame
    }

    /// <summary>Samples the line (~5 m, 200–2000 points) and fits it into the panel, north (+Y model = −Z Godot) up.</summary>
    private void Rebuild(ITrackLine line)
    {
        _line = line;
        int n = Math.Clamp((int)(line.Length / 5), 200, 2000);
        var raw = new (double X, double Y)[n];
        double minX = double.MaxValue, maxX = double.MinValue, minY = double.MaxValue, maxY = double.MinValue;
        for (int i = 0; i < n; i++)
        {
            var p = line.Pose(line.Length * i / n);
            raw[i] = (p.X, p.Y);
            minX = Math.Min(minX, p.X); maxX = Math.Max(maxX, p.X);
            minY = Math.Min(minY, p.Y); maxY = Math.Max(maxY, p.Y);
        }
        double inner = MapSize - 2 * Padding;
        _scale = inner / Math.Max(Math.Max(maxX - minX, maxY - minY), 1);
        _minX = minX;
        _maxY = maxY;
        // Centre the shorter dimension.
        _offset = new Vector2(Padding + (float)((inner - (maxX - minX) * _scale) / 2), Padding + (float)((inner - (maxY - minY) * _scale) / 2));
        _points = new Vector2[n + 1];
        for (int i = 0; i < n; i++) _points[i] = ToMap(raw[i].X, raw[i].Y);
        _points[n] = _points[0];

        // Start/finish: a short bar across the line at s = 0.
        var s0 = line.Pose(0);
        var across = new Vector2((float)-Math.Sin(s0.Psi), (float)-Math.Cos(s0.Psi)) * 7f;
        var c = ToMap(s0.X, s0.Y);
        _startA = c - across;
        _startB = c + across;
    }

    private Vector2 ToMap(double x, double y) => _offset + new Vector2((float)((x - _minX) * _scale), (float)((_maxY - y) * _scale));

    public override void _Draw()
    {
        DrawStyleBox(_panel, new Rect2(Vector2.Zero, new Vector2(MapSize, MapSize)));
        if (_points.Length < 2 || _sim == null) return;
        DrawPolyline(_points, new Color(0, 0, 0, 0.55f), 6f, true);
        DrawPolyline(_points, new Color(0.86f, 0.87f, 0.9f), 2.6f, true);
        DrawLine(_startA, _startB, UiTheme.Accent, 3f, true);

        // Car: triangle along the heading (model ψ; screen y points south).
        var st = _sim.Vehicle.Dynamics.State;
        var at = ToMap(st.X, st.Y);
        var fwd = new Vector2((float)Math.Cos(st.Psi), (float)-Math.Sin(st.Psi));
        var side = new Vector2(-fwd.Y, fwd.X);
        var tri = new[] { at + fwd * 9f, at - fwd * 6f + side * 5.5f, at - fwd * 6f - side * 5.5f };
        DrawColoredPolygon(tri, UiTheme.Accent);
        DrawPolyline(new[] { tri[0], tri[1], tri[2], tri[0] }, new Color(0, 0, 0, 0.8f), 1.5f, true);
    }
}
