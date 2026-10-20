using System;
using System.Globalization;
using Godot;
using Simulator.Simulation;
using VehicleDynamics.Core.Simulation;

namespace Simulator.UI;

/// <summary>Driver HUD. Every number is read from the physics state/outputs, the lap timer or the recorder.</summary>
public partial class RaceHud : Control
{
    private SimulationManager _sim = null!;

    public void Init(SimulationManager sim)
    {
        _sim = sim;
        SetAnchorsAndOffsetsPreset(LayoutPreset.FullRect);
        MouseFilter = MouseFilterEnum.Ignore;
    }

    public override void _Process(double delta) => QueueRedraw();

    public override void _Draw()
    {
        if (_sim?.Vehicle?.Dynamics == null) return;
        var font = ThemeDB.FallbackFont;
        var s = _sim.Vehicle.Dynamics.State;
        var o = _sim.Vehicle.Dynamics.Outputs;
        var p = _sim.Vehicle.Parameters;
        var vp = GetViewportRect().Size;
        string F(double v, string fmt) => v.ToString(fmt, CultureInfo.InvariantCulture);

        // Tachometer + speed, bottom right.
        var c = new Vector2(vp.X - 170, vp.Y - 150);
        float r = 120;
        DrawCircle(c, r + 14, new Color(0, 0, 0, 0.55f));
        const float a0 = Mathf.Pi * 0.75f, a1 = Mathf.Pi * 2.25f;
        double rpmMax = Math.Max(7000, p.MoneyShiftRpm + 300);
        float At(double rpm) => a0 + (a1 - a0) * (float)Math.Clamp(rpm / rpmMax, 0, 1);
        DrawArc(c, r, a0, a1, 64, new Color(1, 1, 1, 0.25f), 6);
        DrawArc(c, r, At(p.FuelCutRpm), At(p.MoneyShiftRpm), 32, new Color(1, 0.55f, 0.1f, 0.9f), 6);
        DrawArc(c, r, At(p.MoneyShiftRpm), a1, 16, new Color(1, 0.1f, 0.1f, 0.9f), 6);
        for (int k = 0; k <= (int)(rpmMax / 1000); k++)
        {
            float a = At(k * 1000);
            var dir = new Vector2(Mathf.Cos(a), Mathf.Sin(a));
            DrawLine(c + dir * (r - 14), c + dir * (r - 2), UiTheme.Text, 2);
            DrawString(font, c + dir * (r - 30) - new Vector2(6, -5), k.ToString(), HorizontalAlignment.Left, -1, 13, UiTheme.Muted);
        }
        if (!s.EngineBlown)
        {
            float na = At(s.EngineRpm);
            DrawLine(c, c + new Vector2(Mathf.Cos(na), Mathf.Sin(na)) * (r - 8), o.RevLimiter ? UiTheme.Bad : UiTheme.Accent, 4);
        }
        DrawCircle(c, 8, UiTheme.Accent);
        DrawString(font, c + new Vector2(-70, 66), F(s.V * 3.6, "0"), HorizontalAlignment.Center, 140, 46, UiTheme.Text);
        DrawString(font, c + new Vector2(-70, 88), "km/h", HorizontalAlignment.Center, 140, 14, UiTheme.Muted);
        string gear = s.EngineBlown ? "X" : s.Gear.ToString();
        DrawString(font, c + new Vector2(-30, -42), gear, HorizontalAlignment.Center, 60, 34, s.EngineBlown ? UiTheme.Bad : UiTheme.Accent);
        DrawString(font, c + new Vector2(-70, 30), (_sim.Vehicle.AutoShift ? "AUTO · " : "") + F(s.EngineRpm, "0") + " rpm", HorizontalAlignment.Center, 140, 13, UiTheme.Muted);

        // Lap panel, top right.
        var lp = new Rect2(vp.X - 300, 16, 284, 128);
        DrawRect(lp, new Color(0, 0, 0, 0.55f));
        var laps = _sim.Laps;
        DrawString(font, lp.Position + new Vector2(14, 26), $"{_sim.Track.TrackName.ToUpperInvariant()}   LAP {Math.Max(laps.Lap, 0)}", HorizontalAlignment.Left, -1, 16, UiTheme.Accent);
        DrawString(font, lp.Position + new Vector2(14, 56), "Current  " + SimulationManager.FormatTime(laps.Lap > 0 ? laps.CurrentLapTime : null) + (laps.LapValid ? "" : "  INVALID"),
            HorizontalAlignment.Left, -1, 17, laps.LapValid ? UiTheme.Text : UiTheme.Bad);
        DrawString(font, lp.Position + new Vector2(14, 84), "Last     " + SimulationManager.FormatTime(laps.LastLapTime), HorizontalAlignment.Left, -1, 15, UiTheme.Muted);
        DrawString(font, lp.Position + new Vector2(14, 110), "Best     " + SimulationManager.FormatTime(laps.BestLapTime), HorizontalAlignment.Left, -1, 15, UiTheme.Good);

        // Status flags, top left.
        float y = 30;
        void Flag(string text, Color col)
        {
            DrawString(font, new Vector2(18, y), text, HorizontalAlignment.Left, -1, 16, col);
            y += 24;
        }
        if (_sim.Telemetry.Recording) Flag("● REC  " + _sim.Telemetry.Active!.Run.Samples.Count + " samples", UiTheme.Bad);
        if (_sim.Vehicle.Paused) Flag("PAUSED (P)", UiTheme.Accent);
        if (o.RevLimiter) Flag("REV LIMITER — fuel cut", UiTheme.Accent);
        if (o.Understeer) Flag("UNDERSTEER — lateral grip saturated [E1]", UiTheme.Accent);
        if (o.Branch == DriveBranch.Drive && o.FTractive > o.FTractiveEff + 1) Flag("TRACTION LIMIT — F_trac capped at F_x,max", UiTheme.Muted);
        if (_sim.CurrentSurface is Surface.Grass or Surface.Barrier) Flag("OFF TRACK — grass [E2]", UiTheme.Bad);
        if (_sim.Collisions.InContact) Flag("BARRIER CONTACT [E3]", UiTheme.Bad);

        if (s.EngineBlown)
        {
            var box = new Rect2(vp.X / 2 - 360, vp.Y / 2 - 70, 720, 120);
            DrawRect(box, new Color(0, 0, 0, 0.75f));
            var col = (int)(Time.GetTicksMsec() / 120) % 2 == 0 ? UiTheme.Bad : new Color(1, 0.9f, 0.2f);
            DrawString(font, box.Position + new Vector2(0, 50), "¡NO SEAS ANIMAL JAJAJA!", HorizontalAlignment.Center, box.Size.X, 34, col);
            DrawString(font, box.Position + new Vector2(0, 84), $"Engine destroyed by a money shift (> {p.MoneyShiftRpm:0} rpm). Press R to pay the mechanic.",
                HorizontalAlignment.Center, box.Size.X, 15, UiTheme.Text);
        }

        if (_sim.LastMessage != null && Time.GetTicksMsec() / 1000.0 - _sim.LastMessageTime < 3.5)
        {
            var w = font.GetStringSize(_sim.LastMessage, HorizontalAlignment.Left, -1, 16).X + 30;
            var m = new Rect2(vp.X / 2 - w / 2, vp.Y - 70, w, 34);
            DrawRect(m, new Color(0, 0, 0, 0.65f));
            DrawString(font, m.Position + new Vector2(15, 23), _sim.LastMessage, HorizontalAlignment.Left, -1, 16, UiTheme.Text);
        }
        DrawString(font, new Vector2(18, vp.Y - 16), "F1 help · Tab setup · F2 telemetry · F3 compare · F5 record · Esc menu", HorizontalAlignment.Left, -1, 13, new Color(1, 1, 1, 0.6f));
    }
}
