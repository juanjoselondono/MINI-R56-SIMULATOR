using System;
using System.Collections.Generic;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;

namespace Simulator.Race;

public readonly record struct LapRecord(int Lap, double Time, bool Valid);

/// <summary>
/// Lap timing from the track arc length of the physical state, updated once per physics step.
/// Standing start: lap 1 starts when the car first moves within 50 m of the line. After that a lap counts only when
/// the start line (s = 0) is crossed forwards after passing checkpoints at 1/3 and 2/3 of the lap in order, so cutting
/// back across the line or driving the wrong way never completes a lap. The crossing time is interpolated linearly
/// between the two steps around the line. A lap is invalid when the car centre spends more than
/// <see cref="OffTrackAllowance"/> s beyond the road + curb, or after a barrier impact.
/// </summary>
public sealed class LapTimer
{
    public ITrackLine Track { get; }
    public double OffTrackAllowance { get; set; } = 1.0;

    /// <summary>Current lap number; 0 until the first forward crossing of the line.</summary>
    public int Lap { get; private set; }
    public double LapStartTime { get; private set; }
    public double CurrentLapTime { get; private set; }
    public bool LapValid { get; private set; } = true;
    public double? LastLapTime { get; private set; }
    public double? BestLapTime { get; private set; }
    public List<LapRecord> Laps { get; } = new();
    public TrackProjection Projection { get; private set; }

    public event Action<LapRecord>? LapCompleted;

    private double _prevS = double.NaN;
    private double _prevT;
    private int _checkpoint;   // 0 none, 1 passed L/3, 2 passed 2L/3
    private double _offTrack;

    public LapTimer(ITrackLine track) => Track = track;

    public void Reset()
    {
        Lap = 0; LapStartTime = 0; CurrentLapTime = 0; LapValid = true;
        LastLapTime = null; BestLapTime = null; Laps.Clear();
        _prevS = double.NaN; _checkpoint = 0; _offTrack = 0;
    }

    public void InvalidateLap() => LapValid = false;

    public void AfterStep(in VehicleState s, double dt)
    {
        var p = Track.Project(s.X, s.Y);
        Projection = p;
        double L = Track.Length;
        double sNow = p.S;

        var (left, right) = Track.Limits(p.S);
        if (p.Lateral > left || p.Lateral < -right) _offTrack += dt;
        if (_offTrack > OffTrackAllowance) LapValid = false;

        if (Lap == 0 && s.V > 0.05 && (sNow < 50 || sNow > L - 50))
        {
            StartLap(_prevT);
        }
        else if (!double.IsNaN(_prevS))
        {
            if (_checkpoint == 0 && Crossed(_prevS, sNow, L / 3)) _checkpoint = 1;
            else if (_checkpoint == 1 && Crossed(_prevS, sNow, 2 * L / 3)) _checkpoint = 2;

            // Forward crossing of s = 0: previous station near the end of the lap, current one just after the line.
            bool forwardCross = _prevS > L - 50 && sNow < 50;
            if (forwardCross)
            {
                double gap = (L - _prevS) + sNow;
                double frac = gap > 0 ? (L - _prevS) / gap : 1;
                double tCross = _prevT + frac * (s.Time - _prevT);
                if (_checkpoint == 2)
                {
                    var rec = new LapRecord(Lap, tCross - LapStartTime, LapValid);
                    Laps.Add(rec);
                    LastLapTime = rec.Time;
                    if (rec.Valid && (BestLapTime is null || rec.Time < BestLapTime)) BestLapTime = rec.Time;
                    LapCompleted?.Invoke(rec);
                    StartLap(tCross);
                }
            }
        }
        _prevS = sNow;
        _prevT = s.Time;
        CurrentLapTime = Lap > 0 ? s.Time - LapStartTime : 0;
    }

    private void StartLap(double t)
    {
        Lap++;
        LapStartTime = t;
        LapValid = true;
        _checkpoint = 0;
        _offTrack = 0;
    }

    private static bool Crossed(double a, double b, double mark) => a < mark && b >= mark && b - a < 50;
}
