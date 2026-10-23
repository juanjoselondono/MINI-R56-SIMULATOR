using System;
using System.Diagnostics;
using Godot;

namespace Simulator.Simulation;

/// <summary>Logs how long a loading stage took (`[load] stage: N ms`); used to find what freezes a session start.</summary>
public readonly struct LoadTimer : IDisposable
{
    private readonly string _stage;
    private readonly long _t0;

    private LoadTimer(string stage)
    {
        _stage = stage;
        _t0 = Stopwatch.GetTimestamp();
    }

    public static LoadTimer Stage(string stage) => new(stage);

    public void Dispose()
        => GD.Print(FormattableString.Invariant($"[load] {_stage}: {(Stopwatch.GetTimestamp() - _t0) * 1000.0 / Stopwatch.Frequency:F0} ms"));
}
