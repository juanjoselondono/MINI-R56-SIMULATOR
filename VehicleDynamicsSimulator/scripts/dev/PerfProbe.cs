using System.Linq;
using Godot;
using Simulator.Simulation;

namespace Simulator.Dev;

/// <summary>
/// `-- --perf=N`: after a 120-frame warm-up, averages N frames of render frame time and of the physics advance
/// (substeps + collision/lap/telemetry hooks), prints the result and quits. Combine with --demo for a moving car.
/// Also reports the GPU time of the main viewport (RenderingServer measurement) and the graphics preset;
/// `--no-vsync` removes the 60 Hz cap so the frame time shows the real headroom.
/// </summary>
public partial class PerfProbe : Node
{
    private SimulationManager _sim = null!;
    private int _frames, _target;
    private double _sumFrame, _sumPhys, _maxFrame, _sumGpu;
    private int _over60;   // frames longer than a 60 Hz refresh (16.7 ms): a dropped frame with vsync
    private long _steps;

    public static void AttachIfRequested(SimulationManager sim)
    {
        var a = OS.GetCmdlineUserArgs().FirstOrDefault(x => x.StartsWith("--perf="));
        if (a == null) return;
        if (OS.GetCmdlineUserArgs().Contains("--no-vsync")) DisplayServer.WindowSetVsyncMode(DisplayServer.VSyncMode.Disabled);
        RenderingServer.ViewportSetMeasureRenderTime(sim.GetViewport().GetViewportRid(), true);
        sim.AddChild(new PerfProbe { Name = "PerfProbe", _sim = sim, _target = int.Parse(a["--perf=".Length..]) });
    }

    public override void _Process(double delta)
    {
        _frames++;
        if (_frames <= 120) return;
        _sumFrame += delta;
        _maxFrame = System.Math.Max(_maxFrame, delta);
        if (delta > 1.0 / 60) _over60++;
        _sumPhys += _sim.Vehicle.LastAdvanceMs;
        _steps += _sim.Vehicle.LastStepsThisFrame;
        _sumGpu += RenderingServer.ViewportGetMeasuredRenderTimeGpu(GetViewport().GetViewportRid());
        int n = _frames - 120;
        if (n < _target) return;
        GD.Print(string.Create(System.Globalization.CultureInfo.InvariantCulture,
            $"PERF frames={n} avg_frame={1000 * _sumFrame / n:F2} ms ({n / _sumFrame:F1} fps) max_frame={1000 * _maxFrame:F1} ms over_16.7ms={_over60} ({100.0 * _over60 / n:F1} %) physics_per_frame={_sumPhys / n:F4} ms for {(double)_steps / n:F2} steps ({_sumPhys / System.Math.Max(1, _steps) * 1000:F1} µs/step) gpu={_sumGpu / n:F2} ms preset={Rendering.GraphicsSettings.Current} size={GetViewport().GetVisibleRect().Size} renderer={RenderingServer.GetVideoAdapterName()}"));
        _target = int.MaxValue;
        AppQuit.Quit(this);
    }
}
