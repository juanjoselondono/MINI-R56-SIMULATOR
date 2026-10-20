using System;
using System.Globalization;
using System.IO;
using System.Linq;
using Godot;
using Simulator.CameraRig;
using Simulator.Simulation;

namespace Simulator.Dev;

/// <summary>
/// Dev utility (needs a rendering run, not --headless): `-- --shots=DIR --shot-times=5,40,85 [--bonnet] [--demo]`
/// saves a PNG of the viewport when the simulation clock passes each time, then quits.
/// </summary>
public partial class ScreenshotRig : Node
{
    private string _dir = "";
    private double[] _times = Array.Empty<double>();
    private int _next;
    private SimulationManager _sim = null!;

    public static void AttachIfRequested(SimulationManager sim)
    {
        var args = OS.GetCmdlineUserArgs();
        var dir = args.FirstOrDefault(a => a.StartsWith("--shots="));
        if (dir == null) return;
        var times = args.FirstOrDefault(a => a.StartsWith("--shot-times="))?["--shot-times=".Length..] ?? "2";
        var rig = new ScreenshotRig
        {
            Name = "ScreenshotRig",
            _dir = dir["--shots=".Length..],
            _times = times.Split(',').Select(t => double.Parse(t, CultureInfo.InvariantCulture)).OrderBy(t => t).ToArray(),
            _sim = sim,
        };
        sim.AddChild(rig);
        if (args.Contains("--bonnet")) sim.Camera?.SetBonnet(true);
    }

    public override void _Process(double delta)
    {
        if (_next >= _times.Length) return;
        if (_sim.Vehicle.Dynamics.State.Time < _times[_next]) return;
        Directory.CreateDirectory(_dir);
        var img = GetViewport().GetTexture().GetImage();
        string path = Path.Combine(_dir, $"{_sim.Track.TrackName}_{_times[_next].ToString("000.0", CultureInfo.InvariantCulture)}s.png");
        img.SavePng(path);
        GD.Print($"screenshot {path}");
        _next++;
        if (_next >= _times.Length) GetTree().Quit();
    }
}
