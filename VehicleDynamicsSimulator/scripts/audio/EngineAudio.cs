using System;
using Godot;
using Simulator.Simulation;
using VehicleDynamics.Core.Simulation;

namespace Simulator.Audio;

/// <summary>
/// Streams <see cref="EngineSynth"/> through an AudioStreamGenerator. Control values come only from the physics core
/// (engine rpm, branch, rev limiter, engine-blown flag, slip indicator, speed) and the driver's throttle input.
/// Fixed 256-frame chunks keep the audio thread fed without allocations.
/// </summary>
public partial class EngineAudio : AudioStreamPlayer
{
    private const int Chunk = 256;
    [Export] public NodePath SimulationPath { get; set; } = new("../SimulationManager");

    private SimulationManager? _sim;
    private EngineSynth _synth = null!;
    private AudioStreamGeneratorPlayback? _playback;
    private readonly float[] _l = new float[Chunk];
    private readonly float[] _r = new float[Chunk];
    private readonly Vector2[] _frames = new Vector2[Chunk];
    private int _lastGear = 1;
    private bool _wasBlown;

    public bool Muted { get; set; }

    public override void _Ready()
    {
        _sim = GetNodeOrNull<SimulationManager>(SimulationPath);
        int rate = (int)AudioServer.GetMixRate();
        Stream = new AudioStreamGenerator { MixRate = rate, BufferLength = 0.1f };
        _synth = new EngineSynth(rate);
        Play();
        _playback = GetStreamPlayback() as AudioStreamGeneratorPlayback;
        if (_sim != null) _sim.Impact += e => _synth.TriggerImpact(e.Severity);
    }

    public override void _ExitTree()
    {
        // The playback is a RefCounted owned by the stream player; drop our reference so it is freed with it.
        Stop();
        _playback?.Dispose();
        _playback = null;
        Stream = null;
    }

    public override void _Process(double delta)
    {
        if (_sim?.Vehicle?.Dynamics == null || _playback == null) return;
        var d = _sim.Vehicle.Dynamics;
        var s = d.State;
        var o = d.Outputs;
        if (s.Gear != _lastGear) { _synth.TriggerShift(); _lastGear = s.Gear; }
        if (s.EngineBlown && !_wasBlown) _synth.TriggerExplosion();
        _wasBlown = s.EngineBlown;

        var c = new SynthControl
        {
            Rpm = s.EngineRpm,
            Throttle = _sim.Vehicle.LastInputs.Throttle,
            SpeedMps = s.V,
            Coasting = o.Branch == DriveBranch.Coast,
            RevLimiter = o.RevLimiter,
            EngineOff = s.EngineBlown || _sim.Vehicle.Paused,
            SlipIndicator = o.SlipIndicator,
            BrakingAtLimit = o.Branch == DriveBranch.Brake && _sim.Vehicle.LastInputs.Brake >= 0.95,
        };
        _synth.MasterGain = Muted ? 0 : 0.6;

        int available = _playback.GetFramesAvailable();
        while (available >= Chunk)
        {
            _synth.Render(c, _l, _r);
            for (int i = 0; i < Chunk; i++) _frames[i] = new Vector2(_l[i], _r[i]);
            _playback.PushBuffer(_frames);
            available -= Chunk;
        }
    }
}
