using Godot;

namespace Simulator.Simulation;

/// <summary>
/// Scripted exit (self-test, autodrive, screenshots, perf). Stops every audio player first and waits ~0.25 s so the
/// audio server releases its generator playbacks before the engine shuts down; quitting at once can leak one.
/// </summary>
public static class AppQuit
{
    public static async void Quit(Node from, int exitCode = 0)
    {
        var tree = from.GetTree();
        foreach (var n in tree.Root.FindChildren("*", nameof(AudioStreamPlayer), true, false))
        {
            if (n is Audio.EngineAudio engine) engine.Shutdown();
            else if (n is AudioStreamPlayer p) p.Stop();
        }
        // The audio thread releases stopped playbacks on its own schedule: give it ~0.25 s of real time.
        ulong until = Time.GetTicksMsec() + 250;
        while (Time.GetTicksMsec() < until) await from.ToSignal(tree, SceneTree.SignalName.ProcessFrame);
        tree.Quit(exitCode);
    }
}
