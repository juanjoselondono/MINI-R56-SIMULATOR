using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using Godot;
using VehicleDynamics.Core.Track;

namespace Simulator.Track;

/// <summary>
/// Disk cache of optimised racing lines (user://cache/racing_lines). The minimum-curvature optimisation takes ~6 s on
/// Monaco; its result depends only on the corridor (track line, limits, walls) and the margin (car width), so the key
/// hashes exactly those, plus an algorithm version. A changed track conversion or car width gives a new key.
/// </summary>
public static class RacingLineCache
{
    /// <summary>Bump when RacingLineOptimizer's result changes for the same input.</summary>
    private const string AlgorithmVersion = "mincurv-v1";
    private static string Dir => ProjectSettings.GlobalizePath("user://cache/racing_lines");

    private sealed record Entry(double[] X, double[] Y, double[] Left, double[] Right);

    public static string Key(string track, double margin, ITrackLine corridor)
    {
        // FNV-1a over the corridor sampled every ~Length/512, rounded to millimetres.
        ulong h = 1469598103934665603UL;
        void Mix(long v)
        {
            for (int b = 0; b < 8; b++) { h ^= (byte)(v >> (8 * b)); h *= 1099511628211UL; }
        }
        foreach (char c in AlgorithmVersion + "|" + track) Mix(c);
        Mix((long)Math.Round(margin * 1000));
        Mix((long)Math.Round(corridor.Length * 1000));
        for (int i = 0; i < 512; i++)
        {
            double s = corridor.Length * i / 512;
            var p = corridor.Pose(s);
            var (l, r) = corridor.Limits(s);
            Mix((long)Math.Round(p.X * 1000)); Mix((long)Math.Round(p.Y * 1000));
            Mix((long)Math.Round(l * 1000)); Mix((long)Math.Round(r * 1000));
        }
        return $"{SafeName(track)}_{margin:F2}_{h:x16}".Replace(',', '.');
    }

    public static bool TryLoad(string key, string name, out PolylineTrack line)
    {
        line = null!;
        string path = Path.Combine(Dir, key + ".json");
        if (!File.Exists(path)) return false;
        try
        {
            var e = JsonSerializer.Deserialize<Entry>(File.ReadAllText(path));
            if (e == null || e.X.Length < 8 || e.X.Length != e.Y.Length) return false;
            line = new PolylineTrack(name, e.X, e.Y, e.Left, e.Right);
            return true;
        }
        catch (Exception ex) when (ex is IOException or JsonException or ArgumentException or UnauthorizedAccessException)
        {
            GD.PushWarning($"Racing line cache {path} unreadable ({ex.Message}); recomputing.");
            return false;
        }
    }

    public static void Save(string key, PolylineTrack line)
    {
        try
        {
            Directory.CreateDirectory(Dir);
            var e = new Entry(line.X.ToArray(), line.Y.ToArray(), line.SideLeft.ToArray(), line.SideRight.ToArray());
            File.WriteAllText(Path.Combine(Dir, key + ".json"), JsonSerializer.Serialize(e));
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            GD.PushWarning("Racing line cache not written: " + ex.Message);
        }
    }

    private static string SafeName(string s) => new(s.Select(c => char.IsLetterOrDigit(c) ? c : '_').ToArray());
}
