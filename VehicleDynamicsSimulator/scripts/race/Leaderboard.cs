using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;

namespace Simulator.Race;

/// <summary>One all-time lap record. <see cref="Photo"/> is the file name of the podium photo (null = none taken).</summary>
public sealed record LeaderboardEntry
{
    public string Id { get; init; } = Guid.NewGuid().ToString("N");
    public string Track { get; init; } = "";
    public string Vehicle { get; init; } = "";
    /// <summary>Lap time [s].</summary>
    public double Time { get; init; }
    /// <summary>When the lap was driven (local time, ISO 8601 in the file).</summary>
    public DateTime Date { get; init; }
    public string? Photo { get; set; }
    /// <summary>Driven with parameters other than the vehicle profile's defaults (shown next to the time).</summary>
    public bool CustomSetup { get; init; }
}

/// <summary>
/// All-time best laps, persisted as one JSON file. Laps are ranked per track and vehicle (a Mini lap and an F1 lap are
/// different competitions); each board keeps its <see cref="Size"/> fastest valid laps. Godot-independent so it can be
/// unit-tested; the game wraps it in LeaderboardService (photos, UI).
/// </summary>
public sealed class Leaderboard
{
    public const int Size = 10;
    private const int FormatVersion = 1;

    private sealed class FileFormat
    {
        public int Version { get; set; } = FormatVersion;
        public List<LeaderboardEntry> Entries { get; set; } = new();
    }

    private static readonly JsonSerializerOptions Options = new() { WriteIndented = true };

    private readonly List<LeaderboardEntry> _entries = new();

    public IReadOnlyList<LeaderboardEntry> All => _entries;

    /// <summary>The board of one track and vehicle, fastest first.</summary>
    public IReadOnlyList<LeaderboardEntry> Top(string track, string vehicle)
        => _entries.Where(e => e.Track == track && e.Vehicle == vehicle).OrderBy(e => e.Time).ThenBy(e => e.Date).Take(Size).ToList();

    /// <summary>1-based position a lap of <paramref name="time"/> would take, or null if it is not a top-<see cref="Size"/> time.</summary>
    public int? RankFor(string track, string vehicle, double time)
    {
        if (!double.IsFinite(time) || time <= 0) return null;
        var top = Top(track, vehicle);
        int rank = top.Count(e => e.Time <= time) + 1;   // ties: the earlier lap keeps the place
        return rank <= Size ? rank : null;
    }

    /// <summary>
    /// Adds a valid lap if it makes the board. Returns the new entry and its rank, and the entries it pushed off the
    /// board (their photos can be deleted); null when the lap is not a top time.
    /// </summary>
    public (LeaderboardEntry Entry, int Rank, IReadOnlyList<LeaderboardEntry> Dropped)? Submit(string track, string vehicle, double time, DateTime date,
        bool customSetup = false)
    {
        if (RankFor(track, vehicle, time) is not { } rank) return null;
        var entry = new LeaderboardEntry { Track = track, Vehicle = vehicle, Time = time, Date = date, CustomSetup = customSetup };
        _entries.Add(entry);
        var keep = Top(track, vehicle).Select(e => e.Id).ToHashSet();
        var dropped = _entries.Where(e => e.Track == track && e.Vehicle == vehicle && !keep.Contains(e.Id)).ToList();
        foreach (var d in dropped) _entries.Remove(d);
        return (entry, rank, dropped);
    }

    public LeaderboardEntry? Find(string id) => _entries.FirstOrDefault(e => e.Id == id);

    /// <summary>
    /// Reads the file; a missing file is an empty board. A file that does not parse is kept aside as
    /// &lt;name&gt;.corrupt-&lt;timestamp&gt; (so a later save does not destroy it) and the board starts empty.
    /// </summary>
    public static Leaderboard Load(string path, out string? error)
    {
        error = null;
        var board = new Leaderboard();
        if (!File.Exists(path)) return board;
        try
        {
            var file = JsonSerializer.Deserialize<FileFormat>(File.ReadAllText(path), Options);
            foreach (var e in file?.Entries ?? new())
                if (double.IsFinite(e.Time) && e.Time > 0 && e.Track.Length > 0 && e.Vehicle.Length > 0 && e.Id.Length > 0)
                    board._entries.Add(e);
            // Re-apply the size limit (a hand-edited file may hold more).
            foreach (var g in board._entries.GroupBy(e => (e.Track, e.Vehicle)).ToList())
                foreach (var extra in g.OrderBy(e => e.Time).ThenBy(e => e.Date).Skip(Size)) board._entries.Remove(extra);
        }
        catch (Exception ex) when (ex is JsonException or IOException or UnauthorizedAccessException or NotSupportedException)
        {
            error = ex.Message;
            try { File.Move(path, $"{path}.corrupt-{DateTime.Now:yyyyMMdd_HHmmss}"); }
            catch (Exception moveEx) when (moveEx is IOException or UnauthorizedAccessException) { }
            board._entries.Clear();
        }
        return board;
    }

    /// <summary>Writes the board atomically (temporary file, then rename), so a crash never leaves half a file.</summary>
    public void Save(string path)
    {
        var dir = Path.GetDirectoryName(Path.GetFullPath(path));
        if (dir != null) Directory.CreateDirectory(dir);
        var file = new FileFormat
        {
            Entries = _entries.OrderBy(e => e.Track, StringComparer.Ordinal).ThenBy(e => e.Vehicle, StringComparer.Ordinal).ThenBy(e => e.Time).ToList(),
        };
        string tmp = path + ".tmp";
        File.WriteAllText(tmp, JsonSerializer.Serialize(file, Options));
        File.Move(tmp, path, overwrite: true);
    }
}
