using Simulator.Race;

namespace GameLogic.Tests;

/// <summary>All-time best laps (Leaderboard): top 10 per track and vehicle, persisted as JSON.</summary>
public class LeaderboardTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "vd_board_" + Guid.NewGuid().ToString("N"));
    private string File_ => Path.Combine(_dir, "leaderboard.json");

    public void Dispose()
    {
        if (Directory.Exists(_dir)) Directory.Delete(_dir, true);
    }

    [Fact]
    public void KeepsTheTenFastest_PerTrackAndVehicle()
    {
        var b = new Leaderboard();
        var t0 = new DateTime(2026, 10, 1);
        for (int i = 0; i < 10; i++) Assert.NotNull(b.Submit("Circuit", "mini_r56", 60 + i, t0.AddMinutes(i)));
        Assert.Null(b.RankFor("Circuit", "mini_r56", 70));          // slower than the 10th
        Assert.Equal(1, b.RankFor("Circuit", "f1_2019", 99));       // another vehicle: its own board
        Assert.Equal(1, b.RankFor("Monaco", "mini_r56", 99));       // another track: its own board
        Assert.Equal(7, b.RankFor("Circuit", "mini_r56", 65));      // a tie goes behind the earlier lap
        Assert.Null(b.RankFor("Circuit", "mini_r56", 69));          // so tying the 10th does not make the board
        Assert.Equal(10, b.RankFor("Circuit", "mini_r56", 68.5));

        var r = b.Submit("Circuit", "mini_r56", 60.5, t0.AddHours(1))!.Value;
        Assert.Equal(2, r.Rank);
        Assert.Single(r.Dropped);
        Assert.Equal(69, r.Dropped[0].Time);
        var top = b.Top("Circuit", "mini_r56");
        Assert.Equal(Leaderboard.Size, top.Count);
        Assert.Equal(new[] { 60, 60.5, 61 }, top.Take(3).Select(e => e.Time));
        Assert.Null(b.Submit("Circuit", "mini_r56", double.NaN, t0));
    }

    [Fact]
    public void RoundTripsThroughJson_WithPhotos()
    {
        var b = new Leaderboard();
        var e = b.Submit("Monaco", "f1_2019", 75.123, new DateTime(2026, 10, 22, 14, 5, 0))!.Value.Entry;
        e.Photo = e.Id + ".png";
        b.Save(File_);
        var back = Leaderboard.Load(File_, out var err);
        Assert.Null(err);
        var only = Assert.Single(back.All);
        Assert.Equal(e with { }, only with { });
        Assert.False(File.Exists(File_ + ".tmp"));
    }

    [Fact]
    public void CorruptFile_IsKeptAside_AndTheBoardStartsEmpty()
    {
        Directory.CreateDirectory(_dir);
        File.WriteAllText(File_, "{ not json");
        var b = Leaderboard.Load(File_, out var err);
        Assert.NotNull(err);
        Assert.Empty(b.All);
        Assert.False(File.Exists(File_));
        Assert.Single(Directory.GetFiles(_dir, "leaderboard.json.corrupt-*"));
        Assert.Empty(Leaderboard.Load(Path.Combine(_dir, "missing.json"), out var none).All);
        Assert.Null(none);
    }
}
