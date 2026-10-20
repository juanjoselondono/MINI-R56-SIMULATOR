using System.Globalization;
using System.Text.Json;

namespace VehicleDynamics.Tests;

/// <summary>Access to MATLAB-generated data shipped in tests/VehicleDynamics.Tests/Data (see Data/README.md).</summary>
internal static class TestData
{
    public static string Dir => Path.Combine(AppContext.BaseDirectory, "Data");

    public static string RepoRoot
    {
        get
        {
            var d = new DirectoryInfo(AppContext.BaseDirectory);
            while (d != null && !File.Exists(Path.Combine(d.FullName, "VehicleDynamics.sln"))) d = d.Parent;
            return d?.FullName ?? throw new InvalidOperationException("Repository root not found.");
        }
    }

    public static Dictionary<string, double[]> ReadCsv(string path)
    {
        var lines = File.ReadAllLines(path).Where(l => l.Length > 0 && !l.StartsWith('#')).ToArray();
        var header = lines[0].Split(',');
        var cols = header.ToDictionary(h => h, _ => new List<double>());
        for (int i = 1; i < lines.Length; i++)
        {
            var parts = lines[i].Split(',');
            for (int j = 0; j < header.Length; j++)
                cols[header[j]].Add(double.Parse(parts[j], CultureInfo.InvariantCulture));
        }
        return cols.ToDictionary(kv => kv.Key, kv => kv.Value.ToArray());
    }

    public static Dictionary<string, double> ReadSummary(string path)
    {
        using var doc = JsonDocument.Parse(File.ReadAllText(path));
        var r = new Dictionary<string, double>();
        foreach (var p in doc.RootElement.EnumerateObject())
            if (p.Value.ValueKind == JsonValueKind.Number) r[p.Name] = p.Value.GetDouble();
        return r;
    }

    public static Dictionary<string, double[]> MlValues()
    {
        var r = new Dictionary<string, double[]>();
        foreach (var line in File.ReadAllLines(Path.Combine(Dir, "ml_values.txt")))
        {
            var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 2 || parts[0] == "colon") continue;
            r[parts[0]] = parts.Skip(1).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
        }
        return r;
    }

    public static double RelErr(double a, double b) => Math.Abs(a - b) / Math.Max(1e-300, Math.Max(Math.Abs(a), Math.Abs(b)));
}
