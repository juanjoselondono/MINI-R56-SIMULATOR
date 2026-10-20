using System.Text.Json;
using System.Text.Json.Serialization;

namespace VehicleDynamics.Core.Simulation;

/// <summary>JSON save/load of <see cref="VehicleParameters"/>. Loading rejects unknown keys and invalid values.</summary>
public static class ConfigSerializer
{
    private static readonly JsonSerializerOptions Options = new()
    {
        WriteIndented = true,
        UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
        Converters = { new JsonStringEnumConverter() },
        NumberHandling = JsonNumberHandling.Strict,
    };

    public static string ToJson(VehicleParameters p) => JsonSerializer.Serialize(p, Options);

    /// <summary>Parses and validates. Missing keys keep model.m defaults. Throws ArgumentException on any problem.</summary>
    public static VehicleParameters FromJson(string json)
    {
        VehicleParameters? p;
        try
        {
            p = JsonSerializer.Deserialize<VehicleParameters>(json, Options);
        }
        catch (JsonException ex)
        {
            throw new ArgumentException("Invalid parameter JSON: " + ex.Message, ex);
        }
        if (p is null) throw new ArgumentException("Empty parameter JSON.");
        p.EnsureValid();
        return p;
    }

    public static void Save(VehicleParameters p, string path)
    {
        p.EnsureValid();
        var dir = Path.GetDirectoryName(Path.GetFullPath(path));
        if (dir != null) Directory.CreateDirectory(dir);
        File.WriteAllText(path, ToJson(p));
    }

    public static VehicleParameters Load(string path) => FromJson(File.ReadAllText(path));

    public static bool TryLoad(string path, out VehicleParameters? p, out string? error)
    {
        try
        {
            p = Load(path);
            error = null;
            return true;
        }
        catch (Exception ex) when (ex is ArgumentException or IOException or UnauthorizedAccessException)
        {
            p = null;
            error = ex.Message;
            return false;
        }
    }
}
