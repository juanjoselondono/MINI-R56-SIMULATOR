using System.Text.Json;
using System.Text.Json.Nodes;
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

    /// <summary>
    /// Parses and validates. Missing keys keep the defaults of the file's vehicle profile when it names a known one
    /// (an F1 file never inherits Mini values), otherwise model.m's. Throws ArgumentException on any problem.
    /// </summary>
    public static VehicleParameters FromJson(string json)
    {
        VehicleParameters? p;
        try
        {
            var node = JsonNode.Parse(json);
            if (node is JsonObject obj && obj[nameof(VehicleParameters.Profile)] is JsonValue pv
                && pv.TryGetValue<string>(out var profile) && VehicleProfiles.IsKnown(profile))
            {
                var merged = JsonNode.Parse(ToJson(VehicleProfiles.Create(profile)))!.AsObject();
                foreach (var (key, value) in obj) merged[key] = value?.DeepClone();
                json = merged.ToJsonString();
            }
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
