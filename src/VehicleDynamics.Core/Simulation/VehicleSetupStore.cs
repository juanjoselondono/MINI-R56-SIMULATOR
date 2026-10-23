namespace VehicleDynamics.Core.Simulation;

/// <summary>
/// Per-vehicle setup files. Each vehicle profile owns its files and never reads another's:
///   &lt;root&gt;/setup_&lt;key&gt;.json   the profile's current setup (written on Apply, loaded when that car is selected);
///   &lt;root&gt;/&lt;key&gt;/*.json         its named setups (Save / Load Configuration).
/// The key is <see cref="VehicleProfiles.SetupKey"/> (mini, f1). A file whose Profile field names another vehicle is
/// rejected, so a Mini setup can never be applied to the F1 or the other way round.
/// </summary>
public sealed class VehicleSetupStore
{
    public string Root { get; }

    public VehicleSetupStore(string root) => Root = root;

    public string ActivePath(string profile) => Path.Combine(Root, $"setup_{VehicleProfiles.SetupKey(profile)}.json");

    public string NamedDirectory(string profile) => Path.Combine(Root, VehicleProfiles.SetupKey(profile));

    /// <summary>
    /// The profile's current setup, or its defaults when there is none. A file that does not load (invalid, wrong
    /// vehicle) also gives the defaults, with the reason in <paramref name="error"/>.
    /// </summary>
    public VehicleParameters LoadActive(string profile, out string? error)
    {
        error = null;
        string path = ActivePath(profile);
        if (!File.Exists(path)) return VehicleProfiles.Create(profile);
        try
        {
            return LoadFor(profile, path);
        }
        catch (Exception ex) when (ex is ArgumentException or IOException or UnauthorizedAccessException)
        {
            error = $"{Path.GetFileName(path)}: {ex.Message}";
            return VehicleProfiles.Create(profile);
        }
    }

    /// <summary>Writes <paramref name="p"/> as the current setup of its own profile; returns the path.</summary>
    public string SaveActive(VehicleParameters p)
    {
        string path = ActivePath(p.Profile);
        ConfigSerializer.Save(p, path);
        return path;
    }

    /// <summary>Names (without .json) of the profile's saved setups, sorted.</summary>
    public IReadOnlyList<string> ListNamed(string profile)
    {
        string dir = NamedDirectory(profile);
        if (!Directory.Exists(dir)) return Array.Empty<string>();
        return Directory.GetFiles(dir, "*.json").Select(f => Path.GetFileNameWithoutExtension(f)).OrderBy(n => n, StringComparer.Ordinal).ToArray();
    }

    public string NamedPath(string profile, string name) => Path.Combine(NamedDirectory(profile), SafeName(name) + ".json");

    /// <summary>Saves <paramref name="p"/> under <paramref name="name"/> in its own profile's folder; returns the path.</summary>
    public string SaveNamed(VehicleParameters p, string name)
    {
        string path = NamedPath(p.Profile, name);
        ConfigSerializer.Save(p, path);
        return path;
    }

    public VehicleParameters LoadNamed(string profile, string name) => LoadFor(profile, NamedPath(profile, name));

    /// <summary>Loads a setup file that must belong to <paramref name="profile"/> (ArgumentException otherwise).</summary>
    public static VehicleParameters LoadFor(string profile, string path)
    {
        var p = ConfigSerializer.Load(path);
        if (p.Profile != profile)
            throw new ArgumentException($"{Path.GetFileName(path)} is a {VehicleProfiles.DisplayName(p.Profile)} setup, not {VehicleProfiles.DisplayName(profile)}.");
        return p;
    }

    /// <summary>
    /// Moves configurations saved before the per-vehicle split (flat *.json in the root) into the folder of the profile
    /// they name. Files that do not load stay where they are. Returns how many moved.
    /// </summary>
    public int MigrateLegacy()
    {
        if (!Directory.Exists(Root)) return 0;
        int moved = 0;
        foreach (string f in Directory.GetFiles(Root, "*.json"))
        {
            string file = Path.GetFileName(f);
            if (VehicleProfiles.All.Any(p => file == $"setup_{VehicleProfiles.SetupKey(p.Id)}.json")) continue;
            if (!ConfigSerializer.TryLoad(f, out var p, out _) || !VehicleProfiles.IsKnown(p!.Profile)) continue;
            string dir = NamedDirectory(p.Profile);
            Directory.CreateDirectory(dir);
            string target = Path.Combine(dir, file);
            for (int i = 2; File.Exists(target); i++) target = Path.Combine(dir, $"{Path.GetFileNameWithoutExtension(file)}_{i}.json");
            File.Move(f, target);
            moved++;
        }
        return moved;
    }

    public static string SafeName(string name)
    {
        name = name.Trim();
        foreach (char c in Path.GetInvalidFileNameChars()) name = name.Replace(c, '_');
        return name.Length == 0 ? "setup" : name;
    }
}
