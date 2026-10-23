namespace VehicleDynamics.Core.Simulation;

/// <summary>
/// Interchangeable vehicle profiles. A profile is a complete <see cref="VehicleParameters"/> set; its
/// <see cref="VehicleParameters.Profile"/> id also selects the 3D model in the game. The class defaults of
/// VehicleParameters remain model.m's (validation), the Mini profile is the game's starting point, and the F1 profile is
/// provisional (no MATLAB model for it).
/// </summary>
public static class VehicleProfiles
{
    public const string MiniR56 = "mini_r56";
    public const string F1 = "f1_2019";

    public static readonly IReadOnlyList<(string Id, string DisplayName)> All = new[]
    {
        (MiniR56, "Mini Cooper R56 (N16, model.m)"),
        (F1, "Formula 1 (2019 car, provisional physics)"),
    };

    public static VehicleParameters Create(string id) => id switch
    {
        MiniR56 => VehicleParameters.MiniCooperR56(),
        F1 => VehicleParameters.F1Provisional(),
        _ => throw new ArgumentException($"Unknown vehicle profile '{id}'."),
    };

    public static string DisplayName(string id) => All.FirstOrDefault(p => p.Id == id).DisplayName ?? id;

    public static bool IsKnown(string id) => All.Any(p => p.Id == id);

    /// <summary>Short key of the profile's setup files (<see cref="VehicleSetupStore"/>): setup_mini.json, setup_f1.json.</summary>
    public static string SetupKey(string id) => id switch
    {
        MiniR56 => "mini",
        F1 => "f1",
        _ => throw new ArgumentException($"Unknown vehicle profile '{id}'."),
    };
}
