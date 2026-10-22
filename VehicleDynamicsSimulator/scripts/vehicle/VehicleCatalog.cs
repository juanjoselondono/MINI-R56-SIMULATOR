using System.Collections.Generic;
using Godot;
using VehicleDynamics.Core.Simulation;

namespace Simulator.Vehicle;

/// <summary>
/// Presentation data of a vehicle profile (the physics lives in <see cref="VehicleProfiles"/>): visuals scene, asset
/// metadata, camera eyes and chase distance, the collision footprint (from the asset pipeline's measured AABB), and the
/// engine's firing events per revolution for the sound synthesis.
/// </summary>
public sealed record VehicleVisualProfile(
    string Id, string VisualScene, string MetaPath,
    Vector3 BonnetEye, Vector3 CockpitEye, float ChaseDistance, float ChaseHeight,
    double HalfLength, double HalfWidth, double CentreBehindCg, float BodyHeight, double FiringsPerRevolution,
    bool HasHeadlights);

public static class VehicleCatalog
{
    private static readonly Dictionary<string, VehicleVisualProfile> Cache = new();

    public static VehicleVisualProfile Get(string id)
    {
        if (Cache.TryGetValue(id, out var v)) return v;
        v = id switch
        {
            VehicleProfiles.F1 => Build(id, "res://scenes/vehicles/F1Visuals.tscn", "res://assets/vehicles/f1/f1_meta.json",
                // The full body including the exposed wheels is the footprint; T-cam above the airbox, driver's eye in the cockpit.
                aabbKey: "overall_aabb", bonnet: null, cockpit: null, chaseDistance: 7.5f, chaseHeight: 2.0f,
                firings: 3.0,                   // V6 four-stroke: three firing events per revolution
                headlights: false),
            _ => Build(VehicleProfiles.MiniR56, "res://scenes/vehicles/MiniJcwVisuals.tscn", "res://assets/vehicles/mini_jcw/mini_jcw_meta.json",
                // Body shell without mirrors (chassis AABB): 3.76 × 1.68 m, centre 0.261 m behind the CG.
                aabbKey: "chassis_aabb",
                bonnet: new Vector3(0f, 1.08f, -0.7f),     // ~0.17 m above the hood, ahead of the glass
                cockpit: new Vector3(-0.35f, 1.1f, 0.42f),  // LHD driver's eye, below the roof lining
                chaseDistance: 6.5f, chaseHeight: 2.3f,
                firings: 2.0,                   // inline-4 four-stroke
                headlights: true),
        };
        Cache[id] = v;
        return v;
    }

    private static VehicleVisualProfile Build(string id, string scene, string metaPath, string aabbKey, Vector3? bonnet,
        Vector3? cockpit, float chaseDistance, float chaseHeight, double firings, bool headlights)
    {
        var meta = Json.ParseString(FileAccess.GetFileAsString(metaPath)).AsGodotDictionary();
        Vector3 V(string key)
        {
            var a = meta[key].AsGodotArray();
            return new Vector3((float)a[0], (float)a[1], (float)a[2]);
        }
        var min = V(aabbKey + "_min");
        var max = V(aabbKey + "_max");
        return new VehicleVisualProfile(id, scene, metaPath,
            bonnet ?? V("tcam_eye"), cockpit ?? V("cockpit_eye"), chaseDistance, chaseHeight,
            HalfLength: (max.Z - min.Z) / 2, HalfWidth: (max.X - min.X) / 2, CentreBehindCg: (max.Z + min.Z) / 2,
            BodyHeight: max.Y, FiringsPerRevolution: firings, HasHeadlights: headlights);
    }
}
