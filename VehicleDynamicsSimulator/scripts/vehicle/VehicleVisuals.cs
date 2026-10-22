using System.Collections.Generic;
using System.Linq;
using Godot;

namespace Simulator.Vehicle;

/// <summary>Which hand-tuned material overrides to apply (the Mini OBJ only has plain diffuse materials).</summary>
public enum VehicleMaterialSet { None = 0, MiniJcw = 1 }

/// <summary>
/// Presentation only: reads values derived from the physics state and never writes back. One scene per vehicle profile
/// (scenes/vehicles/*Visuals.tscn, chosen by <see cref="VehicleCatalog"/>), each converted by the asset pipeline so its
/// origin is on the ground under the CG and its axles sit where the profile's physics puts them:
/// Mini JCW (tools/asset_pipeline/import_jcw_obj.py, wheelbase = model.m's car.L) and F1 2019
/// (tools/asset_pipeline/import_f1_glb.py). Nodes are found by name anywhere below this node: Body, WheelFL/FR/RL/RR
/// (hub) → first child (spin) → meshes, SteeringWheel. Wheel spin angle = odometer / R_w (no wheel-slip model);
/// front wheels show the model's clamped steer angle; the steering wheel turns about its column axis (from the metadata)
/// by delta × <see cref="SteeringWheelRatio"/>, a visual constant. No suspension motion is drawn. Missing meshes fall back
/// to primitive placeholders.
/// </summary>
public partial class VehicleVisuals : Node3D
{
    /// <summary>Asset-pipeline metadata (hubs, footprint, steering axis, camera eyes).</summary>
    [Export] public string MetaPath { get; set; } = "res://assets/vehicles/mini_jcw/mini_jcw_meta.json";
    [Export] public VehicleMaterialSet MaterialSet { get; set; } = VehicleMaterialSet.None;

    public double WheelRadius { get; set; } = 0.3043;
    /// <summary>Steering-wheel angle / road-wheel angle. Visual only; not a model parameter.</summary>
    [Export] public float SteeringWheelRatio { get; set; } = 14f;

    private Node3D[] _spin = System.Array.Empty<Node3D>();
    private Node3D[] _steer = System.Array.Empty<Node3D>();
    private Node3D? _steeringWheel;
    private Basis _steeringRest = Basis.Identity;
    private Vector3 _steeringAxis = new(-0.00052f, 0.40524f, 0.91421f);
    private readonly List<StandardMaterial3D> _brakeLenses = new();
    private readonly List<StandardMaterial3D> _headLenses = new();
    private SpotLight3D[] _headlights = System.Array.Empty<SpotLight3D>();
    private OmniLight3D? _brakeGlow;
    private GpuParticles3D? _smoke;
    private bool? _lastBraking, _lastHeadlights;

    /// <summary>Number of wheel hubs found (4 when the model converted correctly).</summary>
    public int WheelCount => _spin.Length;
    public bool HasSteeringWheel => _steeringWheel != null;

    private T? Find<T>(string name) where T : Node => FindChild(name, true, false) as T;

    public override void _Ready()
    {
        var body = Find<MeshInstance3D>("Body");
        if (body == null || body.Mesh == null)
        {
            GD.PushWarning("Vehicle body mesh missing; using placeholder box.");
            body ??= new MeshInstance3D { Name = "Body" };
            if (body.GetParent() == null) AddChild(body);
            body.Mesh = new BoxMesh { Size = new Vector3(1.68f, 1.3f, 3.76f) };
            body.Position = new Vector3(0, 0.75f, 0.26f);
        }
        else if (MaterialSet == VehicleMaterialSet.MiniJcw)
        {
            TuneMaterials(body);
        }

        string[] wheels = { "WheelFL", "WheelFR", "WheelRL", "WheelRR" };
        var spins = new List<Node3D>();
        foreach (var name in wheels)
        {
            if (Find<Node3D>(name) is not { } hub) continue;
            var spin = hub.GetChildren().OfType<Node3D>().FirstOrDefault() ?? hub;
            var meshes = spin.FindChildren("*", nameof(MeshInstance3D), true, false).OfType<MeshInstance3D>().ToList();
            if (spin is MeshInstance3D own) meshes.Add(own);
            if (meshes.All(m => m.Mesh == null))
            {
                var placeholder = new MeshInstance3D
                {
                    Mesh = new CylinderMesh { TopRadius = 0.3043f, BottomRadius = 0.3043f, Height = 0.205f },
                    RotationDegrees = new Vector3(0, 0, 90),
                };
                spin.AddChild(placeholder);
            }
            else if (MaterialSet == VehicleMaterialSet.MiniJcw)
                foreach (var m in meshes.Where(m => m.Mesh != null)) TuneMaterials(m);
            spins.Add(spin);
        }
        _spin = spins.ToArray();
        _steer = new[] { Find<Node3D>("WheelFL"), Find<Node3D>("WheelFR") }.Where(n => n != null).Cast<Node3D>().ToArray();

        _steeringWheel = Find<Node3D>("SteeringWheel");
        if (_steeringWheel != null)
        {
            _steeringRest = _steeringWheel.Basis;
            if (MaterialSet == VehicleMaterialSet.MiniJcw)
                foreach (var sw in _steeringWheel.FindChildren("*", nameof(MeshInstance3D), true, false).OfType<MeshInstance3D>())
                    if (sw.Mesh != null) TuneMaterials(sw);
            ReadSteeringAxis();
        }

        _brakeGlow = Find<OmniLight3D>("BrakeGlow");
        _headlights = new[] { Find<SpotLight3D>("HeadlightL"), Find<SpotLight3D>("HeadlightR") }
            .Where(l => l != null).Cast<SpotLight3D>().ToArray();
        _smoke = Find<GpuParticles3D>("EngineSmoke");
    }

    /// <summary>The steering column axis measured by the asset pipeline (fallback: the value above).</summary>
    private void ReadSteeringAxis()
    {
        if (!FileAccess.FileExists(MetaPath)) return;
        var json = Json.ParseString(FileAccess.GetFileAsString(MetaPath));
        if (json.VariantType != Variant.Type.Dictionary) return;
        var axis = json.AsGodotDictionary()["steering_wheel"].AsGodotDictionary()["axis"].AsGodotArray();
        _steeringAxis = new Vector3((float)axis[0], (float)axis[1], (float)axis[2]).Normalized();
    }

    /// <summary>
    /// Per-surface material overrides keyed by the source material names (the OBJ materials are plain diffuse).
    /// Duplicates are made so the imported resources stay untouched.
    /// </summary>
    private const string CarPaintPath = "res://assets/vehicles/mini_jcw/materials/car_paint.tres";

    private void TuneMaterials(MeshInstance3D mi)
    {
        var cache = new Dictionary<string, StandardMaterial3D>();
        for (int i = 0; i < mi.Mesh.GetSurfaceCount(); i++)
        {
            if (mi.Mesh.SurfaceGetMaterial(i) is not StandardMaterial3D src) continue;
            string name = src.ResourceName;
            if (!cache.TryGetValue(name, out var m))
            {
                m = (StandardMaterial3D)src.Duplicate();
                switch (name)
                {
                    case "Doorrfok0011Mtl":           // body paint: automotive clear-coat material, in the model's colour
                    {
                        var paint = (StandardMaterial3D)GD.Load<StandardMaterial3D>(CarPaintPath).Duplicate();
                        paint.AlbedoColor = src.AlbedoColor;
                        paint.AlbedoTexture = src.AlbedoTexture;
                        m = paint;
                        break;
                    }
                    case "Doorrfok0051Mtl":           // glass (translucent in the source: d = 0.25)
                        m.Roughness = 0.04f; m.Metallic = 0.1f; m.MetallicSpecular = 0.9f;
                        break;
                    case "Tailight0031Mtl":           // tail-light lens: brake light
                        m.EmissionEnabled = true; m.Emission = new Color(1f, 0.05f, 0.03f); m.EmissionEnergyMultiplier = 0.4f;
                        m.Roughness = 0.2f;
                        _brakeLenses.Add(m);
                        break;
                    case "Frontlight0041Mtl":         // headlight cluster texture
                        m.EmissionEnabled = true; m.EmissionTexture = m.AlbedoTexture; m.Emission = Colors.White;
                        m.EmissionEnergyMultiplier = 0f; m.Roughness = 0.15f;
                        _headLenses.Add(m);
                        break;
                    case "Wheel1Mtl":                 // tyre
                        m.Roughness = 0.92f;
                        break;
                    case "Wheel0011Mtl":              // rim
                        m.Metallic = 0.6f; m.Roughness = 0.35f;
                        break;
                    case "Doorrfok0061Mtl":           // black trim
                        m.Roughness = 0.45f;
                        break;
                    default:
                        m.Roughness = 0.6f;
                        break;
                }
                cache[name] = m;
            }
            m.TextureFilter = BaseMaterial3D.TextureFilterEnum.LinearWithMipmapsAnisotropic;
            mi.SetSurfaceOverrideMaterial(i, m);
        }
    }

    public void UpdateVisuals(double distance, double steerAngle, double brake, bool engineBlown, bool headlights)
    {
        // Forward is -Z; a wheel rolling forward rotates negatively about its +X axle.
        float spin = (float)(-(distance / WheelRadius) % Mathf.Tau);
        foreach (var s in _spin) s.Rotation = new Vector3(spin, 0, 0);
        foreach (var s in _steer) s.Rotation = new Vector3(0, (float)steerAngle, 0);
        // delta > 0 (left) turns the wheel counter-clockwise as the driver sees it: positive about the column axis,
        // which points from the dashboard towards the driver.
        if (_steeringWheel != null)
            _steeringWheel.Basis = new Basis(_steeringAxis, (float)steerAngle * SteeringWheelRatio) * _steeringRest;

        bool braking = brake > 0.05;
        if (braking != _lastBraking)
        {
            foreach (var m in _brakeLenses) m.EmissionEnergyMultiplier = braking ? 5f : 0.4f;
            if (_brakeGlow != null) _brakeGlow.Visible = braking;
            _lastBraking = braking;
        }
        if (headlights != _lastHeadlights)
        {
            foreach (var m in _headLenses) m.EmissionEnergyMultiplier = headlights ? 1.2f : 0f;
            foreach (var h in _headlights) h.Visible = headlights;
            _lastHeadlights = headlights;
        }
        if (_smoke != null) _smoke.Emitting = engineBlown;
    }
}
