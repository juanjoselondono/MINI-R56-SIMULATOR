using System.Collections.Generic;
using System.Linq;
using Godot;

namespace Simulator.Vehicle;

/// <summary>
/// Presentation only: reads values derived from the physics state and never writes back.
/// Model: 2011 Mini JCW (R56) GP, converted by tools/asset_pipeline/import_jcw_obj.py (scaled so its wheelbase equals
/// model.m's car.L; origin under the CG). Wheel spin angle = odometer / R_w (no wheel-slip model exists in model.m);
/// front wheels show the model's clamped steer angle delta; the steering wheel turns about its measured column axis by
/// delta x <see cref="SteeringWheelRatio"/>, a visual constant (model.m has no steering system). No suspension motion is
/// drawn because model.m has no suspension dynamics. Missing meshes fall back to primitive placeholders.
/// </summary>
public partial class VehicleVisuals : Node3D
{
    private const string MetaPath = "res://assets/vehicles/mini_jcw/mini_jcw_meta.json";

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

    public override void _Ready()
    {
        var body = GetNode<MeshInstance3D>("Body");
        if (body.Mesh == null)
        {
            GD.PushWarning("Vehicle body mesh missing; using placeholder box.");
            body.Mesh = new BoxMesh { Size = new Vector3(1.68f, 1.3f, 3.76f) };
            body.Position = new Vector3(0, 0.75f, 0.26f);
        }
        else
        {
            TuneMaterials(body);
        }

        string[] wheels = { "WheelFL", "WheelFR", "WheelRL", "WheelRR" };
        _spin = new Node3D[wheels.Length];
        for (int i = 0; i < wheels.Length; i++)
        {
            var hub = GetNode<Node3D>(wheels[i]);
            _spin[i] = hub.GetNode<Node3D>("Spin");
            var mesh = _spin[i].GetNode<MeshInstance3D>("Mesh");
            if (mesh.Mesh == null)
            {
                mesh.Mesh = new CylinderMesh { TopRadius = 0.3043f, BottomRadius = 0.3043f, Height = 0.205f };
                mesh.RotationDegrees = new Vector3(0, 0, 90);
            }
            else TuneMaterials(mesh);
        }
        _steer = new[] { GetNode<Node3D>("WheelFL"), GetNode<Node3D>("WheelFR") };

        _steeringWheel = GetNodeOrNull<Node3D>("SteeringWheel");
        if (_steeringWheel != null)
        {
            _steeringRest = _steeringWheel.Basis;
            if (_steeringWheel.GetNodeOrNull<MeshInstance3D>("Mesh") is { Mesh: not null } sw) TuneMaterials(sw);
            ReadSteeringAxis();
        }

        _brakeGlow = GetNodeOrNull<OmniLight3D>("BrakeGlow");
        _headlights = new[] { GetNodeOrNull<SpotLight3D>("HeadlightL"), GetNodeOrNull<SpotLight3D>("HeadlightR") }
            .Where(l => l != null).Cast<SpotLight3D>().ToArray();
        _smoke = GetNodeOrNull<GpuParticles3D>("EngineSmoke");
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
                    case "Doorrfok0011Mtl":           // body paint
                        m.Metallic = 0.55f; m.Roughness = 0.3f;
                        m.ClearcoatEnabled = true; m.Clearcoat = 0.8f; m.ClearcoatRoughness = 0.12f;
                        break;
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
