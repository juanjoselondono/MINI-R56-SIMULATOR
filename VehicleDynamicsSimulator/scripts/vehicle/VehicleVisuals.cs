using System.Linq;
using Godot;

namespace Simulator.Vehicle;

/// <summary>
/// Presentation only: reads values derived from the physics state and never writes back.
/// Wheel spin angle = odometer / R_w (no wheel-slip model exists in model.m); front wheels show the model's
/// clamped steer angle delta. No suspension motion is drawn because model.m has no suspension dynamics.
/// Missing meshes fall back to primitive placeholders.
/// </summary>
public partial class VehicleVisuals : Node3D
{
    public double WheelRadius { get; set; } = 0.3043;

    private Node3D[] _spin = System.Array.Empty<Node3D>();
    private Node3D[] _steer = System.Array.Empty<Node3D>();
    private StandardMaterial3D? _brakeLightMaterial;
    private SpotLight3D[] _headlights = System.Array.Empty<SpotLight3D>();
    private OmniLight3D? _brakeGlow;
    private GpuParticles3D? _smoke;

    public override void _Ready()
    {
        var body = GetNode<MeshInstance3D>("Body");
        if (body.Mesh == null)
        {
            GD.PushWarning("Vehicle body mesh missing; using placeholder box.");
            body.Mesh = new BoxMesh { Size = new Vector3(1.68f, 1.4f, 3.7f) };
            body.Position = new Vector3(0, 0.75f, 0.2f);
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
                mesh.Mesh = new CylinderMesh { TopRadius = 0.3043f, BottomRadius = 0.3043f, Height = 0.175f };
                mesh.RotationDegrees = new Vector3(0, 0, 90);
            }
        }
        _steer = new[] { GetNode<Node3D>("WheelFL"), GetNode<Node3D>("WheelFR") };

        var tail = GetNodeOrNull<MeshInstance3D>("TailLightL");
        _brakeLightMaterial = tail?.GetActiveMaterial(0) as StandardMaterial3D;
        _brakeGlow = GetNodeOrNull<OmniLight3D>("BrakeGlow");
        _headlights = new[] { GetNodeOrNull<SpotLight3D>("HeadlightL"), GetNodeOrNull<SpotLight3D>("HeadlightR") }
            .Where(l => l != null).Cast<SpotLight3D>().ToArray();
        _smoke = GetNodeOrNull<GpuParticles3D>("EngineSmoke");
    }

    public void UpdateVisuals(double distance, double steerAngle, double brake, bool engineBlown, bool headlights)
    {
        // Forward is -Z; a wheel rolling forward rotates negatively about its +X axle.
        float spin = (float)(-(distance / WheelRadius) % Mathf.Tau);
        foreach (var s in _spin) s.Rotation = new Vector3(spin, 0, 0);
        foreach (var s in _steer) s.Rotation = new Vector3(0, (float)steerAngle, 0);

        bool braking = brake > 0.05;
        if (_brakeLightMaterial != null) _brakeLightMaterial.EmissionEnergyMultiplier = braking ? 6f : 0.6f;
        if (_brakeGlow != null) _brakeGlow.Visible = braking;
        foreach (var h in _headlights) h.Visible = headlights;
        if (_smoke != null) _smoke.Emitting = engineBlown;
    }
}
