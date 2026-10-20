using Godot;
using Simulator.Track;
using Simulator.UI;
using Simulator.Vehicle;

namespace Simulator.CameraRig;

/// <summary>
/// Chase camera (smoothed, behind the car) and bonnet camera (rigid, above the hood). Toggle with C.
/// A true cockpit view is not offered: Mini_body.stl is a closed shell without interior or transparent glazing.
/// The chase camera is pulled in front of barriers with a ray query against the track's barrier collision body.
/// Impact shake only offsets the view; it never touches the vehicle state.
/// </summary>
public partial class ChaseCamera : Camera3D
{
    [Export] public NodePath TargetPath { get; set; } = new("");
    [Export] public float Distance { get; set; } = 6.5f;
    [Export] public float Height { get; set; } = 2.3f;
    [Export] public float LookAhead { get; set; } = 4f;
    [Export] public float PositionSharpness { get; set; } = 6f;
    /// <summary>Bonnet camera in car coordinates (car faces -Z): 0.2 m above the hood, ahead of the windscreen base.</summary>
    [Export] public Vector3 BonnetEye { get; set; } = new(0f, 1.18f, -0.75f);

    public bool Bonnet { get; private set; }
    private Node3D? _target;
    private float _shake;
    private readonly RandomNumberGenerator _rng = new() { Seed = 3 };

    public override void _Ready()
    {
        _target = GetNodeOrNull<Node3D>(TargetPath);
        if (_target != null) GlobalPosition = DesiredChasePosition();
    }

    /// <summary>Adds view shake; intensity ~ impact severity / 10, decays in about half a second.</summary>
    public void AddShake(float intensity) => _shake = Mathf.Clamp(_shake + intensity, 0, 1.5f);

    public void SetBonnet(bool on) => Bonnet = on;

    /// <summary>Snap to the target (after a reset/teleport) instead of smoothing across the jump.</summary>
    public void Snap()
    {
        if (_target != null) GlobalPosition = DesiredChasePosition();
    }

    public override void _Process(double delta)
    {
        if (_target == null) return;
        if (!UiFocus.IsTyping(GetViewport()) && Input.IsActionJustPressed(InputSetup.ToggleCamera)) Bonnet = !Bonnet;

        var t = _target.GlobalTransform;
        _shake = Mathf.Max(0, _shake - 2.5f * (float)delta);
        var jitter = new Vector3(_rng.RandfRange(-1, 1), _rng.RandfRange(-1, 1), _rng.RandfRange(-1, 1)) * (_shake * _shake * 0.25f);

        if (Bonnet)
        {
            GlobalTransform = new Transform3D(t.Basis, t * BonnetEye + jitter * 0.3f);
            Fov = 72;
            return;
        }

        Fov = 65;
        float k = 1f - Mathf.Exp(-PositionSharpness * (float)delta);
        var desired = DesiredChasePosition();
        var from = t.Origin + Vector3.Up * 1.2f;
        var hit = GetWorld3D().DirectSpaceState.IntersectRay(
            PhysicsRayQueryParameters3D.Create(from, desired, TrackManager.BarrierCollisionLayer));
        if (hit.Count > 0)
        {
            var p = (Vector3)hit["position"];
            desired = p + (from - p).Normalized() * 0.4f;
        }
        GlobalPosition = GlobalPosition.Lerp(desired, k) + jitter;
        var forward = -t.Basis.Z;
        LookAt(t.Origin + forward * LookAhead + Vector3.Up * 0.8f, Vector3.Up);
    }

    private Vector3 DesiredChasePosition()
    {
        var t = _target!.GlobalTransform;
        return t.Origin + t.Basis.Z * Distance + Vector3.Up * Height;
    }
}
