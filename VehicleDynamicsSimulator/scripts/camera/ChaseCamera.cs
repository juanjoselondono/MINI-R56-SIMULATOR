using Godot;
using Simulator.Track;
using Simulator.UI;
using Simulator.Vehicle;

namespace Simulator.CameraRig;

/// <summary>
/// Chase camera (smoothed, behind the car), bonnet camera (rigid, above the hood) and cockpit camera (driver's eye,
/// rigid; the JCW model has a full interior and translucent glass). C cycles chase → bonnet → cockpit.
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
    /// <summary>Bonnet camera in car coordinates (car faces -Z): ~0.17 m above the hood (top 0.91 m at z = -0.7), ahead of the glass.</summary>
    [Export] public Vector3 BonnetEye { get; set; } = new(0f, 1.08f, -0.7f);
    /// <summary>Driver's eye (LHD): on the steering-column line x = -0.35, behind the wheel, below the roof lining (1.22 m).</summary>
    [Export] public Vector3 CockpitEye { get; set; } = new(-0.35f, 1.1f, 0.42f);

    public enum Mode { Chase, Bonnet, Cockpit }
    public Mode View { get; private set; } = Mode.Chase;
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

    public void SetView(Mode mode) => View = mode;

    /// <summary>Snap to the target (after a reset/teleport) instead of smoothing across the jump.</summary>
    public void Snap()
    {
        if (_target != null) GlobalPosition = DesiredChasePosition();
    }

    public override void _Process(double delta)
    {
        if (_target == null) return;
        if (!UiFocus.IsTyping(GetViewport()) && Input.IsActionJustPressed(InputSetup.ToggleCamera)) View = (Mode)(((int)View + 1) % 3);

        var t = _target.GlobalTransform;
        _shake = Mathf.Max(0, _shake - 2.5f * (float)delta);
        var jitter = new Vector3(_rng.RandfRange(-1, 1), _rng.RandfRange(-1, 1), _rng.RandfRange(-1, 1)) * (_shake * _shake * 0.25f);

        if (View != Mode.Chase)
        {
            var eye = View == Mode.Bonnet ? BonnetEye : CockpitEye;
            // Cockpit: look slightly down over the steering wheel towards the road.
            var basis = View == Mode.Cockpit ? t.Basis * new Basis(Vector3.Right, Mathf.DegToRad(-4f)) : t.Basis;
            GlobalTransform = new Transform3D(basis, t * eye + jitter * 0.3f);
            Fov = View == Mode.Cockpit ? 75 : 72;
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
