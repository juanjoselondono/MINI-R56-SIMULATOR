using Godot;
using Simulator.Vehicle;

namespace Simulator.CameraRig;

/// <summary>Chase camera (smoothed, behind the car) and cockpit camera (driver eye point, rigid). Toggle with C.</summary>
public partial class ChaseCamera : Camera3D
{
    [Export] public NodePath TargetPath { get; set; } = new("");
    [Export] public float Distance { get; set; } = 6.5f;
    [Export] public float Height { get; set; } = 2.3f;
    [Export] public float LookAhead { get; set; } = 4f;
    [Export] public float PositionSharpness { get; set; } = 6f;
    /// <summary>Left-hand-drive eye point in car coordinates (car faces -Z, left is -X).</summary>
    [Export] public Vector3 CockpitEye { get; set; } = new(-0.36f, 1.18f, 0.25f);

    public bool Cockpit { get; private set; }
    private Node3D? _target;

    public override void _Ready()
    {
        _target = GetNodeOrNull<Node3D>(TargetPath);
        if (_target != null) GlobalPosition = DesiredChasePosition();
    }

    public override void _Process(double delta)
    {
        if (_target == null) return;
        if (Input.IsActionJustPressed(InputSetup.ToggleCamera)) Cockpit = !Cockpit;

        var t = _target.GlobalTransform;
        if (Cockpit)
        {
            GlobalTransform = new Transform3D(t.Basis, t * CockpitEye);
            Fov = 70;
            return;
        }

        Fov = 65;
        float k = 1f - Mathf.Exp(-PositionSharpness * (float)delta);
        GlobalPosition = GlobalPosition.Lerp(DesiredChasePosition(), k);
        var forward = -t.Basis.Z;
        LookAt(t.Origin + forward * LookAhead + Vector3.Up * 0.8f, Vector3.Up);
    }

    private Vector3 DesiredChasePosition()
    {
        var t = _target!.GlobalTransform;
        return t.Origin + t.Basis.Z * Distance + Vector3.Up * Height;
    }
}
