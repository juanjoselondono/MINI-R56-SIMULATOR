using Godot;

namespace Simulator.Effects;

/// <summary>Visual feedback for barrier impacts (sparks). Purely cosmetic; never touches the physics.</summary>
public static class ImpactEffects
{
    private static ParticleProcessMaterial? _sparkProcess;
    private static Mesh? _sparkMesh;

    /// <summary>One-shot spark burst at a contact point; amount and speed scale with severity |u_n| [m/s].</summary>
    public static void Sparks(Node parent, Vector3 position, Vector3 normal, double severity)
    {
        _sparkProcess ??= new ParticleProcessMaterial
        {
            Spread = 55f,
            InitialVelocityMin = 3f,
            InitialVelocityMax = 9f,
            Gravity = new Vector3(0, -9.8f, 0),
            ScaleMin = 0.4f,
            ScaleMax = 1.0f,
            Color = new Color(1f, 0.75f, 0.3f),
        };
        _sparkMesh ??= new QuadMesh
        {
            Size = new Vector2(0.06f, 0.06f),
            Material = new StandardMaterial3D
            {
                ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded,
                BillboardMode = BaseMaterial3D.BillboardModeEnum.Particles,
                VertexColorUseAsAlbedo = true,
                EmissionEnabled = true,
                Emission = new Color(1f, 0.6f, 0.2f),
                EmissionEnergyMultiplier = 4f,
            },
        };

        float k = Mathf.Clamp((float)severity / 10f, 0.15f, 1.5f);
        var process = (ParticleProcessMaterial)_sparkProcess.Duplicate();
        process.Direction = (normal + Vector3.Up * 0.6f).Normalized();
        process.InitialVelocityMax = 9f * k + 2f;
        var p = new GpuParticles3D
        {
            Amount = (int)(40 * k) + 8,
            Lifetime = 0.6,
            OneShot = true,
            Explosiveness = 0.9f,
            ProcessMaterial = process,
            DrawPass1 = _sparkMesh,
            Position = position,
            Emitting = true,
        };
        parent.AddChild(p);
        p.GetTree().CreateTimer(1.5).Timeout += p.QueueFree;
    }
}
