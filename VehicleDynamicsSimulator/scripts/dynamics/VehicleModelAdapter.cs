using Godot;
using VehicleDynamics.Core.Simulation;

namespace Simulator.Dynamics;

/// <summary>
/// The single place where model-frame quantities (X east, Y north, psi CCW, z up) become Godot quantities
/// (Y up, -Z forward). Delegates the math to <see cref="FrameConversion"/>, which is unit-tested in the core.
/// </summary>
public static class VehicleModelAdapter
{
    public static Vector3 ToGodotPosition(double x, double y, double height = 0)
    {
        var p = FrameConversion.ToGodotPosition(x, y, height);
        return new Vector3((float)p.X, (float)p.Y, (float)p.Z);
    }

    public static Transform3D ToGodotTransform(double x, double y, double psi)
    {
        var basis = new Basis(Vector3.Up, (float)FrameConversion.ToGodotYaw(psi));
        return new Transform3D(basis, ToGodotPosition(x, y));
    }

    /// <summary>Planar pose of a Godot transform (used once to take the spawn pose from the scene).</summary>
    public static (double X, double Y, double Psi) FromGodotTransform(Transform3D t)
    {
        var (x, y) = FrameConversion.FromGodotPosition(new Vec3(t.Origin.X, t.Origin.Y, t.Origin.Z));
        // Heading from the forward (-Z) axis projected on the ground plane.
        var f = -t.Basis.Z;
        double psi = System.Math.Atan2(-f.Z, f.X);
        return (x, y, psi);
    }
}
