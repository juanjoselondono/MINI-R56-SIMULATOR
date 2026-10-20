namespace VehicleDynamics.Core.Simulation;

public readonly record struct Vec3(double X, double Y, double Z);

/// <summary>
/// Model frame (X east, Y north, z up, psi CCW from +X) to Godot (Y up, -Z forward) — pure math twin of the
/// Godot adapter: godot = (X, h, -Y); node Rotation.Y = psi - pi/2.
/// </summary>
public static class FrameConversion
{
    public static Vec3 ToGodotPosition(double x, double y, double height = 0) => new(x, height, -y);

    public static (double X, double Y) FromGodotPosition(Vec3 g) => (g.X, -g.Z);

    public static double ToGodotYaw(double psi) => psi - Math.PI / 2;

    public static double FromGodotYaw(double rotationY) => rotationY + Math.PI / 2;

    /// <summary>Car forward axis in Godot coordinates: the node basis' -Z after Rotation.Y = psi - pi/2.</summary>
    public static Vec3 GodotForward(double psi)
    {
        double ry = ToGodotYaw(psi);
        // Rotation about +Y applied to (0, 0, -1): (-sin ry, 0, -cos ry) = (cos psi, 0, -sin psi)
        return new Vec3(-Math.Sin(ry), 0, -Math.Cos(ry));
    }

    /// <summary>Model-frame vector (vx, vy) expressed in Godot coordinates.</summary>
    public static Vec3 ToGodotVector(double vx, double vy, double vz = 0) => new(vx, vz, -vy);

    /// <summary>Wrap an angle to (-pi, pi].</summary>
    public static double WrapAngle(double a)
    {
        a = Math.IEEERemainder(a, 2 * Math.PI);
        if (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
