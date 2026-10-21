using VehicleDynamics.Core.Simulation;

namespace VehicleDynamics.Core.Collision;

public readonly record struct CollisionInput(
    double X, double Y, double Psi, double Beta, double V,
    double NormalX, double NormalY, double Penetration,
    double Restitution = 0.25, double FrictionLoss = 0.15, double MaxYawKick = 0.6);

/// <param name="SustainedContact">true when no impulse was due (latched) but the approaching normal velocity was removed.</param>
/// <param name="VelocityHeading">World direction of the CG velocity after the response [rad] (NaN if it is zero).</param>
public readonly record struct CollisionResult(
    bool ImpulseApplied, double X, double Y, double Psi, double V, double Severity, double NormalSpeed,
    bool SustainedContact = false, double VelocityHeading = double.NaN);

/// <summary>
/// EXTENSION E3 (not in MATLAB): barrier impact for a car whose velocity is V along psi + beta.
/// n points from the barrier into the track. u_n = u.n; when approaching (u_n &lt; 0):
/// u_n' = -e u_n, u_t' = max(0, 1 - k_t |u_n| / max(V, eps)) u_t, V' = |u'|, heading turned toward u' by at most
/// MaxYawKick. Position is always pushed out by the penetration along n. Severity = |u_n|.
/// Since e &lt;= 1 and the tangential factor is in [0, 1], V' &lt;= V (kinetic energy never increases).
/// </summary>
public static class CollisionResponse
{
    public static double NormalSpeed(in CollisionInput c)
    {
        double h = c.Psi + c.Beta;
        return c.V * (Math.Cos(h) * c.NormalX + Math.Sin(h) * c.NormalY);
    }

    /// <param name="allowImpulse">false while a contact latch says the impulse was already applied.</param>
    public static CollisionResult Resolve(in CollisionInput c, bool allowImpulse = true)
    {
        double nl = Math.Sqrt(c.NormalX * c.NormalX + c.NormalY * c.NormalY);
        if (!(nl > 0)) throw new ArgumentException("Contact normal must be non-zero.");
        double nx = c.NormalX / nl, ny = c.NormalY / nl;
        double pen = Math.Max(0, c.Penetration);
        double x = c.X + nx * pen;
        double y = c.Y + ny * pen;

        double h = c.Psi + c.Beta;
        double ux = c.V * Math.Cos(h), uy = c.V * Math.Sin(h);
        double un = ux * nx + uy * ny;
        if (!allowImpulse || !(un < 0))
            return new CollisionResult(false, x, y, c.Psi, c.V, 0, un, VelocityHeading: c.V > 0 ? h : double.NaN);

        double e = Math.Clamp(c.Restitution, 0, 1);
        double kt = Math.Clamp(c.FrictionLoss, 0, 1);
        double utx = ux - un * nx, uty = uy - un * ny;
        double unNew = -e * un;
        double factor = Math.Max(0, 1 - kt * Math.Abs(un) / Math.Max(c.V, 1e-6));
        double upx = unNew * nx + factor * utx;
        double upy = unNew * ny + factor * uty;
        double vNew = Math.Sqrt(upx * upx + upy * upy);

        double psi = c.Psi;
        double target = double.NaN;
        if (vNew > 1e-9)
        {
            target = Math.Atan2(upy, upx);
            double d = FrameConversion.WrapAngle(target - h);
            psi = c.Psi + Math.Clamp(d, -c.MaxYawKick, c.MaxYawKick);
        }
        return new CollisionResult(true, x, y, psi, vNew, Math.Abs(un), un, VelocityHeading: target);
    }
}

public static class SustainedContact
{
    /// <summary>
    /// Non-penetration constraint for a contact that already received its impulse (latched): if the car still
    /// approaches the wall (u_n &lt; 0), the normal velocity component is removed, V' = sqrt(V² − u_n²), with no bounce
    /// and no heading change. The position is pushed out as in <see cref="CollisionResponse.Resolve"/>. Dissipative
    /// (V' ≤ V), so pressing into a wall cannot build up speed; a head-on press stops the car.
    /// </summary>
    public static CollisionResult Resolve(in CollisionInput c)
    {
        var r = CollisionResponse.Resolve(c, allowImpulse: false);
        if (!(r.NormalSpeed < 0)) return r;
        double vt2 = Math.Max(0, c.V * c.V - r.NormalSpeed * r.NormalSpeed);
        double nl = Math.Sqrt(c.NormalX * c.NormalX + c.NormalY * c.NormalY);
        double h = c.Psi + c.Beta;
        double tx = c.V * Math.Cos(h) - r.NormalSpeed * c.NormalX / nl;
        double ty = c.V * Math.Sin(h) - r.NormalSpeed * c.NormalY / nl;
        double vNew = Math.Sqrt(vt2);
        return r with { V = vNew, SustainedContact = true, VelocityHeading = vNew > 1e-12 ? Math.Atan2(ty, tx) : double.NaN };
    }
}

/// <summary>
/// Contact latching: one impulse on contact entry; no further impulse while the contact persists unless the car
/// first separated (left contact, or moved away with u_n above <see cref="SeparationSpeed"/>) and approaches again.
/// </summary>
public sealed class ContactLatch
{
    public double SeparationSpeed { get; set; } = 0.05;
    public bool Latched { get; private set; }

    /// <summary>Returns true when an impulse must be applied this step.</summary>
    public bool Update(bool inContact, double normalSpeed)
    {
        if (!inContact)
        {
            Latched = false;
            return false;
        }
        if (Latched)
        {
            if (normalSpeed > SeparationSpeed) Latched = false;
            return false;
        }
        if (normalSpeed < 0)
        {
            Latched = true;
            return true;
        }
        return false;
    }

    public void Reset() => Latched = false;
}
