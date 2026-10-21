using VehicleDynamics.Core.Simulation;

namespace VehicleDynamics.Core.Collision;

public readonly record struct CollisionInput(
    double X, double Y, double Psi, double Beta, double V,
    double NormalX, double NormalY, double Penetration,
    double Restitution = 0.25, double FrictionLoss = 0.15, double MaxYawKick = 0.6);

/// <param name="SustainedContact">true when no impulse was due (latched) but the approaching normal velocity was removed.</param>
/// <param name="VelocityHeading">World direction of the CG velocity after the response [rad] (NaN if it is zero).</param>
/// <param name="YawRate">Yaw rate after the response [rad/s] (rigid-body impact); NaN = not computed (E1 responses).</param>
public readonly record struct CollisionResult(
    bool ImpulseApplied, double X, double Y, double Psi, double V, double Severity, double NormalSpeed,
    bool SustainedContact = false, double VelocityHeading = double.NaN, double YawRate = double.NaN);

/// <summary>Planar rigid body hitting a wall at a point (world/model frame; velocities in m/s, yaw rate in rad/s).</summary>
public readonly record struct RigidContactInput(
    double X, double Y, double Psi, double VelX, double VelY, double YawRate, double Mass, double YawInertia,
    double PointX, double PointY, double NormalX, double NormalY, double Penetration,
    double Restitution, double WallFriction);

/// <summary>
/// EXTENSION E3, rigid-body form (used with the E4 dynamic model, which has a yaw-rate state). The car is a planar
/// rigid body (mass M, yaw inertia I_zz); the wall is fixed. With r = contact point − CG, the contact-point velocity is
/// v_P = v + ω × r. If it approaches the wall (v_P·n &lt; 0) one impulse J = j_n n + j_t t is applied at the contact:
///   Δv_P = K J with K = [[1/M + r_n²/I, r_n r_t/I], [r_n r_t/I, 1/M + r_t²/I]], r_n = r × n, r_t = r × t;
///   targets v_n' = −e v_n (Newton restitution at the contact point) and v_t' = 0 (sticking); if the sticking impulse
///   leaves the friction cone |j_t| ≤ μ_w j_n, the contact slides: j_t = −μ_w j_n along the slip, j_n still meets the
///   restitution target (Coulomb wall friction). Then Δv = J / M, Δω = (r × J) / I_zz
/// An off-centre hit therefore turns the car: a front-corner impact yaws it away from the wall, and wall friction on
/// a glancing scrape adds a moment too. Kinetic energy ½M|v|² + ½I_zz ω² never increases: in the rare cases where
/// Newton restitution with friction would add energy, the frictionless impulse (which cannot, for e ≤ 1) is used. The heading itself is not changed by the impulse; it follows from the new yaw rate. Position is pushed out
/// by the penetration along n.
/// </summary>
public static class RigidBodyImpact
{
    public static double ContactNormalSpeed(in RigidContactInput c, out double nx, out double ny)
    {
        double nl = Math.Sqrt(c.NormalX * c.NormalX + c.NormalY * c.NormalY);
        if (!(nl > 0)) throw new ArgumentException("Contact normal must be non-zero.");
        nx = c.NormalX / nl;
        ny = c.NormalY / nl;
        double rx = c.PointX - c.X, ry = c.PointY - c.Y;
        return (c.VelX - c.YawRate * ry) * nx + (c.VelY + c.YawRate * rx) * ny;
    }

    /// <param name="bounce">false = sustained contact: perfectly inelastic (e = 0), no impact event.</param>
    public static CollisionResult Resolve(in RigidContactInput c, bool bounce)
    {
        double vn = ContactNormalSpeed(c, out double nx, out double ny);
        double pen = Math.Max(0, c.Penetration);
        double x = c.X + nx * pen, y = c.Y + ny * pen;
        double vx = c.VelX, vy = c.VelY, w = c.YawRate;
        bool approaching = vn < 0;
        if (approaching)
        {
            double invM = 1 / c.Mass, invI = 1 / c.YawInertia;
            double rx = c.PointX - c.X, ry = c.PointY - c.Y;
            double e = bounce ? Math.Clamp(c.Restitution, 0, 1) : 0;
            double mu = Math.Max(0, c.WallFriction);
            // Tangent along the contact point's sliding direction (any tangent if it is not sliding).
            double vpx = vx - w * ry, vpy = vy + w * rx;
            double tx = vpx - vn * nx, ty = vpy - vn * ny;
            double vt = Math.Sqrt(tx * tx + ty * ty);
            if (vt > 1e-12) { tx /= vt; ty /= vt; } else { tx = -ny; ty = nx; vt = 0; }
            // Contact-point velocity change per unit impulse in the (n, t) basis: Δv = K j.
            double rn = rx * ny - ry * nx, rt = rx * ty - ry * tx;
            double knn = invM + rn * rn * invI, ktt = invM + rt * rt * invI, knt = rn * rt * invI;
            // Targets: v_n' = −e v_n, v_t' = 0 (sticking).
            double bn = -(1 + e) * vn, bt = -vt;
            double det = knn * ktt - knt * knt;
            double jn = (bn * ktt - bt * knt) / det, jt = (bt * knn - bn * knt) / det;
            if (!(jn >= 0) || Math.Abs(jt) > mu * jn)
            {
                // Sliding: friction at the cone edge opposing the slip, normal impulse still meets the restitution target.
                double sgn = vt > 0 ? -1 : 0;
                double den = knn + sgn * mu * knt;
                if (den > 1e-12) { jn = bn / den; jt = sgn * mu * jn; }
                else { jn = bn / knn; jt = 0; }
            }
            (double nvx, double nvy, double nw) Apply(double n_, double t_)
            {
                double jx = n_ * nx + t_ * tx, jy = n_ * ny + t_ * ty;
                return (vx + jx * invM, vy + jy * invM, w + (rx * jy - ry * jx) * invI);
            }
            var (ax, ay, aw) = Apply(jn, jt);
            double e0 = c.Mass * (vx * vx + vy * vy) + c.YawInertia * w * w;
            if (c.Mass * (ax * ax + ay * ay) + c.YawInertia * aw * aw > e0)
                (ax, ay, aw) = Apply(bn / knn, 0);   // frictionless impulse never adds energy (e ≤ 1)
            vx = ax; vy = ay; w = aw;
        }
        double v = Math.Sqrt(vx * vx + vy * vy);
        return new CollisionResult(approaching && bounce, x, y, c.Psi, v, approaching ? -vn : 0, vn,
            SustainedContact: approaching && !bounce, VelocityHeading: v > 1e-12 ? Math.Atan2(vy, vx) : double.NaN, YawRate: w);
    }
}

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
