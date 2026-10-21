using System;
using VehicleDynamics.Core.Collision;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace Simulator.Vehicle;

public readonly record struct ImpactEvent(double Time, double Severity, double PointX, double PointY, double NormalX, double NormalY);

/// <summary>
/// Where barrier contacts come from: analytic walls of a <see cref="TrackGeometry"/>, or shape queries against a
/// collision mesh (converted tracks). Inputs: footprint centre, heading, half extents (model frame).
/// </summary>
public delegate BarrierContactResult? BarrierContactSource(double x, double y, double psi, double halfLength, double halfWidth);

/// <summary>
/// Barrier collisions, run once per physics step (never per render frame). Detection comes from a
/// <see cref="BarrierContactSource"/> (analytic walls or the mesh track's wall query); the response is the documented
/// extension E3. With the E4 dynamic model (default): a rigid-body impulse at the contact point with restitution and
/// wall friction (<see cref="RigidBodyImpact"/>), one bouncing impulse per contact (<see cref="ContactLatch"/>), then the
/// inelastic version while the contact lasts. With E1: the point-mass response (<see cref="CollisionResponse"/>,
/// <see cref="SustainedContact"/>). The result reaches the state only through <see cref="Dyn.ApplyCollision"/>.
/// </summary>
public sealed class CollisionManager
{
    /// <summary>
    /// Footprint of the body shell without mirrors, measured from the JCW model (assets/vehicles/mini_jcw/mini_jcw_meta.json,
    /// chassis_aabb): z from −1.622 m (front bumper) to +2.143 m (rear bumper) relative to the CG, |x| ≤ 0.841 m.
    /// The geometric centre is therefore 0.261 m behind the CG.
    /// </summary>
    public double HalfLength { get; set; } = 1.882;
    public double HalfWidth { get; set; } = 0.841;
    public double CentreBehindCg { get; set; } = 0.261;

    private readonly ContactLatch _latch = new();

    public BarrierContactSource Contacts { get; set; }
    public bool InContact { get; private set; }
    public bool Scraping { get; private set; }
    /// <summary>Impact severity of the last step (|u_n| at entry, m/s), 0 if none.</summary>
    public double LastSeverity { get; private set; }
    public int ImpulseCount { get; private set; }
    /// <summary>Yaw-rate change caused by the last contact response [rad/s] (rigid-body form only).</summary>
    public double LastYawRateChange { get; private set; }
    /// <summary>Contact entries (step without contact followed by a step with contact).</summary>
    public int ContactEpisodes { get; private set; }
    /// <summary>Latch releases during a contact (the car moved away from the wall while still touching it).</summary>
    public int Rearms { get; private set; }
    /// <summary>Impulses applied in two consecutive physics steps (must stay 0).</summary>
    public int ConsecutiveImpulses { get; private set; }
    private bool _impulseLastStep;

    public event Action<ImpactEvent>? Impact;

    public CollisionManager(TrackGeometry track) : this((x, y, psi, hl, hw) => BarrierContact.Check(track, x, y, psi, hl, hw)) { }

    public CollisionManager(BarrierContactSource contacts) => Contacts = contacts;

    public void Reset()
    {
        _latch.Reset();
        InContact = Scraping = false;
        LastSeverity = 0;
    }

    /// <summary>Call after each physics step.</summary>
    public void AfterStep(Dyn dyn)
    {
        LastSeverity = 0;
        Scraping = false;
        var s = dyn.State;
        double c = Math.Cos(s.Psi), sn = Math.Sin(s.Psi);
        double cx = s.X - CentreBehindCg * c, cy = s.Y - CentreBehindCg * sn;

        var contact = Contacts(cx, cy, s.Psi, HalfLength, HalfWidth);
        if (contact is not { } k)
        {
            InContact = false;
            _impulseLastStep = false;
            _latch.Update(false, 0);
            return;
        }
        if (!InContact) ContactEpisodes++;
        InContact = true;

        var p = dyn.Parameters;
        if (dyn.Dynamic)
        {
            ResolveRigid(dyn, k);
            return;
        }
        // E1 (no yaw-rate state): point-mass response at the geometric centre with a bounded heading kick.
        var input = new CollisionInput(cx, cy, s.Psi, dyn.VelocityAngle, dyn.Speed, k.NormalX, k.NormalY, k.Penetration,
            p.CollisionRestitution, p.CollisionFrictionLoss, p.CollisionMaxYawKick);
        bool wasLatched = _latch.Latched;
        double un = CollisionResponse.NormalSpeed(input);
        bool impulse = _latch.Update(true, un);
        // Slow approaches are resting/sliding contact: non-penetration only, no bounce and no impact event.
        if (impulse && -un < p.CollisionBounceThreshold) impulse = false;
        if (wasLatched && !_latch.Latched) Rearms++;
        if (impulse && _impulseLastStep) ConsecutiveImpulses++;
        _impulseLastStep = impulse;
        var r = impulse ? CollisionResponse.Resolve(input) : SustainedContact.Resolve(input);

        // The response is computed for the geometric centre; carry it back to the CG with the new heading.
        var atCg = r with { X = r.X + CentreBehindCg * Math.Cos(r.Psi), Y = r.Y + CentreBehindCg * Math.Sin(r.Psi) };
        dyn.ApplyCollision(atCg);

        if (r.ImpulseApplied)
        {
            ImpulseCount++;
            LastSeverity = r.Severity;
            Impact?.Invoke(new ImpactEvent(s.Time, r.Severity, k.PointX, k.PointY, k.NormalX, k.NormalY));
        }
        Scraping = r.SustainedContact && s.V > 1.0;
    }

    /// <summary>
    /// E4: rigid-body impulse at the contact point (<see cref="RigidBodyImpact"/>), so off-centre hits change the yaw
    /// rate. Latching and the bounce threshold use the contact point's normal speed; latched or slow contacts get the
    /// inelastic (e = 0) version of the same impulse, which also carries wall friction while scraping.
    /// </summary>
    private void ResolveRigid(Dyn dyn, BarrierContactResult k)
    {
        var s = dyn.State;
        var p = dyn.Parameters;
        double c = Math.Cos(s.Psi), sn = Math.Sin(s.Psi);
        var input = new RigidContactInput(s.X, s.Y, s.Psi, s.V * c - s.Vy * sn, s.V * sn + s.Vy * c, s.YawRate,
            p.Mass, p.YawInertia, k.PointX, k.PointY, k.NormalX, k.NormalY, k.Penetration,
            p.CollisionRestitution, p.CollisionWallFriction);
        bool wasLatched = _latch.Latched;
        double vn = RigidBodyImpact.ContactNormalSpeed(input, out _, out _);
        bool impulse = _latch.Update(true, vn);
        if (impulse && -vn < p.CollisionBounceThreshold) impulse = false;
        if (wasLatched && !_latch.Latched) Rearms++;
        if (impulse && _impulseLastStep) ConsecutiveImpulses++;
        _impulseLastStep = impulse;

        var r = RigidBodyImpact.Resolve(input, bounce: impulse);
        double yawBefore = s.YawRate;
        dyn.ApplyCollision(r);
        LastYawRateChange = r.YawRate - yawBefore;
        if (r.ImpulseApplied)
        {
            ImpulseCount++;
            LastSeverity = r.Severity;
            Impact?.Invoke(new ImpactEvent(s.Time, r.Severity, k.PointX, k.PointY, k.NormalX, k.NormalY));
        }
        Scraping = r.SustainedContact && dyn.Speed > 1.0;
    }
}
