using System;
using VehicleDynamics.Core.Collision;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace Simulator.Vehicle;

public readonly record struct ImpactEvent(double Time, double Severity, double PointX, double PointY, double NormalX, double NormalY);

/// <summary>
/// Barrier collisions, run once per physics step (never per render frame). Detection is analytic against the
/// track's barrier walls (core <see cref="BarrierContact"/>); the response is the documented extension E3:
/// one impulse on contact entry (<see cref="CollisionResponse"/>, latched by <see cref="ContactLatch"/>), then a
/// dissipative non-penetration constraint while the contact lasts (<see cref="SustainedContact"/>). The result
/// reaches the state only through <see cref="Dyn.ApplyCollision"/>.
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

    public TrackGeometry Track { get; set; }
    public bool InContact { get; private set; }
    public bool Scraping { get; private set; }
    /// <summary>Impact severity of the last step (|u_n| at entry, m/s), 0 if none.</summary>
    public double LastSeverity { get; private set; }
    public int ImpulseCount { get; private set; }
    /// <summary>Contact entries (step without contact followed by a step with contact).</summary>
    public int ContactEpisodes { get; private set; }
    /// <summary>Latch releases during a contact (the car moved away from the wall while still touching it).</summary>
    public int Rearms { get; private set; }
    /// <summary>Impulses applied in two consecutive physics steps (must stay 0).</summary>
    public int ConsecutiveImpulses { get; private set; }
    private bool _impulseLastStep;

    public event Action<ImpactEvent>? Impact;

    public CollisionManager(TrackGeometry track) => Track = track;

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

        var contact = BarrierContact.Check(Track, cx, cy, s.Psi, HalfLength, HalfWidth);
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
        // Speed and velocity angle of the CG (with E4 these include v_y, not just v_x).
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
}
