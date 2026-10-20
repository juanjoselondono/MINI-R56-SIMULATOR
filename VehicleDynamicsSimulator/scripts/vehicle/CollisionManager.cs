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
    /// Footprint of the body (without mirrors) measured from Mini_body.stl (assets/vehicles/mini_r56/mini_r56_meta.json):
    /// z from −1.756 m (front bumper) to +2.144 m (rear bumper) relative to the CG, |x| ≤ 0.85 m.
    /// The geometric centre is therefore 0.194 m behind the CG.
    /// </summary>
    public double HalfLength { get; set; } = 1.95;
    public double HalfWidth { get; set; } = 0.85;
    public double CentreBehindCg { get; set; } = 0.194;

    private readonly ContactLatch _latch = new();

    public TrackGeometry Track { get; set; }
    public bool InContact { get; private set; }
    public bool Scraping { get; private set; }
    /// <summary>Impact severity of the last step (|u_n| at entry, m/s), 0 if none.</summary>
    public double LastSeverity { get; private set; }
    public int ImpulseCount { get; private set; }

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
            _latch.Update(false, 0);
            return;
        }
        InContact = true;

        var p = dyn.Parameters;
        var input = new CollisionInput(cx, cy, s.Psi, dyn.Outputs.Beta, s.V, k.NormalX, k.NormalY, k.Penetration,
            p.CollisionRestitution, p.CollisionFrictionLoss, p.CollisionMaxYawKick);
        bool impulse = _latch.Update(true, CollisionResponse.NormalSpeed(input));
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
