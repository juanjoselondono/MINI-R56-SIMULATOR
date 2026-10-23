using System;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;

namespace Simulator.Dev;

/// <summary>
/// Scripted driver for tests: pure-pursuit steering on the track centreline (plus an optional lateral offset) and a
/// speed target from upcoming curvature and the car's friction-circle radius. It only produces driver input levels
/// (throttle, brake, normalised steer); the car moves exclusively through the physics model.
/// </summary>
public sealed class CentrelineDriver
{
    public ITrackLine Track { get; }
    /// <summary>Fraction of the lateral grip (F_grip / M) the speed planner uses.</summary>
    public double LateralMargin { get; set; } = 0.75;
    public double PlannedDecel { get; set; } = 4.5;
    public double MaxSpeed { get; set; } = 60;
    public double TargetLateral { get; set; }
    /// <summary>
    /// Pure-pursuit lookahead = max(7 m, time × v). 0.7 s suits the Mini; the F1 at 215 km/h needs a shorter one (0.45 s):
    /// with 42 m it corrected a 1.7 m drift on a sweeper exit too slowly and left the track limits.
    /// </summary>
    public double LookaheadTime { get; set; } = 0.7;

    public CentrelineDriver(ITrackLine track) => Track = track;

    public (double Throttle, double Brake, double Steer) Drive(in VehicleState s, in VehicleOutputs o, VehicleParameters p)
    {
        var pr = Track.Project(s.X, s.Y);
        double v = s.V;

        double ld = Math.Max(7, LookaheadTime * v);
        var (tx, ty) = Track.PointAt(pr.S + ld, TargetLateral);
        double alpha = FrameConversion.WrapAngle(Math.Atan2(ty - s.Y, tx - s.X) - s.Psi);
        double delta = Math.Atan(2 * p.Wheelbase * Math.Sin(alpha) / ld);
        double steer = Math.Clamp(delta / p.MaxSteerAngle, -1, 1);

        // Lateral capacity a(v) = A + B v². A: the static traction limit as before (model.m §10's planning proxy).
        // B (A1, 0 without aero): the downforce's lateral grip as the weaker axle sees it. An axle carrying a share f of
        // the cornering force (front c/L, rear 1 − c/L) gains μ_axle · (its downforce share) / f per unit downforce/mass.
        // Not the derivative of the driven axle's traction limit: for the RWD F1 that overstated the grip of the
        // front-limited car at speed, and it ran wide out of a 215 km/h sweeper.
        double aLat = LateralMargin * p.GripUtilization * TractionLimit.FxMax(p, p.Mu, p.Theta) / p.Mass;
        double frontShare = p.CgToRear / p.Wheelbase;
        double aeroAxle = Math.Min(p.AeroBalanceFront / frontShare, p.RearGripFactor * (1 - p.AeroBalanceFront) / (1 - frontShare));
        double aAero = LateralMargin * p.GripUtilization * p.Mu * aeroAxle * Aerodynamics.SpecificCoefficient(p);
        double target = MaxSpeed;
        for (double d = 0; d <= 160; d += 4)
        {
            double k = Math.Abs(Track.Curvature(pr.S + d));
            if (k < 1e-6 || k <= aAero) continue;
            double vCorner = Math.Sqrt(aLat / (k - aAero));
            target = Math.Min(target, Math.Sqrt(vCorner * vCorner + 2 * PlannedDecel * d));
        }

        if (v > target + 0.8) return (0, Math.Clamp((v - target) / 3, 0.2, 1), steer);
        if (v > target - 0.3) return (0.3, 0, steer);
        return (1, 0, steer);
    }
}
