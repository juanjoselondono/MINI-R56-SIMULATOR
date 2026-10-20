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
    public TrackGeometry Track { get; }
    /// <summary>Fraction of the lateral grip (F_grip / M) the speed planner uses.</summary>
    public double LateralMargin { get; set; } = 0.75;
    public double PlannedDecel { get; set; } = 4.5;
    public double MaxSpeed { get; set; } = 60;
    public double TargetLateral { get; set; }

    public CentrelineDriver(TrackGeometry track) => Track = track;

    public (double Throttle, double Brake, double Steer) Drive(in VehicleState s, in VehicleOutputs o, VehicleParameters p)
    {
        var pr = Track.Project(s.X, s.Y);
        double v = s.V;

        double ld = Math.Max(7, 0.7 * v);
        var (tx, ty) = Track.PointAt(pr.S + ld, TargetLateral);
        double alpha = FrameConversion.WrapAngle(Math.Atan2(ty - s.Y, tx - s.X) - s.Psi);
        double delta = Math.Atan(2 * p.Wheelbase * Math.Sin(alpha) / ld);
        double steer = Math.Clamp(delta / p.MaxSteerAngle, -1, 1);

        double aLat = LateralMargin * p.GripUtilization * TractionLimit.FxMax(p, p.Mu) / p.Mass;
        double target = MaxSpeed;
        for (double d = 0; d <= 160; d += 4)
        {
            double k = Math.Abs(Track.Curvature(pr.S + d));
            if (k < 1e-6) continue;
            double vCorner = Math.Sqrt(aLat / k);
            target = Math.Min(target, Math.Sqrt(vCorner * vCorner + 2 * PlannedDecel * d));
        }

        if (v > target + 0.8) return (0, Math.Clamp((v - target) / 3, 0.2, 1), steer);
        if (v > target - 0.3) return (0.3, 0, steer);
        return (1, 0, steer);
    }
}
