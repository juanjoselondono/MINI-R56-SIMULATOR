namespace VehicleDynamics.Core.Track;

public readonly record struct BarrierContactResult(
    double NormalX, double NormalY, double Penetration, double PointX, double PointY, bool LeftSide, double S, int Corner);

/// <summary>
/// Car footprint (rectangle centred on the CG) against the continuous barrier walls at the cross-section offsets.
/// Each of the 4 corners is projected on the centerline; a corner beyond a barrier offset is penetrating.
/// The normal is the centerline normal at the projection (exact for offset curves), pointing back into the track.
/// </summary>
public static class BarrierContact
{
    public static BarrierContactResult? Check(TrackGeometry track, double x, double y, double psi, double halfLength, double halfWidth)
    {
        double c = Math.Cos(psi), s = Math.Sin(psi);
        BarrierContactResult? best = null;
        int corner = 0;
        foreach (var (fl, fw) in new[] { (1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0) })
        {
            double px = x + fl * halfLength * c - fw * halfWidth * s;
            double py = y + fl * halfLength * s + fw * halfWidth * c;
            var p = track.Project(px, py);
            double nlx = -Math.Sin(p.Heading), nly = Math.Cos(p.Heading);
            double left = track.BarrierOffsetLeft(p.SegmentIndex);
            double right = track.BarrierOffsetRight(p.SegmentIndex);
            BarrierContactResult? r = null;
            if (p.Lateral > left)
                r = new BarrierContactResult(-nlx, -nly, p.Lateral - left, px, py, true, p.S, corner);
            else if (p.Lateral < -right)
                r = new BarrierContactResult(nlx, nly, -right - p.Lateral, px, py, false, p.S, corner);
            if (r is { } rr && (best is null || rr.Penetration > best.Value.Penetration)) best = rr;
            corner++;
        }
        return best;
    }
}
