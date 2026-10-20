using VehicleDynamics.Core.Simulation;

namespace VehicleDynamics.Core.Track;

public readonly record struct TrackProjection(
    double S, double Lateral, int SegmentIndex, double Distance, double CenterX, double CenterY, double Heading);

/// <summary>Cross-section: road |lat| &lt;= RoadHalfWidth, curb up to +CurbWidth, grass (runoff) up to the barrier.</summary>
public sealed record CrossSection(double RoadHalfWidth = 6, double CurbWidth = 1, double Runoff = 10);

/// <summary>
/// Track centerline built from a start pose and an ordered list of segments. Used by both physics queries
/// and rendering (sampling). Lateral offsets are positive to the LEFT of the travel direction.
/// </summary>
public sealed class TrackGeometry
{
    private readonly TrackSegment[] _segments;

    public string Name { get; }
    public Pose2 StartPose { get; }
    public bool IsClosed { get; }
    public CrossSection Section { get; }
    public IReadOnlyList<TrackSegment> Segments => _segments;
    public double Length { get; }
    public Pose2 EndPose { get; }

    public TrackGeometry(string name, Pose2 start, IEnumerable<TrackSegment> segments, bool closed, CrossSection? section = null)
    {
        Name = name;
        StartPose = start;
        IsClosed = closed;
        Section = section ?? new CrossSection();
        _segments = segments.ToArray();
        if (_segments.Length == 0) throw new ArgumentException("Track needs at least one segment.");
        var pose = start;
        double s = 0;
        foreach (var seg in _segments)
        {
            seg.Start = pose;
            seg.StartS = s;
            pose = seg.PoseAt(pose, seg.Length);
            s += seg.Length;
        }
        Length = s;
        EndPose = pose;
    }

    /// <summary>End-pose minus start-pose (position distance [m], heading difference wrapped [rad]).</summary>
    public (double Position, double Heading) ClosureError()
    {
        double dx = EndPose.X - StartPose.X, dy = EndPose.Y - StartPose.Y;
        return (Math.Sqrt(dx * dx + dy * dy), Math.Abs(FrameConversion.WrapAngle(EndPose.Psi - StartPose.Psi)));
    }

    public double NormalizeS(double s)
    {
        if (!IsClosed) return Math.Clamp(s, 0, Length);
        s %= Length;
        if (s < 0) s += Length;
        return s;
    }

    public int SegmentIndexAt(double s)
    {
        s = NormalizeS(s);
        int lo = 0, hi = _segments.Length - 1;
        while (lo < hi)
        {
            int mid = (lo + hi + 1) >> 1;
            if (_segments[mid].StartS <= s) lo = mid; else hi = mid - 1;
        }
        return lo;
    }

    public Pose2 Pose(double s)
    {
        s = NormalizeS(s);
        var seg = _segments[SegmentIndexAt(s)];
        return seg.PoseAt(Math.Min(s - seg.StartS, seg.Length));
    }

    public (double X, double Y) Position(double s)
    {
        var p = Pose(s);
        return (p.X, p.Y);
    }

    public double Heading(double s) => Pose(s).Psi;

    public double Curvature(double s)
    {
        s = NormalizeS(s);
        var seg = _segments[SegmentIndexAt(s)];
        return seg.Curvature(s - seg.StartS);
    }

    /// <summary>World point at arc length s and lateral offset (positive left).</summary>
    public (double X, double Y) PointAt(double s, double lateral)
    {
        var p = Pose(s);
        return (p.X - lateral * Math.Sin(p.Psi), p.Y + lateral * Math.Cos(p.Psi));
    }

    public double RunoffLeft(int segment) => _segments[segment].RunoffLeft ?? Section.Runoff;
    public double RunoffRight(int segment) => _segments[segment].RunoffRight ?? Section.Runoff;
    public double BarrierOffsetLeft(int segment) => Section.RoadHalfWidth + RunoffLeft(segment);
    public double BarrierOffsetRight(int segment) => Section.RoadHalfWidth + RunoffRight(segment);

    public Surface SurfaceAt(int segment, double lateral)
    {
        double a = Math.Abs(lateral);
        if (a <= Section.RoadHalfWidth) return Surface.Asphalt;
        if (a <= Section.RoadHalfWidth + Section.CurbWidth) return Surface.Curb;
        double barrier = lateral >= 0 ? BarrierOffsetLeft(segment) : BarrierOffsetRight(segment);
        return a <= barrier ? Surface.Grass : Surface.Barrier;
    }

    public Surface SurfaceAt(double x, double y)
    {
        var p = Project(x, y);
        return SurfaceAt(p.SegmentIndex, p.Lateral);
    }

    /// <summary>Nearest centerline point over all segments.</summary>
    public TrackProjection Project(double x, double y)
    {
        TrackProjection best = default;
        double bestD = double.PositiveInfinity;
        for (int i = 0; i < _segments.Length; i++)
        {
            double u = ClosestLocal(_segments[i], x, y);
            var pose = _segments[i].PoseAt(u);
            double dx = x - pose.X, dy = y - pose.Y;
            double d = Math.Sqrt(dx * dx + dy * dy);
            if (d < bestD)
            {
                bestD = d;
                double lat = -dx * Math.Sin(pose.Psi) + dy * Math.Cos(pose.Psi);
                double s = _segments[i].StartS + u;
                if (IsClosed && s >= Length) s -= Length;
                best = new TrackProjection(s, lat, i, d, pose.X, pose.Y, pose.Psi);
            }
        }
        return best;
    }

    private static double ClosestLocal(TrackSegment seg, double x, double y)
    {
        var st = seg.Start;
        switch (seg.Kind)
        {
            case SegmentKind.Straight:
            {
                double t = (x - st.X) * Math.Cos(st.Psi) + (y - st.Y) * Math.Sin(st.Psi);
                return Math.Clamp(t, 0, seg.Length);
            }
            case SegmentKind.Arc:
            {
                double k = seg.K0;
                double cx = st.X - Math.Sin(st.Psi) / k, cy = st.Y + Math.Cos(st.Psi) / k;
                double phi0 = Math.Atan2(st.Y - cy, st.X - cx);
                double phi = Math.Atan2(y - cy, x - cx);
                double d = k > 0 ? phi - phi0 : phi0 - phi;
                d %= 2 * Math.PI;
                if (d < 0) d += 2 * Math.PI;
                double u = d / Math.Abs(k);
                if (u <= seg.Length) return u;
                return DistSq(seg, 0, x, y) <= DistSq(seg, seg.Length, x, y) ? 0 : seg.Length;
            }
            default:
            {
                int n = Math.Max(16, (int)Math.Ceiling(seg.Length / 2));
                double bestU = 0, bestD = double.PositiveInfinity;
                for (int i = 0; i <= n; i++)
                {
                    double u = seg.Length * i / n;
                    double d = DistSq(seg, u, x, y);
                    if (d < bestD) { bestD = d; bestU = u; }
                }
                double uu = bestU;
                for (int it = 0; it < 12; it++)
                {
                    var p = seg.PoseAt(uu);
                    double tx = Math.Cos(p.Psi), ty = Math.Sin(p.Psi);
                    double rx = p.X - x, ry = p.Y - y;
                    double g = rx * tx + ry * ty;
                    double gp = 1 + seg.Curvature(uu) * (-rx * ty + ry * tx);
                    if (Math.Abs(gp) < 1e-9) break;
                    double next = Math.Clamp(uu - g / gp, 0, seg.Length);
                    if (Math.Abs(next - uu) < 1e-12) { uu = next; break; }
                    uu = next;
                }
                return DistSq(seg, uu, x, y) <= bestD ? uu : bestU;
            }
        }
    }

    private static double DistSq(TrackSegment seg, double u, double x, double y)
    {
        var p = seg.PoseAt(u);
        return (p.X - x) * (p.X - x) + (p.Y - y) * (p.Y - y);
    }

    /// <summary>Uniform samples (s, pose) along the centerline, including both ends; for rendering.</summary>
    public IEnumerable<(double S, Pose2 Pose)> Sample(double maxSpacing)
    {
        foreach (var seg in _segments)
        {
            int n = Math.Max(1, (int)Math.Ceiling(seg.Length / maxSpacing));
            for (int i = 0; i < n; i++)
            {
                double u = seg.Length * i / n;
                yield return (seg.StartS + u, seg.PoseAt(u));
            }
        }
        yield return (Length, EndPose);
    }
}
