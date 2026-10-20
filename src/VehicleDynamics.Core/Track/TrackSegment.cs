namespace VehicleDynamics.Core.Track;

public enum SegmentKind { Straight, Arc, Clothoid }

public readonly record struct Pose2(double X, double Y, double Psi);

/// <summary>
/// Centerline segment with linearly varying curvature k(u) = K0 + (K1 - K0) u / Length (u = local arc length).
/// Straight and Arc are evaluated in closed form; Clothoid heading is closed form and its position uses
/// composite 10-point Gauss-Legendre quadrature (error ~1e-12 m for the panel size used).
/// Optional per-segment runoff overrides the track default (left/right of the travel direction).
/// </summary>
public sealed class TrackSegment
{
    public SegmentKind Kind { get; }
    public double Length { get; }
    public double K0 { get; }
    public double K1 { get; }
    public double? RunoffLeft { get; init; }
    public double? RunoffRight { get; init; }
    public string Name { get; init; } = "";

    // Set by TrackGeometry
    public Pose2 Start { get; internal set; }
    public double StartS { get; internal set; }

    private TrackSegment(SegmentKind kind, double length, double k0, double k1)
    {
        if (!(length >= 0) || !double.IsFinite(length)) throw new ArgumentOutOfRangeException(nameof(length));
        Kind = kind;
        Length = length;
        K0 = k0;
        K1 = k1;
    }

    public static TrackSegment Straight(double length, string name = "", double? runoffLeft = null, double? runoffRight = null)
        => new(SegmentKind.Straight, length, 0, 0) { Name = name, RunoffLeft = runoffLeft, RunoffRight = runoffRight };

    /// <summary>Circular arc; <paramref name="angleRad"/> &gt; 0 turns left (CCW), &lt; 0 turns right.</summary>
    public static TrackSegment Arc(double radius, double angleRad, string name = "", double? runoffLeft = null, double? runoffRight = null)
    {
        if (!(radius > 0)) throw new ArgumentOutOfRangeException(nameof(radius));
        if (angleRad == 0) throw new ArgumentOutOfRangeException(nameof(angleRad));
        double k = Math.Sign(angleRad) / radius;
        return new(SegmentKind.Arc, radius * Math.Abs(angleRad), k, k) { Name = name, RunoffLeft = runoffLeft, RunoffRight = runoffRight };
    }

    public static TrackSegment Clothoid(double length, double k0, double k1, string name = "", double? runoffLeft = null, double? runoffRight = null)
    {
        if (!(length > 0)) throw new ArgumentOutOfRangeException(nameof(length));
        return new(SegmentKind.Clothoid, length, k0, k1) { Name = name, RunoffLeft = runoffLeft, RunoffRight = runoffRight };
    }

    public TrackSegment WithLength(double length) => Kind switch
    {
        SegmentKind.Straight => Straight(length, Name, RunoffLeft, RunoffRight),
        _ => throw new InvalidOperationException("Only straights can be resized."),
    };

    public double Radius => K0 == 0 ? double.PositiveInfinity : 1 / Math.Abs(K0);
    public double TurnAngle => Kind == SegmentKind.Clothoid ? 0.5 * (K0 + K1) * Length : K0 * Length;

    public double Curvature(double u) => Kind == SegmentKind.Clothoid ? K0 + (K1 - K0) * Math.Clamp(u, 0, Length) / Length : K0;

    public double HeadingAt(Pose2 start, double u) => Kind switch
    {
        SegmentKind.Clothoid => start.Psi + K0 * u + (K1 - K0) * u * u / (2 * Length),
        _ => start.Psi + K0 * u,
    };

    public Pose2 PoseAt(Pose2 start, double u)
    {
        switch (Kind)
        {
            case SegmentKind.Straight:
                return new Pose2(start.X + u * Math.Cos(start.Psi), start.Y + u * Math.Sin(start.Psi), start.Psi);
            case SegmentKind.Arc:
            {
                double k = K0;
                double psi = start.Psi + k * u;
                return new Pose2(start.X + (Math.Sin(psi) - Math.Sin(start.Psi)) / k,
                                 start.Y - (Math.Cos(psi) - Math.Cos(start.Psi)) / k, psi);
            }
            default:
            {
                var (cx, sy) = ClothoidIntegral(start.Psi, u);
                return new Pose2(start.X + cx, start.Y + sy, HeadingAt(start, u));
            }
        }
    }

    public Pose2 PoseAt(double u) => PoseAt(Start, u);

    private static readonly double[] GlX =
    {
        -0.9739065285171717, -0.8650633666889845, -0.6794095682990244, -0.4333953941292472, -0.1488743389816312,
        0.1488743389816312, 0.4333953941292472, 0.6794095682990244, 0.8650633666889845, 0.9739065285171717,
    };
    private static readonly double[] GlW =
    {
        0.0666713443086881, 0.1494513491505806, 0.2190863625159820, 0.2692667193099963, 0.2955242247147529,
        0.2955242247147529, 0.2692667193099963, 0.2190863625159820, 0.1494513491505806, 0.0666713443086881,
    };

    private (double, double) ClothoidIntegral(double psi0, double u)
    {
        if (u <= 0) return (0, 0);
        int panels = Math.Max(1, (int)Math.Ceiling(u / 5.0));
        double h = u / panels;
        double sx = 0, sy = 0;
        var start = new Pose2(0, 0, psi0);
        for (int p = 0; p < panels; p++)
        {
            double a = p * h;
            for (int i = 0; i < GlX.Length; i++)
            {
                double t = a + 0.5 * h * (GlX[i] + 1);
                double th = HeadingAt(start, t);
                sx += GlW[i] * Math.Cos(th);
                sy += GlW[i] * Math.Sin(th);
            }
        }
        return (0.5 * h * sx, 0.5 * h * sy);
    }
}
