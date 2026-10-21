namespace VehicleDynamics.Core.Track;

/// <summary>
/// Closed reference line given by points (model frame), e.g. a measured racing line. Positions are piecewise linear in
/// arc length; the heading is interpolated between vertex tangents and the curvature is a central difference of the
/// heading over ±<see cref="CurvatureWindow"/> metres (smooths the polyline's vertex kinks). Projection uses a uniform
/// grid of segments plus a brute-force fallback, so it is cheap enough for every physics step.
/// Optional per-point track-edge distances (left/right of the line) define <see cref="Limits"/>.
/// </summary>
public sealed class PolylineTrack : ITrackLine
{
    public const double CurvatureWindow = 5.0;
    private const double GridCell = 25.0;

    private readonly double[] _x, _y, _s, _segHeading, _vertexHeading, _left, _right;
    private readonly Dictionary<(int, int), List<int>> _grid = new();

    public string Name { get; }
    public double Length { get; }
    public bool IsClosed => true;
    public int Count => _x.Length;

    public PolylineTrack(string name, IReadOnlyList<double> x, IReadOnlyList<double> y,
        IReadOnlyList<double>? sideLeft = null, IReadOnlyList<double>? sideRight = null, double defaultHalfWidth = 5)
    {
        if (x.Count != y.Count || x.Count < 3) throw new ArgumentException("Need at least 3 points.");
        Name = name;
        int n = x.Count;
        _x = x.ToArray();
        _y = y.ToArray();
        _left = sideLeft?.ToArray() ?? Enumerable.Repeat(defaultHalfWidth, n).ToArray();
        _right = sideRight?.ToArray() ?? Enumerable.Repeat(defaultHalfWidth, n).ToArray();
        _s = new double[n + 1];
        _segHeading = new double[n];
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            double dx = _x[j] - _x[i], dy = _y[j] - _y[i];
            _s[i + 1] = _s[i] + Math.Sqrt(dx * dx + dy * dy);
            _segHeading[i] = Math.Atan2(dy, dx);
        }
        Length = _s[n];
        _vertexHeading = new double[n];
        for (int i = 0; i < n; i++)
        {
            double a = _segHeading[(i - 1 + n) % n], b = _segHeading[i];
            _vertexHeading[i] = a + 0.5 * Simulation.FrameConversion.WrapAngle(b - a);
        }
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            int cx0 = Cell(Math.Min(_x[i], _x[j])), cx1 = Cell(Math.Max(_x[i], _x[j]));
            int cy0 = Cell(Math.Min(_y[i], _y[j])), cy1 = Cell(Math.Max(_y[i], _y[j]));
            for (int cx = cx0; cx <= cx1; cx++)
                for (int cy = cy0; cy <= cy1; cy++)
                {
                    if (!_grid.TryGetValue((cx, cy), out var list)) _grid[(cx, cy)] = list = new List<int>();
                    list.Add(i);
                }
        }
    }

    private static int Cell(double v) => (int)Math.Floor(v / GridCell);

    public double NormalizeS(double s)
    {
        s %= Length;
        if (s < 0) s += Length;
        return s;
    }

    private int SegmentAt(double s)
    {
        int i = Array.BinarySearch(_s, s);
        if (i < 0) i = ~i - 1;
        return Math.Clamp(i, 0, _x.Length - 1);
    }

    public Pose2 Pose(double s)
    {
        s = NormalizeS(s);
        int i = SegmentAt(s);
        int j = (i + 1) % _x.Length;
        double len = _s[i + 1] - _s[i];
        double t = len > 0 ? (s - _s[i]) / len : 0;
        double h0 = _vertexHeading[i];
        double h1 = h0 + Simulation.FrameConversion.WrapAngle(_vertexHeading[j] - h0);
        return new Pose2(_x[i] + t * (_x[j] - _x[i]), _y[i] + t * (_y[j] - _y[i]), h0 + t * (h1 - h0));
    }

    public double Curvature(double s)
    {
        double a = Pose(s - CurvatureWindow).Psi, b = Pose(s + CurvatureWindow).Psi;
        return Simulation.FrameConversion.WrapAngle(b - a) / (2 * CurvatureWindow);
    }

    public (double X, double Y) PointAt(double s, double lateral)
    {
        var p = Pose(s);
        return (p.X - lateral * Math.Sin(p.Psi), p.Y + lateral * Math.Cos(p.Psi));
    }

    public (double Left, double Right) Limits(double s)
    {
        s = NormalizeS(s);
        int i = SegmentAt(s);
        int j = (i + 1) % _x.Length;
        double len = _s[i + 1] - _s[i];
        double t = len > 0 ? (s - _s[i]) / len : 0;
        return (_left[i] + t * (_left[j] - _left[i]), _right[i] + t * (_right[j] - _right[i]));
    }

    public TrackProjection Project(double x, double y)
    {
        int cx = Cell(x), cy = Cell(y);
        TrackProjection best = default;
        double bestD = double.PositiveInfinity;
        bool found = false;
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                if (_grid.TryGetValue((cx + dx, cy + dy), out var list))
                    foreach (int i in list) found |= Consider(i, x, y, ref best, ref bestD);
        // Grid neighbourhood empty, or the nearest candidate is farther than a cell (far off the line): search all.
        if (!found || bestD > GridCell)
            for (int i = 0; i < _x.Length; i++) Consider(i, x, y, ref best, ref bestD);
        return best;
    }

    private bool Consider(int i, double x, double y, ref TrackProjection best, ref double bestD)
    {
        int j = (i + 1) % _x.Length;
        double ex = _x[j] - _x[i], ey = _y[j] - _y[i];
        double len2 = ex * ex + ey * ey;
        double t = len2 > 0 ? Math.Clamp(((x - _x[i]) * ex + (y - _y[i]) * ey) / len2, 0, 1) : 0;
        double px = _x[i] + t * ex, py = _y[i] + t * ey;
        double d = Math.Sqrt((x - px) * (x - px) + (y - py) * (y - py));
        if (d >= bestD) return true;
        bestD = d;
        double h = _segHeading[i];
        double lateral = -(x - px) * Math.Sin(h) + (y - py) * Math.Cos(h);
        double s = _s[i] + t * (_s[i + 1] - _s[i]);
        if (s >= Length) s -= Length;
        best = new TrackProjection(s, lateral, i, d, px, py, h);
        return true;
    }
}
