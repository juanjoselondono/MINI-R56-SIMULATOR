namespace VehicleDynamics.Core.Reference;

/// <summary>
/// Port of MATLAB interp1(x, y, xq, 'pchip') / pchip(): interior slopes from pchipslopes (weighted harmonic
/// mean, zero at sign changes), end slopes from pchipend (3-point shape-preserving), piecewise cubic Hermite
/// coefficients as in pwch.m, evaluated with ppval's Horner scheme. Outside [x1, xn] the end polynomials
/// extrapolate (MATLAB pchip default).
/// </summary>
public sealed class Pchip
{
    private readonly double[] _x;
    private readonly double[] _y;
    private readonly double[,] _coefs; // per interval: [c3, c2, c1, c0] in powers of (xq - x_k)

    public IReadOnlyList<double> X => _x;
    public IReadOnlyList<double> Y => _y;

    public Pchip(IReadOnlyList<double> x, IReadOnlyList<double> y)
    {
        if (x.Count != y.Count) throw new ArgumentException("x and y must have the same length.");
        if (x.Count < 2) throw new ArgumentException("pchip needs at least 2 points.");
        for (int i = 1; i < x.Count; i++)
            if (!(x[i] > x[i - 1])) throw new ArgumentException("x must be strictly increasing.");

        int n = x.Count;
        _x = x.ToArray();
        _y = y.ToArray();
        var h = new double[n - 1];
        var del = new double[n - 1];
        for (int k = 0; k < n - 1; k++)
        {
            h[k] = _x[k + 1] - _x[k];
            del[k] = (_y[k + 1] - _y[k]) / h[k];
        }
        var d = Slopes(h, del, n);

        _coefs = new double[n - 1, 4];
        for (int k = 0; k < n - 1; k++)
        {
            double dzzdx = (del[k] - d[k]) / h[k];
            double dzdxdx = (d[k + 1] - del[k]) / h[k];
            _coefs[k, 0] = (dzdxdx - dzzdx) / h[k];
            _coefs[k, 1] = 2 * dzzdx - dzdxdx;
            _coefs[k, 2] = d[k];
            _coefs[k, 3] = _y[k];
        }
    }

    private static double[] Slopes(double[] h, double[] del, int n)
    {
        var d = new double[n];
        if (n == 2)
        {
            d[0] = del[0];
            d[1] = del[0];
            return d;
        }
        for (int k = 0; k < n - 2; k++)
        {
            if (Math.Sign(del[k]) * Math.Sign(del[k + 1]) > 0)
            {
                double hs = h[k] + h[k + 1];
                double w1 = (h[k] + hs) / (3 * hs);
                double w2 = (hs + h[k + 1]) / (3 * hs);
                double dmax = Math.Max(Math.Abs(del[k]), Math.Abs(del[k + 1]));
                double dmin = Math.Min(Math.Abs(del[k]), Math.Abs(del[k + 1]));
                d[k + 1] = dmin / (w1 * (del[k] / dmax) + w2 * (del[k + 1] / dmax));
            }
        }
        d[0] = PchipEnd(h[0], h[1], del[0], del[1]);
        d[n - 1] = PchipEnd(h[n - 2], h[n - 3], del[n - 2], del[n - 3]);
        return d;
    }

    private static double PchipEnd(double h1, double h2, double del1, double del2)
    {
        double d = ((2 * h1 + h2) * del1 - h1 * del2) / (h1 + h2);
        if (Math.Sign(d) != Math.Sign(del1)) d = 0;
        else if (Math.Sign(del1) != Math.Sign(del2) && Math.Abs(d) > Math.Abs(3 * del1)) d = 3 * del1;
        return d;
    }

    public double Evaluate(double xq)
    {
        if (double.IsNaN(xq)) return double.NaN;
        int k = Interval(xq);
        double s = xq - _x[k];
        double v = _coefs[k, 0];
        v = s * v + _coefs[k, 1];
        v = s * v + _coefs[k, 2];
        v = s * v + _coefs[k, 3];
        return v;
    }

    public double[] Evaluate(IReadOnlyList<double> xq)
    {
        var r = new double[xq.Count];
        for (int i = 0; i < r.Length; i++) r[i] = Evaluate(xq[i]);
        return r;
    }

    /// <summary>ppval convention: interval k with x_k &lt;= xq &lt; x_{k+1}; first/last interval used outside.</summary>
    private int Interval(double xq)
    {
        int last = _x.Length - 2;
        if (xq < _x[1]) return 0;
        if (xq >= _x[last]) return last;
        int lo = 1, hi = last;
        while (hi - lo > 1)
        {
            int mid = (lo + hi) >> 1;
            if (xq >= _x[mid]) lo = mid; else hi = mid;
        }
        return lo;
    }
}
