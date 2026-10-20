namespace VehicleDynamics.Core.Reference;

/// <summary>
/// MATLAB colon operator a:d:b. Port of the published algorithm (Cleve Moler, colonop.m): the number of
/// intervals is round((b-a)/d) corrected by a 2*eps*max(|a|,|b|) tolerance, the right end point is snapped
/// to b when within tolerance, and the vector is built symmetrically from both ends.
/// Verified against MATLAB R2026a output in tests (Data/ml_colon.txt).
/// </summary>
public static class MatlabColon
{
    public static double[] Range(double a, double d, double b)
    {
        if (!double.IsFinite(a) || !double.IsFinite(d) || !double.IsFinite(b)) return new[] { double.NaN };
        if (d == 0 || (a < b && d < 0) || (b < a && d > 0)) return Array.Empty<double>();

        double tol = 2.0 * Eps(Math.Max(Math.Abs(a), Math.Abs(b)));
        double sig = Math.Sign(d);
        double nd;
        if (a == Math.Floor(a) && d == 1)
        {
            nd = Math.Floor(b) - a;
        }
        else if (a == Math.Floor(a) && d == Math.Floor(d))
        {
            nd = Math.Floor((b - a) / d);
        }
        else
        {
            nd = Math.Round((b - a) / d, MidpointRounding.AwayFromZero);
            if (sig * (a + nd * d - b) > tol) nd -= 1;
        }
        int n = (int)nd;
        double c = a + n * d;
        if (sig * (c - b) > -tol) c = b;

        var v = new double[n + 1];
        int half = n / 2;
        for (int k = 0; k <= half; k++)
        {
            v[k] = a + k * d;
            v[n - k] = c - k * d;
        }
        // MATLAB R2026a: a + (c - a)/2, not (a + c)/2 (verified on Data/ml_colon_random.txt)
        if (n % 2 == 0) v[n / 2] = a + (c - a) / 2;
        return v;
    }

    public static int Length(double a, double d, double b) => Range(a, d, b).Length;

    /// <summary>MATLAB eps(x): spacing of doubles at |x| (eps(0) = 2^-1074).</summary>
    public static double Eps(double x)
    {
        x = Math.Abs(x);
        if (x == 0) return double.Epsilon;
        if (!double.IsFinite(x)) return double.NaN;
        return Math.BitIncrement(x) - x;
    }
}
