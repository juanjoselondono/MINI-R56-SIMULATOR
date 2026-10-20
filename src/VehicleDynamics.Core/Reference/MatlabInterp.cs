namespace VehicleDynamics.Core.Reference;

/// <summary>Small MATLAB idioms used by model.m §11.</summary>
public static class MatlabInterp
{
    /// <summary>[u, idx] = unique(v, 'stable'): first occurrences, original order.</summary>
    public static int[] UniqueStableIndices(IReadOnlyList<double> v)
    {
        var seen = new HashSet<double>();
        var idx = new List<int>(v.Count);
        for (int i = 0; i < v.Count; i++)
            if (seen.Add(v[i])) idx.Add(i);
        return idx.ToArray();
    }

    /// <summary>
    /// interp1(x, y, xq, 'linear', 'extrap'); x must be sorted ascending (unique). Interval: x_k &lt;= xq &lt; x_{k+1},
    /// first/last interval for extrapolation. Formula t = (xq - x_k)/(x_{k+1} - x_k); (1 - t)*y_k + t*y_{k+1}:
    /// of seven algebraically equivalent forms tried, it is the only one that reproduces the MATLAB R2026a
    /// R1/R2 histories of all golden scenarios bit-for-bit (the others differ by 1 ulp).
    /// </summary>
    public static double LinearExtrap(IReadOnlyList<double> x, IReadOnlyList<double> y, double xq)
    {
        int n = x.Count;
        if (n == 0) return double.NaN;
        if (n == 1) return xq == x[0] ? y[0] : double.NaN;
        int k;
        if (xq < x[1]) k = 0;
        else if (xq >= x[n - 2]) k = n - 2;
        else
        {
            int lo = 1, hi = n - 2;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) >> 1;
                if (xq >= x[mid]) lo = mid; else hi = mid;
            }
            k = lo;
        }
        double t = (xq - x[k]) / (x[k + 1] - x[k]);
        return (1 - t) * y[k] + t * y[k + 1];
    }
}
