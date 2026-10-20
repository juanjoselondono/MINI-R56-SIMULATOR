namespace VehicleDynamics.Core.Reference;

public sealed class StraightResult
{
    public required double[] HistoryT { get; init; }
    public required double[] HistoryD { get; init; }
    /// <summary>km/h, as stored by model.m.</summary>
    public required double[] HistoryVKmh { get; init; }
    public required double DFrenado { get; init; }
    public required double TFrenado { get; init; }
    public required int BrakeSteps { get; init; }
    public required double DAccEfectiva { get; init; }
    public required double TAcc { get; init; }
    public required double TTotal { get; init; }
    public required double VMaxKmh { get; init; }
}

public sealed class OvalResult
{
    public required double VCurvaAdherenciaKmh { get; init; }
    public required int OptimalGearCurva { get; init; }
    public required double VMaxGearMotrizKmh { get; init; }
    public required double VCurvaFinalKmh { get; init; }
    public required double VCurvaFinalMs { get; init; }
    public required double LCurva { get; init; }
    public required StraightResult R1 { get; init; }
    public required StraightResult R2 { get; init; }
    public required double TCurva1Total { get; init; }
    public required double TCurva2Total { get; init; }
    public required double TVueltaCompleta { get; init; }
    public required double DistanciaLap { get; init; }
}

/// <summary>Literal port of model.m §10 (curve speed) and §11 (four-segment oval lap).</summary>
public static class ReferenceOval
{
    public static OvalResult Run(ReferenceParameters p, ReferenceTables t)
    {
        double theta = t.Theta;
        double fxMax = t.FxMax;

        // §10.1-10.2
        double fLimiteUtil = p.MarginSafety * fxMax;
        double[] vSearchKmh = MatlabColon.Range(p.VSearchStart, p.VSearchStep, p.VSearchEnd);
        var vSearchMs = new double[vSearchKmh.Length];
        for (int i = 0; i < vSearchKmh.Length; i++) vSearchMs[i] = vSearchKmh[i] / 3.6;

        double vCurvaAdhMs = 0;
        for (int i = 0; i < vSearchMs.Length; i++)
        {
            double vTest = vSearchMs[i];
            double vMphTest = vSearchKmh[i] / 1.60934;
            double fY = p.M * (vTest * vTest) / p.RCurva;
            double fDragC = 0.5 * p.RhoAir * p.A * p.Cd * (vTest * vTest);
            double fRollingC = (0.01 * (1 + (vMphTest / 100))) * (p.M * p.G * Math.Cos(theta));
            double fPendC = p.M * p.G * Math.Sin(theta);
            double fXReq = fDragC + fRollingC + fPendC;
            double fRes = Math.Sqrt(fXReq * fXReq + fY * fY);
            if (fRes > fLimiteUtil)
            {
                vCurvaAdhMs = i > 0 ? vSearchMs[i - 1] : vSearchMs[0];
                break;
            }
        }
        if (vCurvaAdhMs == 0) vCurvaAdhMs = vSearchMs.Max();
        double vCurvaAdhKmh = vCurvaAdhMs * 3.6;

        // §10.3
        int optimalGear = 0;
        double vMaxGearMotriz = 0;
        int gMax = Math.Min(p.CurveGearSearchMax, t.NumGears);
        for (int g = 0; g < gMax; g++)
        {
            double vMinG = t.VGearKmh[g].Min();
            double vMaxG = t.VGearKmh[g].Max();
            if (vCurvaAdhKmh >= vMinG && vCurvaAdhKmh <= vMaxG)
            {
                optimalGear = g + 1;
                int idxPos = FindLastNonNegative(t.AxMs2[g]);
                vMaxGearMotriz = idxPos >= 0 ? t.VGearKmh[g][idxPos] : vMaxG;
                break;
            }
        }
        if (optimalGear == 0)
        {
            optimalGear = 5;
            int idxPos = FindLastNonNegative(t.AxMs2[4]);
            vMaxGearMotriz = idxPos >= 0 ? t.VGearKmh[4][idxPos] : t.VGearKmh[4].Max();
        }

        // §10.4
        double vCurvaFinalKmh = ReferenceModel.MatlabMin(vCurvaAdhKmh, vMaxGearMotriz);
        double vCurvaFinalMs = vCurvaFinalKmh / 3.6;

        // §11.1
        double lCurva = Math.PI * p.RCurva;
        double muFrenado = p.BrakeMuFactor * p.Mu;
        double fBrakeTotal = muFrenado * (p.M * p.G * Math.Cos(theta));
        double mEqFrenado = p.M * (1 + p.BrakeMassFactor);
        double vTarget = vCurvaFinalMs;

        double vStartR1 = t.VGearKmh[0].Min() / 3.6;
        var r1 = Straight(p, t, theta, vStartR1, vTarget, fBrakeTotal, mEqFrenado);
        double tCurva1 = lCurva / vTarget;
        var r2 = Straight(p, t, theta, vTarget, vTarget, fBrakeTotal, mEqFrenado);
        double tCurva2 = lCurva / vTarget;

        double tVuelta = r1.TTotal + tCurva1 + r2.TTotal + tCurva2;
        double distLap = 2 * p.LRecta + 2 * lCurva;

        return new OvalResult
        {
            VCurvaAdherenciaKmh = vCurvaAdhKmh, OptimalGearCurva = optimalGear, VMaxGearMotrizKmh = vMaxGearMotriz,
            VCurvaFinalKmh = vCurvaFinalKmh, VCurvaFinalMs = vCurvaFinalMs, LCurva = lCurva, R1 = r1, R2 = r2,
            TCurva1Total = tCurva1, TCurva2Total = tCurva2, TVueltaCompleta = tVuelta, DistanciaLap = distLap,
        };
    }

    private static StraightResult Straight(ReferenceParameters p, ReferenceTables t, double theta, double vStart,
        double vTarget, double fBrakeTotal, double mEqFrenado)
    {
        double dt = p.Dt;
        double tt = 0, d = 0, v = vStart;
        var hT = new List<double>();
        var hD = new List<double>();
        var hV = new List<double>();
        int ng = t.NumGears;

        // Unique (stable) tables per gear are loop-invariant; compute once (same values MATLAB recomputes each step).
        var uniqX = new double[ng][];
        var uniqA = new double[ng][];
        for (int g = 0; g < ng; g++)
        {
            int[] idx = MatlabInterp.UniqueStableIndices(t.VGearKmh[g]);
            uniqX[g] = idx.Select(i => t.VGearKmh[g][i]).ToArray();
            uniqA[g] = idx.Select(i => t.AxReal[g][i]).ToArray();
        }
        var gearMax = new double[ng];
        for (int g = 0; g < ng; g++) gearMax[g] = t.VGearKmh[g].Max();

        while (d < p.LRecta && tt < p.TimeCap)
        {
            int currGear = 0; // MATLAB default curr_gear = 1 (kept when v exceeds every gear's max)
            for (int g = 0; g < ng; g++)
            {
                if (v * 3.6 <= gearMax[g]) { currGear = g; break; }
            }
            // MATLAB's x must be sorted for interp1; V_gear_kmh is strictly increasing so unique() is a no-op here.
            double aInst = MatlabInterp.LinearExtrap(uniqX[currGear], uniqA[currGear], v * 3.6);
            aInst = Math.Max(0, aInst);

            hT.Add(tt);
            hD.Add(d);
            hV.Add(v * 3.6);

            v = v + aInst * dt;
            d = d + v * dt + 0.5 * aInst * (dt * dt);
            tt = tt + dt;
        }

        double vEnd = hV[^1] / 3.6;
        double dFr = 0, tFr = 0;
        int brakeSteps = 0;
        if (vEnd > vTarget)
        {
            double[] vBrk = MatlabColon.Range(vEnd, -p.BrakeSpeedStep, vTarget);
            brakeSteps = vBrk.Length;
            for (int k = 0; k < vBrk.Length; k++)
            {
                double vk = vBrk[k];
                double vkMph = (vk * 3.6) / 1.60934;
                double fResK = 0.5 * p.RhoAir * p.A * p.Cd * (vk * vk)
                               + (0.01 * (1 + (vkMph / 100))) * (p.M * p.G * Math.Cos(theta))
                               + p.M * p.G * Math.Sin(theta);
                double aBrk = (fBrakeTotal + fResK) / mEqFrenado;
                double dtK = p.BrakeSpeedStep / aBrk;
                dFr = dFr + vk * dtK;
                tFr = tFr + dtK;
            }
        }

        double dAcc = Math.Max(0, p.LRecta - dFr);
        int idxC = hD.FindIndex(x => x >= dAcc);
        if (idxC < 0) idxC = hD.Count - 1;
        double tAcc = hT[idxC];
        return new StraightResult
        {
            HistoryT = hT.ToArray(), HistoryD = hD.ToArray(), HistoryVKmh = hV.ToArray(),
            DFrenado = dFr, TFrenado = tFr, BrakeSteps = brakeSteps, DAccEfectiva = dAcc, TAcc = tAcc,
            TTotal = tAcc + tFr, VMaxKmh = hV[idxC],
        };
    }

    private static int FindLastNonNegative(double[] a)
    {
        for (int i = a.Length - 1; i >= 0; i--) if (a[i] >= 0) return i;
        return -1;
    }
}
