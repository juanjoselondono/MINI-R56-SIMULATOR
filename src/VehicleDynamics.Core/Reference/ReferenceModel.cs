namespace VehicleDynamics.Core.Reference;

/// <summary>Tables of model.m sections 1-7 (+ the §9 acceleration envelope). Row index = gear-1.</summary>
public sealed class ReferenceTables
{
    public required ReferenceParameters Parameters { get; init; }
    public required double Theta { get; init; }
    public required double Rw { get; init; }
    public required double[] RpmAxis { get; init; }
    public required double[] Omega { get; init; }
    public required double[] TEngine { get; init; }
    public required double[] PEngineW { get; init; }
    public required double[] HpEngine { get; init; }

    public required double[] VKmh { get; init; }
    public required double[] VMs { get; init; }
    public required double[] FDrag { get; init; }
    public required double[] FRolling { get; init; }
    public required double[] FPend { get; init; }
    public required double[] FResistive { get; init; }
    public required double[] PDemandedW { get; init; }
    public required double[] PDemandedKW { get; init; }
    public required double[] AxEnvelope { get; init; }

    public required double[][] VGearKmh { get; init; }
    public required double[][] PAvailableKW { get; init; }
    public required double[][] TWheelNm { get; init; }
    public required double[][] FTractiveN { get; init; }
    public required double[][] AxMs2 { get; init; }
    public required double[][] FTractiveEffective { get; init; }
    public required double[][] AxReal { get; init; }

    public required double FxMax { get; init; }
    public int NumGears => VGearKmh.Length;
}

/// <summary>Literal port of model.m §1-§7 and the §9 envelope (same loop and arithmetic order).</summary>
public static class ReferenceModel
{
    public static ReferenceTables Build(ReferenceParameters p)
    {
        // §1-2
        double theta = p.Theta;
        double rw = p.Rw;
        double kPhiR = p.KPhiR;
        double kPhi = p.KPhi;

        // §3
        double[] rpmAxis = MatlabColon.Range(p.RpmAxisStart, p.RpmAxisStep, p.RpmAxisEnd);
        int nr = rpmAxis.Length;
        double wFactor = 2 * Math.PI / 60;
        var omega = new double[nr];
        for (int i = 0; i < nr; i++) omega[i] = rpmAxis[i] * wFactor;
        var tEngine = new Pchip(p.RpmRaw, p.TorqueRaw).Evaluate(rpmAxis);
        var pEngine = new double[nr];
        var hp = new double[nr];
        for (int i = 0; i < nr; i++)
        {
            pEngine[i] = tEngine[i] * omega[i];
            hp[i] = pEngine[i] / 745.7;
        }

        // §4
        double[] vKmh = MatlabColon.Range(p.VGridStart, p.VGridStep, p.VGridEnd);
        int nv = vKmh.Length;
        var vMs = new double[nv];
        var fDrag = new double[nv];
        var fRolling = new double[nv];
        var fPend = new double[nv];
        var fRes = new double[nv];
        var pDemW = new double[nv];
        var pDemKW = new double[nv];
        double dragK = 0.5 * p.RhoAir * p.A * p.Cd;
        double fNormal = p.M * p.G * Math.Cos(theta);
        double fPendScalar = p.M * p.G * Math.Sin(theta);
        for (int i = 0; i < nv; i++)
        {
            vMs[i] = vKmh[i] / 3.6;
            double vMph = vKmh[i] / 1.60934;
            fDrag[i] = dragK * (vMs[i] * vMs[i]);
            double frDyn = 0.01 * (1 + (vMph / 100));
            fRolling[i] = frDyn * fNormal;
            fPend[i] = fPendScalar;
            fRes[i] = fDrag[i] + fRolling[i] + fPend[i];
            pDemW[i] = fRes[i] * vMs[i];
            pDemKW[i] = pDemW[i] / 1000;
        }

        // §5
        int ng = p.GearRatios.Length;
        var vGear = NewJagged(ng, nr);
        var pAvail = NewJagged(ng, nr);
        var tWheel = NewJagged(ng, nr);
        var fTract = NewJagged(ng, nr);
        var ax = NewJagged(ng, nr);
        for (int g = 0; g < ng; g++)
        {
            double totalRatio = p.GearRatios[g] * p.FinalDrive;
            double mEq = p.M * (1 + p.MassFactors[g]);
            for (int i = 0; i < nr; i++)
            {
                double wWheel = omega[i] / totalRatio;
                double vWheel = wWheel * rw;
                vGear[g][i] = vWheel * 3.6;
                pAvail[g][i] = (pEngine[i] * p.EtaT) / 1000;
                tWheel[g][i] = tEngine[i] * totalRatio * p.EtaT;
                fTract[g][i] = tWheel[g][i] / rw;
                double fResG = GearResistance(p, theta, dragK, vWheel, vGear[g][i]);
                ax[g][i] = (fTract[g][i] - fResG) / mEq;
            }
        }

        // §6
        double fxMax = FxMax(p, theta, p.Mu, rw, kPhiR, kPhi);

        // §7
        var fEff = NewJagged(ng, nr);
        var axReal = NewJagged(ng, nr);
        for (int g = 0; g < ng; g++)
        {
            double totalRatio = p.GearRatios[g] * p.FinalDrive;
            double mEq = p.M * (1 + p.MassFactors[g]);
            for (int i = 0; i < nr; i++)
            {
                fEff[g][i] = MatlabMin(fTract[g][i], fxMax);
                double wWheel = omega[i] / totalRatio;
                double vWheel = wWheel * rw;
                double fResG = GearResistance(p, theta, dragK, vWheel, vGear[g][i]);
                axReal[g][i] = (fEff[g][i] - fResG) / mEq;
            }
        }

        // §9 envelope
        double pMax = double.NegativeInfinity;
        foreach (var v in pEngine) pMax = Math.Max(pMax, v);
        double pMaxAvail = pMax * p.EtaT;
        double mfSum = 0;
        foreach (var m in p.MassFactors) mfSum += m;
        double mEqAvg = p.M * (1 + mfSum / p.MassFactors.Length);
        var env = new double[nv];
        for (int i = 0; i < nv; i++)
        {
            double pExcess = Math.Max(0, pMaxAvail - pDemW[i]);
            double fExcess = pExcess / vMs[i];
            env[i] = fExcess / mEqAvg;
        }

        return new ReferenceTables
        {
            Parameters = p, Theta = theta, Rw = rw, RpmAxis = rpmAxis, Omega = omega, TEngine = tEngine,
            PEngineW = pEngine, HpEngine = hp, VKmh = vKmh, VMs = vMs, FDrag = fDrag, FRolling = fRolling,
            FPend = fPend, FResistive = fRes, PDemandedW = pDemW, PDemandedKW = pDemKW, AxEnvelope = env,
            VGearKmh = vGear, PAvailableKW = pAvail, TWheelNm = tWheel, FTractiveN = fTract, AxMs2 = ax,
            FTractiveEffective = fEff, AxReal = axReal, FxMax = fxMax,
        };
    }

    // model.m lines 127-131 / 161-166
    private static double GearResistance(ReferenceParameters p, double theta, double dragK, double vWheelMs, double vGearKmh)
    {
        double vMphGear = vGearKmh / 1.60934;
        double fDragG = dragK * (vWheelMs * vWheelMs);
        double fRollingG = (0.01 * (1 + (vMphGear / 100))) * (p.M * p.G * Math.Cos(theta));
        double fPendG = p.M * p.G * Math.Sin(theta);
        return fDragG + fRollingG + fPendG;
    }

    /// <summary>model.m §6, same arithmetic order.</summary>
    public static double FxMax(ReferenceParameters p, double theta, double mu, double rw, double kPhiR, double kPhi)
    {
        double w = p.M * p.G * Math.Cos(theta);
        double num = mu * (w * p.C / p.L);
        double den = 1 + (p.H / p.L) * mu + ((2 * mu * rw) / (p.FinalDrive * p.Tf)) * (kPhiR / kPhi);
        return num / den;
    }

    /// <summary>MATLAB min(a,b) ignores NaN.</summary>
    public static double MatlabMin(double a, double b)
    {
        if (double.IsNaN(a)) return b;
        if (double.IsNaN(b)) return a;
        return a <= b ? a : b;
    }

    private static double[][] NewJagged(int rows, int cols)
    {
        var r = new double[rows][];
        for (int i = 0; i < rows; i++) r[i] = new double[cols];
        return r;
    }
}
