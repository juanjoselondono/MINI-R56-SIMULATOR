using System.Globalization;

namespace VehicleDynamics.Core.Reference;

/// <summary>
/// Every input of reference/matlab/model.m. Defaults are the model.m values.
/// Derived quantities (theta, R_w, K_phi) are computed with the same arithmetic order as model.m.
/// </summary>
public sealed record ReferenceParameters
{
    // §1 environment
    public double G { get; init; } = 9.81;
    public double RhoAir { get; init; } = 1.225;
    public string SlopeMode { get; init; } = "pct";
    public double SlopeValue { get; init; } = 0;
    public double FuelLHV { get; init; } = 44e6;
    public double FuelAFR { get; init; } = 14.7;

    // §2 vehicle
    public double M { get; init; } = 1420;
    public double Cd { get; init; } = 0.378;
    public double A { get; init; } = 1.99;
    public double[] GearRatios { get; init; } = { 3.214, 1.792, 1.194, 0.914, 0.784, 0.683 };
    public double FinalDrive { get; init; } = 4.353;
    public double EtaT { get; init; } = 0.92;
    public double[] MassFactors { get; init; } = { 0.50, 0.20, 0.18, 0.16, 0.14, 0.11 };
    public double Vd { get; init; } = 1.598e-3;
    public double CR { get; init; } = 11.0;
    public double TireWidth { get; init; } = 0.175;
    public double AspectRatio { get; init; } = 0.65;
    public double RimDiameterInch { get; init; } = 15;
    public double Mu { get; init; } = 0.90;
    public double L { get; init; } = 2.467;
    public double C { get; init; } = 1.529;
    public double H { get; init; } = 0.503;
    public double Tf { get; init; } = 1.459;
    public double Tr { get; init; } = 1.467;
    public double KsF { get; init; } = 28000;
    public double KsR { get; init; } = 37100;
    public double SF { get; init; } = 1.059;
    public double SR { get; init; } = 1.067;

    // §3 engine
    public double RpmAxisStart { get; init; } = 1000;
    public double RpmAxisStep { get; init; } = 50;
    public double RpmAxisEnd { get; init; } = 6500;
    public double[] RpmRaw { get; init; } = { 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4250, 4500, 5000, 5500, 6000, 6400, 6500 };
    public double[] TorqueRaw { get; init; } = { 75, 120, 135, 145, 150, 155, 158, 160, 158, 155, 148, 142, 125, 118 };

    // §4 speed grid v_kmh = 0.1:1:220
    public double VGridStart { get; init; } = 0.1;
    public double VGridStep { get; init; } = 1;
    public double VGridEnd { get; init; } = 220;

    // §10
    public double RCurva { get; init; } = 80;
    public double MarginSafety { get; init; } = 0.80;
    public double VSearchStart { get; init; } = 1;
    public double VSearchStep { get; init; } = 0.1;
    public double VSearchEnd { get; init; } = 220;
    public int CurveGearSearchMax { get; init; } = 5;

    // §11
    public double LRecta { get; init; } = 400;
    public double BrakeMuFactor { get; init; } = 0.90;
    public double BrakeMassFactor { get; init; } = 0.05;
    public double Dt { get; init; } = 0.005;
    public double TimeCap { get; init; } = 60;
    public double BrakeSpeedStep { get; init; } = 0.05;

    public static ReferenceParameters Default { get; } = new();

    public double Theta => SlopeMode.ToLowerInvariant() switch
    {
        "deg" => (Math.PI / 180) * SlopeValue,
        "pct" => Math.Atan(SlopeValue / 100),
        "rad" => SlopeValue,
        _ => throw new ArgumentException("Modo de pendiente no valido. Use 'deg', 'pct' o 'rad'."),
    };

    public double RimDiameter => RimDiameterInch * 0.0254;
    public double Rw => (RimDiameter / 2) + (TireWidth * AspectRatio);
    public double KPhiF => 0.5 * KsF * (SF * SF);
    public double KPhiR => 0.5 * KsR * (SR * SR);
    public double KPhi => KPhiF + KPhiR;

    /// <summary>
    /// Applies an override addressed by the model.m variable name (as in validation/scenarios.json).
    /// </summary>
    public ReferenceParameters WithOverride(string modelName, double value)
    {
        return modelName switch
        {
            "env.g" => this with { G = value },
            "env.rho_air" => this with { RhoAir = value },
            "slope_value" => this with { SlopeValue = value },
            "car.M" => this with { M = value },
            "car.Cd" => this with { Cd = value },
            "car.A" => this with { A = value },
            "car.final_drive" => this with { FinalDrive = value },
            "car.eta_t" => this with { EtaT = value },
            "car.mu" => this with { Mu = value },
            "car.L" => this with { L = value },
            "car.c" => this with { C = value },
            "car.h" => this with { H = value },
            "car.t_f" => this with { Tf = value },
            "car.t_r" => this with { Tr = value },
            "car.K_s_f" => this with { KsF = value },
            "car.K_s_r" => this with { KsR = value },
            "car.s_f" => this with { SF = value },
            "car.s_r" => this with { SR = value },
            "tire_width" => this with { TireWidth = value },
            "aspect_ratio" => this with { AspectRatio = value },
            "R_curva" => this with { RCurva = value },
            "margin_safety" => this with { MarginSafety = value },
            "L_recta" => this with { LRecta = value },
            "dt" => this with { Dt = value },
            _ => throw new ArgumentException($"Unknown or unsupported model.m override '{modelName}'."),
        };
    }

    public ReferenceParameters WithOverrides(IEnumerable<KeyValuePair<string, double>> overrides)
    {
        var p = this;
        foreach (var kv in overrides) p = p.WithOverride(kv.Key, kv.Value);
        return p;
    }

    /// <summary>Flat key=value view (model.m names) for CSV headers.</summary>
    public IReadOnlyList<KeyValuePair<string, string>> ToKeyValues()
    {
        string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);
        string Arr(double[] a) => "[" + string.Join(" ", a.Select(F)) + "]";
        return new List<KeyValuePair<string, string>>
        {
            new("env.g", F(G)), new("env.rho_air", F(RhoAir)), new("slope_mode", SlopeMode), new("slope_value", F(SlopeValue)),
            new("car.M", F(M)), new("car.Cd", F(Cd)), new("car.A", F(A)), new("car.gear_ratios", Arr(GearRatios)),
            new("car.final_drive", F(FinalDrive)), new("car.eta_t", F(EtaT)), new("car.mass_factors", Arr(MassFactors)),
            new("car.R_w", F(Rw)), new("car.mu", F(Mu)), new("car.L", F(L)), new("car.c", F(C)), new("car.h", F(H)),
            new("car.t_f", F(Tf)), new("car.t_r", F(Tr)), new("car.K_s_f", F(KsF)), new("car.K_s_r", F(KsR)),
            new("car.s_f", F(SF)), new("car.s_r", F(SR)), new("rpm_raw", Arr(RpmRaw)), new("torque_raw", Arr(TorqueRaw)),
            new("R_curva", F(RCurva)), new("margin_safety", F(MarginSafety)), new("L_recta", F(LRecta)), new("dt", F(Dt)),
        };
    }
}
