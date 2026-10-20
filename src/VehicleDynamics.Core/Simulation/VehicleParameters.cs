using System.Text.Json.Serialization;
using VehicleDynamics.Core.Reference;

namespace VehicleDynamics.Core.Simulation;

public enum SlopeMode { Deg, Pct, Rad }

public enum Surface { Asphalt = 0, Curb = 1, Grass = 2, Barrier = 3 }

/// <summary>
/// Parameters of the interactive model. Group 1 = model.m inputs (defaults identical to model.m).
/// Group 2 = seed-game rules (reference/js_seed/monaco.html). Group 3 = extensions E1-E3 (not in MATLAB,
/// provisional gameplay values). Call <see cref="Validate"/> (or <see cref="EnsureValid"/>) after editing.
/// </summary>
public sealed class VehicleParameters
{
    // ---- Group 1: model.m ----
    public double G { get; set; } = 9.81;
    public double RhoAir { get; set; } = 1.225;
    public SlopeMode SlopeMode { get; set; } = SlopeMode.Pct;
    public double SlopeValue { get; set; } = 0;
    public double Mass { get; set; } = 1420;
    public double Cd { get; set; } = 0.378;
    public double FrontalArea { get; set; } = 1.99;
    public double[] GearRatios { get; set; } = { 3.214, 1.792, 1.194, 0.914, 0.784, 0.683 };
    public double FinalDrive { get; set; } = 4.353;
    public double EtaT { get; set; } = 0.92;
    public double[] MassFactors { get; set; } = { 0.50, 0.20, 0.18, 0.16, 0.14, 0.11 };
    public double TireWidth { get; set; } = 0.175;
    public double AspectRatio { get; set; } = 0.65;
    public double RimDiameterInch { get; set; } = 15;
    public double Mu { get; set; } = 0.90;
    public double Wheelbase { get; set; } = 2.467;
    /// <summary>model.m car.c: CG to REAR axle [m].</summary>
    public double CgToRear { get; set; } = 1.529;
    public double CgHeight { get; set; } = 0.503;
    public double TrackFront { get; set; } = 1.459;
    public double TrackRear { get; set; } = 1.467;
    public double SpringFront { get; set; } = 28000;
    public double SpringRear { get; set; } = 37100;
    public double SpringSepFront { get; set; } = 1.059;
    public double SpringSepRear { get; set; } = 1.067;
    public double[] RpmRaw { get; set; } = { 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4250, 4500, 5000, 5500, 6000, 6400, 6500 };
    public double[] TorqueRaw { get; set; } = { 75, 120, 135, 145, 150, 155, 158, 160, 158, 155, 148, 142, 125, 118 };
    public double BrakeMuFactor { get; set; } = 0.90;
    public double BrakeMassFactor { get; set; } = 0.05;
    /// <summary>model.m §10 planning margin; used by validation/planning only, not by the driving model.</summary>
    public double MarginSafety { get; set; } = 0.80;
    /// <summary>Fixed physics step [s]; model.m dt.</summary>
    public double Dt { get; set; } = 0.005;

    // ---- Group 2: seed-game rules (monaco.html), not in MATLAB ----
    public double IdleRpm { get; set; } = 1000;
    public double FuelCutRpm { get; set; } = 6500;
    public double ClutchTargetRpm { get; set; } = 3500;
    public double ClutchThrottleThreshold { get; set; } = 0.1;
    public bool ClutchLaunch { get; set; } = true;
    public double EngineBrakeTorque { get; set; } = 30;
    public double MoneyShiftRpm { get; set; } = 7200;
    public bool MechanicalDamage { get; set; } = true;
    public double BrakeInputThreshold { get; set; } = 0.05;
    public double ThrottleInputThreshold { get; set; } = 0.01;

    // ---- Group 3: extensions (provisional) ----
    /// <summary>E1: steering angle at full input [rad]. PROVISIONAL (seed used 0.06).</summary>
    public double MaxSteerAngle { get; set; } = 0.30;
    /// <summary>E1: friction-circle radius = GripUtilization * F_x,max(mu_surface).</summary>
    public double GripUtilization { get; set; } = 1.0;
    /// <summary>E3 collision restitution e.</summary>
    public double CollisionRestitution { get; set; } = 0.25;
    /// <summary>E3 tangential friction loss k_t.</summary>
    public double CollisionFrictionLoss { get; set; } = 0.15;
    /// <summary>E3 maximum heading change per impact [rad].</summary>
    public double CollisionMaxYawKick { get; set; } = 0.6;
    /// <summary>E2 grip factor multiplying mu: [asphalt, curb, grass].</summary>
    public double[] SurfaceGripFactor { get; set; } = { 1.0, 0.95, 0.6 };
    /// <summary>E2 factor multiplying the rolling coefficient: [asphalt, curb, grass]. Curb value 1.0 is a provisional choice.</summary>
    public double[] SurfaceRollingFactor { get; set; } = { 1.0, 1.0, 4.0 };

    [JsonIgnore]
    public double Theta => SlopeMode switch
    {
        SlopeMode.Deg => (Math.PI / 180) * SlopeValue,
        SlopeMode.Pct => Math.Atan(SlopeValue / 100),
        _ => SlopeValue,
    };

    [JsonIgnore]
    public double WheelRadius => (RimDiameterInch * 0.0254 / 2) + (TireWidth * AspectRatio);
    [JsonIgnore]
    public double KPhiFront => 0.5 * SpringFront * (SpringSepFront * SpringSepFront);
    [JsonIgnore]
    public double KPhiRear => 0.5 * SpringRear * (SpringSepRear * SpringSepRear);
    [JsonIgnore]
    public double KPhi => KPhiFront + KPhiRear;
    [JsonIgnore]
    public int NumGears => GearRatios.Length;

    public VehicleParameters Clone()
    {
        var c = (VehicleParameters)MemberwiseClone();
        c.GearRatios = (double[])GearRatios.Clone();
        c.MassFactors = (double[])MassFactors.Clone();
        c.RpmRaw = (double[])RpmRaw.Clone();
        c.TorqueRaw = (double[])TorqueRaw.Clone();
        c.SurfaceGripFactor = (double[])SurfaceGripFactor.Clone();
        c.SurfaceRollingFactor = (double[])SurfaceRollingFactor.Clone();
        return c;
    }

    /// <summary>Maps the model.m inputs of a reference parameter set (validation runs).</summary>
    public static VehicleParameters FromReference(ReferenceParameters r)
    {
        return new VehicleParameters
        {
            G = r.G, RhoAir = r.RhoAir,
            SlopeMode = r.SlopeMode.ToLowerInvariant() switch { "deg" => SlopeMode.Deg, "rad" => SlopeMode.Rad, _ => SlopeMode.Pct },
            SlopeValue = r.SlopeValue, Mass = r.M, Cd = r.Cd, FrontalArea = r.A,
            GearRatios = (double[])r.GearRatios.Clone(), FinalDrive = r.FinalDrive, EtaT = r.EtaT,
            MassFactors = (double[])r.MassFactors.Clone(), TireWidth = r.TireWidth, AspectRatio = r.AspectRatio,
            RimDiameterInch = r.RimDiameterInch, Mu = r.Mu, Wheelbase = r.L, CgToRear = r.C, CgHeight = r.H,
            TrackFront = r.Tf, TrackRear = r.Tr, SpringFront = r.KsF, SpringRear = r.KsR, SpringSepFront = r.SF,
            SpringSepRear = r.SR, RpmRaw = (double[])r.RpmRaw.Clone(), TorqueRaw = (double[])r.TorqueRaw.Clone(),
            BrakeMuFactor = r.BrakeMuFactor, BrakeMassFactor = r.BrakeMassFactor, MarginSafety = r.MarginSafety, Dt = r.Dt,
        };
    }

    /// <summary>Returns every violated rule (empty = valid).</summary>
    public IReadOnlyList<string> Validate()
    {
        var e = new List<string>();
        void R(string name, double v, double lo, double hi)
        {
            if (!double.IsFinite(v) || v < lo || v > hi) e.Add($"{name}={v} outside [{lo}, {hi}]");
        }
        R(nameof(G), G, 1, 30);
        R(nameof(RhoAir), RhoAir, 0.5, 2);
        switch (SlopeMode)
        {
            case SlopeMode.Pct: R(nameof(SlopeValue), SlopeValue, -40, 40); break;
            case SlopeMode.Deg: R(nameof(SlopeValue), SlopeValue, -22, 22); break;
            default: R(nameof(SlopeValue), SlopeValue, -0.39, 0.39); break;
        }
        if (!Enum.IsDefined(SlopeMode)) e.Add("SlopeMode invalid");
        R(nameof(Mass), Mass, 300, 5000);
        R(nameof(Cd), Cd, 0.1, 1.5);
        R(nameof(FrontalArea), FrontalArea, 0.5, 10);
        if (GearRatios is null || GearRatios.Length < 1 || GearRatios.Length > 10) e.Add("GearRatios must have 1..10 entries");
        else
        {
            for (int i = 0; i < GearRatios.Length; i++) R($"GearRatios[{i}]", GearRatios[i], 0.2, 8);
            for (int i = 1; i < GearRatios.Length; i++)
                if (!(GearRatios[i] < GearRatios[i - 1])) e.Add("GearRatios must be strictly decreasing");
        }
        if (MassFactors is null || GearRatios is null || MassFactors.Length != GearRatios.Length) e.Add("MassFactors must have one entry per gear");
        else for (int i = 0; i < MassFactors.Length; i++) R($"MassFactors[{i}]", MassFactors[i], 0, 2);
        R(nameof(FinalDrive), FinalDrive, 1, 10);
        R(nameof(EtaT), EtaT, 0.5, 1);
        R(nameof(TireWidth), TireWidth, 0.1, 0.4);
        R(nameof(AspectRatio), AspectRatio, 0.2, 1);
        R(nameof(RimDiameterInch), RimDiameterInch, 10, 24);
        R(nameof(Mu), Mu, 0.05, 2);
        R(nameof(Wheelbase), Wheelbase, 1.5, 5);
        R(nameof(CgToRear), CgToRear, 0.05 * Wheelbase, 0.95 * Wheelbase);
        R(nameof(CgHeight), CgHeight, 0.1, 2);
        R(nameof(TrackFront), TrackFront, 0.8, 2.5);
        R(nameof(TrackRear), TrackRear, 0.8, 2.5);
        R(nameof(SpringFront), SpringFront, 1000, 500000);
        R(nameof(SpringRear), SpringRear, 1000, 500000);
        R(nameof(SpringSepFront), SpringSepFront, 0.2, 2.5);
        R(nameof(SpringSepRear), SpringSepRear, 0.2, 2.5);
        if (RpmRaw is null || TorqueRaw is null || RpmRaw.Length != TorqueRaw.Length || RpmRaw.Length < 3)
            e.Add("RpmRaw/TorqueRaw must have the same length (>= 3)");
        else
        {
            for (int i = 1; i < RpmRaw.Length; i++) if (!(RpmRaw[i] > RpmRaw[i - 1])) e.Add("RpmRaw must be strictly increasing");
            for (int i = 0; i < TorqueRaw.Length; i++) R($"TorqueRaw[{i}]", TorqueRaw[i], 0, 2000);
            R("RpmRaw[0]", RpmRaw[0], 100, 20000);
        }
        R(nameof(BrakeMuFactor), BrakeMuFactor, 0, 1.5);
        R(nameof(BrakeMassFactor), BrakeMassFactor, 0, 1);
        R(nameof(MarginSafety), MarginSafety, 0.1, 1.5);
        R(nameof(Dt), Dt, 1e-4, 0.05);
        R(nameof(IdleRpm), IdleRpm, 100, 5000);
        R(nameof(FuelCutRpm), FuelCutRpm, IdleRpm + 1, 20000);
        R(nameof(ClutchTargetRpm), ClutchTargetRpm, IdleRpm, FuelCutRpm);
        R(nameof(ClutchThrottleThreshold), ClutchThrottleThreshold, 0, 1);
        R(nameof(EngineBrakeTorque), EngineBrakeTorque, 0, 500);
        R(nameof(MoneyShiftRpm), MoneyShiftRpm, FuelCutRpm, 30000);
        R(nameof(BrakeInputThreshold), BrakeInputThreshold, 0, 1);
        R(nameof(ThrottleInputThreshold), ThrottleInputThreshold, 0, 1);
        R(nameof(MaxSteerAngle), MaxSteerAngle, 0.01, 0.7);
        R(nameof(GripUtilization), GripUtilization, 0.1, 1.5);
        R(nameof(CollisionRestitution), CollisionRestitution, 0, 1);
        R(nameof(CollisionFrictionLoss), CollisionFrictionLoss, 0, 1);
        R(nameof(CollisionMaxYawKick), CollisionMaxYawKick, 0, Math.PI);
        if (SurfaceGripFactor is null || SurfaceGripFactor.Length != 3) e.Add("SurfaceGripFactor must have 3 entries [asphalt, curb, grass]");
        else for (int i = 0; i < 3; i++) R($"SurfaceGripFactor[{i}]", SurfaceGripFactor[i], 0.05, 2);
        if (SurfaceRollingFactor is null || SurfaceRollingFactor.Length != 3) e.Add("SurfaceRollingFactor must have 3 entries [asphalt, curb, grass]");
        else for (int i = 0; i < 3; i++) R($"SurfaceRollingFactor[{i}]", SurfaceRollingFactor[i], 0, 20);
        return e;
    }

    public void EnsureValid()
    {
        var e = Validate();
        if (e.Count > 0) throw new ArgumentException("Invalid VehicleParameters: " + string.Join("; ", e));
    }

    public double GripFactor(Surface s) => s switch
    {
        Surface.Curb => SurfaceGripFactor[1],
        Surface.Grass or Surface.Barrier => SurfaceGripFactor[2],
        _ => SurfaceGripFactor[0],
    };

    public double RollingFactor(Surface s) => s switch
    {
        Surface.Curb => SurfaceRollingFactor[1],
        Surface.Grass or Surface.Barrier => SurfaceRollingFactor[2],
        _ => SurfaceRollingFactor[0],
    };
}
