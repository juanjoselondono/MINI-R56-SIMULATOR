using System;
using System.Collections.Generic;
using VehicleDynamics.Core.Simulation;

namespace Simulator.UI;

public enum ParamSource { ModelM, JsSeed, ExtE1, ExtE2, ExtE3, ExtE4, ExtE5, CorrectionC1, UserData, DerivedModelM }

public sealed record ParamEntry(
    string Group, string Name, string Unit, double Min, double Max, double Step, string Help, ParamSource Source,
    Func<VehicleParameters, double> Get, Action<VehicleParameters, double> Set, bool IsToggle = false, bool RestartOnly = false);

/// <summary>
/// The parameters the setup panel exposes. Only quantities that exist in the model are listed; every range lies inside
/// <see cref="VehicleParameters.Validate"/>. Cross-parameter rules (strictly decreasing gear ratios, c within the
/// wheelbase, idle &lt; fuel cut &lt; money shift…) are checked by Validate on the edited copy before Apply.
/// model.m's MarginSafety is not listed: it is a planning margin used only by the reference/validation layer.
/// </summary>
public static class ParameterCatalog
{
    public static string Tag(ParamSource s) => s switch
    {
        ParamSource.ModelM => "model.m",
        ParamSource.JsSeed => "JS seed",
        ParamSource.ExtE1 => "Ext. E1 (provisional)",
        ParamSource.ExtE2 => "Ext. E2 (provisional)",
        ParamSource.ExtE4 => "Ext. E4 (estimate)",
        ParamSource.ExtE5 => "Ext. E5 (provisional)",
        ParamSource.CorrectionC1 => "Correction C1 of model.m",
        ParamSource.UserData => "user-supplied",
        ParamSource.DerivedModelM => "derived from model.m",
        _ => "Ext. E3 (provisional)",
    };

    public static List<ParamEntry> Build()
    {
        var l = new List<ParamEntry>();
        void Add(string g, string n, string u, double min, double max, double step, string help, ParamSource src,
            Func<VehicleParameters, double> get, Action<VehicleParameters, double> set, bool restartOnly = false)
            => l.Add(new ParamEntry(g, n, u, min, max, step, help, src, get, set, RestartOnly: restartOnly));
        void Toggle(string g, string n, string help, ParamSource src, Func<VehicleParameters, bool> get, Action<VehicleParameters, bool> set)
            => l.Add(new ParamEntry(g, n, "", 0, 1, 1, help, src, p => get(p) ? 1 : 0, (p, v) => set(p, v >= 0.5), IsToggle: true));

        const string body = "Vehicle body & environment";
        Add(body, "Mass M", "kg", 600, 3000, 10, "Running mass. model.m: car.M = 1420 kg; game default 1150 kg (user-supplied with I_zz). Enters F_x,max, rolling, grade, every inertia term and the E4 lateral dynamics.", ParamSource.UserData, p => p.Mass, (p, v) => p.Mass = v);
        Add(body, "Drag coefficient Cd", "–", 0.15, 0.8, 0.001, "Aerodynamic drag coefficient (car.Cd): F_drag = ½ρ A Cd V².", ParamSource.ModelM, p => p.Cd, (p, v) => p.Cd = v);
        Add(body, "Frontal area A", "m²", 1.0, 4.0, 0.01, "Projected frontal area (car.A).", ParamSource.ModelM, p => p.FrontalArea, (p, v) => p.FrontalArea = v);
        Add(body, "Air density ρ", "kg/m³", 0.9, 1.4, 0.005, "env.rho_air (1.225 = sea level, 15 °C).", ParamSource.ModelM, p => p.RhoAir, (p, v) => p.RhoAir = v);
        Add(body, "Road grade", "%", -20, 20, 0.5, "model.m slope (pct mode). One grade for the whole track, as in model.m: + = uphill.", ParamSource.ModelM,
            p => p.SlopeMode == SlopeMode.Pct ? p.SlopeValue : Math.Tan(p.Theta) * 100, (p, v) => { p.SlopeMode = SlopeMode.Pct; p.SlopeValue = v; });

        const string tyre = "Tyres & traction limit (F_x,max, FWD)";
        Add(tyre, "Friction coefficient μ", "–", 0.3, 1.6, 0.01, "Tyre–road friction (car.mu = 0.90 in model.m; game default 1.05 for performance road tyres). Scales F_x,max, braking and the lateral grip.", ParamSource.ModelM, p => p.Mu, (p, v) => p.Mu = v);
        Toggle(tyre, "Grip limit on tyre force (C1)", "On: F_x,max caps the tyre force (driveline force minus what spins up the rotating parts); a grip-limited tyre accelerates the bare mass. Off: model.m's formula (driveline force capped, divided by M_eq).", ParamSource.CorrectionC1, p => p.GripLimitOnTyreForce, (p, v) => p.GripLimitOnTyreForce = v);
        Add(tyre, "Tyre width", "m", 0.145, 0.255, 0.005, "175/65 R15 → 0.175. With aspect ratio and rim gives R_w.", ParamSource.ModelM, p => p.TireWidth, (p, v) => p.TireWidth = v);
        Add(tyre, "Aspect ratio", "–", 0.3, 0.8, 0.05, "Sidewall height / width.", ParamSource.ModelM, p => p.AspectRatio, (p, v) => p.AspectRatio = v);
        Add(tyre, "Rim diameter", "in", 13, 20, 1, "Wheel rim diameter.", ParamSource.ModelM, p => p.RimDiameterInch, (p, v) => p.RimDiameterInch = v);
        Add(tyre, "Wheelbase L", "m", 2.0, 3.2, 0.001, "car.L. Also the E1 kinematic steering geometry.", ParamSource.ModelM, p => p.Wheelbase, (p, v) => p.Wheelbase = v);
        Add(tyre, "CG to rear axle c", "m", 0.5, 2.5, 0.001, "car.c (must stay within 5–95 % of L). Front-axle load share = c/L.", ParamSource.ModelM, p => p.CgToRear, (p, v) => p.CgToRear = v);
        Add(tyre, "CG height h", "m", 0.3, 0.8, 0.001, "car.h. Longitudinal load transfer term (h/L)μ of F_x,max.", ParamSource.ModelM, p => p.CgHeight, (p, v) => p.CgHeight = v);
        Add(tyre, "Front track t_f", "m", 1.2, 1.8, 0.001, "car.t_f. Enters the torque-induced lateral load-transfer term of F_x,max.", ParamSource.ModelM, p => p.TrackFront, (p, v) => p.TrackFront = v);
        Add(tyre, "Rear track t_r", "m", 1.2, 1.8, 0.001, "car.t_r (defined in model.m; not used by its equations).", ParamSource.ModelM, p => p.TrackRear, (p, v) => p.TrackRear = v);
        Add(tyre, "Front spring K_s,f", "N/m", 10000, 100000, 100, "car.K_s_f → roll stiffness K_φ,f = ½ K s².", ParamSource.ModelM, p => p.SpringFront, (p, v) => p.SpringFront = v);
        Add(tyre, "Rear spring K_s,r", "N/m", 10000, 100000, 100, "car.K_s_r → K_φ,r. F_x,max uses K_φ,r / K_φ.", ParamSource.ModelM, p => p.SpringRear, (p, v) => p.SpringRear = v);
        Add(tyre, "Front spring spacing s_f", "m", 0.8, 1.3, 0.001, "car.s_f.", ParamSource.ModelM, p => p.SpringSepFront, (p, v) => p.SpringSepFront = v);
        Add(tyre, "Rear spring spacing s_r", "m", 0.8, 1.3, 0.001, "car.s_r.", ParamSource.ModelM, p => p.SpringSepRear, (p, v) => p.SpringSepRear = v);

        const string drive = "Drivetrain";
        Add(drive, "Final drive i_fd", "–", 2.5, 6.0, 0.001, "car.final_drive.", ParamSource.ModelM, p => p.FinalDrive, (p, v) => p.FinalDrive = v);
        Add(drive, "Transmission efficiency η_t", "–", 0.6, 1.0, 0.005, "car.eta_t.", ParamSource.ModelM, p => p.EtaT, (p, v) => p.EtaT = v);
        for (int i = 0; i < 6; i++)
        {
            int g = i;
            Add(drive, $"Gear {g + 1} ratio", "–", 0.4, 5.0, 0.001, "car.gear_ratios. Must decrease strictly from 1st to 6th.", ParamSource.ModelM,
                p => p.GearRatios[g], (p, v) => p.GearRatios[g] = v);
        }
        for (int i = 0; i < 6; i++)
        {
            int g = i;
            Add(drive, $"Mass factor gear {g + 1}", "–", 0.0, 1.0, 0.01, "car.mass_factors: rotational inertia, M_eq = M(1 + factor).", ParamSource.ModelM,
                p => p.MassFactors[g], (p, v) => p.MassFactors[g] = v);
        }

        const string engine = "Engine full-load torque (model.m nodes, PCHIP)";
        var defaults = VehicleParameters.GameDefaults();
        for (int i = 0; i < defaults.RpmRaw.Length; i++)
        {
            int k = i;
            Add(engine, $"Torque @ {defaults.RpmRaw[k]:0} rpm", "N·m", 0, 300, 1, "torque_raw node; the curve between nodes is PCHIP-interpolated as in model.m.", ParamSource.ModelM,
                p => p.TorqueRaw[k], (p, v) => p.TorqueRaw[k] = v);
        }

        const string brake = "Braking";
        Add(brake, "Braking μ factor", "–", 0.3, 1.2, 0.01, "model.m §11: F_brake = factor · μ · M g cosθ (0.90).", ParamSource.ModelM, p => p.BrakeMuFactor, (p, v) => p.BrakeMuFactor = v);
        Add(brake, "Braking mass factor", "–", 0, 0.2, 0.005, "model.m §11: M_eq,brake = M(1 + factor) (0.05).", ParamSource.ModelM, p => p.BrakeMassFactor, (p, v) => p.BrakeMassFactor = v);

        const string seed = "Engine rules from the JS seed game";
        Add(seed, "Idle speed", "rpm", 600, 1500, 10, "Engine speed floor; torque below the first node uses the 1000-rpm value.", ParamSource.JsSeed, p => p.IdleRpm, (p, v) => p.IdleRpm = v);
        Add(seed, "Fuel cut", "rpm", 5000, 7000, 10, "Torque = 0 at or above (rev limiter).", ParamSource.JsSeed, p => p.FuelCutRpm, (p, v) => p.FuelCutRpm = v);
        Add(seed, "Seed clutch target", "rpm", 2000, 5000, 10, "Only with the seed launch rule: engine held at idle + throttle·(target − idle) in 1st until the wheels catch up.", ParamSource.JsSeed, p => p.ClutchTargetRpm, (p, v) => p.ClutchTargetRpm = v);
        Add(seed, "Engine braking torque", "N·m", 0, 80, 1, "Coasting drag torque at the crank (seed: 30).", ParamSource.JsSeed, p => p.EngineBrakeTorque, (p, v) => p.EngineBrakeTorque = v);
        Add(seed, "Money-shift limit", "rpm", 6600, 9000, 10, "Wheel-driven engine speed that destroys the engine (seed: 7200).", ParamSource.JsSeed, p => p.MoneyShiftRpm, (p, v) => p.MoneyShiftRpm = v);
        Toggle(seed, "Mechanical damage", "Over-revving above the money-shift limit destroys the engine.", ParamSource.JsSeed, p => p.MechanicalDamage, (p, v) => p.MechanicalDamage = v);
        Toggle(seed, "Seed launch rule instead of E5", "Use the seed's rpm rule in 1st gear instead of the physical launch clutch.", ParamSource.JsSeed, p => p.LaunchModel == LaunchModel.SeedRule, (p, v) => p.LaunchModel = v ? LaunchModel.SeedRule : LaunchModel.PhysicalClutch);

        const string e5 = "Launch clutch (extension E5)";
        Toggle(e5, "Launch model on", "Off: model.m as is (engine always tied to the wheels, idle clamp); the car cannot pull away cleanly from 0 km/h.", ParamSource.ExtE5, p => p.ClutchLaunch, (p, v) => p.ClutchLaunch = v);
        Add(e5, "Engine inertia I_e", "kg·m²", 0.05, 1.0, 0.01, "Engine + flywheel at the crank. Derived from model.m's 1st/2nd-gear mass factors (0.29). Sets how fast the engine revs while the clutch slips.", ParamSource.DerivedModelM, p => p.EngineInertia, (p, v) => p.EngineInertia = v);
        Add(e5, "Clutch capacity", "N·m", 80, 400, 5, "Torque the fully engaged clutch can transmit (1.25 × the 160 N·m peak).", ParamSource.ExtE5, p => p.ClutchCapacity, (p, v) => p.ClutchCapacity = v);
        Add(e5, "Clutch bite speed", "rpm", 1500, 5000, 50, "Engine speed at which the automatic launch clutch is fully engaged; full throttle settles near 2300 rpm while slipping.", ParamSource.ExtE5, p => p.ClutchBiteRpm, (p, v) => p.ClutchBiteRpm = v);

        const string e1 = "Steering & lateral dynamics (E4 dynamic / E1 kinematic)";
        Toggle(e1, "Dynamic single-track (E4)", "On: slip angles, tyre forces, yaw inertia (the car can understeer, oversteer and spin). Off: E1 kinematic path with a friction circle.", ParamSource.ExtE4, p => p.LateralModel == LateralModel.Dynamic, (p, v) => p.LateralModel = v ? LateralModel.Dynamic : LateralModel.Kinematic);
        Add(e1, "Yaw inertia I_zz", "kg·m²", 500, 3000, 10, "Yaw moment of inertia (user-supplied 1100). For reference, M·l_f·l_r = 1650 kg·m² at 1150 kg.", ParamSource.UserData, p => p.YawInertia, (p, v) => p.YawInertia = v);
        Add(e1, "Tyre load sensitivity k", "–", 0, 0.4, 0.01, "Friction coefficient falls with tyre load: μ_i = μ(1 − k(F_zi − F_z,nom)/F_z,nom). Makes the heavier front axle and lateral load transfer cost grip (limit understeer). 0 = load-independent.", ParamSource.ExtE4, p => p.TyreLoadSensitivity, (p, v) => p.TyreLoadSensitivity = v);
        Add(e1, "Front roll-stiffness share", "–", 0.2, 0.8, 0.01, "Share of the lateral load transfer carried by the front axle (anti-roll bar balance). Higher = more understeer. model.m's springs alone give 0.43.", ParamSource.ExtE4, p => p.FrontRollStiffnessShare, (p, v) => p.FrontRollStiffnessShare = v);
        Add(e1, "Front cornering stiffness C_αf", "N/rad", 30000, 250000, 1000, "Front axle (both tyres). Estimate: ~12/rad × front axle load; with C_αr gives ~2.6°/g understeer (linear range).", ParamSource.ExtE4, p => p.CorneringStiffnessFront, (p, v) => p.CorneringStiffnessFront = v);
        Add(e1, "Rear cornering stiffness C_αr", "N/rad", 30000, 250000, 1000, "Rear axle (both tyres, incl. compliance). Estimate: ~27/rad × rear axle load. Lower it relative to C_αf for a more nervous, oversteering car.", ParamSource.ExtE4, p => p.CorneringStiffnessRear, (p, v) => p.CorneringStiffnessRear = v);
        Add(e1, "Max steer angle δ_max", "rad", 0.05, 0.6, 0.005, "Road-wheel angle at full input. Not in model.m; the seed used 0.06.", ParamSource.ExtE1, p => p.MaxSteerAngle, (p, v) => p.MaxSteerAngle = v);
        Add(e1, "Grip utilisation (E1 only)", "–", 0.5, 1.3, 0.01, "E1 kinematic model only: friction-circle radius = factor · F_x,max (model.m §10 uses 0.8 as a planning margin).", ParamSource.ExtE1, p => p.GripUtilization, (p, v) => p.GripUtilization = v);

        const string e2 = "Surfaces (extension E2)";
        Add(e2, "Curb grip factor", "–", 0.3, 1.2, 0.01, "μ multiplier on the curb band.", ParamSource.ExtE2, p => p.SurfaceGripFactor[1], (p, v) => p.SurfaceGripFactor[1] = v);
        Add(e2, "Grass grip factor", "–", 0.1, 1.0, 0.01, "μ multiplier on grass.", ParamSource.ExtE2, p => p.SurfaceGripFactor[2], (p, v) => p.SurfaceGripFactor[2] = v);
        Add(e2, "Curb rolling factor", "–", 0.5, 5, 0.1, "Rolling-coefficient multiplier on the curb band.", ParamSource.ExtE2, p => p.SurfaceRollingFactor[1], (p, v) => p.SurfaceRollingFactor[1] = v);
        Add(e2, "Grass rolling factor", "–", 1, 15, 0.1, "Rolling-coefficient multiplier on grass.", ParamSource.ExtE2, p => p.SurfaceRollingFactor[2], (p, v) => p.SurfaceRollingFactor[2] = v);

        const string e3 = "Barrier impacts (extension E3)";
        Add(e3, "Restitution e", "–", 0, 0.8, 0.01, "Normal velocity after impact = −e · before.", ParamSource.ExtE3, p => p.CollisionRestitution, (p, v) => p.CollisionRestitution = v);
        Add(e3, "Tangential loss k_t", "–", 0, 0.6, 0.01, "Scrape loss of the tangential velocity, proportional to impact severity.", ParamSource.ExtE3, p => p.CollisionFrictionLoss, (p, v) => p.CollisionFrictionLoss = v);
        Add(e3, "Max yaw kick", "rad", 0, 1.2, 0.01, "Largest heading change one impact may cause.", ParamSource.ExtE3, p => p.CollisionMaxYawKick, (p, v) => p.CollisionMaxYawKick = v);
        Add(e3, "Bounce threshold", "m/s", 0, 3, 0.05, "Approach speeds below this are sliding contact (no bounce, no impact effects).", ParamSource.ExtE3, p => p.CollisionBounceThreshold, (p, v) => p.CollisionBounceThreshold = v);

        const string sim = "Numerics";
        Add(sim, "Physics time step dt", "s", 0.001, 0.02, 0.0005, "model.m dt = 0.005 s. Semi-implicit Euler as in model.m; see the convergence study in docs/validation_report.md.", ParamSource.ModelM, p => p.Dt, (p, v) => p.Dt = v, restartOnly: false);
        return l;
    }
}
