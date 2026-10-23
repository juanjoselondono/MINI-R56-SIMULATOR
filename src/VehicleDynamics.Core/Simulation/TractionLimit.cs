namespace VehicleDynamics.Core.Simulation;

/// <summary>
/// Traction limit of the driven axle. FWD = model.m §6 (unchanged). RWD (profile extension): Gillespie's rear-drive form
/// with independent suspension, F_x,max = μ W_r / (1 − μ h / L), W_r = W (L − c) / L (longitudinal load transfer onto the
/// driven rear axle raises the limit; no solid-axle torque term). A1: the driven axle's share of the downforce adds to
/// its load (front share = aero balance).
/// </summary>
public static class TractionLimit
{
    /// <summary>F_x,max = mu (W c / L) / (1 + (h/L) mu + (2 mu R_w / (i_fd t_f)) (K_phi_r / K_phi)), W = M g cos(theta).</summary>
    public static double FxMax(VehicleParameters p, double mu) => FxMax(p, mu, p.Theta);

    /// <summary>
    /// Same with an explicit road slope <paramref name="theta"/> [rad] (local track grade, E6) and the current downforce
    /// [N] (A1; 0 for model.m).
    /// </summary>
    public static double FxMax(VehicleParameters p, double mu, double theta, double downforce = 0)
    {
        double w = p.Mass * p.G * Math.Cos(theta);
        if (p.DriveLayout == DriveLayout.RearWheelDrive)
        {
            double rearLoad = w * (p.Wheelbase - p.CgToRear) / p.Wheelbase + downforce * (1 - p.AeroBalanceFront);
            double muR = mu * p.RearGripFactor;
            return muR * rearLoad / (1 - (p.CgHeight / p.Wheelbase) * muR);
        }
        double num = mu * (w * p.CgToRear / p.Wheelbase + downforce * p.AeroBalanceFront);
        double den = 1 + (p.CgHeight / p.Wheelbase) * mu
                     + ((2 * mu * p.WheelRadius) / (p.FinalDrive * p.TrackFront)) * (p.KPhiRear / p.KPhi);
        return num / den;
    }
}
