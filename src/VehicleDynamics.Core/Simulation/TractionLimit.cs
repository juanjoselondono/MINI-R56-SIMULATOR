namespace VehicleDynamics.Core.Simulation;

/// <summary>model.m §6 traction limit of the driven (front) axle.</summary>
public static class TractionLimit
{
    /// <summary>F_x,max = mu (W c / L) / (1 + (h/L) mu + (2 mu R_w / (i_fd t_f)) (K_phi_r / K_phi)), W = M g cos(theta).</summary>
    public static double FxMax(VehicleParameters p, double mu) => FxMax(p, mu, p.Theta);

    /// <summary>Same with an explicit road slope <paramref name="theta"/> [rad] (local track grade, E6).</summary>
    public static double FxMax(VehicleParameters p, double mu, double theta)
    {
        double w = p.Mass * p.G * Math.Cos(theta);
        double num = mu * (w * p.CgToRear / p.Wheelbase);
        double den = 1 + (p.CgHeight / p.Wheelbase) * mu
                     + ((2 * mu * p.WheelRadius) / (p.FinalDrive * p.TrackFront)) * (p.KPhiRear / p.KPhi);
        return num / den;
    }
}
