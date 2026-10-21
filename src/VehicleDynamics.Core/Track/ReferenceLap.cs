using VehicleDynamics.Core.Simulation;

namespace VehicleDynamics.Core.Track;

/// <summary>Driver input class of a stretch of the reference lap (racing-line colouring).</summary>
public enum DriveZone { Accelerate, Coast, Brake }

/// <param name="Throttle">Estimated throttle [0..1]: (a_req − a_coast) / (a_full throttle − a_coast).</param>
/// <param name="Brake">Estimated brake [0..1]: (a_req − a_coast) / (a_full brake − a_coast).</param>
public readonly record struct ReferencePoint(double S, double V, double Throttle, double Brake, DriveZone Zone, double Curvature);

/// <summary>
/// EXTENSION (driver aid, not in MATLAB): quasi-steady-state reference lap along a track line, the classic
/// "speed profile" of lap-time simulation.
///   1. Corner limit: v_c = √(f_lat μ g cos θ / |κ|) from the line curvature κ(s).
///   2. Forward pass: v_{i+1}² = v_i² + 2 a_drive(v_i) Δs, where a_drive is the core's own full-throttle longitudinal
///      acceleration (model.m engine map, gears by the §11 auto-shift rule, F_x,max, drag, rolling, grade) scaled by the
///      friction circle √(1 − (a_y / μg)²).
///   3. Backward pass with the core's full-brake deceleration (S7 brake force + resistances), same friction circle.
///   v(s) = min of the three. The required acceleration a_req = v dv/ds, placed between the core's coasting, full-throttle
///   and full-brake accelerations at that speed, gives the reference throttle and brake. Zones: Brake if brake ≥ <see cref="HeavyBrake"/>, Accelerate if throttle ≥
///   <see cref="FullThrottle"/> and the car gains speed, otherwise Coast (lift, partial throttle, trail braking, holding a corner speed).
/// The lateral side is a point-mass friction circle (no tyre slip, no load transfer), so this is a guide for where to
/// brake and accelerate, not a validated lap time.
/// </summary>
public static class ReferenceLap
{
    public const double HeavyBrake = 0.3;
    public const double FullThrottle = 0.5;
    /// <summary>Accelerate zones must actually gain speed [m/s²]; holding a corner speed on part throttle is Coast.</summary>
    public const double MinAcceleration = 0.3;

    /// <param name="spacing">Target point spacing along the line [m].</param>
    /// <param name="grade">Road grade at s [rad, + uphill] (E6), or null for flat.</param>
    /// <param name="gripFactor">Surface friction factor on μ (e.g. surfaces.ini TARMAC); 1 = asphalt.</param>
    /// <param name="lateralFraction">Share of μg used in corners (a real driver's margin).</param>
    /// <param name="brakeFraction">Share of the full-brake deceleration planned for.</param>
    /// <param name="curvatureSmoothing">Half-width of the moving average on |κ| [m] (0 = none).</param>
    public static ReferencePoint[] Compute(ITrackLine line, VehicleParameters p, double spacing = 2.0,
        Func<double, double>? grade = null, double gripFactor = 1.0, double lateralFraction = 0.85, double brakeFraction = 0.9,
        double curvatureSmoothing = 6.0)
    {
        int n = Math.Max(8, (int)Math.Ceiling(line.Length / spacing));
        double ds = line.Length / n;
        var kin = p.Clone();
        kin.LateralModel = LateralModel.Kinematic;   // straight-line longitudinal model; lateral handled below
        var dyn = new Simulation.VehicleDynamics(kin);

        var s = new double[n];
        var kappa = new double[n];
        var theta = new double[n];
        var muG = new double[n];
        var vLim = new double[n];
        for (int i = 0; i < n; i++)
        {
            s[i] = i * ds;
            // Moving average of |κ| over ±curvatureSmoothing: a measured line (e.g. AC fast_lane.ai) has point jitter
            // that would otherwise flip short stretches between braking and accelerating.
            int m = Math.Max(1, (int)Math.Round(curvatureSmoothing));   // 1 m sample spacing
            double sum = 0;
            for (int k = -m; k <= m; k++) sum += Math.Abs(line.Curvature(s[i] + k * curvatureSmoothing / m));
            kappa[i] = sum / (2 * m + 1);
            theta[i] = grade?.Invoke(s[i]) ?? 0;
            muG[i] = p.Mu * gripFactor * p.G * Math.Cos(p.Theta + theta[i]);
            vLim[i] = kappa[i] > 1e-6 ? Math.Sqrt(lateralFraction * muG[i] / kappa[i]) : double.PositiveInfinity;
        }

        double Circle(int i, double v)
        {
            double ay = v * v * kappa[i];
            double r = ay / muG[i];
            return r >= 1 ? 0 : Math.Sqrt(1 - r * r);
        }
        // Straight-line acceleration of the core at speed v: pedal = +1 full throttle, 0 coast, −1 full brake.
        double Accel(int i, double v, int pedal)
        {
            var st = VehicleState.AtRest();
            st.V = Math.Max(v, 0.1);
            st.Gear = dyn.AutoGear(st.V);
            st.ClutchLocked = true;
            var u = new VehicleInputs { Throttle = pedal > 0 ? 1 : 0, Brake = pedal < 0 ? 1 : 0, RoadGrade = theta[i], SurfaceGrip = gripFactor };
            return dyn.Evaluate(st, u).Ax;
        }
        double Drive(int i, double v) { double a = Accel(i, v, 1); return a > 0 ? a * Circle(i, v) : a; }
        double FullBrake(int i, double v) => Math.Max(0.1, -Accel(i, v, -1) * Circle(i, v));
        double Decel(int i, double v) => FullBrake(i, v) * brakeFraction;

        // Closed line: two laps of each pass so the start of the lap sees the end of the previous one.
        var fwd = new double[n];
        double vc = Math.Min(vLim[0], 20);
        for (int k = 0; k < 2 * n; k++)
        {
            int i = k % n, j = (k + 1) % n;
            fwd[i] = vc;
            vc = Math.Min(vLim[j], Math.Sqrt(Math.Max(0, vc * vc + 2 * Drive(i, vc) * ds)));
        }
        var v = new double[n];
        double vb = fwd[0];
        for (int k = 2 * n - 1; k >= 0; k--)
        {
            int i = k % n;
            vb = Math.Min(fwd[i], vb);
            v[i] = vb;
            int prev = (i - 1 + n) % n;
            vb = Math.Sqrt(vb * vb + 2 * Decel(i, vb) * ds);
            vb = Math.Min(vb, fwd[prev]);
        }

        var pts = new ReferencePoint[n];
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            double aReq = (v[j] * v[j] - v[i] * v[i]) / (2 * ds);
            double throttle = 0, brakeIn = 0;
            double aCoast = Accel(i, v[i], 0);
            if (aReq >= aCoast)
            {
                // Throttle share between coasting and full throttle (holding speed against drag needs some throttle).
                double aMax = Drive(i, v[i]);
                throttle = aMax > aCoast + 1e-6 ? Math.Clamp((aReq - aCoast) / (aMax - aCoast), 0, 1) : 1;
            }
            else
            {
                double aBrake = -FullBrake(i, v[i]);
                brakeIn = Math.Clamp((aReq - aCoast) / (aBrake - aCoast), 0, 1);
            }
            var zone = brakeIn >= HeavyBrake ? DriveZone.Brake
                : throttle >= FullThrottle && aReq > MinAcceleration ? DriveZone.Accelerate : DriveZone.Coast;
            pts[i] = new ReferencePoint(s[i], v[i], throttle, brakeIn, zone, kappa[i]);
        }
        return pts;
    }

    /// <summary>Lap time of a profile [s] (trapezoidal in 1/v).</summary>
    public static double LapTime(IReadOnlyList<ReferencePoint> pts, double length)
    {
        double ds = length / pts.Count, t = 0;
        for (int i = 0; i < pts.Count; i++)
        {
            double a = pts[i].V, b = pts[(i + 1) % pts.Count].V;
            t += 2 * ds / Math.Max(a + b, 1e-3);
        }
        return t;
    }
}
