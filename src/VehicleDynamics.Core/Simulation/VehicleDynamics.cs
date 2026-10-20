namespace VehicleDynamics.Core.Simulation;

public enum Integrator
{
    /// <summary>model.m scheme: V += a dt; s += V dt + 0.5 a dt^2 (default, used by the game).</summary>
    SemiImplicitEuler,
    /// <summary>Classic RK4 with gear and inputs frozen over the step. Only for the timestep convergence study.</summary>
    RungeKutta4,
}

/// <summary>
/// Interactive vehicle model: model.m longitudinal equations evaluated continuously with driver inputs,
/// plus extension E1 (kinematic single-track + friction circle; not in MATLAB) and E2 surface factors.
/// No yaw inertia, cornering stiffness or tyre model exists in MATLAB, so the car cannot spin.
/// </summary>
public sealed class VehicleDynamics
{
    private VehicleParameters _p;
    private EngineModel _engine;
    private double[] _gearMaxKmh = Array.Empty<double>();

    public VehicleState State;
    public VehicleOutputs Outputs;
    public Integrator Integrator { get; set; } = Integrator.SemiImplicitEuler;
    public VehicleParameters Parameters => _p;

    public VehicleDynamics(VehicleParameters parameters, VehicleState? initial = null)
    {
        _p = null!;
        _engine = null!;
        SetParameters(parameters);
        State = initial ?? VehicleState.AtRest();
    }

    /// <summary>Applies a new parameter set (call between steps). Throws if invalid.</summary>
    public void SetParameters(VehicleParameters parameters)
    {
        parameters.EnsureValid();
        _p = parameters.Clone();
        _engine = new EngineModel(_p);
        int ng = _p.NumGears;
        _gearMaxKmh = new double[ng];
        double omegaMax = _p.RpmRaw[^1] * (2 * Math.PI / 60);
        for (int g = 0; g < ng; g++)
        {
            double totalRatio = _p.GearRatios[g] * _p.FinalDrive;
            _gearMaxKmh[g] = (omegaMax / totalRatio) * _p.WheelRadius * 3.6;
        }
        if (State.Gear > ng) State.Gear = ng;
    }

    public EngineModel Engine => _engine;

    /// <summary>Speed [km/h] at the last torque node (6500 rpm) in gear g (1-based), computed like max(V_gear_kmh(g,:)).</summary>
    public double GearMaxSpeedKmh(int gear) => _gearMaxKmh[gear - 1];

    /// <summary>MATLAB §11 rule: lowest gear whose max speed >= V. Above every gear: top gear (MATLAB would keep 1; unreachable in practice).</summary>
    public int AutoGear(double v)
    {
        for (int g = 0; g < _gearMaxKmh.Length; g++)
            if (v * 3.6 <= _gearMaxKmh[g]) return g + 1;
        return _gearMaxKmh.Length;
    }

    public double TotalRatio(int gear) => _p.GearRatios[gear - 1] * _p.FinalDrive;

    /// <summary>Advances one fixed step. Shift edges in <paramref name="u"/> are applied once here.</summary>
    public void Step(in VehicleInputs u, double dt)
    {
        if (!(dt > 0)) throw new ArgumentOutOfRangeException(nameof(dt));
        var s = State;
        bool blownNow = false;

        if (!s.EngineBlown)
        {
            if (u.AutoShift) s.Gear = AutoGear(s.V);
            else
            {
                if (u.ShiftUp && s.Gear < _p.NumGears) s.Gear++;
                if (u.ShiftDown && s.Gear > 1) s.Gear--;
            }
            double wheelRpm = EngineModel.WheelRpm(s.V, _p.WheelRadius, TotalRatio(s.Gear));
            if (_p.MechanicalDamage && wheelRpm > _p.MoneyShiftRpm)
            {
                s.EngineBlown = true; // money shift (seed rule)
                blownNow = true;
            }
        }

        var o = Evaluate(s, u, s.V);
        o.EngineBlownThisStep = blownNow;

        if (Integrator == Integrator.SemiImplicitEuler)
        {
            double a = o.Ax;
            double vNew = s.V + a * dt;
            if (vNew < 0) { vNew = 0; }
            double ds = vNew * dt + 0.5 * a * (dt * dt);
            if (ds < 0) ds = 0;
            else s.Distance = s.Distance + vNew * dt + 0.5 * a * (dt * dt);
            // Heading change from path curvature (r / V) over ds; position along the chord (midpoint heading).
            double kappa = s.V > 1e-9 ? o.YawRate / s.V : 0;
            double dPsi = kappa * ds;
            double heading = s.Psi + 0.5 * dPsi + o.Beta;
            s.X += ds * Math.Cos(heading);
            s.Y += ds * Math.Sin(heading);
            s.Psi += dPsi;
            s.V = vNew;
        }
        else
        {
            Rk4(ref s, u, dt);
        }

        s.Time += dt;
        s.EngineRpm = o.EngineRpm;
        State = s;
        Outputs = o;
    }

    private void Rk4(ref VehicleState s, VehicleInputs u, double dt)
    {
        var s0 = s;
        bool auto = u.AutoShift && !s0.EngineBlown;
        (double dv, double dpsi, double dx, double dy) F(double v, double psi)
        {
            v = Math.Max(0, v);
            // With auto-shift the gear is a function of V (model.m rule), so every stage re-selects it;
            // freezing it would let stages run past 6500 rpm into the fuel cut.
            var sv = s0;
            if (auto) sv.Gear = AutoGear(v);
            var o = Evaluate(sv, u, v);
            double h = psi + o.Beta;
            return (o.Ax, o.YawRate, v * Math.Cos(h), v * Math.Sin(h));
        }
        var k1 = F(s0.V, s0.Psi);
        var k2 = F(s0.V + 0.5 * dt * k1.dv, s0.Psi + 0.5 * dt * k1.dpsi);
        var k3 = F(s0.V + 0.5 * dt * k2.dv, s0.Psi + 0.5 * dt * k2.dpsi);
        var k4 = F(s0.V + dt * k3.dv, s0.Psi + dt * k3.dpsi);
        double v1 = Math.Max(0, s0.V + 0.5 * dt * k1.dv);
        double v2 = Math.Max(0, s0.V + 0.5 * dt * k2.dv);
        double v3 = Math.Max(0, s0.V + dt * k3.dv);
        double ds = dt / 6 * (s0.V + 2 * v1 + 2 * v2 + v3);
        double vNew = s0.V + dt / 6 * (k1.dv + 2 * k2.dv + 2 * k3.dv + k4.dv);
        s.V = Math.Max(0, vNew);
        s.Distance += Math.Max(0, ds);
        s.Psi += dt / 6 * (k1.dpsi + 2 * k2.dpsi + 2 * k3.dpsi + k4.dpsi);
        s.X += dt / 6 * (k1.dx + 2 * k2.dx + 2 * k3.dx + k4.dx);
        s.Y += dt / 6 * (k1.dy + 2 * k2.dy + 2 * k3.dy + k4.dy);
    }

    /// <summary>
    /// Applies an explicit collision response (extension E3) computed by <see cref="Collision.CollisionResponse.Resolve"/>
    /// or <see cref="Collision.SustainedContact.Resolve"/>: the only sanctioned external write to the state. Position
    /// (depenetration), heading and speed are taken from the result; a response may never increase the speed.
    /// </summary>
    public void ApplyCollision(in Collision.CollisionResult r)
    {
        if (!double.IsFinite(r.X) || !double.IsFinite(r.Y) || !double.IsFinite(r.Psi) || !double.IsFinite(r.V))
            throw new ArgumentException("Non-finite collision result.", nameof(r));
        if (r.V > State.V + 1e-9)
            throw new ArgumentException("A collision response must not increase speed.", nameof(r));
        var s = State;
        s.X = r.X;
        s.Y = r.Y;
        s.Psi = r.Psi;
        s.V = Math.Max(0, r.V);
        State = s;
    }

    /// <summary>Forces, accelerations and yaw rate at speed <paramref name="v"/> for the gear/flags in <paramref name="s"/>.</summary>
    public VehicleOutputs Evaluate(in VehicleState s, in VehicleInputs u, double v)
    {
        var p = _p;
        var o = new VehicleOutputs();
        double throttle = Math.Clamp(u.Throttle, 0, 1);
        double brake = Math.Clamp(u.Brake, 0, 1);
        double rw = p.WheelRadius;
        double tr = TotalRatio(s.Gear);
        double cosT = Math.Cos(p.Theta);

        // Longitudinal (model.m §3-§7)
        o.WheelRpm = EngineModel.WheelRpm(v, rw, tr);
        if (s.EngineBlown)
        {
            o.EngineRpm = 0;
            o.EngineTorque = 0;
        }
        else
        {
            o.EngineRpm = _engine.EngineRpm(o.WheelRpm, s.Gear, throttle);
            o.RevLimiter = o.EngineRpm > p.FuelCutRpm;
            o.EngineTorque = _engine.Torque(o.EngineRpm);
        }
        o.FTractive = o.EngineTorque * throttle * tr * p.EtaT / rw;

        double muEff = p.Mu * p.GripFactor(u.Surface);
        o.FxMax = TractionLimit.FxMax(p, muEff);
        o.FGrip = p.GripUtilization * o.FxMax;

        // Lateral: extension E1 (kinematic single-track + friction circle)
        double delta = Math.Clamp(u.SteerAngle, -p.MaxSteerAngle, p.MaxSteerAngle);
        o.SteerAngle = delta;
        double lr = p.CgToRear;
        double tanD = Math.Tan(delta);
        o.Beta = Math.Atan((lr / p.Wheelbase) * tanD);
        double rKin = v * Math.Cos(o.Beta) * tanD / p.Wheelbase;
        o.FyDemand = p.Mass * v * Math.Abs(rKin);
        double r = rKin;
        double fy = o.FyDemand;
        if (o.FyDemand > o.FGrip && v > 0)
        {
            r = Math.Sign(delta) * o.FGrip / (p.Mass * v);
            fy = o.FGrip;
            o.Understeer = true;
        }
        o.YawRate = r;
        o.Fy = fy;
        o.Ay = v * r;
        o.Vx = v * Math.Cos(o.Beta);
        o.Vy = v * Math.Sin(o.Beta);
        double fxAvail = fy == 0 ? o.FGrip : Math.Sqrt(Math.Max(0, o.FGrip * o.FGrip - fy * fy));
        double circleScale = o.FGrip > 0 ? fxAvail / o.FGrip : 0;

        o.FTractiveEff = Math.Min(o.FTractive, fxAvail);
        o.FDrag = Resistance.Drag(p, v);
        o.FRolling = Resistance.Rolling(p, v, p.RollingFactor(u.Surface));
        o.FGrade = Resistance.Grade(p);
        double fRes = o.FDrag + o.FRolling + o.FGrade;
        double fBrakeMax = (p.BrakeMuFactor * muEff) * (p.Mass * p.G * cosT);
        o.FBrake = fBrakeMax * brake * circleScale;
        o.FEngineBrake = p.EngineBrakeTorque * tr / rw;

        double fxUsed;
        double fxCap;
        if (s.EngineBlown)
        {
            // Seed rule: locked-wheel friction + drag over the bare mass.
            o.Branch = DriveBranch.Blown;
            o.MassEquivalent = p.Mass;
            o.Ax = -(p.Mass * p.G * muEff + o.FDrag) / p.Mass;
            o.FTractiveEff = 0;
            fxUsed = 0; fxCap = 1;
        }
        else if (brake > p.BrakeInputThreshold)
        {
            o.Branch = DriveBranch.Brake;
            o.MassEquivalent = p.Mass * (1 + p.BrakeMassFactor);
            o.Ax = -(o.FBrake + fRes) / o.MassEquivalent;
            o.FTractiveEff = 0;
            fxUsed = o.FBrake; fxCap = fBrakeMax;
        }
        else if (throttle > p.ThrottleInputThreshold)
        {
            o.Branch = DriveBranch.Drive;
            o.MassEquivalent = p.Mass * (1 + p.MassFactors[s.Gear - 1]);
            o.Ax = (o.FTractiveEff - fRes) / o.MassEquivalent;
            fxUsed = o.FTractiveEff; fxCap = o.FGrip;
        }
        else
        {
            o.Branch = DriveBranch.Coast;
            o.MassEquivalent = p.Mass * (1 + p.MassFactors[s.Gear - 1]);
            o.Ax = -(fRes + o.FEngineBrake) / o.MassEquivalent;
            o.FTractiveEff = 0;
            fxUsed = 0; fxCap = 1;
        }

        double latUse = o.FGrip > 0 ? fy / o.FGrip : 0;
        double lonUse = fxCap > 0 ? fxUsed / fxCap : 0;
        o.GripUsage = Math.Sqrt(latUse * latUse + lonUse * lonUse);
        double latDemand = o.FGrip > 0 ? o.FyDemand / o.FGrip : 0;
        double lonDemand = 0;
        if (o.Branch == DriveBranch.Drive && o.FTractive > 0)
            lonDemand = fxAvail > 0 ? o.FTractive / fxAvail : 10;
        o.SlipIndicator = Math.Min(10, Math.Max(latDemand, lonDemand));
        return o;
    }
}
