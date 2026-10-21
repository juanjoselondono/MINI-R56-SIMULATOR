namespace VehicleDynamics.Core.Simulation;

public enum Integrator
{
    /// <summary>model.m scheme: V += a dt; s += V dt + 0.5 a dt^2 (default, used by the game).</summary>
    SemiImplicitEuler,
    /// <summary>Classic RK4 with gear, clutch mode and inputs frozen over the step. Only for the timestep convergence study.</summary>
    RungeKutta4,
}

/// <summary>
/// Interactive vehicle model: model.m longitudinal equations evaluated continuously with driver inputs, plus
/// extensions (not in MATLAB, see docs/vehicle_model.md): E1 kinematic or E4 dynamic single-track lateral model,
/// E2 surface factors, E5 physical launch clutch, and correction C1 (grip limit on the tyre force).
/// With δ = 0 and no lateral motion every extension term is exactly zero and the straight-line update uses model.m's
/// expressions, so the MATLAB validation (C1 off, no launch model) is unaffected.
/// </summary>
public sealed class VehicleDynamics
{
    private const double RpmToRad = 2 * Math.PI / 60;

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

    private bool PhysicalClutch => _p.ClutchLaunch && _p.LaunchModel == LaunchModel.PhysicalClutch;
    private double OmegaIdle => _p.IdleRpm * RpmToRad;
    private bool Dynamic => _p.LateralModel == LateralModel.Dynamic;

    /// <summary>Crank speed imposed by the wheels in gear g [rad/s].</summary>
    private double WheelSideOmega(double v, int gear) => v / _p.WheelRadius * TotalRatio(gear);

    /// <summary>Magnitude of the CG velocity [m/s] (E4: √(v_x² + v_y²); E1: the path speed).</summary>
    public double Speed => Dynamic ? Math.Sqrt(State.V * State.V + State.Vy * State.Vy) : State.V;

    /// <summary>Angle of the CG velocity relative to the heading [rad] (E4: atan2(v_y, v_x); E1: kinematic β).</summary>
    public double VelocityAngle => Dynamic ? (State.V > 0 || State.Vy != 0 ? Math.Atan2(State.Vy, State.V) : 0) : Outputs.Beta;

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
        }
        UpdateClutchMode(ref s);
        if (!s.EngineBlown && s.ClutchLocked)
        {
            double wheelRpm = EngineModel.WheelRpm(s.V, _p.WheelRadius, TotalRatio(s.Gear));
            if (_p.MechanicalDamage && wheelRpm > _p.MoneyShiftRpm)
            {
                s.EngineBlown = true; // money shift (seed rule)
                blownNow = true;
                s.ClutchLocked = false;
            }
        }

        var o = Evaluate(s, u);
        o.EngineBlownThisStep = blownNow;

        if (Integrator == Integrator.SemiImplicitEuler) IntegrateEuler(ref s, o, u, dt);
        else if (Dynamic) Rk4Dynamic(ref s, u, dt);
        else Rk4Kinematic(ref s, u, dt);

        IntegrateClutch(ref s, o, dt);
        s.Time += dt;
        s.EngineRpm = o.EngineRpm;
        s.AxPrev = o.Ax;
        State = s;
        Outputs = o;
    }

    /// <summary>
    /// E5 clutch mode at the start of a step. Without the physical launch clutch the engine is always tied to the
    /// wheels (model.m). With it: it opens when the wheel-driven engine speed would fall below idle, and closes as soon
    /// as the wheel side turns at least as fast as the (slipping) engine.
    /// </summary>
    private void UpdateClutchMode(ref VehicleState s)
    {
        if (!PhysicalClutch) { s.ClutchLocked = true; return; }
        if (s.EngineBlown) { s.ClutchLocked = false; return; }
        double idle = OmegaIdle;
        double win = WheelSideOmega(s.V, s.Gear);
        if (s.EngineOmega < idle) s.EngineOmega = idle;
        if (s.ClutchLocked)
        {
            if (win < idle * (1 - 1e-9)) { s.ClutchLocked = false; s.EngineOmega = idle; }
            else s.EngineOmega = win;
        }
        else if (win >= idle * (1 - 1e-9) && s.EngineOmega <= win * (1 + 1e-12))
        {
            s.ClutchLocked = true;
            s.EngineOmega = win;
        }
    }

    /// <summary>E5: engine speed while slipping (idle governor floor); locks when the slip changes sign.</summary>
    private void IntegrateClutch(ref VehicleState s, in VehicleOutputs o, double dt)
    {
        if (!PhysicalClutch || s.EngineBlown || o.ClutchLocked) return;
        double idle = OmegaIdle;
        double we = Math.Max(idle, s.EngineOmega + o.EngineOmegaDot * dt);
        double win = WheelSideOmega(s.V, s.Gear);
        if (win >= idle && we <= win)
        {
            s.ClutchLocked = true;
            s.EngineOmega = win;
        }
        else s.EngineOmega = we;
    }

    private void IntegrateEuler(ref VehicleState s, in VehicleOutputs o, in VehicleInputs u, double dt)
    {
        double a = o.Ax;
        double vNew = s.V + a * dt;
        if (vNew < 0) { vNew = 0; }

        if (!Dynamic)
        {
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
            return;
        }

        double vyNew = s.Vy + o.VyDot * dt;
        double rNew = s.YawRate + o.YawAccel * dt;
        BlendTowardsKinematic(vNew, o.SteerAngle, ref vyNew, ref rNew);

        double dx = vNew * dt + 0.5 * a * (dt * dt);
        if (dx < 0) dx = 0;
        double dy = vyNew * dt;
        if (dy == 0 && rNew == 0)
        {
            // Straight line: model.m's distance update verbatim.
            if (dx > 0) s.Distance = s.Distance + vNew * dt + 0.5 * a * (dt * dt);
            s.X += dx * Math.Cos(s.Psi);
            s.Y += dx * Math.Sin(s.Psi);
        }
        else
        {
            double psiMid = s.Psi + 0.5 * rNew * dt;
            double c = Math.Cos(psiMid), sn = Math.Sin(psiMid);
            s.X += dx * c - dy * sn;
            s.Y += dx * sn + dy * c;
            s.Distance += Math.Sqrt(dx * dx + dy * dy);
        }
        s.Psi += rNew * dt;
        s.V = vNew;
        s.Vy = vyNew;
        s.YawRate = rNew;
    }

    /// <summary>
    /// E4 numerics: slip angles are ill-defined near standstill, so below <see cref="VehicleParameters.DynamicBlendHighSpeed"/>
    /// the lateral states are blended (smoothstep) towards the kinematic single-track values, fully kinematic below
    /// <see cref="VehicleParameters.DynamicBlendLowSpeed"/>.
    /// </summary>
    private void BlendTowardsKinematic(double vx, double delta, ref double vy, ref double r)
    {
        if (vx <= 0) { vy = 0; r = 0; return; }
        double x = (vx - _p.DynamicBlendLowSpeed) / (_p.DynamicBlendHighSpeed - _p.DynamicBlendLowSpeed);
        double w = Smoothstep(x);
        if (w >= 1) return;
        double tanD = Math.Tan(delta);
        double vyKin = vx * (_p.CgToRear / _p.Wheelbase) * tanD;
        double rKin = vx * tanD / _p.Wheelbase;
        vy = w * vy + (1 - w) * vyKin;
        r = w * r + (1 - w) * rKin;
    }

    private static double Smoothstep(double x)
    {
        x = Math.Clamp(x, 0, 1);
        return x * x * (3 - 2 * x);
    }

    private void Rk4Kinematic(ref VehicleState s, VehicleInputs u, double dt)
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

    /// <summary>RK4 of (v_x, v_y, r, ψ, X, Y, distance) with gear (auto-shift re-selected), clutch mode and inputs frozen.</summary>
    private void Rk4Dynamic(ref VehicleState s, VehicleInputs u, double dt)
    {
        var s0 = s;
        bool auto = u.AutoShift && !s0.EngineBlown;
        double[] F(double[] y)
        {
            var sv = s0;
            sv.V = Math.Max(0, y[0]); sv.Vy = y[1]; sv.YawRate = y[2]; sv.Psi = y[3];
            if (auto) sv.Gear = AutoGear(sv.V);
            var o = Evaluate(sv, u);
            double c = Math.Cos(sv.Psi), sn = Math.Sin(sv.Psi);
            return new[]
            {
                o.Ax, o.VyDot, o.YawAccel, sv.YawRate,
                sv.V * c - sv.Vy * sn, sv.V * sn + sv.Vy * c, Math.Sqrt(sv.V * sv.V + sv.Vy * sv.Vy),
            };
        }
        double[] y0 = { s0.V, s0.Vy, s0.YawRate, s0.Psi, s0.X, s0.Y, s0.Distance };
        double[] Add(double[] y, double[] k, double h) => y.Select((v, i) => v + h * k[i]).ToArray();
        var k1 = F(y0);
        var k2 = F(Add(y0, k1, 0.5 * dt));
        var k3 = F(Add(y0, k2, 0.5 * dt));
        var k4 = F(Add(y0, k3, dt));
        var y1 = y0.Select((v, i) => v + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])).ToArray();
        double vNew = Math.Max(0, y1[0]), vy = y1[1], r = y1[2];
        BlendTowardsKinematic(vNew, Math.Clamp(u.SteerAngle, -_p.MaxSteerAngle, _p.MaxSteerAngle), ref vy, ref r);
        s.V = vNew; s.Vy = vy; s.YawRate = r; s.Psi = y1[3]; s.X = y1[4]; s.Y = y1[5]; s.Distance = Math.Max(s0.Distance, y1[6]);
    }

    /// <summary>
    /// Applies an explicit collision response (extension E3) computed by <see cref="Collision.CollisionResponse.Resolve"/>
    /// or <see cref="Collision.SustainedContact.Resolve"/>: the only sanctioned external write to the state. Position
    /// (depenetration), heading and speed are taken from the result; a response may never increase the speed. With the
    /// E4 model the new velocity is expressed in the new body frame (v_x clamped at 0: no reverse) and the yaw rate is
    /// scaled by V'/V.
    /// </summary>
    public void ApplyCollision(in Collision.CollisionResult r)
    {
        if (!double.IsFinite(r.X) || !double.IsFinite(r.Y) || !double.IsFinite(r.Psi) || !double.IsFinite(r.V))
            throw new ArgumentException("Non-finite collision result.", nameof(r));
        double speed = Speed;
        if (r.V > speed + 1e-9)
            throw new ArgumentException("A collision response must not increase speed.", nameof(r));
        var s = State;
        s.X = r.X;
        s.Y = r.Y;
        s.Psi = r.Psi;
        if (!Dynamic)
        {
            s.V = Math.Max(0, r.V);
        }
        else
        {
            double heading = double.IsFinite(r.VelocityHeading) ? r.VelocityHeading : r.Psi;
            double rel = FrameConversion.WrapAngle(heading - r.Psi);
            s.V = Math.Max(0, r.V * Math.Cos(rel));
            s.Vy = r.V * Math.Sin(rel);
            s.YawRate *= speed > 1e-9 ? Math.Min(1, r.V / speed) : 0;
        }
        State = s;
    }

    /// <summary>Outputs at speed <paramref name="v"/> (overrides the state's v_x).</summary>
    public VehicleOutputs Evaluate(in VehicleState s, in VehicleInputs u, double v)
    {
        var sv = s;
        sv.V = v;
        return Evaluate(sv, u);
    }

    /// <summary>Forces, accelerations, yaw rate and the E4/E5 derivatives for the state <paramref name="s"/>.</summary>
    public VehicleOutputs Evaluate(in VehicleState s, in VehicleInputs u)
    {
        var p = _p;
        var o = new VehicleOutputs { LateralModel = p.LateralModel };
        double v = s.V;
        double throttle = Math.Clamp(u.Throttle, 0, 1);
        double brake = Math.Clamp(u.Brake, 0, 1);
        double rw = p.WheelRadius;
        double tr = TotalRatio(s.Gear);
        double cosT = Math.Cos(p.Theta);
        bool slipping = PhysicalClutch && !s.ClutchLocked && !s.EngineBlown;
        o.ClutchLocked = !slipping;

        // Longitudinal (model.m §3-§7)
        o.WheelRpm = EngineModel.WheelRpm(v, rw, tr);
        double massFactorMass = p.Mass * (1 + p.MassFactors[s.Gear - 1]);
        if (s.EngineBlown)
        {
            o.EngineRpm = 0;
            o.EngineTorque = 0;
        }
        else if (slipping)
        {
            // E5: engine speed is its own state; the clutch transmits engagement · capacity.
            double rpmE = Math.Max(p.IdleRpm, s.EngineOmega / RpmToRad);
            o.EngineRpm = rpmE;
            o.RevLimiter = rpmE > p.FuelCutRpm;
            o.EngineTorque = _engine.Torque(rpmE);
            double engagement = Smoothstep((rpmE - p.IdleRpm) / (p.ClutchBiteRpm - p.IdleRpm));
            double win = WheelSideOmega(v, s.Gear);
            double we = rpmE * RpmToRad;
            o.ClutchSlipRpm = (we - win) / RpmToRad;
            o.ClutchTorque = engagement * p.ClutchCapacity * Math.Sign(we - win);
            double friction = throttle > p.ThrottleInputThreshold ? 0 : p.EngineBrakeTorque;
            o.EngineOmegaDot = (o.EngineTorque * throttle - friction - o.ClutchTorque) / p.EngineInertia;
        }
        else
        {
            o.EngineRpm = _engine.EngineRpm(o.WheelRpm, s.Gear, throttle);
            o.RevLimiter = o.EngineRpm > p.FuelCutRpm;
            o.EngineTorque = _engine.Torque(o.EngineRpm);
        }
        o.FTractive = slipping
            ? o.ClutchTorque * tr * p.EtaT / rw
            : o.EngineTorque * throttle * tr * p.EtaT / rw;
        // While slipping the engine inertia is decoupled: only the driveline downstream of the clutch is reflected.
        double driveMass = slipping
            ? Math.Max(p.Mass, massFactorMass - p.EngineInertia * (tr / rw) * (tr / rw))
            : massFactorMass;

        double muEff = p.Mu * p.GripFactor(u.Surface);
        o.FxMax = TractionLimit.FxMax(p, muEff);
        double delta = Math.Clamp(u.SteerAngle, -p.MaxSteerAngle, p.MaxSteerAngle);
        o.SteerAngle = delta;

        o.FDrag = Resistance.Drag(p, v);
        o.FRolling = Resistance.Rolling(p, v, p.RollingFactor(u.Surface));
        o.FGrade = Resistance.Grade(p);
        double fRes = o.FDrag + o.FRolling + o.FGrade;
        double fBrakeMax = (p.BrakeMuFactor * muEff) * (p.Mass * p.G * cosT);
        o.FEngineBrake = slipping ? 0 : p.EngineBrakeTorque * tr / rw;

        if (p.LateralModel == LateralModel.Kinematic)
            EvaluateKinematic(ref o, s, p, v, throttle, brake, fRes, fBrakeMax, muEff, driveMass);
        else
            EvaluateDynamic(ref o, s, p, v, throttle, brake, fRes, fBrakeMax, muEff, driveMass, cosT);
        return o;
    }

    /// <summary>
    /// Drive branch acceleration with the traction limit <paramref name="cap"/>.
    /// model.m (C1 off): a = (min(F, cap) − F_res) / M_eq. C1: the tyre force is F − (M_eq − M)·a_u; if it exceeds the
    /// cap the tyre force is the cap and it accelerates the bare mass.
    /// </summary>
    private static (double A, double FEff, double TyreForce, double Mass) DriveAcceleration(
        VehicleParameters p, double fDrive, double cap, double fRes, double mEq)
    {
        if (!p.GripLimitOnTyreForce)
        {
            double fEff = Math.Min(fDrive, cap);
            double a = (fEff - fRes) / mEq;
            return (a, fEff, fEff - (mEq - p.Mass) * a, mEq);
        }
        double aU = (fDrive - fRes) / mEq;
        double tyre = fDrive - (mEq - p.Mass) * aU;
        if (tyre <= cap) return (aU, fDrive, tyre, mEq);
        return ((cap - fRes) / p.Mass, cap, cap, p.Mass);
    }

    /// <summary>E1: kinematic single-track + friction circle of radius GripUtilization · F_x,max.</summary>
    private static void EvaluateKinematic(ref VehicleOutputs o, in VehicleState s, VehicleParameters p, double v,
        double throttle, double brake, double fRes, double fBrakeMax, double muEff, double driveMass)
    {
        o.FGrip = p.GripUtilization * o.FxMax;
        double delta = o.SteerAngle;
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
        o.FBrake = fBrakeMax * brake * circleScale;

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
        else if (throttle > p.ThrottleInputThreshold || !o.ClutchLocked)
        {
            o.Branch = DriveBranch.Drive;
            var (a, fEff, _, m) = DriveAcceleration(p, o.FTractive, fxAvail, fRes, driveMass);
            o.MassEquivalent = m;
            o.Ax = a;
            o.FTractiveEff = fEff;
            fxUsed = o.FTractiveEff; fxCap = o.FGrip;
        }
        else
        {
            o.Branch = DriveBranch.Coast;
            o.MassEquivalent = driveMass;
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
    }

    /// <summary>
    /// E4: dynamic single-track (bicycle) model in the body frame (x forward, y left, r CCW):
    ///   v̇_x = [model.m longitudinal force + F_xf (cos δ − 1) − F_yf sin δ] / M_eq + v_y r
    ///   v̇_y = (F_yf cos δ + F_xf sin δ + F_yr) / M − v_x r
    ///   ṙ   = [l_f (F_yf cos δ + F_xf sin δ) − l_r F_yr] / I_zz
    /// Slip angles α_f = δ − atan2(v_y + l_f r, v_x), α_r = −atan2(v_y − l_r r, v_x); tyre forces
    /// F_y = F_y,max tanh(C_α α / F_y,max) (linear for small α, saturating at the axle friction limit).
    /// Axle loads: static split by c/L plus longitudinal transfer M a_x h / L (a_x of the previous step).
    /// Friction circle per axle, longitudinal first: F_y,max = √((μ F_z)² − F_x²). Drive (FWD) is limited by model.m's
    /// F_x,max; braking (model.m force) is split by axle load.
    /// </summary>
    private void EvaluateDynamic(ref VehicleOutputs o, in VehicleState s, VehicleParameters p, double v,
        double throttle, double brake, double fRes, double fBrakeMax, double muEff, double driveMass, double cosT)
    {
        double delta = o.SteerAngle;
        double L = p.Wheelbase, lr = p.CgToRear, lf = L - lr;
        double m = p.Mass;
        double w = m * p.G * cosT;
        double transfer = m * s.AxPrev * p.CgHeight / L;
        double fzF = Math.Clamp(w * lr / L - transfer, 0.05 * w, 0.95 * w);
        double fzR = w - fzF;
        o.FzFront = fzF;
        o.FzRear = fzR;
        double gF = muEff * fzF, gR = muEff * fzR;

        // Longitudinal (model.m branches) and the per-axle tyre forces they imply.
        double fxF = 0, fxR = 0, lonDemand = 0;
        o.FBrake = 0;
        if (s.EngineBlown)
        {
            o.Branch = DriveBranch.Blown;
            o.MassEquivalent = m;
            o.Ax = -(m * p.G * muEff + o.FDrag) / m;
            o.FTractiveEff = 0;
            double locked = m * p.G * muEff;
            fxF = -locked * fzF / w;
            fxR = -locked - fxF;
        }
        else if (brake > p.BrakeInputThreshold)
        {
            o.Branch = DriveBranch.Brake;
            o.MassEquivalent = m * (1 + p.BrakeMassFactor);
            o.FBrake = fBrakeMax * brake;
            o.Ax = -(o.FBrake + fRes) / o.MassEquivalent;
            o.FTractiveEff = 0;
            fxF = -o.FBrake * fzF / w;
            fxR = -o.FBrake - fxF;
        }
        else if (throttle > p.ThrottleInputThreshold || !o.ClutchLocked)
        {
            o.Branch = DriveBranch.Drive;
            var (a, fEff, tyre, mass) = DriveAcceleration(p, o.FTractive, o.FxMax, fRes, driveMass);
            o.MassEquivalent = mass;
            o.Ax = a;
            o.FTractiveEff = fEff;
            fxF = Math.Min(tyre, gF);
            if (o.FTractive > 0) lonDemand = o.FxMax > 0 ? o.FTractive / o.FxMax : 10;
        }
        else
        {
            o.Branch = DriveBranch.Coast;
            o.MassEquivalent = driveMass;
            o.Ax = -(fRes + o.FEngineBrake) / o.MassEquivalent;
            o.FTractiveEff = 0;
            fxF = -o.FEngineBrake;
        }

        // Lateral tyre forces within what the longitudinal forces leave of each friction circle.
        double fyMaxF = Math.Sqrt(Math.Max(0, gF * gF - fxF * fxF));
        double fyMaxR = Math.Sqrt(Math.Max(0, gR * gR - fxR * fxR));
        double vxSafe = Math.Max(v, 0.5);
        double vy = s.Vy, r = s.YawRate;
        o.AlphaFront = delta - Math.Atan2(vy + lf * r, vxSafe);
        o.AlphaRear = -Math.Atan2(vy - lr * r, vxSafe);
        double linF = p.CorneringStiffnessFront * o.AlphaFront;
        double linR = p.CorneringStiffnessRear * o.AlphaRear;
        o.FyFront = fyMaxF > 0 ? fyMaxF * Math.Tanh(linF / fyMaxF) : 0;
        o.FyRear = fyMaxR > 0 ? fyMaxR * Math.Tanh(linR / fyMaxR) : 0;

        double cosD = Math.Cos(delta), sinD = Math.Sin(delta);
        double fyFrontBody = o.FyFront * cosD + fxF * sinD;
        double coupling = fxF * (cosD - 1) - o.FyFront * sinD;
        o.Ax = o.Ax + coupling / o.MassEquivalent + vy * r;
        double fyBody = fyFrontBody + o.FyRear;
        o.VyDot = fyBody / m - v * r;
        o.YawAccel = (lf * fyFrontBody - lr * o.FyRear) / p.YawInertia;

        o.Ay = fyBody / m;
        o.YawRate = r;
        o.Vx = v;
        o.Vy = vy;
        o.Beta = v > 0 || vy != 0 ? Math.Atan2(vy, v) : 0;
        o.Fy = fyBody;
        o.FGrip = gF + gR;
        o.FyDemand = Math.Abs(linF) + Math.Abs(linR);
        double demandF = fyMaxF > 0 ? Math.Abs(linF) / fyMaxF : (linF != 0 ? 10 : 0);
        double demandR = fyMaxR > 0 ? Math.Abs(linR) / fyMaxR : (linR != 0 ? 10 : 0);
        o.Understeer = demandF > 1 && demandF >= demandR;
        o.Oversteer = demandR > 1 && demandR > demandF;
        double useF = gF > 0 ? Math.Sqrt(fxF * fxF + o.FyFront * o.FyFront) / gF : 0;
        double useR = gR > 0 ? Math.Sqrt(fxR * fxR + o.FyRear * o.FyRear) / gR : 0;
        o.GripUsage = Math.Max(useF, useR);
        o.SlipIndicator = Math.Min(10, Math.Max(lonDemand, Math.Max(demandF, demandR)));
    }
}
