# Vehicle model (Phase 1): formulation of `model.m` and of the game extensions

Source of truth: `reference/matlab/model.m` (SHA-256 `b665b69c…697b`). Sections 0–7 and 9 describe that file
*as coded*, including quirks. Everything that is not in `model.m` is marked **[EXT]** with its source.
Numbers marked "(MATLAB)" were printed by MATLAB R2026a running the unmodified file (figures hidden);
numbers marked "(Python)" come from an independent numpy check and are not MATLAB output.

## 0. Nature of the model

`model.m` is **not** a time-domain vehicle simulator. It is:
1. a quasi-static **performance map**: per gear, tractive force, resistances and acceleration as functions of engine
   speed on a fixed 50-rpm grid (§3–§7);
2. a **lap-time estimator** for an oval (§10–§11): curve speed from a friction-circle check, full-throttle straights
   integrated in time with Euler steps on the pre-computed acceleration table, braking computed by a velocity-domain sum,
   curves at constant speed.

There is no lateral, yaw, vertical, pitch/roll, wheel-spin or tyre dynamics. The only integrated "state" (§11) is
$(t, d, v)$ along a straight, with the gear an algebraic function of $v$.

## 1. Notation, units, inputs

| Symbol | model.m | Value (default) | Unit |
|---|---|---|---|
| $g$ | `env.g` | 9.81 | m/s² |
| $\rho$ | `env.rho_air` | 1.225 | kg/m³ |
| $\theta$ | `env.theta` from `slope_mode`, `slope_value` | pct 0 → 0 | rad (+ = uphill) |
| $M$ | `car.M` | 1420 | kg |
| $C_d$, $A$ | `car.Cd`, `car.A` | 0.378, 1.99 | –, m² |
| $i_g$ | `car.gear_ratios` | 3.214, 1.792, 1.194, 0.914, 0.784, 0.683 | – |
| $i_{fd}$ | `car.final_drive` | 4.353 | – |
| $\eta_t$ | `car.eta_t` | 0.92 | – |
| $\lambda_g$ | `car.mass_factors` | 0.50, 0.20, 0.18, 0.16, 0.14, 0.11 | – |
| $R_w$ | `car.R_w` | 0.30425 (MATLAB) | m |
| $\mu$ | `car.mu` | 0.90 | – |
| $L$, $c$, $h$ | `car.L`, `car.c`, `car.h` | 2.467, 1.529 (CG→**rear** axle), 0.503 | m |
| $t_f$, $t_r$ | `car.t_f`, `car.t_r` | 1.459, 1.467 (`t_r` unused) | m |
| $K_{s,f}$, $K_{s,r}$ | `car.K_s_f`, `car.K_s_r` | 28000, 37100 | N/m |
| $s_f$, $s_r$ | `car.s_f`, `car.s_r` | 1.059, 1.067 | m |
| $R$ | `R_curva` | 80 | m |
| $k_s$ | `margin_safety` | 0.80 | – |
| $L_s$ | `L_recta` | 400 | m |
| $\Delta t$ | `dt` | 0.005 | s |

Declared but unused by any computation: `fuel.LHV`, `fuel.AFR`, `car.V_d`, `car.CR`, `car.t_r`; `car.HP_engine`,
`P_available_kW`, `P_demanded_*`, `a_x_envelope` feed plots only. Drivers' inputs do not exist: throttle is
implicitly 1 on straights and braking is implicitly maximal.

## 2. Equations, section by section

### §1 Slope
$$\theta=\begin{cases}\tfrac{\pi}{180}\,\text{slope\_value} & \text{deg}\\ \arctan(\text{slope\_value}/100) & \text{pct}\\ \text{slope\_value} & \text{rad}\end{cases}$$
(`deg2rad(x)` is `(pi/180)*x` in MATLAB.)

### §2 Wheel radius and roll stiffness
$$R_w=\tfrac{15\cdot0.0254}{2}+0.175\cdot0.65=0.1905+0.11375=0.30425\ \text{m}$$
$$K_{\phi f}=\tfrac12K_{s,f}s_f^2,\quad K_{\phi r}=\tfrac12K_{s,r}s_r^2,\quad K_\phi=K_{\phi f}+K_{\phi r}$$
(MATLAB) $K_{\phi f}=15700.734$, $K_{\phi r}=21118.97095$, $K_\phi=36819.70495$ N·m/rad, $K_{\phi r}/K_\phi=0.5735779518$.

### §3 Engine map
Engine-speed grid `rpm_axis = 1000:50:6500` (111 points), $\omega=n\cdot2\pi/60$.
Nodes $n_j$ = 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4250, 4500, 5000, 5500, 6000, 6400, 6500 rpm,
$T_j$ = 75, 120, 135, 145, 150, 155, 158, 160, 158, 155, 148, 142, 125, 118 Nm.
$$T_e(n)=\mathrm{PCHIP}(n_j,T_j)(n),\qquad P_e=T_e\,\omega,\qquad HP=P_e/745.7$$
(MATLAB) $T_{max}=160$ Nm at 4250 rpm; $T_e(6500)=118$ Nm; $P_{max}=89231.094$ W (119.661 HP) at 6050 rpm on the grid.
PCHIP node slopes (MATLAB, Nm/rpm): 0.12, 0.045, 0.024, 0.013333, 0.01, 0.0075, 0.0069677, 0, −0.0069677, −0.0084,
−0.0129231, −0.0191117, −0.0556075, −0.0755. Algorithm in §9.1.

### §4 Resistances on a speed grid
`v_kmh = 0.1:1:220` (220 points, last 219.1), $v=v_{kmh}/3.6$, $v_{mph}=v_{kmh}/1.60934$.
$$F_{drag}=\tfrac12\rho A C_d v^2,\qquad F_{roll}=0.01\left(1+\tfrac{v_{mph}}{100}\right)Mg\cos\theta,\qquad F_{grade}=Mg\sin\theta$$
$$F_{res}=F_{drag}+F_{roll}+F_{grade},\qquad P_{dem}=F_{res}\,v$$

### §5 Per-gear kinematics and ideal acceleration (for each gear $g$, each grid rpm)
$$i_{tot}=i_g\,i_{fd},\quad v_g=\frac{\omega}{i_{tot}}R_w,\quad V_{g,kmh}=3.6\,v_g$$
$$T_{wheel}=T_e\,i_{tot}\,\eta_t,\quad F_{trac}=T_{wheel}/R_w,\quad P_{avail}=P_e\eta_t/1000\ \text{(kW)}$$
$$M_{eq,g}=M(1+\lambda_g),\qquad a_{x}=\frac{F_{trac}-F_{res}(v_g)}{M_{eq,g}}$$
with $F_{res}(v_g)$ the §4 formulas evaluated at $v_g$ (rolling uses `V_gear_kmh/1.60934`).
(MATLAB) speed ranges [1000, 6500 rpm]: g1 8.198–53.289, g2 14.704–95.576, g3 22.068–143.444, g4 28.829–187.387,
g5 33.609–218.459, g6 38.579–250.764 km/h.

### §6 Traction limit $F_{x,max}$ (labelled FWD in model.m)
$$W=Mg\cos\theta,\qquad F_{x,max}=\frac{\mu\,W\,c/L}{1+\mu\,\dfrac{h}{L}+\dfrac{2\mu R_w}{i_{fd}\,t_f}\,\dfrac{K_{\phi r}}{K_\phi}}$$
(MATLAB, baseline) $W=13930.2$ N; numerator $=7770.307345$ N; denominator $=1+0.1835022294+0.04945971184=1.232961941$;
$F_{x,max}=6302.146956$ N $=0.452409\,W$. Static front-axle share $c/L=0.619781$.
Reading: numerator = μ × static front-axle load; the $\mu h/L$ term = longitudinal load transfer off the front
axle; the last term has the form of the lateral load transfer caused by drive-shaft torque reaction
($T_{ds}=F_xR_w/i_{fd}$) shared by roll stiffness, as in the textbook solid-drive-axle traction-limit expression.
model.m cites no source; using it for a transverse-engine FWD car, with front track and *rear* roll-stiffness share,
is an assumption of model.m that this project does not validate. It is gear-independent.

### §7 Effective traction and real acceleration
$$F_{trac,eff}=\min(F_{trac},F_{x,max}),\qquad a_{x,real}=\frac{F_{trac,eff}-F_{res}(v_g)}{M_{eq,g}}$$
(MATLAB) only 1st gear is traction-limited (51 of 111 grid points; max $F_{trac}$ 6768.80 N at 4250 rpm);
gear 2 peaks at 3774.02 N.

### §8–§9 Plots; envelope
$$a_{env}(v)=\frac{\max(0,\;\max(P_e)\eta_t-P_{dem}(v))/v}{M(1+\overline{\lambda})}$$
Plot only; it diverges as $v\to0$ (MATLAB: 1712.86 m/s² at 0.1 km/h). Not used for dynamics.

### §10 Curve speed (quasi-static friction circle)
$F_{lim}=k_s F_{x,max}$ (MATLAB: 5041.717565 N). Search grid `v_search_kmh = 1:0.1:220` (2191 points). For
$i=1,2,\dots$: $F_y=Mv_i^2/R$, $F_{x,req}=F_{res}(v_i)$ (§4 formulas, constant speed),
$$\text{stop at the first } i \text{ with } \sqrt{F_{x,req}^2+F_y^2}>F_{lim};\quad v_{adh}=v_{i-1}\ (\text{or } v_1 \text{ if } i=1)$$
If never exceeded, $v_{adh}=\max(v_{search})=220$ km/h. Then gear check over $g=1..5$ only: first $g$ with
$\min V_g\le v_{adh}\le\max V_g$; $v_{mot}=V_g[\text{last } k: a_x(g,k)\ge0]$ (uses the *ideal* `a_x_ms2`, not
`a_x_real`; falls back to $\max V_g$ if none). If no gear contains $v_{adh}$, gear 5 is used the same way.
$$v_{curva}=\min(v_{adh},v_{mot})$$
(MATLAB, baseline) $v_{adh}=60.6$ km/h (closed form without $F_{x,req}$ would give 60.6726 km/h); gear 2;
$v_{mot}=95.5759$ km/h; $v_{curva}=60.6$ km/h. Note: $F_{x,max}$ (a driven-axle longitudinal limit, 0.452 g) is used
as the radius of the *total* friction circle, i.e. the car may corner at ≈0.36 g at the 0.8 margin.

### §11 Oval lap
Geometry $L_c=\pi R$ (251.3274 m). Brakes: $F_b=0.90\,\mu\,Mg\cos\theta$ (11283.462 N), $M_b=1.05M$ (1491 kg).

**Gear rule** (each step): $g$ = first gear with $3.6v\le\max V_g$; initialised to 1, so if $v$ exceeds the 6th-gear
top speed (250.76 km/h) the loop leaves $g=1$.

**Acceleration** (each step):
$$a=\max\Big(0,\ \mathrm{interp1}\big(V_{g,kmh},a_{x,real}(g,\cdot),3.6v,\ \text{linear, extrap}\big)\Big)$$
after `unique(V_g,'stable')` (a no-op: every $V_g$ row is strictly increasing — MATLAB check).

**Straight R1** ($t=0,d=0,v=v_0=\min V_1/3.6$ = 8.198363 km/h):
```
while d < L_recta && t < 60
    choose g, compute a            (from v at start of step)
    log t, d, 3.6 v                (BEFORE the update)
    v = v + a dt
    d = d + v dt + 0.5 a dt^2      (uses the NEW v)
    t = t + dt                     (accumulated sum)
end
```
**Braking distance** (velocity-domain, after the loop): $v_e=$ `history_v(end)/3.6` (the *last logged* speed, i.e. near
the end of the straight, not the speed at the braking point). If $v_e>v_{curva}$:
$$v_k\in v_e:-0.05:v_{curva},\quad a_k=\frac{F_b+F_{res}(v_k)}{M_b},\quad \Delta t_k=\frac{0.05}{a_k},\quad d_b=\sum_k v_k\Delta t_k,\ t_b=\sum_k\Delta t_k$$
(a left Riemann sum of $\int v/a\,dv$ that always covers $n\cdot0.05\ge v_e-v_{curva}$, $n=$ length of the colon range).
**Brake point**: $d_{acc}=\max(0,L_s-d_b)$; $k^*$ = first index with `history_d >= d_acc` (else the last index);
$t_{acc}=$ `history_t(k*)`, $v_{max}=$ `history_v(k*)`, $t_{R}=t_{acc}+t_b$.
**Curves**: $t_C=L_c/v_{curva}$. **R2**: identical to R1 with $v_0=v_{curva}$. **Lap**: $t=t_{R1}+t_{C1}+t_{R2}+t_{C2}$,
distance $2L_s+2L_c$.

(MATLAB, baseline) R1: 3713 logged steps, gears 1–3, last logged $d=399.952673$ m, $v_e=123.291832$ km/h; braking
loop 349 points, $d_b=56.141391$ m, $t_b=2.202438$ s; $d_{acc}=343.858609$ m, $k^*=3378$, $t_{acc}=16.885$ s,
$v_{max}=117.620243$ km/h, $t_{R1}=19.087438$ s. C: 14.930341 s each. R2: 2878 steps, $v_e=128.343171$ km/h, 377 points,
$d_b=62.183887$ m, $t_b=2.375000$ s, $t_{acc}=12.605$ s, $v_{max}=122.659432$ km/h, $t_{R2}=14.98$ s.
Lap 63.928121245 s over 1302.654825 m. These match DESIGN_CONTRACT §0.

## 3. State, inputs, outputs

| Layer | State | Inputs | Algebraic |
|---|---|---|---|
| model.m §11 | $t$ [s], $d$ [m], $v$ [m/s] (logged as km/h) | none (full throttle, then max braking) | gear $g(v)$, $a(v)$ |
| [EXT] interactive (DESIGN_CONTRACT §2b) | `X, Y` [m], `Psi` [rad], `V` [m/s], `Gear`, `Distance` [m], `Time` [s], `EngineBlown` | `Throttle`, `Brake` [0..1], `SteerAngle` [rad], `ShiftUp/Down`, `AutoShift` | `EngineRpm`, forces, a_x, a_y, r, β |

## 4. Initial conditions

R1: $t=0$, $d=0$, $v=\min(V_{1,kmh})/3.6$ = speed at exactly 1000 rpm in 1st (8.1983634217 km/h). R2: $t=0$, $d=0$,
$v=v_{curva}$ (gear chosen by the rule; 2nd at baseline). Curves start/end at $v_{curva}$ (no transient).

## 5. Integration and discretisation (exactly as coded)

### 5.1 Straights
Velocity: explicit Euler with $a$ evaluated at the start-of-step speed. Position: $d_{n+1}=d_n+v_{n+1}\Delta t+\tfrac12a_n\Delta t^2
=d_n+v_n\Delta t+\tfrac32a_n\Delta t^2$. Compared with the trapezoid $d_n+\tfrac12(v_n+v_{n+1})\Delta t$ it adds
$a_n\Delta t^2$ per step, i.e. a total bias of $\Delta t\,(v_{end}-v_0)$: 0.15985 m on baseline R1 and 0.2375 m on S07
(Python, from the golden histories; equals $\Delta t\,\Delta v$ to 1e-12). This is the "d-update quirk"; the reference
layer reproduces it literally.
Termination tests use the pre-update $d$ and the accumulated $t$; the last logged sample has $d<L_s$.
Time is a running sum (`t = t + dt`), e.g. 59.99500000001224 after 11999 additions, so the `t < 60` cap stops after
exactly 12000 steps (S07).

### 5.2 Braking
Not time-integrated: velocity grid of step 0.05 m/s from $v_e$ downwards, left Riemann sum. Overestimates the exact
integral slightly (baseline R1: 56.141 m vs 56.013 m for $\int_{v_{curva}}^{v_e}v/a\,dv$, Python) and, more importantly,
starts from $v_e$ (123.29 km/h) although the car leaves the straight's acceleration phase at $v_{max}$ = 117.62 km/h
(exact braking from 117.62 km/h would need 49.49 m / 2.003 s, Python). Lap time therefore is not kinematically
self-consistent; reproduce, don't fix, in the reference layer.

### 5.3 Time cap
If the 60 s cap ends the loop before $d\ge d_{acc}$, `find` is empty, $k^*$ = last index, and the unreached distance is
silently dropped from the lap time. Example (golden S07, L_recta = 3000): $d$ at cap 2238.96 m, $d_b$ 134.60 m, so
626.44 m of the straight are not timed.

## 6. Saturations and branches
- $F_{trac,eff}=\min(F_{trac},F_{x,max})$ pointwise on the grid (§7).
- $a=\max(0,a)$ on straights: with $a_{x,real}<0$ (e.g. steep uphill) the car holds speed instead of slowing down
  (not triggered in S01/S02 — MATLAB check: 0 clamps).
- Gear rule falls back to gear 1 above the 6th-gear top speed; curve gear search is limited to gears 1–5 with fallback 5.
- Braking block skipped when $v_e\le v_{curva}$ ($d_b=0$, $k^*$ = last).
- $d_{acc}=\max(0,\cdot)$.
- No reverse, no $v<0$ handling (never needed: $a\ge0$ on straights).

## 7. Validity range and assumptions
1. Longitudinal, quasi-static, point mass; rotational inertia lumped in per-gear mass factors (driving) and 5 % (braking).
2. Engine data only on [1000, 6500] rpm. Below $\min V_g$ the acceleration is a linear *extrapolation* of `a_x_real`
   in speed (never reached in baseline: R1 starts exactly at 1000 rpm; MATLAB check 0 extrapolations); above 6500 rpm
   the gear rule always upshifts; above the 6th-gear top speed (250.76 km/h) the rule falls back to gear 1 and `interp1`
   extrapolates the 1st-gear row (unreachable at baseline).
3. Steady full throttle; no shift time, no clutch, no torque interruption at shifts.
4. $F_{x,max}$ is a driven-axle traction limit (≈0.45 g) but §10 uses it as the full friction-circle radius and §11 brakes
   at 0.81 g on all four wheels: the three grip levels are mutually inconsistent.
5. Rolling resistance $0.01(1+v_{mph}/100)$ is an empirical passenger-tyre fit, independent of surface and load
   distribution; drag uses constant $C_dA$, still air.
6. Load transfer appears only inside the $F_{x,max}$ formula; no pitch/roll dynamics.
7. Grids: speed grid 0.1–219.1 km/h (§4) and search grid 1–220 km/h (§10). A curve speed above 220 km/h is reported as 220.
8. Mass changes do not change $c$, $h$, $K_\phi$ (no CG model).

## 8. MATLAB → C# mapping

Reference layer names are those present in `src/VehicleDynamics.Core/Reference/*.cs` at the time of writing.
Interactive-layer names come from DESIGN_CONTRACT §2b (`Core.Simulation` did not exist yet).

| model.m | C# reference layer | C# interactive layer |
|---|---|---|
| `env.g`, `env.rho_air` | `ReferenceParameters.G`, `.RhoAir` | same parameter set (contract: defaults = model.m) |
| `slope_mode`, `slope_value`, `env.theta` | `.SlopeMode`, `.SlopeValue`, `.Theta` | slope parameter → θ |
| `car.M`, `car.Cd`, `car.A` | `.M`, `.Cd`, `.A` | |
| `car.gear_ratios`, `car.final_drive`, `car.eta_t` | `.GearRatios`, `.FinalDrive`, `.EtaT` | |
| `car.mass_factors` | `.MassFactors` | |
| `tire_width`, `aspect_ratio`, `rim_diameter`, `car.R_w` | `.TireWidth`, `.AspectRatio`, `.RimDiameterInch`, `.Rw` | |
| `car.mu`, `car.L`, `car.c`, `car.h`, `car.t_f`, `car.t_r` | `.Mu`, `.L`, `.C`, `.H`, `.Tf`, `.Tr` | `l_r = C`, `l_f = L − C` (E1) |
| `car.K_s_f/r`, `car.s_f/r`, `car.K_phi_f/r`, `car.K_phi` | `.KsF`, `.KsR`, `.SF`, `.SR`, `.KPhiF`, `.KPhiR`, `.KPhi` | |
| `fuel.LHV`, `fuel.AFR`, `car.V_d`, `car.CR` | `.FuelLHV`, `.FuelAFR`, `.Vd`, `.CR` (unused) | – |
| `rpm_raw`, `torque_raw` | `.RpmRaw`, `.TorqueRaw`; `Pchip` | `Pchip.Evaluate(rpm)` directly |
| `car.rpm_axis`, `car.omega`, `car.T_engine`, `car.P_engine_W`, `car.HP_engine` | `ReferenceTables.RpmAxis`, `.Omega`, `.TEngine`, `.PEngineW`, `.HpEngine` | `EngineRpm` (state) |
| `v_kmh`, `v_ms`, `F_drag`, `F_rolling`, `F_pend`, `F_resistive`, `P_demanded_W/kW`, `a_x_envelope` | `.VKmh`, `.VMs`, `.FDrag`, `.FRolling`, `.FPend`, `.FResistive`, `.PDemandedW/KW`, `.AxEnvelope` | CSV `f_drag_n`, `f_rolling_n`, `f_grade_n` |
| `V_gear_kmh`, `P_available_kW`, `T_wheel_Nm`, `F_tractive_N`, `a_x_ms2` | `.VGearKmh`, `.PAvailableKW`, `.TWheelNm`, `.FTractiveN`, `.AxMs2` | CSV `f_tractive_n` |
| `car.F_x_max` | `.FxMax`, `ReferenceModel.FxMax(...)` | CSV `f_x_max_n` |
| `F_tractive_effective`, `a_x_real` | `.FTractiveEffective`, `.AxReal` | CSV `f_tractive_eff_n`, `ax_mps2` |
| `R_curva`, `margin_safety`, `v_search_kmh` | `.RCurva`, `.MarginSafety`, `.VSearchStart/Step/End`, `.CurveGearSearchMax` | track radius / gripUtilization (E1) |
| `v_curva_adherencia_kmh`, `optimal_gear_curva`, `v_max_gear_motriz_kmh`, `v_curva_final_kmh/_ms` | `OvalResult.VCurvaAdherenciaKmh`, `.OptimalGearCurva`, `.VMaxGearMotrizKmh`, `.VCurvaFinalKmh/Ms` | – |
| `L_recta`, `L_curva` | `.LRecta`, `OvalResult.LCurva` | Oval track geometry |
| `0.90`, `0.05` (braking), `dt`, `60`, `0.05` (brake grid) | `.BrakeMuFactor`, `.BrakeMassFactor`, `.Dt`, `.TimeCap`, `.BrakeSpeedStep` | brake force, `SimulationClock` dt |
| `history_t/d/v_r1/r2` | `StraightResult.HistoryT`, `.HistoryD`, `.HistoryVKmh` | `Time`, `Distance`, `V` |
| `d/t_frenado_r*`, `d_acc_efectiva_r*`, `t_acc_r*`, `t_recta*_total`, `v_max_recta*` | `.DFrenado`, `.TFrenado`, `.BrakeSteps`, `.DAccEfectiva`, `.TAcc`, `.TTotal`, `.VMaxKmh` | – |
| `t_curva1/2_total`, `t_vuelta_completa`, `distancia_lap` | `OvalResult.TCurva1Total`, `.TCurva2Total`, `.TVueltaCompleta`, `.DistanciaLap` | lap timer |

## 9. MATLAB → C# porting notes

### 9.1 `interp1(x,y,xq,'pchip')`
Shape-preserving slopes as in MATLAB `pchip` (Fritsch–Carlson family, weighted harmonic mean): $h_k=x_{k+1}-x_k$, $\delta_k=(y_{k+1}-y_k)/h_k$.
Interior: if $\delta_{k-1}\delta_k>0$, $d_k=(w_1+w_2)/(w_1/\delta_{k-1}+w_2/\delta_k)$ with $w_1=2h_k+h_{k-1}$, $w_2=h_k+2h_{k-1}$;
else $d_k=0$. Ends (3-point): $d_1=((2h_1+h_2)\delta_1-h_1\delta_2)/(h_1+h_2)$; if $\mathrm{sign}(d_1)\ne\mathrm{sign}(\delta_1)$ then
$d_1=0$, else if $\mathrm{sign}(\delta_1)\ne\mathrm{sign}(\delta_2)$ and $|d_1|>|3\delta_1|$ then $d_1=3\delta_1$ (mirror for $d_n$).
Evaluate in `ppval` form: $s=x_q-x_k$, $c_3=(d_k-2\delta_k+d_{k+1})/h_k^2$, $c_2=(3\delta_k-2d_k-d_{k+1})/h_k$,
$T=((c_3s+c_2)s+d_k)s+y_k$; interval $k$ with $x_k\le x_q<x_{k+1}$, last node in the last interval.
Verified in MATLAB: coefficients equal `pchip(x,y).coefs` to 5e-20, values equal `interp1` to 2.8e-14 abs (2.2e-16 rel)
on `rpm_axis`. Outside the node range `interp1 pchip` *extrapolates* the end cubic (950 rpm → 68.93 Nm, 6550 rpm →
114.42 Nm); the interactive layer clamps instead (contract).

### 9.2 `interp1(x,y,xq,'linear','extrap')`
MATLAB R2026a result equals, **bit for bit**, $s=(x_q-x_k)/(x_{k+1}-x_k)$, $y=(1-s)\,y_k+s\,y_{k+1}$ (0 mismatches on
161 946 queries: all R1/R2 histories + 120 000 random in-range + 2 400 extrapolated queries, all six gears).
The forms $y_k+(x_q-x_k)\,((y_{k+1}-y_k)/(x_{k+1}-x_k))$ and $y_k+s(y_{k+1}-y_k)$ differ in the last ulp in ~32 % of
those queries. Interval: $k$ with $x_k\le x_q<x_{k+1}$; first/last interval for extrapolation.

### 9.3 Colon operator `a:d:b`
Element values are built symmetrically, not as $a+kd$ (on the R1 braking range the naive form differs by up to
3.6e-15). Empirical model that reproduced MATLAB R2026a on 20 003 ranges (ascending 0.1 / 1 / 50 steps and descending
−0.05 steps, endpoints hit and not hit, including `1:0.1:220`, `0.1:1:220`, `1000:50:6500`):
```
tol = 2*eps(max(|a|,|b|));   n = floor((b-a)/d + tol/|d|)       (number of intervals)
c = a + n*d;  if |c-b| < tol: c = b
v[k] = a + k*d,  v[n-k] = c - k*d   for k = 0..floor(n/2)   (write the c-side first, then the a-side)
if n even: v[n/2] = a + (c-a)/2      <- NOT (a+c)/2: that failed on ~9-11 % of the random ascending 0.1-step ranges tested
```
For model.m's own ranges `(a+c)/2` and `a+(c-a)/2` coincide (checked), but a general port should use `a+(c-a)/2`.
Baseline R1 braking range: length 349 = $\lfloor(v_e-v_{curva})/0.05\rfloor+1$ (MATLAB).

### 9.4 `unique(v,'stable')`
Returns first occurrences in original order; on `V_gear_kmh` rows it is the identity (checked). Port it anyway.

### 9.5 `find(x >= v, 1, 'first')` / `find(a >= 0, 1, 'last')`
1-based in MATLAB → 0-based in C#; empty result → fallback (last index / `max(V_g)`). Keep the fallback order.

### 9.6 `min`, `max`, `sign`
`min(F, Fx)` with NaN returns the non-NaN operand in MATLAB (`Math.Min` returns NaN) — irrelevant for finite data but
keep a `MatlabMin` helper. `max(0,a)` same. `sign(0)=0`.

### 9.7 Floating-point ordering
Keep MATLAB's operation order literally: `0.5*rho*A*Cd*(v^2)` is $(((0.5\rho)A)C_d)v^2$; `car.M*env.g*cos(theta)`
left to right; `(0.01*(1+(v_mph/100)))*(M*g*cos θ)`; `T_engine*total_ratio*eta_t`; `V/3.6` and `/1.60934` divisions
(not multiplications by reciprocals); `omega = rpm*(2*pi/60)` (parenthesised constant). Accumulate `t = t + dt`
(do not use `k*dt`). MATLAB `x^2` / `x.^2` equal `x*x` bit for bit (MATLAB check, 1e6 random values).

## 10. UI-exposable parameters

The setup panel exposes exactly the entries of `VehicleDynamicsSimulator/scripts/ui/ParameterCatalog.cs`. The table below
is generated from that file. Every range lies inside `VehicleParameters.Validate()`. Cross-parameter rules (strictly
decreasing gear ratios, c within 5–95 % of L, idle < fuel cut < money shift, clutch target between idle and fuel cut) are
checked on the edited copy before Apply. Not exposed: `g` (a physical constant), `margin_safety` (used only by the
reference/validation layer), the rpm positions of the torque nodes (only their torque values are editable), and the
asphalt surface factors (μ itself covers asphalt).

| Group | Parameter | Unit | Range | Step | Source | Meaning |
|---|---|---|---|---|---|---|
| Vehicle body & environment | Mass M | kg | 600 – 3000 | 10 | model.m | Running mass (car.M). Enters F_x,max, rolling, grade and every inertia term. |
| Vehicle body & environment | Drag coefficient Cd | – | 0.15 – 0.8 | 0.001 | model.m | Aerodynamic drag coefficient (car.Cd): F_drag = ½ρ A Cd V². |
| Vehicle body & environment | Frontal area A | m² | 1.0 – 4.0 | 0.01 | model.m | Projected frontal area (car.A). |
| Vehicle body & environment | Air density ρ | kg/m³ | 0.9 – 1.4 | 0.005 | model.m | env.rho_air (1.225 = sea level, 15 °C). |
| Vehicle body & environment | Road grade | % | -20 – 20 | 0.5 | model.m | model.m slope (pct mode). One grade for the whole track, as in model.m: + = uphill. |
| Tyres & traction limit (F_x,max, FWD) | Friction coefficient μ | – | 0.3 – 1.6 | 0.01 | model.m | Tyre–road friction (car.mu). Scales F_x,max, braking force and (E1) cornering grip. |
| Tyres & traction limit (F_x,max, FWD) | Tyre width | m | 0.145 – 0.255 | 0.005 | model.m | 175/65 R15 → 0.175. With aspect ratio and rim gives R_w. |
| Tyres & traction limit (F_x,max, FWD) | Aspect ratio | – | 0.3 – 0.8 | 0.05 | model.m | Sidewall height / width. |
| Tyres & traction limit (F_x,max, FWD) | Rim diameter | in | 13 – 20 | 1 | model.m | Wheel rim diameter. |
| Tyres & traction limit (F_x,max, FWD) | Wheelbase L | m | 2.0 – 3.2 | 0.001 | model.m | car.L. Also the E1 kinematic steering geometry. |
| Tyres & traction limit (F_x,max, FWD) | CG to rear axle c | m | 0.5 – 2.5 | 0.001 | model.m | car.c (must stay within 5–95 % of L). Front-axle load share = c/L. |
| Tyres & traction limit (F_x,max, FWD) | CG height h | m | 0.3 – 0.8 | 0.001 | model.m | car.h. Longitudinal load transfer term (h/L)μ of F_x,max. |
| Tyres & traction limit (F_x,max, FWD) | Front track t_f | m | 1.2 – 1.8 | 0.001 | model.m | car.t_f. Enters the torque-induced lateral load-transfer term of F_x,max. |
| Tyres & traction limit (F_x,max, FWD) | Rear track t_r | m | 1.2 – 1.8 | 0.001 | model.m | car.t_r (defined in model.m; not used by its equations). |
| Tyres & traction limit (F_x,max, FWD) | Front spring K_s,f | N/m | 10000 – 100000 | 100 | model.m | car.K_s_f → roll stiffness K_φ,f = ½ K s². |
| Tyres & traction limit (F_x,max, FWD) | Rear spring K_s,r | N/m | 10000 – 100000 | 100 | model.m | car.K_s_r → K_φ,r. F_x,max uses K_φ,r / K_φ. |
| Tyres & traction limit (F_x,max, FWD) | Front spring spacing s_f | m | 0.8 – 1.3 | 0.001 | model.m | car.s_f. |
| Tyres & traction limit (F_x,max, FWD) | Rear spring spacing s_r | m | 0.8 – 1.3 | 0.001 | model.m | car.s_r. |
| Drivetrain | Final drive i_fd | – | 2.5 – 6.0 | 0.001 | model.m | car.final_drive. |
| Drivetrain | Transmission efficiency η_t | – | 0.6 – 1.0 | 0.005 | model.m | car.eta_t. |
| Drivetrain | Gear 1…6 ratio | – | 0.4 – 5.0 | 0.001 | model.m | car.gear_ratios. Must decrease strictly from 1st to 6th. |
| Drivetrain | Mass factor gear 1…6 | – | 0.0 – 1.0 | 0.01 | model.m | car.mass_factors: rotational inertia, M_eq = M(1 + factor). |
| Engine full-load torque (model.m nodes, PCHIP) | Torque @ each torque_raw node (14) | N·m | 0 – 300 | 1 | model.m | torque_raw node; the curve between nodes is PCHIP-interpolated as in model.m. |
| Braking | Braking μ factor | – | 0.3 – 1.2 | 0.01 | model.m | model.m §11: F_brake = factor · μ · M g cosθ (0.90). |
| Braking | Braking mass factor | – | 0 – 0.2 | 0.005 | model.m | model.m §11: M_eq,brake = M(1 + factor) (0.05). |
| Engine rules from the JS seed game | Idle speed | rpm | 600 – 1500 | 10 | JS seed | Engine speed floor; torque below the first node uses the 1000-rpm value. |
| Engine rules from the JS seed game | Fuel cut | rpm | 5000 – 7000 | 10 | JS seed | Torque = 0 at or above (rev limiter). |
| Engine rules from the JS seed game | Clutch launch target | rpm | 2000 – 5000 | 10 | JS seed | 1st-gear launch: engine held at idle + throttle·(target − idle) until the wheels catch up. |
| Engine rules from the JS seed game | Engine braking torque | N·m | 0 – 80 | 1 | JS seed | Coasting drag torque at the crank (seed: 30). |
| Engine rules from the JS seed game | Money-shift limit | rpm | 6600 – 9000 | 10 | JS seed | Wheel-driven engine speed that destroys the engine (seed: 7200). |
| Engine rules from the JS seed game | Mechanical damage |  | on / off |  | JS seed | Over-revving above the money-shift limit destroys the engine. |
| Engine rules from the JS seed game | Clutch launch |  | on / off |  | JS seed | Use the seed's clutch-slip launch rule in 1st gear. |
| Steering & cornering (extension E1) | Max steer angle δ_max | rad | 0.05 – 0.6 | 0.005 | Ext. E1 | Road-wheel angle at full input. Not in model.m; the seed used 0.06. |
| Steering & cornering (extension E1) | Grip utilisation | – | 0.5 – 1.3 | 0.01 | Ext. E1 | Friction-circle radius = factor · F_x,max (model.m §10 uses 0.8 as a planning margin). |
| Surfaces (extension E2) | Curb grip factor | – | 0.3 – 1.2 | 0.01 | Ext. E2 | μ multiplier on the curb band. |
| Surfaces (extension E2) | Grass grip factor | – | 0.1 – 1.0 | 0.01 | Ext. E2 | μ multiplier on grass. |
| Surfaces (extension E2) | Curb rolling factor | – | 0.5 – 5 | 0.1 | Ext. E2 | Rolling-coefficient multiplier on the curb band. |
| Surfaces (extension E2) | Grass rolling factor | – | 1 – 15 | 0.1 | Ext. E2 | Rolling-coefficient multiplier on grass. |
| Barrier impacts (extension E3) | Restitution e | – | 0 – 0.8 | 0.01 | Ext. E3 | Normal velocity after impact = −e · before. |
| Barrier impacts (extension E3) | Tangential loss k_t | – | 0 – 0.6 | 0.01 | Ext. E3 | Scrape loss of the tangential velocity, proportional to impact severity. |
| Barrier impacts (extension E3) | Max yaw kick | rad | 0 – 1.2 | 0.01 | Ext. E3 | Largest heading change one impact may cause. |
| Numerics | Physics time step dt | s | 0.001 – 0.02 | 0.0005 | model.m | model.m dt = 0.005 s. Semi-implicit Euler as in model.m; see the convergence study in docs/validation_report.md. |

## 11. Effects not in model.m (unsupported unless an extension says otherwise)
Lateral dynamics, yaw inertia, sideslip build-up, tyre force model (slip ratio/angle, cornering stiffness, combined slip),
wheel spin dynamics, suspension and roll/pitch dynamics, load transfer beyond the F_x,max formula, steering system,
reverse gear, clutch and shift time, launch from rest, engine inertia/idle control, engine braking, coasting,
partial throttle, ABS/brake balance, brake thermal model, tyre temperature/wear, fuel consumption (constants declared
but unused), aerodynamic lift and wind, banking, kerbs and off-road surfaces, collisions.

## 12. Extensions and seed-game rules (all [EXT])

| Id | Rule | Source |
|---|---|---|
| E1 | Kinematic single-track: $\beta=\arctan(\tfrac{l_r}{L}\tan\delta)$, $r=V\cos\beta\tan\delta/L$, $F_y=MVr$, grip radius $F_{grip}=$ gripUtilization·$F_{x,max}(\mu_{surf})$; if $F_y>F_{grip}$: $r=F_{grip}/(MV)$, SlipIndicator = demand/available; longitudinal limited to $\sqrt{F_{grip}^2-F_y^2}$; $\dot\psi=r$, $\dot X=V\cos(\psi+\beta)$, $\dot Y=V\sin(\psi+\beta)$, $a_y=Vr$. Cannot spin (no yaw inertia). | DESIGN_CONTRACT §2b; friction circle from model.m §10 |
| E2 | Surface factors on μ and $f_r$ (asphalt 1.0/1, curb 0.95, grass 0.6/4) | DESIGN_CONTRACT §2b, provisional gameplay values |
| E3 | Barrier impulse: $u_n'=-e\,u_n$, $u_t'=(1-k_t|u_n|/\max(V,\varepsilon))u_t$, heading re-aligned ≤ maxYawKick, push-out, severity $|u_n|$, contact latching; while latched, a sustained-contact constraint removes the approaching normal velocity ($V'=\sqrt{V^2-u_n^2}$, no bounce, no heading change) so pressing into a wall cannot build speed | DESIGN_CONTRACT §2b |
| S1 | Clutch-slip launch: gear 1, wheel rpm < 3500, throttle > 0.1 → rpm = max(wheel, 1000 + throttle·2500) | monaco.html l. 209–215 |
| S2 | Torque clamp rpm < 1000 → T(1000) | monaco.html `getTorque` |
| S3 | Fuel cut rpm ≥ 6500 → T = 0; limiter flag for HUD/audio | monaco.html `getTorque` + l. 218–223 (rpm remap dropped) |
| S4 | Coast engine brake 30 Nm·i_g·i_fd/R_w | monaco.html l. 244 |
| S5 | Branch thresholds: brake > 0.05 → brake; throttle > 0.01 → drive; else coast | monaco.html l. 242–244 |
| S6 | Money shift: wheel rpm > 7200 → engine blown, T = 0, hard friction deceleration | monaco.html l. 183–207 |
| S7 | Interactive braking $F=0.9\mu Mg\cos\theta\cdot$Brake with $M_b=1.05M$ | model.m §11 force, applied in time (contract) |

## 13. Coordinate transformations
Model frame: X east, Y north, z up, ψ CCW from +X, δ > 0 left, r > 0 CCW (ISO 8855 signs). Godot: Y up, −Z forward.
$$p_{godot}=(X,\,0,\,-Y),\qquad \text{Rotation.Y}=\psi-\tfrac{\pi}{2}$$
Check (Python): Godot's basis rotates $(0,0,-1)$ by angle $\vartheta$ about +Y to $(-\sin\vartheta,0,-\cos\vartheta)$; with
$\vartheta=\psi-\pi/2$ this is $(\cos\psi,0,-\sin\psi)$, the image of the model heading $(\cos\psi,\sin\psi)$ under the
position map, for ψ = 0, 0.7, −2.1, π/2. Yaw rate $r$ (CCW seen from above) maps to angular velocity $+r$ about
Godot +Y (also CCW seen from above). The Mini mesh needs its own
fixed import rotation (STL: X width, Y height, Z length), applied once in the asset, not in the physics adapter.

## 14. Discrepancies found between DESIGN_CONTRACT.md and model.m / the seed (to resolve)

1. **Torque at 6500 rpm.** Contract §2b: "rpm ≥ 6500 → torque 0 (… matches the edge of the MATLAB axis)". In model.m
   $T_e(6500)=118$ Nm and the auto-shift rule keeps the gear up to and including the 6500-rpm speed, so
   MATLAB accelerates with 118 Nm at the shift point; the interactive layer will have a torque step 118 → 0 there
   (differs from R1 at every shift unless the auto-shift upshifts strictly before 6500 rpm). In the seed, the 6500 node
   is 0 Nm and the curve ramps linearly 125 → 0 over 6400–6500, not a step.
2. **Rev limiter in the seed.** The contract calls the fuel cut "the seed game's rule"; the seed actually remaps rpm > 6500
   to 6200 *before* `getTorque`, so an over-revved engine (6500–7200 rpm after a downshift) produces T(6200) = 133.5 Nm·throttle.
   The contract's choice (T = 0 above 6500) is a deliberate change from the seed and should be labelled as such.
3. **E1 friction-circle coupling vs MATLAB braking.** Contract E1 limits the longitudinal force magnitude to
   $\sqrt{F_{grip}^2-F_y^2}$ with $F_{grip}=$ gripUtilization·$F_{x,max}$ (6302 N at baseline). Applied to braking, that caps
   straight-line braking at 6302 N, while model.m §11 (and contract S7) brake with $0.9\mu Mg$ = 11283 N. The coupling must
   apply only to traction, or braking needs its own radius, otherwise the interactive braking cannot match MATLAB R1.
4. **Below 1000 rpm.** Contract clamps T(rpm < 1000) = T(1000); model.m extrapolates `a_x_real` linearly in speed. Not
   exercised by the MATLAB scenarios (start exactly at 1000 rpm) but it is a model difference, not just a port detail.
5. **Colon operator wording.** Contract §2a gives only the element count; element values are symmetric (§9.3).
   `MatlabColon.Range` now uses `a + (c-a)/2` for the middle element (fixed by the validation agent; before it used `(a+c)/2`,
   which differed from MATLAB on 32 of 1200 random ranges in `tests/.../Data/ml_colon_random.txt`, now 0).
6. **`interp1` linear formula.** `MatlabInterp.LinearExtrap` uses $(1-s)y_0+sy_1$, the form MATLAB R2026a matches bit for
   bit (§9.2); an earlier draft of this note described a different form, which is not what the code does.
7. Contract §0 baseline numbers (F_x,max 6302.146956 N, v_curva 60.6 km/h, lap 63.928121 s, R1 117.62 km/h, 343.9 m /
   16.89 s, 56.1 m / 2.20 s, R2 122.66 km/h) are confirmed. Contract interpretation `car.c` = CG→rear (l_r = c) agrees with
   model.m's comment and with the F_x,max numerator (front static load = W c/L).
8. Contract "Oval … CCW" is a game choice; model.m has no direction or planar geometry.
