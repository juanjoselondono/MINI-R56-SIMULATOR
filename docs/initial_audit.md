# Initial audit (Phase 0)

Date: 2026-10-19. Scope: what exists before the Godot/C# port, what is reused, and how the two existing
"physics" sources (MATLAB `model.m`, JS `monaco.html`) differ. Every number quoted here was produced by
running MATLAB R2026a on the unmodified file (probe scripts kept outside the repo) or, where stated, by a
Python re-implementation of the JS code.

## 1. Repository structure

| Location | Content | Status |
|---|---|---|
| `../model.m` | Authoritative MATLAB physics (sections 1–12) | original, untouched |
| `../model.asv` | MATLAB editor autosave of an older `model.m` (Sep 14) | original, untouched |
| `../monaco.html` | Seed browser game "Super Monaco GP: R56 – Hardcore Edition" | original, untouched |
| `../torque.png`, `../potsubeono.png` | Reference images (torque curve / screenshot) | original |
| `../mini_ws/` | ROS 2 / Gazebo Harmonic workspace, package `mini_r56_gazebo` (`urdf/mini_r56.urdf.xacro`, `meshes/Mini_body.stl`, `worlds/custom_sonoma.sdf`, `launch/spawn_mini.launch.py`, `scripts/custom_teleop.py`, `config/joystick_params.yaml`) | original, not used at runtime |
| `game/reference/` | Read-only snapshot: `matlab/model.m`, `matlab/model.asv`, `js_seed/monaco.html`, `assets/{Mini_body.stl, mini_r56.urdf.xacro, torque.png, potsubeono.png}` + `SHA256SUMS` | `sha256sum -c SHA256SUMS` → all 7 OK; `cmp` against `../` originals → identical |
| `game/docs/DESIGN_CONTRACT.md` | Binding decisions (layout, layers, CSV schema, scenarios) | |
| `game/src`, `tests`, `tools`, `validation`, `VehicleDynamicsSimulator` | Being created in parallel by other agents | not audited here |

Asset facts (measured, not assumed):
- `Mini_body.stl`: binary STL, 11 430 triangles, bounding box extent 1.929 × 1.435 × 3.900 (STL units, metres
  per the URDF `scale="1 1 1"`); axes: X ≈ width, Y ≈ height, Z ≈ length. The URDF visual applies
  `rpy="1.5708 0 1.5708"` and `xyz="0 0 -0.3"` to bring it into the ROS body frame.
- URDF properties: wheelbase 2.467, front/rear track 1.459/1.467, wheel radius **0.3043** (MATLAB computes
  0.30425), wheel width 0.175, wheel mass 15 kg, chassis inertial mass **1150 kg** (+4 × 15 kg wheels +
  2 × 5 kg steer links) with hand-tuned inertia ("Drastically reduced moment of inertia…"), steering limit
  ±0.88 rad, Gazebo `AckermannSteering` plugin. The URDF masses/inertias do **not** match `model.m`
  (`car.M = 1420`) and are tuned for Gazebo behaviour, so they are not physics sources; only the mesh is reused.

## 2. Toolchain found

| Tool | Version / location | Check |
|---|---|---|
| .NET SDK | 8.0.426 at `~/.dotnet` | `dotnet --version` |
| Godot .NET | 4.7.2-stable mono, `~/Applications/godot/Godot_v4.7.2-stable_mono_linux_x86_64/` | directory present |
| MATLAB | R2026a, `~/MATLAB/bin/matlab` | ran `model.m` headless with `matlab -batch … < /dev/null > log 2>&1` |
| Python 3 | numpy 1.26.4, matplotlib 3.6.3 | imported |
| Node.js | **not installed** | the JS seed was checked by a Python re-implementation of `updatePhysics` |

## 3. Inventory of `model.m`

| § | Lines | Content | Used by dynamics? |
|---|---|---|---|
| 1 | 6–27 | g = 9.81, ρ = 1.225, slope (`slope_mode` deg/pct/rad → `env.theta`), `fuel.LHV`, `fuel.AFR` | fuel.* unused |
| 2 | 29–65 | Mass, Cd, A, gear ratios, final drive, η_t, mass factors per gear, `V_d`, `CR` (unused), tyre 175/65 R15 → R_w, F_x,max inputs (μ, L, c, h, t_f, t_r, K_s_f/r, s_f/r), roll stiffness K_φ | `t_r`, `V_d`, `CR` unused |
| 3 | 67–80 | Engine map: 14 torque nodes, `interp1(...,'pchip')` on `rpm_axis = 1000:50:6500` (111 pts), power, HP | yes |
| 4 | 82–101 | Resistance curves on `v_kmh = 0.1:1:220` (220 pts): drag, rolling 0.01(1+v_mph/100), grade, power demand | plots + envelope only |
| 5 | 103–135 | Per gear: V(rpm), wheel power, wheel torque, F_tractive, ideal a_x (no grip limit) | yes |
| 6 | 137–147 | F_x,max (FWD traction limit with load transfer) | yes |
| 7 | 149–172 | F_tractive_effective = min(F_tractive, F_x,max), a_x_real | yes (drives §11) |
| 8 | 174–238 | Figure 1 (5 plots) | no |
| 9 | 240–313 | Figure 2 (3 plots) + `a_x_envelope` (P_max·η − P_dem)/v / M_eq,avg | envelope plot only |
| 10 | 315–392 | Oval curve speed: friction-circle search at 80 % of F_x,max, gear check (gears 1–5), v_curva_final | yes |
| 11 | 394–558 | Oval lap: R1 accel from 1000 rpm in 1st (Euler, dt 0.005), braking distance loop, C1 constant speed, R2, C2, lap time | yes |
| 12 | 560–591 | Console report | no |

Detailed equations: `docs/vehicle_model.md`.

## 4. Inventory of `monaco.html` and reuse decision

| Feature (JS) | What it does | Decision | Reason |
|---|---|---|---|
| Physics `updatePhysics` (5 ms fixed step, accumulator, 0.25 s frame clamp) | Longitudinal + pseudo-lateral | **Reimplement** in `VehicleDynamics.Core.Simulation` (C#) | Must follow `model.m` equations; JS deviates (§6) |
| Fixed-step accumulator | `while (acc >= 0.005) step` | **Reuse conceptually** (`SimulationClock`) | Sound pattern; add interpolation alpha |
| Controls W/↑ throttle, S/↓ brake, A/D/←/→ steer, E/Q shift, R reset, start button | Per-frame ramps (+0.2/frame throttle, −0.3 brake release, ±0.15 steer, ×0.7 return) | **Reimplement** in Godot InputMap | JS ramps are per *frame* → frame-rate dependent; contract §5 requires per-second ramps |
| Shift flags | Latched booleans | **Reimplement** as one-step edge events | JS bug: a flag that cannot be served (E in 6th, Q in 1st, or while engine blown) stays latched and fires on a later step |
| Engine torque `getTorque` | Linear interpolation, 6500 rpm node = 0, rpm ≤ 1000 → 75 Nm, rpm ≥ 6500 → 0 | **Drop** the linear map; keep the domain rules | MATLAB is PCHIP with T(6500) = 118 Nm |
| Clutch-slip launch rule | 1st gear, wheel rpm < 3500, throttle > 0.1 → rpm = max(wheel, 1000 + throttle·2500) | **Reuse** (labelled seed rule) | MATLAB cannot start below 1000 rpm |
| Rev limiter | rpm > 6500 → displayed rpm = 6200 + red HUD | **Reuse conceptually** (flag for HUD/audio); **drop** the rpm remap | Remap makes the engine produce T(6200) while over-revved (§6.14) |
| Money-shift engine blow-up | wheel rpm > 7200 → engine dead, hard deceleration, explosion sound, overlay text | **Reuse** (toggleable, labelled seed rule) | Gameplay; not in MATLAB |
| Engine brake on coast | 30 Nm × i_g·i_fd / R_w | **Reuse** (labelled seed rule) | MATLAB has no coasting case |
| Lateral `lat` pseudo-physics | Normalised lateral offset driven by arbitrary gains | **Drop** | No physical meaning; replaced by extension E1 |
| Off-road | \|lat\| > 1 → v ×= 0.992 per step | **Drop** | Replaced by E2 surfaces + E3 barriers |
| Track `getSmoothedCurve` | Curvature vs. arc length only, 50 m linear ramps, no 2-D geometry | **Drop** | Replaced by exact `Core.Track` geometry |
| Pseudo-3D renderer | 10 m segments, 120-segment draw distance, camH 1200, projW = 3500/z, rumble/centre stripes, parallax mountains | **Drop** | Godot renders real 3-D from track geometry |
| Car sprite + body-roll wobble | Canvas drawing | **Drop** | `Mini_body.stl` mesh |
| HUD | RPM, gear, km/h, "LAP TIME" (actually total time since reset), RPM bar scaled 1000→7200 | **Reimplement** in Godot UI | Real lap timing needed |
| Audio synth | Sawtooth, low-pass 800 Hz, f = 30 + 0.035·rpm Hz, gain 0.05 + 0.15·throttle | **Reuse conceptually**, reimplement in Godot | Simple, effective; Godot `AudioStreamGenerator` |
| Explosion sound | 2 s white noise, exp decay τ = 0.3 s, low-pass 800→10 Hz exponential sweep | **Reuse conceptually**, reimplement | |
| Blown-engine overlay text | Flashing message, "press R" | **Reimplement** (wording optional) | |
| Telemetry canvas | 200-sample ring buffer every 0.1 s of *wall* time; plots v·1.5 and rpm/60; `a_x` stored but not plotted | **Reimplement** (Godot plots + CSV recorder at sim-time interval, contract §5–6) | JS sampling is render-rate based |
| Sliders | M 1000–2000 step 10, Cd 0.2–0.6 step 0.01, μ 0.5–1.5 step 0.05 | **Reimplement**, extended set (vehicle_model.md §10) | |

## 5. `model.asv` vs `model.m` (`diff model.asv model.m`)

`model.asv` is an older autosave (no line-ending differences; `.asv` lacks a final newline). Differences:
1. **Slope**: `slope_value = 10` (10 %, θ = atan 0.1) in `.asv`; `0` in `.m`. (Scenario S02 uses the `.asv` value.)
2. `.asv` has **no F_x,max parameters** (μ, L, c, h, t_f, t_r, K_s_f/r, s_f/r) and no roll stiffness K_φ.
3. §5: `.asv` computes a local `F_tractive` per gear; `.m` stores the matrix `F_tractive_N(g,:)` (values identical).
4. `.asv` has **no §6 F_x,max, no §7 effective traction/a_x_real**; its acceleration is the ungripped `a_x_ms2` only.
5. Plot sections renumbered (`.asv` §6/§7 → `.m` §8/§9). `.asv` second figure is a single axes (gear curves dashed,
   envelope solid magenta width 3, `ylim([-0.5 max(a_x_ms2(1,:))*1.1])`); `.m` second figure has three subplots
   (ideal a_x, F_tractive vs F_x,max, a_x_real) and envelope dashed width 2.
6. `.asv` has **no §10 curve speed, no §11 oval lap, no §12 report**.
7. Cosmetic: comments ("evitamos 0 exacto para división por velocidad", "(Restaurado)", "Línea de referencia a_x = 0"),
   figure names, blank lines.
The envelope formula (P_max·η − P_dem)/v / M(1 + mean(mass_factors)) is identical in both.

## 6. JS seed physics vs `model.m` (exhaustive)

Notation: W = M·g. "per step" = per 5 ms physics step.

1. **Torque interpolation**: JS piecewise-linear between nodes; MATLAB PCHIP (shape-preserving cubic). Example at
   6450 rpm: JS 62.5 Nm, MATLAB 121.749 Nm.
2. **6500 rpm node**: JS torque node is **0 Nm** (MATLAB: 118 Nm) and `rpm >= 6500 → 0`. Between 6400 and 6500 JS
   ramps 125 → 0 linearly. Consequence (Python emulation, full throttle, level road, gear held, 60 s): the car
   settles just below 6500 rpm in every gear 1–3 (e.g. 53.25 km/h in 1st vs. MATLAB 1st-gear top 53.29 km/h)
   and never reaches the limiter or 7200 rpm.
3. **Below 1000 rpm**: JS uses T = 75 Nm (clamp, plus clutch rule); MATLAB never evaluates torque below 1000 rpm
   — it linearly *extrapolates* `a_x_real` in speed (`interp1 … 'extrap'`).
4. **Evaluation path**: JS evaluates T(rpm) every step; MATLAB pre-tabulates a_x_real on the 50-rpm grid and
   linearly interpolates it in km/h.
5. **Throttle**: JS multiplies torque by throttle ∈ [0,1]; MATLAB is always full throttle.
6. **R_w**: JS 0.3043 m (rounded); MATLAB 0.30425 m from 175/65 R15.
7. **K_φ**: JS hard-codes K_φf = 15703, K_φr = 21118; MATLAB computes 15700.734 and 21118.971 N·m/rad.
   F_x,max: JS 6302.126 N vs MATLAB 6302.147 N (baseline).
8. **Slope**: JS θ = 0 fixed, no grade force, W = M·g (no cos θ); MATLAB supports θ in rolling, grade, F_x,max, brakes.
9. **Braking**: JS a = −(0.9·μ·W·brake + F_drag + F_roll) / **M(1 + mass_factor[gear])**, integrated in time;
   MATLAB uses **M·1.05** and full braking only, evaluated by a velocity-domain sum (no time integration), and adds grade.
10. **Coasting**: JS engine-brake 30 Nm·i_g·i_fd/R_w (no η); MATLAB has no coast state.
11. **Branch logic**: JS brake > 0.05 → brake branch, else throttle > 0.01 → drive, else coast. MATLAB: drive on
    straights, then an analytic braking phase.
12. **Integration**: JS `v += a dt; x += v dt` (symplectic Euler); MATLAB `d += v_new dt + 0.5 a dt²`
    (see vehicle_model.md §5.1).
13. **Saturation**: JS clamps v ≥ 0 (and sets a = 0); MATLAB clamps a ≥ 0 under drive (`max(0,a)`): with throttle
    the MATLAB car can never slow down.
14. **Rev limiter**: JS sets rpm = 6200 when rpm > 6500 and then computes torque at **6200 rpm = 133.5 Nm·throttle**
    (linear map). So an over-revved engine (e.g. after a downshift into 6500–7200 rpm) still pulls. MATLAB has no
    limiter; its gear rule upshifts exactly at the 6500 rpm table edge.
15. **Money shift**: JS wheel rpm > 7200 → engine blown: a = −(μ·W + F_drag)/M (full μ, no 0.9, no rolling, no mass
    factor, no grade), rpm shown 0; not in MATLAB.
16. **Gears**: JS manual E/Q at any speed; MATLAB automatic: lowest gear whose 6500-rpm speed ≥ v (falls back to
    gear 1 above 6th-gear top speed). MATLAB R1 uses gears 1–3 only.
17. **Start**: JS from v = 0 with clutch-slip rule; MATLAB from 1000 rpm in 1st = 8.198 km/h.
18. **Lateral** (JS only; arbitrary, unitless gains): δ = steer·0.06 rad; R = L/tan δ (≥ 41.07 m); F_y = M v²/R;
    grip_margin = **0.85**·F_x,max (MATLAB uses 0.80 for planning); resultant uses **max(F_tract_eff, F_brake)**
    (not the applied force; F_tract_eff is computed even while braking); `lat_speed = steer·v·0.018`;
    if resultant > margin and v > 10 m/s: `lat_speed −= sign(steer)(F − margin)·0.0004` and `v ×= 0.9995` per step
    (≈ ×0.905 per second); curve drift `lat_speed −= curvature·v·0.022`; `lat` clamped to ±1, off-road `v ×= 0.992`
    per step (≈ ×0.20 per second). None of this has a MATLAB counterpart.
19. **Track**: same nominal oval (2 × 400 m, R 80) but represented only as curvature(s) with 50 m linear ramps
    placed in the last 50 m of each straight and of each arc; no planar geometry, the car never actually turns.
20. **Time**: `state.time` is total time since reset (HUD labels it LAP TIME). Inputs and telemetry are
    frame-rate dependent.
21. **Parameter coupling**: sliders change M, Cd, μ live; F_x,max is recomputed every step (MATLAB once per run),
    K_φ, h, L, etc. are fixed.
22. MATLAB-only features absent from JS: curve-speed planning (§10), braking-point computation, lap-time
    estimate, power/envelope analysis, slope modes.
