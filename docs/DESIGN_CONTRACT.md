# Design Contract (binding for all implementation work)

This file fixes the decisions every component must agree on. If an implementation needs to deviate,
update this file in the same change and say why.

## 0. Environment (verified 2026-10-19)

| Tool | Location | Notes |
|---|---|---|
| .NET SDK 8.0.426 | `~/.dotnet/dotnet` | `export PATH=$HOME/.dotnet:$PATH DOTNET_ROOT=$HOME/.dotnet` |
| Godot 4.7.2-stable .NET | `~/Applications/godot/Godot_v4.7.2-stable_mono_linux_x86_64/Godot_v4.7.2-stable_mono_linux.x86_64` | headless works: `--headless --import --path <proj>` then `--headless --path <proj>` |
| MATLAB R2026a | `matlab` on PATH | **Always** run as `matlab -batch "..." < /dev/null > log.txt 2>&1` — otherwise a helper process keeps the pipe open and the shell hangs. ~12 s per run. |
| Python 3 | numpy 1.26, matplotlib 3.6 | allowed for asset conversion / plotting helpers |

Godot csproj template that is known to build: `<Project Sdk="Godot.NET.Sdk/4.7.2">`, `TargetFramework net8.0`,
`EnableDynamicLoading true`. `project.godot` needs `config/features=PackedStringArray("4.7", "C#")` and
`[dotnet] project/assembly_name=...`.

Baseline MATLAB output of the unmodified `reference/matlab/model.m`:
`car.F_x_max = 6302.146956 N`, `v_curva_final_kmh = 60.600000`, `t_vuelta_completa = 63.928121 s`,
R1: v_max 117.62 km/h, accel 343.9 m / 16.89 s, braking 56.1 m / 2.20 s; R2: v_max 122.66 km/h.

## 1. Repository layout (repo root = `game/`)

```
reference/                     READ-ONLY snapshot of the originals (+ SHA256SUMS). Never edit.
  matlab/model.m               authoritative physics reference
  js_seed/monaco.html          seed browser game
  assets/Mini_body.stl, mini_r56.urdf.xacro, torque.png, potsubeono.png
src/VehicleDynamics.Core/      net8.0 class library, namespace VehicleDynamics.Core[.*]; NO Godot dependency
tests/VehicleDynamics.Tests/   xUnit tests for the core (run: dotnet test VehicleDynamics.sln)
tools/ValidationRunner/        net8.0 console app: runs the C# side of every validation scenario -> CSV
validation/
  scenarios.json               single source of scenario definitions for MATLAB and C#
  matlab/                      run_reference.m, compare_results.m, helpers
  golden/                      MATLAB-generated CSV/JSON (committed; C# tests read these, no MATLAB needed)
  out/                         generated, git-ignored
  plots/                       comparison plots (committed PNG)
VehicleDynamicsSimulator/      Godot 4.7 C# project (ProjectReference ../src/VehicleDynamics.Core)
docs/                          initial_audit.md, vehicle_model.md, architecture.md, validation_report.md, run_instructions.md
VehicleDynamics.sln            contains Core, Tests, ValidationRunner (NOT the Godot csproj; build that separately)
```

## 2. Physics: two layers in the core

### 2a. Reference layer — `VehicleDynamics.Core.Reference` (bit-faithful port of model.m)
Purpose: reproduce `model.m` sections 1–11 exactly (target |err| < 1e-9 relative on every exported quantity).
- `ReferenceParameters` record with every model.m input (env.g, rho_air, slope_mode/value, car.*, R_curva,
  margin_safety, L_recta, dt=0.005, braking 0.90/0.05, v grids...). Defaults = model.m values.
- `Pchip` — exact port of MATLAB `interp1(...,'pchip')` (Fritsch–Butland/MATLAB `pchip` slopes incl. the
  MATLAB 3-point shape-preserving end conditions). Must reproduce `car.T_engine` to ~1e-12.
- `ReferenceModel.Build(params)` → tables: rpm_axis, omega, T_engine, P_engine_W, V_gear_kmh[g,:],
  T_wheel, F_tractive_N, a_x_ms2, F_x_max, F_tractive_effective, a_x_real, v_kmh grid resistances, a_x_envelope.
- `ReferenceOval.Run(params, tables)` → section 10 (curve speed search, gear choice, v_curva_final) and
  section 11 (R1, C1, R2, C2 incl. histories, braking loop, lap time). Reproduce quirks literally:
  gear selection loop, `unique(...,'stable')`, `interp1 'linear','extrap'`, `max(0,a)`, the distance update
  `d = d + v_new*dt + 0.5*a*dt^2`, `while d < L && t < 60`, the braking loop over `v_end:-0.05:v_target`
  (MATLAB colon semantics: n = floor((a-b)/0.05 + eps-tolerance)+1 — replicate MATLAB colon exactly),
  `find(history_d >= d_acc, 1)`, etc.
- Port notes for every MATLAB idiom go in docs/vehicle_model.md §"MATLAB→C# porting notes".

### 2b. Interactive layer — `VehicleDynamics.Core.Simulation` (what the game drives)
Same equations, evaluated continuously with driver inputs. Default parameters identical to model.m.

State `VehicleState` (SI): `X, Y` [m] planar world position of the CG (model frame), `Psi` [rad] heading,
`V` [m/s] speed along the path (>= 0, MATLAB has no reverse), `Gear` (1..6), `EngineRpm` (algebraic, stored for
telemetry), `Distance` [m] odometer, `Time` [s], `EngineBlown` (bool, seed-game rule), plus derived
telemetry fields (ax, ay, yaw rate r, beta, Fx terms, grip usage...). Keep derived values in a separate
`VehicleOutputs` struct computed by the same step.

Inputs `VehicleInputs`: `Throttle` [0..1], `Brake` [0..1], `SteerAngle` δ [rad, + = left], `ShiftUp/ShiftDown` edge
requests (consumed once per physics step), `AutoShift` flag.

Longitudinal (MATLAB §3–§7, §11):
- rpm = V/R_w · i_g·i_fd · 60/2π. Engine torque T(rpm) = PCHIP(model.m nodes), evaluated directly
  (not via the 50-rpm table), clamped to the table domain: rpm < 1000 → use 1000 (idle);
  rpm >= 6500 → torque 0 (fuel cut; this is the seed game's rule and matches the edge of the MATLAB axis).
- Launch below 1000 rpm in 1st gear: seed-game clutch slip rule (monaco.html): engine rpm =
  max(wheel_rpm, 1000 + throttle·(3500−1000)) while gear==1 and wheel_rpm < 3500 and throttle > 0.1.
  Documented as "taken from the JS seed, not in MATLAB".
- F_trac = T·throttle·i_g·i_fd·η_t / R_w ; F_trac_eff = min(F_trac, F_x,max·gripScale) (MATLAB §6–7 formula for F_x,max
  evaluated with current parameters and surface μ).
- Resistances exactly as MATLAB: drag 0.5ρACdV², rolling 0.01(1+V_mph/100)·M g cosθ, grade M g sinθ.
- Braking (MATLAB §11): F_brake = 0.90·μ·M·g·cosθ·Brake, with equivalent mass M·1.05.
- Coasting (no throttle, no brake): seed-game engine brake F = 30 Nm · i_g·i_fd / R_w (label: from JS seed).
- Mass: M_eq = M(1 + mass_factor[g]) when driving/coasting; M·1.05 when braking (MATLAB). Which branch applies
  follows the seed game: brake > 0.05 → braking branch; else throttle > 0.01 → drive branch; else coast.
- V clamps at 0 (no reverse). Resistances must not drive V negative.
- Integration: MATLAB's semi-implicit Euler at dt = 0.005 s: `V += a·dt; s += V·dt + 0.5·a·dt²`.
  Same scheme kept by default; RK4 available only as an alternative for the convergence study.
- Money shift (seed game): wheel-driven rpm > 7200 (e.g. downshift at speed) → EngineBlown, engine
  torque 0, car decelerates with brake-like friction. Toggleable ("Mechanical damage" option, default on).
- Rev limiter flag when rpm > 6500 (telemetry/audio only; torque already 0).
- Auto-shift (optional, default OFF for driving, ON for validation): MATLAB rule = lowest gear whose
  max speed (at 6500 rpm) >= V.

Lateral — EXTENSION E1 (not in MATLAB; must be labelled as an extension everywhere):
MATLAB has no lateral/yaw dynamics; it only has the quasi-static friction-circle cornering check of §10:
sqrt(F_x_req² + F_y²) <= margin·F_x,max with F_y = M V²/R. Extension E1 = kinematic single-track
(bicycle) model + that same friction circle:
- model.m `car.c` = CG→REAR axle distance (1.529 m), so l_r = c and l_f = L − c.
  Kinematic sideslip at the CG: β = atan( (l_r/L)·tan δ ).
- Desired yaw rate r_kin = V·cosβ·tanδ / L ; lateral demand F_y = M·V·r (≈ M V²/R).
- Grip radius F_grip = gripUtilization · F_x,max(μ_surface). (Default gripUtilization = 1.0 for driving;
  MATLAB's planning margin 0.80 is reproduced in the reference layer and in validation.)
- If F_y > F_grip: saturate r = F_grip/(M·V) (understeer, path radius grows) and report
  `SlipIndicator = demand/available` (> 1 ⇒ sliding; drives tire-squeal audio).
- Friction circle couples axes: longitudinal force magnitude limited to sqrt(F_grip² − F_y²).
- Kinematics: ψ̇ = r ; Ẋ = V cos(ψ+β) ; Ẏ = V sin(ψ+β) ; a_y = V·r. Telemetry v_y = V sinβ (kinematic).
- Steering input → δ: δ = steer·δ_max with δ_max a PROVISIONAL parameter (default 0.30 rad); the seed game
  used 0.06 rad which cannot negotiate radii below ~41 m. Speed-sensitive steering reduction is a
  gameplay input assist that lives in the Godot input layer, never in the core.
- No yaw inertia / cornering stiffness / tire model exists in MATLAB → the car cannot spin. State this.

Surfaces (EXTENSION E2, provisional, exposed in UI): `SurfaceGripFactor` multiplies μ (asphalt 1.0, curb 0.95,
grass 0.6) and `SurfaceRollingFactor` multiplies f_r (asphalt 1, grass 4). Values are provisional gameplay
parameters, not from MATLAB.

Collision response (EXTENSION E3, `VehicleDynamics.Core.Collision`, pure function, documented):
inputs: contact normal n (unit, planar, pointing from barrier into track), penetration depth, restitution e
(default 0.25), tangential friction loss k_t (default 0.15). Velocity vector u = V·(cos(ψ+β), sin(ψ+β));
u_n = u·n; only respond when u_n < 0 (approaching). u_n' = −e·u_n ; u_t' = (1 − k_t·|u_n|/max(V,ε))·u_t clamped ≥ 0;
new V = |u'| (but heading re-aligned toward u' by at most `maxYawKick` rad, default 0.6) ; position pushed out by
penetration along n. Severity = |u_n| (m/s) is returned for effects/telemetry. Cooldown/contact latching:
an impulse is applied only on contact entry or when u_n < 0 again after separation; never every frame.

Mass/inertia params for UI: only parameters that exist in model.m (+ the clearly-labelled extension params).

## 3. Track geometry — `VehicleDynamics.Core.Track` (pure C#, used by BOTH rendering and physics queries)
- Centerline = ordered list of segments: `Straight(length)`, `Arc(radius, angleRad, +left/−right)`, optional
  `Clothoid(length, k0, k1)`. Built from a start pose. Exact closed-form sampling: Position(s), Heading(s),
  Curvature(s). Closure must be verified by a test (end pose == start pose within 1e-6 m / 1e-9 rad).
- `Project(x, y)` → (s, lateralOffset, segment index) — nearest point; robust for a car anywhere within 60 m.
- Cross-section: road half-width 6 m, curbs 1 m beyond road edge, runoff (grass) to barrier at half-width + runoff
  (runoff default 10 m on oval, 6–15 m on circuit). Barrier = continuous wall along both sides.
- Tracks: (1) "Oval" = exact model.m geometry (2× 400 m straight + 2× R80 semicircles, CCW).
  (2) "Circuit" = closed track with long straights, a tight hairpin (R≈25–30 m), wide sweepers (R≥120 m),
  a left-right chicane, ≥ 2.5 km, closes exactly.
- Godot builds all road/curb/grass/barrier meshes by sampling this geometry so rendering == physics.

## 4. Coordinates
Model frame: planar X (east), Y (north), ψ CCW from +X, δ>0 = left turn, r>0 = CCW (ISO 8855 sign convention, z up).
Godot (Y-up, −Z forward): `godot = (X, 0, −Y)`; node `Rotation.Y = ψ − π/2` (yields forward −Z basis ↦ (cosψ,0,−sinψ)).
Put the conversion in ONE place (`VehicleModelAdapter`) with a unit-testable pure-math twin in core
(`FrameConversion`) and tests for straight, ±steer, ±yaw.

## 5. Simulation clock & threading
- `SimulationClock` (core, pure): accumulator, fixed dt (default 0.005 s, configurable), max substeps per frame (e.g.
  0.25 s clamp), returns step count and interpolation alpha. Inputs are sampled once per render frame as *levels*
  and held for all substeps; keyboard ramping (throttle/brake/steer rate limits) is applied per simulated second,
  not per frame. Shift requests are edge events consumed by exactly one physics step.
- Godot `SimulationManager` drives the clock from `_Process`, keeps previous+current state, renders with
  interpolation alpha. Single-threaded. Pause stops the clock; parameter changes are applied between steps.
- Recording at fixed simulation interval (default every 0.02 s sim time = every 4 steps), never at render rate.

## 6. Telemetry CSV schema (C# recorder AND validation outputs use snake_case headers, SI units in header)
`t_s, x_m, y_m, psi_rad, v_mps, v_kmh, vx_mps, vy_mps, yaw_rate_rps, beta_rad, ax_mps2, ay_mps2, gear, engine_rpm,
throttle, brake, steer_rad, f_tractive_n, f_tractive_eff_n, f_x_max_n, f_drag_n, f_rolling_n, f_grade_n, f_brake_n,
grip_usage, slip_indicator, track_s_m, track_lateral_m, surface, lap, lap_time_s, collision_severity_mps, event`
Each CSV starts with `# key=value` comment lines holding the full parameter set + scenario/run id + git sha if available.

## 7. Validation scenarios (`validation/scenarios.json`)
Each scenario = id + description + parameter overrides (model.m variable names). MATLAB side: `run_reference.m`
reads model.m TEXT, strips `clear; clc; close all;`, applies overrides by regex on the exact assignment lines
(error if an override matches ≠ 1 line), forces figures invisible, runs it in a temp file under validation/out/,
then exports workspace variables to `validation/golden/<id>/*.csv` (+ `summary.json`). model.m itself is never modified.
Minimum scenario set: S01 baseline; S02 slope 10 % (model.asv value); S03 slope −5 %; S04 mass 1800; S05 mu 0.7;
S06 Cd 0.30; S07 long straight L_recta = 3000 (long-run error growth, t<60 cap); S08 R_curva 40; S09 mu 1.2 (traction not
limiting in 1st?) — the actual effect is reported, not assumed.
C# side: ValidationRunner produces the same files under `validation/out/csharp/<id>/`. `compare_results.m` (and an
equivalent xUnit test reading golden/) computes max abs err, RMSE, relative err, error-vs-time plots → validation/plots.
Additionally compare the INTERACTIVE layer (full throttle, auto-shift, from 1000 rpm in 1st) against MATLAB R1/S07
histories; tolerance justified in validation_report.md (it differs by design: direct PCHIP vs 50-rpm table + linear interp).
Steering/transient scenarios: NOT comparable (no lateral model in MATLAB) — say so; test E1 against analytic results instead.

## 8. Amendments (2026-10-20, made while implementing the game layer)

| § | Change | Why |
|---|---|---|
| 3 | Barrier offsets are a continuous function of s (`BarrierOffsetLeftAt/RightAt`), blended linearly over ±20 m (or half the shorter segment) around each joint. Contact, surface lookup and rendering all use it. | Per-segment offsets made the walls step where the runoff width changes. A car sliding along a wall would then be pushed 2–5 m sideways in one step. |
| 2b / E3 | While a contact is latched (impulse already applied), `SustainedContact.Resolve` removes the approaching normal velocity: V' = √(V² − u_n²), no bounce, no heading change. `VehicleDynamics.ApplyCollision` is the only external writer of the state and rejects speed gains and non-finite values. | Without it, full throttle against a wall kept adding speed that the depenetration then discarded, so V grew while the car stood still. |
| 5 | `SimulationLoop.BeforeStep`: per-step input adjustment from the current state. Used for the E2 surface lookup and for keyboard ramping plus the steering assist (`VehicleController.ShapeInputs`). | Input shaping per frame made the input history depend on the frame rate. Per step, constant inputs give bit-identical states at 10–1000 fps. |
| 3 | Spawn on the start/finish line (s = 0), which is model.m's oval start point. The lap timer uses a standing start. | Spawning behind the line put the car inside the Circuit's final corner. |
| 4 | Chase + **bonnet** camera instead of a cockpit camera. | `Mini_body.stl` is a closed shell without interior or transparent glazing. |
| 4 | (2026-10-20, later) The STL car was replaced by the 2011 Mini JCW (R56) GP OBJ, scaled to model.m's wheelbase. Its interior and glass make a **cockpit** camera possible, so the camera cycles chase → bonnet → cockpit. The collision footprint is now 3.76 × 1.68 m, centre 0.261 m behind the CG. | User-supplied model with interior and textures. |
| 2b | (2026-10-20) **Game mass 1150 kg** (user-supplied with I_zz), via `VehicleParameters.GameDefaults()`. The class defaults stay equal to model.m (1420 kg) and validation uses those. | User decision. |
| 2b | **C1** (default on): F_x,max limits the tyre force, not the driveline force; a grip-limited tyre force accelerates the bare mass. Off = model.m. Validation runs with it off. | model.m applies the rotating-inertia mass factor to a force that never passes through the rotating parts. User-approved. |
| 2b | **E4** dynamic single-track lateral model (`LateralModel.Dynamic`, default) replaces E1 as the default; E1 stays selectable. | User request: slip angles, yaw rate, agility. I_zz user-supplied, C_α estimated. |
| 2b | **E5** physical launch clutch (`LaunchModel.PhysicalClutch`, default) replaces the seed rule S1 as the default; S1 stays selectable. | User request: realistic 1st-gear pull from 0 km/h. |
| E3 | Bounce threshold 1 m/s: slower approaches are sliding contact without an impulse. | With E4 the tyres keep pushing a car into a wall, which caused about 9 Hz micro-impact chatter. |
| 5 | Gamepad: bindings live in `project.godot`'s Input Map. Analog devices bypass the keyboard ramps. | User request (Logitech F710). |
| 2b / E4 | (2026-10-20, second pass) Game μ = 1.05 (model.m 0.90). E4 gains tyre load sensitivity (k = 0.15) and a lateral-load-transfer split (front share 0.60), C_αf/C_αr retuned to 85/115 kN/rad, wheel-frame slip angles valid when rolling backwards, v_x may go negative while sliding (spins), dissipative forces follow the direction of travel, and the low-speed blend became a 0.2 s relaxation on total speed. | Driving feedback: too much oversteer, too little grip, and a spin glitch (velocity snapped to zero). The glitch was a bug in the first E4 version. |
| 3 | (2026-10-20) Converted **Monaco** track (Assetto Corsa mod) alongside the procedural tracks. It has no analytic geometry: lap timing, track position, recovery and the scripted driver use the AC racing line (`fast_lane.ai`) as an `ITrackLine` (`PolylineTrack`, s = 0 at the timing gate), and track limits come from that file's per-point left/right edge distances. | User request. The procedural tracks are unchanged. |
| 2b / E2 | On mesh tracks the surface comes from the hidden physics mesh under the CG (TARMAC / KERB / OFFTRACK). μ is multiplied by that surface's `surfaces.ini` FRICTION (`VehicleInputs.SurfaceGrip`), not by the generic E2 factor. Rolling resistance stays asphalt (the off-mesh areas at Monaco are pavement). | AC track data supplies its own friction values. |
| 2b / **E6** | Road grade per step (`VehicleInputs.RoadGrade`, rad, + uphill), added to model.m's global slope θ in every θ term (F_x,max, rolling, grade force, braking). On mesh tracks it is measured from the ground heights under the front axle and the rear wheels. It is 0 on procedural tracks, which keeps the MATLAB equivalence. | Monaco has about 42 m of elevation change; model.m's θ is the same quantity, but constant. |
| 4 / E3 | Mesh-track walls: contact comes from a Godot shape query of the car footprint box against the WALL bodies (deepest pair = penetration, rest-info normal made horizontal). The response is still the core's E3 (`CollisionManager` takes a `BarrierContactSource`). Godot physics never moves the car. | The physics mesh is the only wall description AC provides. |
| 5 | Start/finish **Area3D** on the AC timing gate, detecting the car's `TriggerVolume` (layer 5). It logs the telemetry event `sf_gate`. `LapTimer` stays authoritative: it interpolates the crossing within a 5 ms physics step, while the Area3D only sees Godot's 60 Hz physics tick. | User request for an Area3D trigger; step-exact timing kept. |
| E3 | (2026-10-20) With E4, wall impacts are a **rigid-body impulse at the contact point** (`RigidBodyImpact`): restitution e at the contact point, Coulomb wall friction μ_w = 0.4 (new, provisional), I_zz = 1100 kg·m². An off-centre hit changes the yaw rate, and the heading follows from it (no direct heading kick). The guard on `ApplyCollision` moves from "speed must not increase" to "kinetic energy ½MV² + ½I_zz r² must not increase": a spinning car may trade rotation for translation. E1 keeps the point-mass response (it has no yaw-rate state). | User request: restitution and angular momentum transfer for off-centre impacts. |
| 3 / 4 | Teleports (restart, recover, first spawn) on mesh tracks snap to the ground with a full-height vertical `RayCast3D` against the collision mesh (`MeshTrack.SnapToGround`), never a fixed height. The per-step ground search re-acquires from that probe if it ever loses the ground. | Bug: after driving uphill, restart placed the car about 40 m above the grid. The ground search window (last height +4/−12 m) could not reach the grid, so the render used y = 0. |
| 4 | **Racing-line driver aid** (`RacingLine`, toggle L / D-pad up): the track line drawn on the road and coloured by a quasi-steady reference lap (`ReferenceLap`, core) computed with the core's own longitudinal model. Green = accelerate, yellow = coast/transition, red = heavy braking (≥ 30 % brake). Rebuilt when the parameters change. | User request. A computed profile rather than a recorded lap, so it follows the car's setup. |
| 2b / **R1** | (2026-10-21) **Reverse gear** (EXTENSION R1; model.m has none): ratio 3.143, from BMW Group "Technical Data – MINI Hatch", MINI UK 03/2011, MINI Cooper N16 6-speed manual, gearbox Getrag G255 (GS6-55BG). The forward ratios and the 4.35 final drive in that sheet equal model.m's. `Gear = −1`. The driveline uses the speed along the gear's direction; rotating-mass factor = 1st gear's (assumption). Requires E4. The "stop at 0" rule and the near-zero direction factor are mirrored in reverse. | User request. |
| 2b / R1 | **Transmission damage** (game rule): engaging reverse while v_x > 10 km/h (or 1st while rolling back faster than 10 km/h) sets `TransmissionDamaged`: engine torque 0, no drive and no engine braking, shifting disabled; brakes and tyres work. HUD: "Transmission Damaged! Press R to restart". Restart (R / Back) clears it; recover keeps it. The lap is invalidated. | User request (mirrors the money-shift rule S6). |
| 4 / 5 | Bindings: X (pad and key) = reverse toggle; A (pad) added to the racing-line toggle; telemetry recording is F5 only (was also pad X). | User request. |
| 4 | Racing line **source**: by default a minimum-curvature line (`RacingLineOptimizer`) inside the track limits, with a 1.2 m margin. On mesh tracks the limits are tightened to the walls of the physics mesh (≥ 2.0 m CG-to-wall). Alternatives: the track's reference line, or the fastest valid lap of a telemetry CSV (`RecordedLapLine`). Lap timing and validity stay on the track's reference line. | User report: the drawn line followed the centreline (procedural tracks) or a cautious AI line (Monaco) and ran too wide. |
| 4 | (2026-10-21) **Graphics overhaul** (Forward+): ACES, SSAO, SSIL, SSR, low-density volumetric fog, thresholded glow; TAA (MSAA/FXAA removed), FSR 2.2 Quality/Balanced presets; clear-coat car paint resource; `CameraAttributesPractical` far DoF; motion blur as a compositor compute effect, because Godot 4.7 has no built-in motion blur and `CameraAttributesPractical` has only DoF/exposure. Presets via F6 / `--gfx=`; default Ultra (native + TAA) from measurements. Rendering only: no effect on the physics. | User request. |
