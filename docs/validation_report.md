# Validation report: C# core vs MATLAB `model.m`

Date: 2026-10-19. MATLAB R2026a, .NET 8.0.426. The physics reference is `reference/matlab/model.m`
(SHA-256 `b665b69c…0697b`, which matches the `model_sha256` stored in every `validation/golden/*/summary.json`).
Every number in this report comes from the runs listed in §9. Nothing here is estimated.

## 1. Summary

| Layer | Criterion | Result |
|---|---|---|
| Reference (`VehicleDynamics.Core.Reference`) | every table, history and summary value within \|e\| ≤ 1e-9 + 1e-9·\|golden\|, for all 9 scenarios | **PASS. Max abs error 0, max rel error 0** (bit-identical) for 312,900 values |
| Interactive (`VehicleDynamics.Core.Simulation`), MATLAB R1 conditions | \|Δv\| ≤ 0.05 km/h, \|Δd\| ≤ (0.05/3.6)·t, same step count to reach `L_recta` / 60 s cap | **PASS.** Max \|Δv\| 3.4e-3 to 6.9e-3 km/h; max \|Δd\| 0.011 to 0.052 m; time to reach identical in all 9 scenarios |
| Timestep convergence (interactive) | Euler order ~1; RK4 better | Euler order 0.96 to 1.00. At dt = 0.005 s the Euler distance error after 10 s is 0.086 m (0.06 %) |

`dotnet test`: `Passed! - Failed: 0, Passed: 87, Skipped: 0, Total: 87` on 2026-10-19. On 2026-10-20: 94/94 core tests and 11/11 game-logic tests (§10, §11). MATLAB `compare_results()` reports PASS for the reference
layer and the interactive layer in all 9 scenarios (18/18).

## 2. Method

1. **Golden data (MATLAB).** `validation/matlab/run_reference.m` reads the text of `model.m`, strips `clear; clc; close all;`,
   and applies each scenario override from `validation/scenarios.json` by regex. Each override must match exactly one
   assignment line. The script then runs a temporary copy with invisible figures and exports workspace variables to
   `validation/golden/<id>/`. Files and columns are listed in `validation/FILE_CONTRACT.md`. `model.m` is never modified.
2. **C# outputs.** `tools/ValidationRunner` builds the same files from `ReferenceModel.Build` / `ReferenceOval.Run` and writes
   them to `validation/out/csharp/<id>/`. It also writes the interactive R1 run, the error-source split (§5.3) and the
   convergence study (§6).
3. **Comparison.** There are two independent implementations:
   - `validation/matlab/compare_results.m` writes `validation/out/comparison.{json,md}` and the PNGs in `validation/plots/`.
   - The xUnit class `tests/VehicleDynamics.Tests/GoldenValidationTests.cs` computes the C# values in-process and compares
     them with `validation/golden/**`, so `dotnet test` validates without MATLAB. It fails if a golden folder is missing.

   Both read their tolerances from `validation/matlab/compare_config.json`.
4. **Element tolerance (reference layer).** `|c − g| ≤ abs_tol + rel_tol·|g|` with `abs_tol = rel_tol = 1e-9`, applied to
   every element of every column. Metrics per column are max abs, RMSE and max rel over elements with g ≠ 0.

## 3. Scenarios (`validation/scenarios.json`) and the MATLAB outputs they produce

| id | override | F_x,max [N] | curve gear | v_curva_final [km/h] | v_max R1 [km/h] | t_acc R1 [s] | d_brake R1 [m] | lap [s] |
|---|---|---|---|---|---|---|---|---|
| S01 | none | 6302.146956 | 2 | 60.6 | 117.620 | 16.885 | 56.141 | 63.928121 |
| S02 | slope_value = 10 | 6270.870599 | 2 | 58.7 | 98.448 | 20.720 | 28.630 | 69.802263 |
| S03 | slope_value = −5 | 6294.284012 | 2 | 60.5 | 125.858 | 15.285 | 75.170 | 62.324916 |
| S04 | car.M = 1800 | 7988.636986 | 2 | 60.6 | 109.522 | 18.850 | 45.532 | 66.297570 |
| S05 | car.mu = 0.7 | 5116.500283 | 2 | 54.5 | 114.755 | 16.885 | 74.611 | 68.850050 |
| S06 | car.Cd = 0.30 | 6302.146956 | 2 | 60.6 | 118.523 | 16.775 | 58.021 | 63.813235 |
| S07 | L_recta = 3000 | 6302.146956 | 2 | 60.6 | 179.187 | 59.995 | 134.598 | 158.055094 |
| S08 | R_curva = 40 | 6302.146956 | 1 | 42.8 | 116.632 | 16.605 | 65.297 | 57.129910 |
| S09 | car.mu = 1.2 | 7904.993081 | 2 | 67.8 | 119.504 | 17.345 | 39.181 | 59.772101 |

S07 runs into `model.m`'s `t < 60` cap on both straights. R1 stops at d = 2238.96 m and R2 at 2472 m, both short of 3000 m.
The lap time inherits `model.m`'s fallback (the last sample is used as the brake point). The reference layer reproduces this
exactly, but 158.06 s is not a physically meaningful lap time.

## 4. Reference layer vs MATLAB golden

### 4.1 Per scenario

Values compared per scenario:
- the 6 gear tables (111 × 8 each) and `resist.csv` (220 × 7);
- `r1_history` and `r2_history` (3 columns each);
- `engine_torque` (111 × 2) and `pchip_probe` (787 × 2);
- the 21 `summary.json` keys.

| id | values compared | R1 / R2 rows | max abs err | RMSE (worst column) | max rel err | elements failing | status |
|---|---|---|---|---|---|---|---|
| S01 | 28,458 | 3713 / 2878 | 0 | 0 | 0 | 0 | PASS |
| S02 | 31,740 | 4352 / 3333 | 0 | 0 | 0 | 0 | PASS |
| S03 | 27,246 | 3472 / 2715 | 0 | 0 | 0 | 0 | PASS |
| S04 | 30,006 | 4064 / 3043 | 0 | 0 | 0 | 0 | PASS |
| S05 | 29,085 | 3829 / 2971 | 0 | 0 | 0 | 0 | PASS |
| S06 | 28,362 | 3698 / 2861 | 0 | 0 | 0 | 0 | PASS |
| S07 | 80,685 | 12000 / 12000 | 0 | 0 | 0 | 0 | PASS |
| S08 | 29,199 | 3713 / 3125 | 0 | 0 | 0 | 0 | PASS |
| S09 | 28,119 | 3701 / 2777 | 0 | 0 | 0 | 0 | PASS |

Every column of every file is bit-identical; `validation/out/comparison.md` has the full per-file, per-column table. The baseline
values printed by C# are `F_x_max = 6302.146955955833 N`, `v_curva_final_kmh = 60.599999999999994` and
`t_vuelta_completa = 63.92812124496099 s`, all equal to MATLAB.

Because all errors are zero, there is no residual that would need an irreducibility proof. Bit-exactness depends on these port
choices, each of which is checked against MATLAB data:
- `MatlabInterp.LinearExtrap` uses `(1−s)·y_k + s·y_{k+1}`. Other algebraically equivalent forms differ by 1 ulp on the
  R1/R2 histories (`docs/vehicle_model.md` §9.2).
- `Pchip` reproduces `interp1(...,'pchip')`. The 787-point off-node probe matches at max rel error 0; the test threshold is
  1e-12.
- **Fixed in this pass:** `MatlabColon.Range` built the middle element of an even-length range as `(a+c)/2`. MATLAB R2026a
  uses `a + (c−a)/2`. This made no difference on `model.m`'s own ranges, so the golden data was unaffected, but a new MATLAB
  dataset showed the gap: 1200 random ranges (steps 0.1, −0.05, 0.05, 1, 0.3, −0.1, 7, 0.01; 59,836 elements; script
  `tests/VehicleDynamics.Tests/Data/matlab_scripts/colons_random.m`). The old rule differed on 32 of those ranges; the new
  one differs on 0 (test `MatlabColon_RandomRangesBitExact`). This matters because the braking loop `v_end:-0.05:v_target`
  has data-dependent endpoints.

### 4.2 Plots

- `validation/plots/<id>_r1_error.png` and `<id>_r2_error.png`: these show "all errors exactly 0".
- `<id>_r1_overlay.png` and `<id>_r2_overlay.png`.
- `<id>_tables_error.png`: no zero errors are drawn.

## 5. Interactive layer vs MATLAB R1

### 5.1 Set-up

R1 conditions as defined in `FILE_CONTRACT.md`:
- straight line, throttle 1, brake 0, steer 0, auto-shift on;
- start in 1st gear at 1000 rpm (8.198 km/h), dt = 0.005 s, model.m's Euler update;
- stop at d ≥ `L_recta` or t ≥ 60 s.

The seed-game clutch launch rule is **off**. With it on, the run is not MATLAB-comparable (§5.5). Samples are compared
point by point on the same time grid. Every scenario has the same step count as MATLAB.

### 5.2 Results

From `validation/out/csharp/interactive_vs_matlab.csv`, `interactive_r1.csv` and `comparison.md`. Δ = interactive − MATLAB.

| id | n | max \|Δv\| [km/h] | RMSE Δv [km/h] | max rel Δv | max \|Δd\| [m] | RMSE Δd [m] | max \|Δv\| at equal distance [km/h] | t to reach L_recta / cap, MATLAB = C# [s] | final v MATLAB / C# [km/h] |
|---|---|---|---|---|---|---|---|---|---|
| S01 | 3713 | 3.748e-3 | 2.290e-3 | 1.31e-4 | 0.0108 | 0.0070 | 2.209e-3 | 18.565 | 123.2918 / 123.2929 |
| S02 | 4352 | 5.128e-3 | 4.264e-3 | 2.21e-4 | 0.0253 | 0.0141 | 4.015e-3 | 21.760 | 99.6186 / 99.6228 |
| S03 | 3472 | 3.444e-3 | 2.694e-3 | 1.10e-4 | 0.0127 | 0.0069 | 2.013e-3 | 17.360 | 135.1298 / 135.1332 |
| S04 | 4064 | 3.720e-3 | 2.484e-3 | 1.33e-4 | 0.0134 | 0.0082 | 2.215e-3 | 20.320 | 113.7034 / 113.7052 |
| S05 | 3829 | 4.202e-3 | 3.420e-3 | 1.96e-4 | 0.0177 | 0.0096 | 2.559e-3 | 19.145 | 122.5604 / 122.5638 |
| S06 | 3698 | 3.764e-3 | 3.081e-3 | 1.31e-4 | 0.0155 | 0.0085 | 2.365e-3 | 18.490 | 124.6344 / 124.6375 |
| **S07** (long run) | 12000 | 6.937e-3 | 3.496e-3 | 1.31e-4 | 0.0522 | 0.0253 | 6.491e-3 | 60.000 (cap, both) | 179.1870 / 179.1936 |
| S08 | 3713 | 3.748e-3 | 2.290e-3 | 1.31e-4 | 0.0108 | 0.0070 | 2.209e-3 | 18.565 | 123.2918 / 123.2929 |
| S09 | 3701 | 3.701e-3 | 3.028e-3 | 1.31e-4 | 0.0153 | 0.0083 | 2.363e-3 | 18.505 | 123.3682 / 123.3711 |

`compare_results.m` interpolates distance to find the time at which each run reaches the common final distance. MATLAB and C#
differ there by 0.0003 to 0.0009 s, which is below one step.

S08 is identical to S01 because R1 does not depend on `R_curva`.

**S07 long run:** over 60 s and 2239 m, the speed error stays below 0.007 km/h and the distance error below 0.053 m (2.3e-5
relative). The error does not grow without bound; it follows the sign changes of the torque-interpolation error as the
operating point moves through the 50-rpm cells (`validation/plots/S07_interactive_r1.png`). At t = 59.995 s the odometers
read 2238.965 m (MATLAB) and 2239.017 m (C#).

### 5.3 Sources of the difference (measured, `validation/out/csharp/interactive_error_sources.csv`)

The two sides use the same equations, gear rule and Euler update. Along the MATLAB R1 speed trace, the acceleration
difference was evaluated and split into two parts:
- **(a) Torque:** direct PCHIP torque (interactive) vs linear interpolation in speed of the 50-rpm `T_engine` table (MATLAB,
  through `interp1` on `a_x_real`), with the same traction cap.
- **(b) Residual:** linear interpolation of the quadratic drag/rolling terms between table nodes, plus linear interpolation
  across the `min(F_trac, F_x,max)` kink.

`∫|Δa|dt` is the open-loop speed-error bound in km/h.

| id | max \|Δa\| total [m/s²] | max \|Δa\| torque | max \|Δa\| residual | ∫\|Δa\|dt total [km/h] | torque part | residual part | engine rpm range | share traction-limited |
|---|---|---|---|---|---|---|---|---|
| S01 | 3.26e-3 | 2.07e-3 | 3.26e-3 | 6.99e-3 | 6.62e-3 | 5.2e-4 | 1000.000 – 6499.807 | 10.9 % |
| S02 | 2.81e-3 | 2.07e-3 | 2.96e-3 | 1.09e-2 | 1.05e-2 | 5.4e-4 | 1000.000 – 6499.510 | 12.8 % |
| S03 | 2.07e-3 | 2.07e-3 | 1.85e-3 | 6.70e-3 | 6.41e-3 | 4.1e-4 | 1000.000 – 6499.117 | 10.6 % |
| S04 | 1.63e-3 | 1.63e-3 | 5.1e-6 | 7.02e-3 | 6.98e-3 | 1.3e-4 | 1000.000 – 6499.891 | 0 % |
| S05 | 1.27e-2 | 1.44e-3 | 1.23e-2 | 7.32e-3 | 5.21e-3 | 2.2e-3 | 1000.000 – 6499.096 | 25.5 % |
| S06 | 3.05e-3 | 2.07e-3 | 3.06e-3 | 6.90e-3 | 6.55e-3 | 4.8e-4 | 1000.000 – 6499.458 | 10.9 % |
| S07 | 3.26e-3 | 2.07e-3 | 3.26e-3 | 2.19e-2 | 2.09e-2 | 1.5e-3 | 1000.000 – 6499.807 | 3.4 % |
| S08 | 3.26e-3 | 2.07e-3 | 3.26e-3 | 6.99e-3 | 6.62e-3 | 5.2e-4 | 1000.000 – 6499.807 | 10.9 % |
| S09 | 2.06e-3 | 2.06e-3 | 6.5e-6 | 7.23e-3 | 7.20e-3 | 1.7e-4 | 1000.000 – 6499.325 | 0 % |

Conclusions:
- **Direct PCHIP vs 50-rpm table + linear interpolation is the dominant source.** It accounts for 71 to 99.7 % of `∫|Δa|dt`.
  The residual peaks where the traction cap starts or stops binding (S05 has the largest traction-limited share and the
  largest residual peak), but it integrates to at most 2.2e-3 km/h.
- **Gear-switch handling contributes nothing on these runs.** Both sides pick the lowest gear whose 6500-rpm speed is ≥ V, at
  the start of the step. The highest engine speed reached is 6499.9 rpm, so the interactive fuel cut (torque 0 at ≥ 6500 rpm,
  which differs from MATLAB's 118 Nm at the node) is never triggered. Moving the fuel cut to 7000 rpm changes the
  interactive R1 speed trace by exactly 0 in all 9 scenarios (`max_abs_dv_fuelcut7000_kmh`). The contract discrepancy
  reported in `docs/vehicle_model.md` therefore does not affect R1 comparisons. It would matter for a driver who holds a gear
  past 6500 rpm, and there is no MATLAB counterpart for that case.
- **Idle extrapolation contributes nothing on these runs.** The lowest engine speed is exactly 1000 rpm, at the start sample,
  so neither MATLAB's linear `extrap` of `a_x_real` below the table nor the interactive idle clamp is ever evaluated.
- **`max(0, a)` is never active.** In all scenarios the acceleration on the run is positive, as the MATLAB agent also observed.
- Observed max |Δv| is 0.32 to 0.54 of the open-loop bound `∫|Δa|dt`. Errors of opposite sign in different rpm cells partly
  cancel.

### 5.4 Tolerance and its justification

These values are in `compare_config.json` and used by both `compare_results.m` and `GoldenValidationTests`.

- **Speed: |Δv| ≤ 0.05 km/h** at every sample. The only modelling difference on these runs is (a) + (b) above. Their
  integrated open-loop effect is at most 0.022 km/h (S07, 60 s, the longest run). 0.05 km/h is about 2× that bound and 7× the
  largest observed error. It is tight enough to catch a real port regression: for example, a 1 % torque error would build up
  a Δa of 0.015 to 0.03 m/s² (3rd to 1st gear), or 0.5 to 1 km/h within 10 s.
- **Distance: |Δd(t)| ≤ (0.05/3.6)·t m.** This follows from integrating the speed bound (0.83 m at 60 s). The largest
  observed value is 0.052 m.
- **Time to reach `L_recta` (or the cap): same step count.** This held in all scenarios. In `compare_results.m` the
  interpolated time-to-reach must agree within one step (0.005 s); observed agreement is ≤ 0.0009 s.
- The previous provisional values (1 km/h + 1 %, 0.25 s) were 20 to 50× looser than needed and have been replaced.

### 5.5 Seed-game clutch launch rule (extension, not MATLAB)

With `ClutchLaunch = true` (engine held at ≥ 3500 rpm in 1st while throttle > 0.1, from `monaco.html`), the car launches
harder:
- Max |Δv| against MATLAB is 0.97 to 4.62 km/h.
- `L_recta` is reached 0.10 to 0.52 s earlier: S01 18.285 vs 18.565 s, S02 21.240 vs 21.760 s, S05 19.040 vs 19.145 s.
- S07 still hits the 60 s cap.

This is by design, so `interactive_r1_clutch.csv` is reported and not tested against MATLAB.

## 6. Timestep convergence study (interactive layer)

`ConvergenceStudy.Run(10)`, written to `validation/out/csharp/convergence_study.csv`, plot in
`validation/plots/convergence_study.png`. The state at t = 10 s is compared with RK4 at dt = 1e-5 s. Reference uncertainty
(RK4 at 1e-5 vs 2e-5 s) is ≤ 7.3e-11 m and ≤ 1.2e-11 m/s in all cases.

Cases:
- `r1_launch`: MATLAB R1 conditions with shifts, so a(V) jumps.
- `wot_3rd`: full throttle held in 3rd from 60 km/h, no shift.
- `coast_4th`: coasting from 100 km/h in 4th, with the seed engine brake.

Distance error at t = 10 s [m]:

| case | method | dt = 0.001 | 0.002 | 0.005 | 0.01 | 0.02 |
|---|---|---|---|---|---|---|
| r1_launch | Euler (model.m) | 0.0167 | 0.0352 | 0.0863 | 0.1679 | 0.3419 |
| r1_launch | RK4 | 1.6e-4 | 2.5e-5 | 1.6e-4 | 2.9e-3 | 2.6e-3 |
| wot_3rd | Euler (model.m) | 0.0119 | 0.0238 | 0.0594 | 0.1189 | 0.2378 |
| wot_3rd | RK4 | 1.4e-11 | 1.7e-11 | 1.1e-11 | 1.2e-10 | 2.4e-9 |
| coast_4th | Euler (model.m) | 0.0057 | 0.0113 | 0.0283 | 0.0567 | 0.1134 |
| coast_4th | RK4 | 1.2e-11 | 1.2e-11 | 1.2e-11 | 1.2e-11 | 1.2e-11 |

Speed error at t = 10 s for Euler [m/s]:

| case | dt = 0.001 | 0.002 | 0.005 | 0.01 | 0.02 |
|---|---|---|---|---|---|
| r1_launch | 1.97e-4 | 8.49e-5 | 4.81e-4 | 1.76e-3 | 2.47e-3 |
| wot_3rd | 3.80e-5 | 7.61e-5 | 1.90e-4 | 3.80e-4 | 7.60e-4 |
| coast_4th | 4.00e-5 | 8.00e-5 | 2.00e-4 | 4.00e-4 | 8.00e-4 |

Findings:
- **Euler order.** The observed order between dt = 0.01 and 0.005 is 0.959 for `r1_launch` and 1.000 for `wot_3rd` and
  `coast_4th`. The distance update `d += v_new·dt + ½a·dt²` is equivalent to `v_old·dt + 1.5·a·dt²`, so it carries an O(dt)
  bias of 0.5·a·dt per step. That bias is the 0.086 m (0.06 %) at the game step. The R1 speed error in the launch case is not
  monotone in dt, because the shift instants fall at different points within a step.
- **RK4.** On smooth cases RK4 sits at round-off level (≤ 2.4e-9 m). On the launch case it loses its order because of the
  acceleration jumps at shifts, but it beats Euler at every dt.
- **Fixed in this pass:** the RK4 stages previously kept the gear from the start of the step. Stages that crossed a shift speed
  then evaluated the engine above 6500 rpm, into the fuel cut. That was an artefact of the study, not of the game, which uses
  Euler. With auto-shift on, each stage now re-selects the gear with the same model.m rule, The previous run (frozen gear, reference RK4
  at dt = 0.0005 s) reported an RK4 launch error of 0.022 m at dt = 0.005 s; it is now 1.6e-4 m.
- **Is dt = 0.005 s adequate?** Yes, for the game and for MATLAB comparisons. The integration error after 10 s (0.086 m, 4.8e-4
  m/s) is of the same order as the model-to-model gap in §5 (0.011 to 0.052 m) and far below gameplay relevance. The Euler
  scheme is kept because it is MATLAB's scheme. Switching to RK4 would remove the 0.5·a·dt bias but would break the
  step-for-step agreement with `model.m`.

## 7. What is NOT validated against MATLAB

`model.m` has no lateral or yaw dynamics, no surfaces, no collisions, no driver-controlled braking transients, no coasting
or engine braking, and no manual shifting. The following parts are therefore checked only by analytic or unit tests
(`tests/VehicleDynamics.Tests`), never against MATLAB data:

| Extension / feature | Source | Tests (analytic expectation) |
|---|---|---|
| E1 kinematic single-track + friction circle | DESIGN_CONTRACT §2b (extension) | `KinematicBicycle_ConstantRadiusBelowGrip` (path radius = L/(cosβ·tanδ)), `Understeer_SaturatesLateralForce` (F_y capped at F_grip, r = F_grip/(M·V)), `FrictionCircle_CouplesLongitudinalAndLateral` (F_x ≤ √(F_grip² − F_y²)), `Frame_SteerSignGivesYawSign`, `Frame_StraightAhead_MapsToGodot` |
| E1 braking while cornering (braking force scaled by remaining circle share) | C# agent's choice, labelled extension | covered only through `FrictionCircle_CouplesLongitudinalAndLateral`; straight-line braking = MATLAB formula via `Braking_DecelerationMatchesMatlabFormula` |
| E2 surfaces (grip and rolling factors, provisional values) | DESIGN_CONTRACT §2b | `Surfaces_ScaleGripAndRolling`, `CrossSection_SurfaceLookup` |
| E3 collision response | DESIGN_CONTRACT §2b | `Collision_EnergyNeverIncreases`, `Collision_HeadOnAndGlancing`, `Collision_LatchPreventsRepeatedImpulses`, `BarrierContact_ReportsPenetrationAndInwardNormal` |
| Seed-game rules (clutch launch, idle clamp, fuel cut, engine brake, money shift) | `monaco.html` | `Engine_IdleClampFuelCutAndClutch`, `FuelCut_NoTractionAboveLimit`, `MoneyShift_BlowsEngine_AndToggleDisablesIt`, `ZeroThrottle_OnSlope_DocumentedBehaviour`, `ZeroInput_FlatGround_StaysAtRest` |
| Track geometry | DESIGN_CONTRACT §3 | `Oval_MatchesModelGeometryAndCloses`, `Circuit_RequirementsAndClosure`, `Projection_RoundTripInsideBarriers`, `Track_PoseContinuousAtJoints`, `Clothoid_HeadingClosedFormAndPositionAccurate` |
| Clock, recorder, config | DESIGN_CONTRACT §5–6 | `Clock_*`, `Loop_InputsHeldAcrossSubsteps_ShiftEdgeConsumedOnce`, `InputShaper_RampsPerSimulatedSecond`, `Recorder_*`, `Config_*` |

Also not validated:
- The interactive braking phase as a time history. MATLAB's braking is a Riemann sum over speed, not a time integration, so
  only the deceleration formula is compared.
- R2 and the curve phases in the interactive layer.
- Any steering or transient scenario.

## 8. Remaining issues (honest list)

- **No failing tests or comparisons.**
- **The tolerance was tightened, not loosened.** The interactive tolerance (0.05 km/h) is justified for straight-line R1
  runs only. It says nothing about E1/E2/E3.
- **Known `model.m` quirks are reproduced on purpose, not fixed:**
  - the a·dt² distance bias;
  - braking computed from the end-of-straight speed;
  - the 60 s cap in S07;
  - the F_x,max grip radius used for cornering vs 0.81 g used for braking.
- **The contract discrepancies listed in `docs/vehicle_model.md` remain open for the contract owner:**
  - torque 0 at ≥ 6500 rpm vs 118 Nm in MATLAB. This is shown above to have no effect on R1.
  - the E1 braking radius.

## 9. Reproduce

```bash
cd /home/juan-londono/Documents/vehicleDynamics/game
export PATH=$HOME/.dotnet:$PATH DOTNET_ROOT=$HOME/.dotnet

# 1. MATLAB golden data (only needed if model.m or scenarios.json change; golden/ is committed)
(cd validation/matlab && matlab -batch "run_reference()" < /dev/null > ../out/run_reference.log 2>&1)

# 2. C# side: reference + interactive outputs, error sources, convergence study
dotnet run --project tools/ValidationRunner -c Release

# 3. MATLAB comparison -> validation/out/comparison.{json,md}, validation/plots/*.png
(cd validation/matlab && matlab -batch "compare_results()" < /dev/null > ../out/compare.log 2>&1)

# 4. Convergence plot -> validation/plots/convergence_study.png
python3 validation/plot_convergence.py

# 5. Tests (reads validation/golden, no MATLAB needed)
dotnet build VehicleDynamics.sln
dotnet test VehicleDynamics.sln

# Random colon-operator dataset (tests/VehicleDynamics.Tests/Data/ml_colon_random.txt)
(cd tests/VehicleDynamics.Tests/Data && cp matlab_scripts/colons_random.m . && \
 matlab -batch "colons_random" < /dev/null > colons.log 2>&1; rm -f colons_random.m colons.log)
```

## 10. Re-run and closure (2026-10-20)

The whole chain was re-run from a clean build on 2026-10-20, without reusing any earlier output:

| Step | Command | Result |
|---|---|---|
| Reference integrity | `cd reference && sha256sum -c SHA256SUMS` | all 7 files OK; all 9 `golden/*/summary.json` carry `model_sha256 = b665b69c…0697b` |
| C# build | `dotnet build VehicleDynamics.sln` | 0 errors, 0 warnings |
| Unit + golden tests | `dotnet test VehicleDynamics.sln` | `Passed! - Failed: 0, Passed: 90, Total: 90` |
| C# validation outputs | `dotnet run --project tools/ValidationRunner` | S01–S09: golden max rel err `0.000E+000`; interactive R1 max \|Δv\| 3.4e-3 … 6.9e-3 km/h |
| MATLAB comparison | `compare_results` | reference layer PASS 9/9 (max abs 0, max rel 0); interactive layer PASS 9/9 |

Baseline values reproduced by the C# reference layer (S01): F_x,max = 6302.146955955833 N, v_curva_final = 60.6 km/h,
lap time = 63.92812124496099 s (MATLAB prints 63.93 s).

`LoopHookTests` covers the game-facing hooks rather than the MATLAB model:
- `SimulationLoop.BeforeStep` runs once per physics step, and the surface it sets reaches F_x,max.
- `VehicleDynamics.ApplyCollision` writes pose, heading and speed from a collision result, and rejects speed gains and
  non-finite values.
- `SustainedContact` (§11.5) removes only the approaching normal velocity.

**Not done.** The planned independent adversarial review of the port (a separate equation-by-equation and numerics review)
did not run: the workflow that would have run it stopped after this report was written. The bit-exact agreement above is
strong evidence for the reference layer. It says nothing about the extensions of §7, which rest on the unit tests only.

Phase 9 status: **closed** for everything MATLAB can check. The game layer (Phase 10) is covered in §11.

After the game-layer work below changed core files (s-continuous barrier offsets, sustained contact, two hooks), the
chain was run again: ValidationRunner S01–S09 golden max rel err `0.000E+000`; `compare_results` 18/18 PASS;
reference checksums 7/7 OK. The reference layer is untouched by those changes.

## 11. Game layer tests (Phase 10, 2026-10-20)

None of this is a comparison with MATLAB: model.m has no 3D world, steering, collisions or audio. These are
consistency and behaviour checks of the game built on the validated core.

### 11.1 Unit tests (`dotnet test VehicleDynamics.sln`)

`VehicleDynamics.Tests`: **94/94 passed**. `GameLogic.Tests`: **11/11 passed**. Measured values printed by the tests:

| Test | Result |
|---|---|
| Engine pitch locked to model rpm (f0 = rpm/30 Hz) | 1000 rpm: 33.37 Hz (+0.12 %); 3000 rpm: 99.98 Hz (−0.02 %); 6000 rpm: 200.02 Hz (+0.01 %); tolerance ±2 % |
| Squeal only when slip indicator > 1 | slip = 1.0 adds nothing (output exactly 0 at rest; RMS 4.3e-4 at 10 m/s, which is wind only); slip = 1.6 gives RMS 4.9e-2 |
| Synth cost / allocations | 102.5 µs per 256-frame chunk (real-time budget 5805 µs); 0 bytes allocated |
| Angled barrier impact at 30 m/s, then 10 s of full throttle into the wall | 1 impulse; 31.0 → 25.1 m/s at impact; CG never closer than 1.66 m to the wall |
| Pressing into a wall head-on at full throttle | speed stays below 0.019 m/s (sustained-contact constraint) |
| Scripted centreline lap (pure pursuit, through the model inputs) | Oval 64.03 s, Circuit 118.13 s; valid; 0 impacts; max \|lateral\| 0.35 m / 0.52 m |
| Lap timer | forward lap counted once, crossing time interpolated; backwards crossing and cutting the line count nothing |
| Barrier offsets continuous in s (both tracks) | largest change 0.0033 m per 0.01 m; nominal value away from joints |

The Oval lap time of the scripted driver (64.03 s) is close to model.m's lap estimate (63.93 s), but it is not a
validation. The two use different strategies: the driver follows a curvature-based speed plan through E1, while
model.m uses constant curve speed and its braking-distance rule.

### 11.2 In-game self-test (`-- --selftest`, headless, `--fixed-fps 60`)

The real scene is driven through the normal input path. **21/21 passed, exit code 0.**

| Check (each on Oval and Circuit) | Oval | Circuit |
|---|---|---|
| Spawn at rest on the grid (model.m oval start point) | PASS | PASS |
| Fixed physics step: steps × dt = simulated time | 124 steps / 0.620 s | 124 steps / 0.620 s |
| Scripted lap counted, valid, no impacts | 64.07 s | 118.23 s |
| Speedometer = physics state (telemetry sample vs `State.V`) | \|Δ\| < 1e-9 | \|Δ\| < 1e-9 |
| CSV export: header = schema, metadata, config JSON, sample interval | 3209 rows, interval error 1.8e-14 s | 5917 rows, 1.8e-14 s |
| Barrier response (steering into the wall for 12 s) | 15 impulses / 15 contact entries, 0 on consecutive steps, 16.5 → 12.2 m/s, CG ≥ 0.87 m from wall | 15 / 15, 0, 16.4 → 11.9 m/s, ≥ 0.88 m |
| Pause stops the clock | PASS | PASS |
| Parameter change reaches the equations (M = 1800 kg → F_x,max) | 7988.6 N = formula | 7988.6 N = formula |
| Money shift (2nd → 1st at 68 km/h) destroys the engine | PASS | PASS |
| Restart (grid, laps cleared, engine repaired) | PASS | PASS |
| UI panels open + run comparison of the two recorded runs | PASS (n = 3209 time-aligned samples) | |

### 11.3 Frame-rate independence

`-- --track=Oval --autodrive=10` (constant full throttle, auto-shift) at `--fixed-fps` 10, 60, 144 and 1000 gave
bit-identical states at the 2000th step: `V = 23.748988458642845 m/s`, `X = 129.9374357076798 m`. This holds because
input ramping and the steering assist run per physics step (`VehicleController.ShapeInputs`). Inputs that change from
frame to frame (a human, or the scripted driver, which samples the state once per frame) are themselves frame-rate
dependent, as with any real-time input device.

### 11.4 Performance (`-- --perf`)

| Run | Frame time | Physics per frame | Physics per step (incl. hooks) |
|---|---|---|---|
| Headless, Circuit, scripted driver, 3000 frames | 6.90 ms | 0.104 ms (1.38 steps) | 75 µs |
| Rendered 1280×720 on llvmpipe (software Vulkan), 150 frames, old STL car | 133 ms (7.5 fps) | 0.92 ms (26.7 steps) | 34 µs |
| Rendered 1280×720 on the RTX 3050 (Vulkan Forward+), 150 frames, JCW model (160k triangles) | 16.70 ms (59.9 fps, vsync-limited; max 18.2 ms) | 0.25 ms (3.34 steps) | 74 µs |

The first rendered measurement used llvmpipe (software), not the GPU. A later session rendered on the RTX 3050 and
reached the 60 Hz vsync limit with the detailed JCW model. The physics kept pace with real time even
at 7.5 fps, because the fixed-step clock runs about 27 steps per frame.

### 11.5 Defects found and fixed while testing the game layer

| Defect | Fix |
|---|---|
| Barrier walls stepped at segment joints with different runoff widths. A car scraping the wall would be pushed 2–5 m sideways in one step. | `BarrierOffsetLeftAt/RightAt(s)`, blended ±20 m around joints, used by physics and rendering (`BarrierOffsets_AreContinuousAndNominalAwayFromJoints`, `BarrierContact_UsesBlendedOffsetAtJoints`) |
| Pressing into a wall built up speed that the depenetration then discarded | `SustainedContact.Resolve` (`SustainedContact_RemovesApproachingNormalVelocityOnly`, `PressingIntoWall_CannotBuildSpeed`) |
| Input ramps applied per render frame (frame-rate dependent) | per-step shaping (§11.3) |
| Spawn behind the line was inside the Circuit's last corner; zero-steer autodrive hit the wall | spawn at s = 0 with a standing-start lap |
| HUD dial and lap panel drawn off-screen (control size 0) | draw against the viewport rectangle |
| Cockpit camera inside the closed STL shell | bonnet camera; later replaced by a real cockpit view once the JCW model (interior + glass) was integrated |
| "Recover to track" repaired a destroyed engine; shift presses while paused were applied on resume | damage kept; paused shifts dropped |
| `AudioStreamGeneratorPlayback` leaked at exit | released in `EngineAudio._ExitTree` |

### 11.6 Vehicle model replacement (JCW R56 GP)

The self-test was re-run after the car model and collision footprint were replaced: **21/21 passed**, exit code 0.
Oval lap 64.07 s and Circuit lap 118.23 s, both unchanged. Barrier test: 15 impulses / 15 contact entries, 0 on
consecutive steps, CG margin 0.86 m. Unit tests: 94/94 + 11/11.

The model's scale cross-checks against independent data:
- tyre radius 0.3032 m vs model.m R_w = 0.3043 m;
- shell width 1.68 m vs the R56's 1.683 m;
- length 3.76 m vs the JCW GP's about 3.73 m.

### 11.7 Still not validated

- E1, E2 and E3 against any measured vehicle data. Their values are provisional and the tests only check internal
  consistency.
- Rendering performance above 60 fps (vsync limited the GPU measurement) and at higher resolutions.
- Audio by ear: only pitch, gating and the slip rule were checked numerically.
- Godot editor workflows (opening the project in the editor UI). Every check here used the command-line engine.

## 12. Gamepad, launch clutch (E5), dynamic lateral model (E4) and correction C1 (2026-10-20)

### 12.1 MATLAB validation after the change

The validation chain was re-run after these changes:

| Check | Result |
|---|---|
| ValidationRunner, golden max rel err (S01–S09) | `0.000E+000` |
| Interactive R1 max \|Δv\| | identical to before, e.g. S01 3.748e-3 km/h |
| `compare_results` | 18/18 PASS |

These checks run with C1 off and no launch model (`FromReference`, `ClutchLaunch = false`). With δ = 0 the E4 terms are
exactly zero. `Dynamic_StraightLine_IsBitIdenticalToKinematic` asserts bit-identical speed and distance over 4000 steps.

### 12.2 1st-gear launch: what changed and why

Full throttle from rest with auto-shift (`dotnet run` scratch study using the core):

| Configuration | 0–50 km/h | 0–100 km/h | a_x at 10 km/h | rpm at 10 km/h | clutch locks |
|---|---|---|---|---|---|
| Before: 1420 kg, seed clutch S1, model.m formula | 4.86 s | 12.76 s | 2.89 m/s² | 3500 (pinned) | – |
| 1420 kg, S1, C1 | 4.67 s | 12.58 s | 3.01 | 3500 | – |
| 1420 kg, E5, C1 | 4.70 s | 12.60 s | 3.76 | 2263 | 18.7 km/h |
| **Game now: 1150 kg, E5, C1** | **3.89 s** | **10.20 s** | **4.33** | 2245 | 18.8 km/h |

Diagnosis:
- **Before.** 1st gear's 0.50 mass factor divided the F_x,max-capped driveline force by 1.5·M. The seed rule pinned the
  engine at 3500 rpm (flat tach and sound) up to 28 km/h.
- **C1.** With the rotating inertia accounted for, the locked 1st-gear tyre force is below F_x,max, so C1 adds only a few
  per cent.
- **E5.** The launch phase becomes genuinely grip-limited, at 4.34 m/s² on 1150 kg, because the engine inertia is
  decoupled while the clutch slips.
- **Mass.** The 1150 kg mass gives the largest overall gain.

### 12.3 New tests

13 tests in `tests/VehicleDynamics.Tests/DynamicModelTests.cs`, all passing. Total: core **107/107**, game logic **11/11**.

| Test | Result |
|---|---|
| Steady-state yaw rate vs linear bicycle, δ = 0.01 rad, 20.4 m/s | r = 0.065213 vs 0.065048 rad/s (0.25 %), K = 1.60e-3 s²/m (1420 kg) |
| Step steer 0.02 rad at 25 m/s | peak 0.156, settled 0.143 rad/s at 3 s and 8 s |
| Full lock 0.3 rad at 25 m/s | max \|a_y\| 8.459 m/s² ≤ μg 8.829; front axle saturates (understeer) |
| Left/right symmetry; low-speed kinematic limit; 60 s random inputs | exact symmetry; r = v tanδ/L at 1 m/s; all states finite, v_x ≥ 0, rpm ≥ idle |
| E5 full-throttle launch (1150 kg) | slips from rest, grip-limited a_x = 4.336 m/s² = (F_x,max − F_res)/M; locks at 1.485 s / 18.8 km/h; rpm 2279 → 2287 at lock |
| E5 throttle 0.3 | max a_x 1.24 m/s², locks, never below idle |
| E5 at rest | no creep on flat ground; on −10 % rolls, clutch locks, 8.2 km/h after 20 s with engine braking |
| C1 | grip-limited case: a = (F_x,max − F_res)/M with C1, /M_eq without; not grip-limited: identical |
| Collision with E4 state | velocity re-expressed in the new body frame, r scaled by V'/V, speed never increases |

### 12.4 Wall-contact chatter found and fixed

A scripted driver steering into a wall for 12 s gave 113 impulses with E4, against 19 with E1. The median severity was
0.46 m/s: the tyres pushed the car back into the wall after each small bounce. With the 1 m/s bounce threshold there
were 17 impulses (median 1.67 m/s); the rest is sliding contact. In the self-test that is 3 impulses over 46 contact
entries.

In the same abusive scenario the E4 car can spin (peak |r| 2.86 rad/s, oversteer flag set). The yaw rate changes
smoothly (no sign flips), so this is physical, not numerical.

### 12.5 In-game self-test

**29/29 passed**, exit code 0. Six checks are new: three on each track.

| Check | Result |
|---|---|
| Gamepad RT + stick (events injected through Godot's input pipeline) | RT 0.6 → throttle 0.5789 (deadzone-remapped, no ramp); stick −0.5 → δ = 0.0889 rad (exponent 1.5) |
| Gamepad LT | brake 1.0 |
| Gamepad RB / LB | gear 1 → 2 → 1 |
| Standing start (E5) | 0–50 km/h in 4.00 s; slips from rest, locks at 18.7 km/h; engine never below 1000 rpm |
| Scripted laps (now with E4 and 1150 kg) | Oval 62.16 s, Circuit 114.97 s, valid, no impacts |

The F710 itself was not connected in this environment. The gamepad checks inject the events a standard-layout pad
produces, so the device-specific mapping (XInput mode) remains to be confirmed on the real pad.

## 13. Handling tuning and the spin fix (2026-10-20, second pass)

### 13.1 Spin glitch: cause

The glitch was a bug in the first E4 version, not a breakdown of the tyre curve. There were three mechanisms:
- **Clamp:** $v_x$ was clamped at 0 (no reverse) even while the body slid sideways.
- **Reset and snap:** the low-speed code reset $v_y = r = 0$ whenever $v_x \le 0$, and below 4 m/s of $v_x$ it pulled
  $v_y$ and $r$ to kinematic values by a fixed fraction per step. A spinning car therefore lost its sideways speed
  within a few steps, which measured about 60 m/s².
- **Jitter:** the load transfer used $\dot v_x$, including the $v_y r$ term that swings in a spin, and slip angles had
  the form $\delta - \operatorname{atan2}(\dots, v_x)$, which is discontinuous for backward-rolling wheels.

The fixes are listed in `docs/vehicle_model.md` §15.

### 13.2 Spin regression test

Car at 22 m/s with r = 2.5 rad/s and v_y = −3 m/s, held at δ = 0.1 rad for 8 s (`Spin_SlidesSmoothlyWithoutSnapping`):

| Input | max \|β\| | min v_x | speed 0 → 0.5 s → 8 s | max \|ΔU\| per step | max \|Δr\| per step |
|---|---|---|---|---|---|
| none | 3.11 rad | −14.4 m/s | 22.2 → 19.2 → 0.4 m/s | 0.047 m/s | 0.058 rad/s |
| full throttle | 3.14 rad | −11.1 m/s | 22.2 → 20.1 → 0.9 m/s | 0.047 m/s | 0.034 rad/s |
| 60 % brake | 3.12 rad | −13.3 m/s | 22.2 → 18.1 → 0 m/s | 0.047 m/s | 0.053 rad/s |

A speed change of 0.047 m/s per 5 ms step is 9.4 m/s², i.e. friction deceleration. Before the fix the same test showed
steps of 0.36 m/s, about 72 m/s².

### 13.3 Handling balance

Steady 0.7 g corner at 20 m/s, then 2 s of the action with the steering held (game defaults, 1150 kg):

| Tuning | Action | max \|β\| | Result |
|---|---|---|---|
| before (μ 0.9, C 100/75 kN/rad, no load sensitivity) | lift-off | 1.9° | rear saturated |
| before | brake 50 % | 157° | **spin** |
| before | brake 100 % | 124° | **spin** |
| now | lift-off | 0.2° | stable |
| now | brake 50 % | 1.1° | stable |
| now | brake 100 % | 2.0° | front-limited (understeer) |

In slow ramp steers at 22 m/s (`LimitBalance_IsUndersteer`), both with speed held and coasting, the front axle
saturates first. Peak lateral acceleration is 0.92 / 0.93 g. In the linear range the understeer gradient is
K = 5.66e-3 s²/m at 1420 kg (yaw gain matches the linear bicycle within 0.25 %).

### 13.4 Results after the change

- **Tests:** core 112/112 and game logic 11/11.
- **In-game self-test:** 29/29.
  - Scripted laps: Oval 60.20 s and Circuit 111.61 s (were 62.16 s and 114.97 s).
  - 0–50 km/h: 3.90 s.
- **MATLAB chain:** golden max rel err 0 for S01–S09, interactive R1 errors identical to before, and `compare_results`
  18/18 PASS.

### 13.5 Caveats

The tuning values are estimates, not measurements of a Mini: C_αf and C_αr, the load sensitivity k, the front
roll-stiffness share, and μ = 1.05. Spins now need provocation, such as a flick or arriving with yaw already built up.
Braking in a corner no longer rotates the car. If you want some lift-off adjustability back, lower C_αr or the front
roll-stiffness share in the setup panel.

## 14. Monaco (converted Assetto Corsa track, 2026-10-20)

As in §11, this is a consistency check of the game layer, not a MATLAB comparison. E6 with RoadGrade = 0 and
SurfaceGrip = 0 is exactly the previous code path, so §4–§10 are unaffected. The golden tests still pass.

### 14.1 Unit tests

`VehicleDynamics.Tests` **116/116** (4 new), `GameLogic.Tests` **11/11**.

| New test | Result |
|---|---|
| `Polyline_SampledOval_MatchesAnalyticGeometry` (model.m oval sampled every 1 m) | length error < 1e-4 relative; curvature on the arc 1/80 to 4 decimals, 0 on straights; Project(PointAt(s, lat)) round trip within 0.05 m in s and 0.01 m in lateral, including across the closing seam; brute-force fallback far from the line |
| `Polyline_Limits_InterpolateTrackEdges` | linear between points |
| `RoadGrade_EqualsGlobalSlope` (6 %, 10 s full throttle) | bit-identical to model.m's global slope |
| `SurfaceGrip_OverridesSurfaceFactor` | F_x,max = formula with μ·0.95; 0 falls back to the E2 factor |

### 14.2 In-game self-test

**37/37 passed** (Oval 14, Circuit 14, UI 1, Monaco 8), exit code 0. Monaco checks:

| Check | Result |
|---|---|
| Spawn on the AC pole slot | pose (−347.18, −216.51, ψ 1.653) from `AC_START_0` |
| Ground under the grid | ray hit at −14.15 m (the AC grid object sits 0.55 m higher), car body at the ground, TARMAC, friction 0.99 |
| Kerb layer at the track limits | 161 of 1316 probes just inside the AC edge lines read KERB (friction 0.95) |
| Scripted lap on the AC racing line | 190.20 s over 3287 m, valid, 0 wall impacts (cautious scripted driver, not a lap-time reference) |
| Start/finish Area3D | 2 events (start and lap crossing), logged as `sf_gate` in the recorded run |
| Mesh surfaces → μ | F_x,max equals μ·friction·F_z(θ_eff) on every step (max error 0 N) |
| Elevation and grade | ground from −16.8 to 25.0 m; max \|grade\| 10.4 % |
| Mesh wall contact (10 s steering into the right-hand wall) | 8 impulses, 0 on consecutive steps, CG never crossed a wall |

During the lap, 15 steps (0.075 s) at s ≈ 2095 m read OFFTRACK. That is a small hole in the AC physics mesh where a
wheel ray also misses, and the friction differs by 0.04.

### 14.3 Frame-rate independence and performance

`--track=Monaco --autodrive=30` at `--fixed-fps` 30, 60 and 144 gave bit-identical states
(`V = 13.496965172003158 m/s`, `X = −274.3447834216452 m`), including the ray-based ground and wall queries.

Rendered on the RTX 3050 (Vulkan Forward+, 1600×900, scripted driver, 600 frames): **16.84 ms average (59.4 fps,
vsync-limited)**. Max frame 133 ms, a one-off hitch while shaders and textures load. Physics including the ray and shape
queries costs 95 µs per step (0.32 ms per frame).

## 15. Respawn, rigid-body impacts, racing line (2026-10-20)

Unit tests: `VehicleDynamics.Tests` **122/122** (6 new), `GameLogic.Tests` **11/11**. Self-test **39/39** (Monaco 10).

| Test | Result |
|---|---|
| Centred head-on impact (15 m/s, e = 0.25) | v' = 3.75 m/s = e·v, yaw rate 0 |
| Front-left corner head-on (15 m/s) | r' = +3.61 rad/s (car yaws left); front-right mirrors it exactly; contact-point normal velocity after = −e · before |
| Glancing scrape (25 m/s, 2 m/s into the wall, front-left corner) | wall friction: V' 24.71 vs 25.03 m/s without friction; r' −1.07 vs −1.14 rad/s |
| 20,000 random contacts (any normal, point, velocity, spin, e, μ_w) | kinetic energy never increases (worst relative change 6.7e-16); the contact point never keeps approaching |
| Spinning car: rotational → translational transfer | accepted by `ApplyCollision`; an energy-increasing result is rejected |
| Angled oval impact at 30°, 10 s pressing (GameLogic) | 2 separate impacts (the rear swings in), 0 on consecutive steps, energy never increases, CG ≥ 0.84 m from the wall |
| Reference lap, Oval (game parameters) | arc speed = √(0.85 μ g R) exactly; throttle out of the turns, braking from s = 352 m; 43.0 s; heavier car with less grip brakes no later |
| Respawn on Monaco | car moved to the highest point (s = 920 m, ground 24.98 m), then restart: body at −14.15 m = ground under the grid (before the fix the ground search could not reach it); physics ground found on the next steps |
| Racing line on Monaco | 1644 points, 72 zone changes; accelerate 77 %, coast 5 %, brake 18 %; reference lap 139.8 s. Sainte-Dévote braking 138 → 66 km/h, hairpin ≈ 40 km/h, tunnel exit 143 km/h. |
| Mesh walls with rigid-body impacts | 6 impulses, 0 consecutive, no tunnelling; largest yaw-rate change 2.28 rad/s |

The barrier checks on the procedural tracks now report a larger loss of forward speed at the first impact (Oval
20.9 → 6.2 m/s v_x, previously 14.5). The rotation turns part of the velocity sideways, and the wall friction removes
more. All barrier invariants still hold: impulses only on contact entry, no gain in energy, no tunnelling.
