# Validation file contract (MATLAB golden <-> C# outputs)

Written by the C# core agent before implementation. Both sides MUST produce exactly these files and columns.
If you need to change something, edit this file and say why.

Directories: MATLAB -> `validation/golden/<id>/`, C# -> `validation/out/csharp/<id>/`, where `<id>` is the
`id` field of a scenario in `validation/scenarios.json`.

## General format
- CSV: comma separated, UTF-8, `\n` line endings, first non-comment line = header (exact names below, snake_case),
  no index column, no quoting. Numbers in round-trip precision (MATLAB `%.17g`, C# `"R"`/`G17`),
  invariant culture (dot decimal). Lines starting with `#` are comments and must be skipped by readers
  (the reference-layer files below carry NO comment lines; `interactive_r1.csv` does).
- JSON: a single flat object, numeric values as JSON numbers (full double precision), keys exactly as below.

## Files per scenario

### `tables_gear<g>.csv`, g = 1..6 (one row per element of `car.rpm_axis`, 111 rows)
| column | MATLAB source |
|---|---|
| rpm | `car.rpm_axis` |
| v_kmh | `V_gear_kmh(g,:)` |
| t_engine_nm | `car.T_engine` |
| t_wheel_nm | `T_wheel_Nm(g,:)` |
| f_tractive_n | `F_tractive_N(g,:)` |
| a_x_ms2 | `a_x_ms2(g,:)` |
| f_tractive_eff_n | `F_tractive_effective(g,:)` |
| a_x_real | `a_x_real(g,:)` |

### `resist.csv` (one row per element of `v_kmh = 0.1:1:220`, 220 rows)
| column | MATLAB source |
|---|---|
| v_kmh | `v_kmh` |
| f_drag_n | `F_drag` |
| f_rolling_n | `F_rolling` |
| f_pend_n | `F_pend` |
| f_resistive_n | `F_resistive` |
| p_demanded_kw | `P_demanded_kW` |
| a_x_envelope | `a_x_envelope` |

### `r1_history.csv`, `r2_history.csv`
| column | MATLAB source |
|---|---|
| t_s | `history_t_r1` / `history_t_r2` |
| d_m | `history_d_r1` / `history_d_r2` |
| v_kmh | `history_v_r1` / `history_v_r2` (already km/h in MATLAB) |

### `summary.json`
Keys (MATLAB variable name = key): `F_x_max` (= `car.F_x_max`), `v_curva_adherencia_kmh`, `optimal_gear_curva`,
`v_max_gear_motriz_kmh`, `v_curva_final_kmh`, `d_frenado_r1`, `t_frenado_r1`, `d_acc_efectiva_r1`, `t_acc_r1`,
`v_max_recta1`, `t_recta1_total`, `d_frenado_r2`, `t_frenado_r2`, `d_acc_efectiva_r2`, `t_acc_r2`,
`v_max_recta2`, `t_recta2_total`, `t_curva1_total`, `t_curva2_total`, `t_vuelta_completa`, `distancia_lap`.
Optional extra key: `scenario_id` (string). Readers ignore unknown keys.

### `interactive_r1.csv` (C# ONLY, not produced by MATLAB)
Interactive layer (`VehicleDynamics.Core.Simulation`) run under R1 conditions: straight line, auto-shift ON,
throttle = 1, brake = 0, steer = 0, start in 1st gear at `min(V_gear_kmh(1,:))` (1000 rpm), dt = scenario dt
(0.005 s), stop when distance >= `L_recta` or t >= 60 s (same as MATLAB R1 loop). One row per physics step
(sample interval = dt) so it can be compared point-by-point with `r1_history.csv`.
Columns = DESIGN_CONTRACT §6 telemetry schema, preceded by `# key=value` comment lines.
Comparison: `interactive_r1.csv` (t_s, v_kmh, distance column `track_s_m` = odometer) vs golden `r1_history.csv`.

## `validation/scenarios.json` schema (created by the C# agent; do not reshape)
`{ "version": 1, "notes": str, "scenarios": [ { "id": "S01", "name": str, "description": str,
"overrides": { "<model.m variable name>": number, ... }, "compare_interactive": bool } ] }`.
Override keys used: `slope_value`, `car.M`, `car.mu`, `car.Cd`, `L_recta`, `R_curva`.

## Additions by the MATLAB harness agent (2026-10-19)
Added (not changing anything above) so the C# side knows what else is in `golden/<id>/`:
- `summary.json` also carries `overrides` (string, `name=value` pairs joined by `;`, empty for baseline) and
  `model_sha256` (string, SHA-256 of `reference/matlab/model.m`). Both are metadata; readers ignore them.
  Kept as strings so the object stays flat.
- `engine_torque.csv` (MATLAB only): `rpm, t_engine_nm` = `car.rpm_axis`, `car.T_engine` (same as the table columns,
  provided standalone for the PCHIP unit test).
- `pchip_probe.csv` (MATLAB only): `rpm, t_engine_nm` on `unique([1000:7:6500, 6500])` (787 rows) =
  `interp1(rpm_raw, torque_raw, rpm, 'pchip')` — off-node probe for the C# `Pchip` port.
- If C# also writes `engine_torque.csv` / `pchip_probe.csv` in `out/csharp/<id>/`, `compare_results.m` compares them
  with the reference tolerances; otherwise they are skipped.
- Comparison tolerances live in `validation/matlab/compare_config.json` (scenarios.json is not reshaped).

## Additions by the C# agent (2026-10-19, after reading the MATLAB additions above)
- C# also writes `engine_torque.csv` and `pchip_probe.csv` (same columns/rows as the MATLAB files; probe rpm =
  `unique([1000:7:6500, 6500])`) in `out/csharp/<id>/`.
- `interactive_r1.csv` is produced with the seed-game clutch launch rule OFF (`ClutchLaunch=false`, recorded in its
  `# key=value` header) so that it uses exactly the MATLAB equations at 1000 rpm in 1st; the only modelling difference
  left vs `r1_history.csv` is direct PCHIP torque vs the 50-rpm table + linear interpolation. One row per step, rows
  are the states for which the MATLAB loop condition `d < L_recta && t < 60` holds (same count as `r1_history.csv`).
- Optional extra `interactive_r1_clutch.csv` (same schema): identical run with the seed launch rule ON
  (engine held at >= 3500 rpm in 1st while throttle > 0.1) — shows the effect of that seed-game rule; not MATLAB-comparable.
- `validation/out/csharp/convergence_study.csv`: `case,integrator,dt,v_end_mps,d_end_m,err_v_mps,err_d_m`
  (interactive layer, t_end = 10 s, reference = RK4 with dt = 0.0005 s).
- `validation/out/csharp/golden_comparison.csv`: `id,file,column,n,max_abs_err,max_rel_err` (C# reference layer vs
  golden, written only when golden files exist), plus `interactive_vs_matlab.csv`:
  `id,n,max_abs_dv_kmh,rmse_dv_kmh,max_abs_dd_m,t_reach_matlab_s,t_reach_interactive_s` (time to reach L_recta).

## Additions by the validation agent (2026-10-19)
- `validation/out/csharp/interactive_error_sources.csv` (ValidationRunner): `id,n,max_abs_da_total,max_abs_da_torque,
  max_abs_da_residual,int_abs_da_total_kmh,int_abs_da_torque_kmh,int_abs_da_residual_kmh,min_engine_rpm,max_engine_rpm,
  frac_traction_limited,max_abs_dv_fuelcut7000_kmh,clutch_max_abs_dv_kmh,clutch_t_reach_s,t_reach_matlab_s`.
  Acceleration gap [m/s^2] interactive minus MATLAB `interp1` evaluated on the golden R1 speeds, split into the torque part
  (direct PCHIP vs linear interpolation of the 50-rpm torque table) and the residual (linear interpolation of the resistances
  and across the traction-limit kink); `int_*` = sum |da|*dt in km/h (open-loop speed-error bound); rpm range of the run;
  effect of moving the fuel cut from 6500 to 7000 rpm; clutch-rule run vs MATLAB.
- `convergence_study.csv` now has three cases (`r1_launch`, `wot_3rd`, `coast_4th`); reference = RK4 with dt = 1e-5 s
  (was 0.0005 s). Columns unchanged.
- `validation/plot_convergence.py` writes `validation/plots/convergence_study.png` from that CSV.
- Interactive tolerances in `validation/matlab/compare_config.json` are no longer provisional (0.05 km/h, 0.005 s; see
  `docs/validation_report.md`). The xUnit test `GoldenValidationTests` reads the same file.
