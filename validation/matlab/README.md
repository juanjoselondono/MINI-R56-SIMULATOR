# MATLAB reference / validation harness

Requires MATLAB (tested with R2026a). Always run MATLAB non-interactively with stdin closed and output
redirected, otherwise a helper process keeps the pipe open and the shell hangs:

```bash
cd validation/matlab
matlab -batch "run_reference()" < /dev/null > ../out/run_reference.log 2>&1
```

## Files
| file | purpose |
|---|---|
| `run_reference.m` | generates `validation/golden/<id>/` from the unmodified `reference/matlab/model.m` |
| `compare_results.m` | compares `validation/out/csharp/<id>/` (C# outputs) with `validation/golden/<id>/` |
| `compare_config.json` | tolerances (reference layer 1e-9 rel / 1e-9 abs; interactive layer 0.05 km/h, 0.005 s, justified in docs/validation_report.md) |
| `../scenarios.json` | scenario definitions shared with C# (S01..S09) |
| `../FILE_CONTRACT.md` | exact file names / columns / summary keys |

## Regenerating golden data
```bash
matlab -batch "run_reference()" < /dev/null > ../out/run_reference.log 2>&1            # all scenarios
matlab -batch "run_reference({'S01','S07'})" < /dev/null > ../out/run_reference.log 2>&1  # subset
```
What it does, per scenario:
1. reads `reference/matlab/model.m` as text (the file is never modified; its SHA-256 goes into `summary.json`);
2. comments out the `clear; clc; close all;` line (exactly one match required);
3. for every override `"<name>": value` in `scenarios.json` replaces the right-hand side of the single assignment
   line `<name> = ...;` (error if the key matches 0 or >1 lines). JSON keys like `car.M` are mangled by
   `jsondecode` to `car_M`; they are mapped back to the unique model.m left-hand side with that mangled name;
4. writes the modified text as a function `validation/out/tmp/model_ref_<id>.m` (clean function workspace,
   figures forced invisible), runs it and captures the workspace;
5. exports `tables_gear1..6.csv`, `resist.csv`, `r1_history.csv`, `r2_history.csv`, `summary.json`
   (+ MATLAB-only `engine_torque.csv`, `pchip_probe.csv`) with `%.17g`.

Baseline check (S01): `F_x_max = 6302.146956 N`, `v_curva_final_kmh = 60.60`, `t_vuelta_completa = 63.928121 s`.
Golden files are deterministic; regenerate only if model.m or scenarios.json change, then commit them.

## Running the comparison
First produce the C# side (`tools/ValidationRunner`, writes `validation/out/csharp/<id>/`), then:
```bash
matlab -batch "compare_results()" < /dev/null > ../out/compare.log 2>&1
matlab -batch "compare_results('Scenarios',{'S01'})" < /dev/null > ../out/compare.log 2>&1
```
Options: `'CsharpDir'`, `'GoldenDir'`, `'PlotDir'`, `'OutDir'`, `'Config'`.

Outputs:
- `validation/out/comparison.json` and `validation/out/comparison.md`
- `validation/plots/<id>_r1_error.png`, `<id>_r2_error.png` (|error| vs time), `<id>_r1_overlay.png`,
  `<id>_r2_overlay.png`, `<id>_tables_error.png`, and `<id>_interactive_r1.png` when present.

Reference layer: every column of every table/history file and every numeric summary key, element-wise
pass criterion `|c - g| <= abs_tol + rel_tol*|g|`; reported metrics are max abs error, RMSE and max relative
error (over non-zero golden elements). Histories are matched on common time samples (`ismembertol`, 1e-9 s);
a length mismatch is reported and counts as a failure.

Interactive layer (separate section, differs by design): `out/csharp/<id>/interactive_r1.csv` (columns `t_s`,
`v_kmh`, `track_s_m`) against golden `r1_history.csv`, with speed interpolated linearly on time and on distance,
plus the time to reach the common final distance. Tolerances in `compare_config.json` are justified in `docs/validation_report.md` §5.

Smoke test without C# outputs (golden vs a copy of itself; all errors must be exactly 0):
```bash
cp -r ../golden /tmp/cs_copy
matlab -batch "compare_results('CsharpDir','/tmp/cs_copy','PlotDir','/tmp/cs_plots','OutDir','/tmp/cs_cmp')" < /dev/null > /tmp/cs_cmp.log 2>&1
```
