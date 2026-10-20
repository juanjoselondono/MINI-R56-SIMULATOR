# R56 Vehicle Dynamics Simulator

A Godot 4.7 (C#) 3D driving simulator of a Mini Cooper R56 (N16). The car is driven by a C# port of the MATLAB model
`reference/matlab/model.m`, which is validated against MATLAB: the reference layer reproduces it bit-exactly in 9
scenarios.

- **Physics core** (`src/VehicleDynamics.Core`). model.m's longitudinal model (engine PCHIP torque map, gears, traction
  limit F_x,max, drag/rolling/grade, braking) runs at its own fixed step. Steering (E1), surfaces (E2) and barrier
  impacts (E3) are labelled extensions, because model.m has no lateral dynamics.
- **Game** (`VehicleDynamicsSimulator/`):
  - two tracks: model.m's oval, and a 2.6 km road circuit;
  - analytic barrier collisions, lap timing;
  - a live setup panel exposing the real model parameters;
  - engineering telemetry with CSV export, and run comparison;
  - procedural engine audio locked to the model's rpm.
- **Validation** (`validation/`, `tests/`). Golden data comes from the unmodified model.m; 105 unit tests and a
  21-check in-game self-test.

## Quick start

```bash
export PATH=$HOME/.dotnet:$PATH DOTNET_ROOT=$HOME/.dotnet
export GODOT=~/Applications/godot/Godot_v4.7.2-stable_mono_linux_x86_64/Godot_v4.7.2-stable_mono_linux.x86_64

dotnet test VehicleDynamics.sln                               # core + game-logic tests
cd VehicleDynamicsSimulator && dotnet build && $GODOT --headless --import --path .
$GODOT --path .                                               # play (F1 in game for controls)
$GODOT --headless --fixed-fps 60 --path . -- --selftest       # in-game self-test, exit code 0 = pass
```

## Documentation

| Document | Contents |
|---|---|
| [docs/run_instructions.md](docs/run_instructions.md) | requirements, build, launch, controls, panels, output paths, test modes, troubleshooting |
| [docs/architecture.md](docs/architecture.md) | layers, per-frame/per-step data flow, coordinates, collisions, telemetry, audio, extending |
| [docs/vehicle_model.md](docs/vehicle_model.md) | model.m equations, MATLAB→C# mapping, exposed parameters, extensions and their sources |
| [docs/validation_report.md](docs/validation_report.md) | MATLAB comparison, convergence study, game-layer tests, what is not validated |
| [docs/initial_audit.md](docs/initial_audit.md) | audit of the original MATLAB model and JS seed game |
| [docs/DESIGN_CONTRACT.md](docs/DESIGN_CONTRACT.md) | binding design decisions and their amendments |

`reference/` holds read-only copies of the original files, with SHA-256 checksums. The originals in the parent
directory are untouched.
