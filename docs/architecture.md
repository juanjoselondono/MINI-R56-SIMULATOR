# Architecture

The simulator has four layers. Each depends only on the layers below it:

| Layer | Where | Depends on | Responsibility |
|---|---|---|---|
| Physics core | `src/VehicleDynamics.Core` (net8.0, no Godot) | – | model.m equations, extensions E1–E3, track geometry, collision response, clock, recorder, config |
| Validation | `validation/`, `tools/ValidationRunner`, `tests/` | core, MATLAB (optional) | golden data from the unmodified `model.m`, comparison, unit tests |
| Game logic (Godot-free C#) | `VehicleDynamicsSimulator/scripts/{race,vehicle/CollisionManager.cs,audio/EngineSynth.cs,dev/CentrelineDriver.cs}` | core | lap timing, per-step collision handling, sound synthesis, scripted driver. Unit-tested by `tests/GameLogic.Tests`, which compiles these files directly. |
| Presentation | `VehicleDynamicsSimulator/` (Godot 4.7, C#) | all of the above | rendering, input, camera, UI, audio output, scene flow |

**Rule:** nothing in the presentation layer moves the car or invents a physical value. The vehicle state changes
only through four paths:
- `VehicleDynamics.Step`, which integrates the equations;
- `VehicleDynamics.ApplyCollision`, which applies the documented E3 response and rejects any speed gain;
- `VehicleController.ResetVehicleAt`, which is an explicit reset at rest (grid restart or recover-to-track);
- `VehicleController.ApplyParameters`, which validates parameters and applies them between steps.

Every number on screen, in the CSV and in the audio comes from `VehicleState`, `VehicleOutputs`, the lap timer or the
driver inputs.

## 1. Physics core

```
VehicleDynamics.Core
├── Reference/      bit-exact port of model.m §1–11 (ReferenceModel, ReferenceOval, Pchip, MatlabColon, MatlabInterp)
├── Simulation/     interactive model
│   ├── VehicleParameters   typed parameters, Validate()/EnsureValid(); groups: model.m / JS seed / extensions
│   ├── VehicleDynamics     Step(u, dt): longitudinal model.m equations + E1 lateral + E2 surfaces; ApplyCollision
│   ├── EngineModel, TractionLimit, Resistance
│   ├── SimulationClock     fixed-dt accumulator, frame clamp 0.25 s, interpolation alpha
│   ├── SimulationLoop      per-frame Advance: BeforeStep → Step → AfterStep → Recorder, previous state for interpolation
│   ├── InputShaper         rate limits in simulated time
│   ├── SimulationRecorder  fixed-interval samples, CSV (DESIGN_CONTRACT §6), RunComparison
│   ├── ConfigSerializer    strict JSON (unknown keys and invalid values rejected)
│   └── FrameConversion     model ↔ Godot pure math
├── Track/          TrackGeometry (straight/arc/clothoid, exact pose, projection, s-continuous barrier offsets,
│                   surfaces), TrackLibrary (Oval = model.m geometry, Circuit), BarrierContact
└── Collision/      CollisionResponse (E3 impulse), SustainedContact, ContactLatch
```

The reference layer reproduces MATLAB to max relative error 0 (`docs/validation_report.md`). The interactive layer uses
the same equations with driver inputs. Everything that model.m does not contain is labelled as an extension and
grouped separately in `VehicleParameters`:
- **E1:** kinematic single-track model limited by the friction circle.
- **E2:** surface factors for curb and grass.
- **E3:** barrier impulse and the sustained-contact constraint.
- **Seed-game rules:** clutch launch, idle clamp, fuel cut, engine braking, money shift.

The model and its extensions are described in `docs/vehicle_model.md`.

## 2. Frame and step data flow

```mermaid
sequenceDiagram
    participant G as Godot frame (_Process)
    participant VC as VehicleController
    participant L as SimulationLoop
    participant SM as SimulationManager hooks
    participant VD as VehicleDynamics (core)
    G->>VC: delta
    VC->>VC: sample raw targets once (keyboard/gamepad or Autopilot), shift edges
    VC->>L: Advance(delta, levels)
    loop n = clock steps (fixed dt = 0.005 s)
        L->>VC: BeforeStep: InputShaper.Update(dt), steering assist, δ = steer·δmax
        L->>SM: BeforeStep: surface = Track.SurfaceAt(X, Y)   [E2]
        L->>VD: Step(u, dt)   (semi-implicit Euler as in model.m)
        L->>SM: AfterStep: CollisionManager → ApplyCollision [E3]; LapTimer; telemetry events/live buffer
        L->>L: Recorder.OnStep (every SampleInterval/dt steps)
    end
    VC->>G: node transform = interpolated pose (prev, current, alpha)
    G->>G: camera, HUD, panels, EngineAudio read state/outputs
```

Properties that follow from this design and are checked by tests:
- **The physics step count depends only on simulated time.** `fixed physics step` in the self-test. With constant
  inputs the states are bit-identical at 10, 60, 144 and 1000 fps (`docs/validation_report.md` §11).
- **Shift edges reach exactly one step.** `Loop_InputsHeldAcrossSubsteps_ShiftEdgeConsumedOnce`. Presses made while
  paused are dropped.
- **Render interpolation never feeds back into the physics.** `SimulationLoop.InterpolatedPose` is read only by
  `VehicleController.ApplyPose`.
- **Parameter changes are applied between frames,** never inside a step. A new `Dt` rebuilds the clock but keeps the
  state. A recording in progress is split, so each CSV has a single sample interval.

## 3. Coordinates

The model frame is ISO-style: X east, Y north, z up, ψ counter-clockwise from +X, δ > 0 to the left, r > 0
counter-clockwise. Godot is Y-up with −Z forward. The one conversion is `VehicleModelAdapter`, which delegates to
`FrameConversion`:
- position: `godot = (X, h, −Y)`
- heading: `Rotation.Y = ψ − π/2`, so the car's −Z axis maps to (cos ψ, 0, −sin ψ).

The track meshes, scenery, barrier collision body, spark positions and camera all go through the same functions.
The vehicle mesh (2011 Mini JCW R56 GP, converted by `tools/asset_pipeline/import_jcw_obj.py`) is exported facing
−Z, scaled so its wheelbase equals model.m's 2.467 m, with its origin on the ground below the CG. The CG sits `car.c` = 1.529 m ahead of the rear axle.

### Vehicle visuals

`VehicleVisuals` only reads values derived from the state:
- **Wheels:** spin = odometer / R_w. The front wheels turn by the model's clamped δ.
- **Steering wheel:** turns by δ × 14 about its measured column axis. The ratio is a visual constant; model.m has no
  steering system.
- **Brake lights:** the tail-light lens glows when the brake input exceeds 0.05.
- **Headlights:** the headlight texture glows when the headlights are on.
- **Materials:** tuned per source material name (metallic clearcoat paint, translucent glass, rough tyres, metallic
  rims).

No body roll or pitch is drawn, because model.m has no suspension dynamics.

## 4. Track, surfaces and collisions

The road, curbs, runoff and barriers are generated by `TrackMeshBuilder` from the same `TrackGeometry` the physics
uses:
- straights are sampled every 4 m and curves every ≤ 1° of heading;
- the barrier inner face sits exactly at `BarrierOffsetLeftAt/RightAt(s)`.

So what you see is what you hit.

| Concern | Implementation | Notes |
|---|---|---|
| Surface (E2) | `BeforeStep` → `TrackGeometry.SurfaceAt(CG)` | One surface for the whole car, evaluated at the CG. model.m has a single μ, and per-wheel contact is not modelled. |
| Barrier detection | `CollisionManager.AfterStep` → `BarrierContact.Check` on the 3.76 × 1.68 m body footprint (measured from the JCW model's shell without mirrors, centre 0.261 m behind the CG) | Analytic, deterministic, per physics step. No Godot physics is involved in the car's motion. |
| Response (E3) | First step of a contact: `CollisionResponse.Resolve` (restitution e, tangential loss k_t, yaw kick ≤ maxYawKick). While latched: `SustainedContact.Resolve`. Both applied via `ApplyCollision`. | One impulse per contact (`ContactLatch`); a new impulse only after separation. Speed can never increase. |
| Effects | `SimulationManager.OnImpact` → sparks, camera shake, `EngineSynth.TriggerImpact`, telemetry event `impact:<severity>`, lap invalidated | Cosmetic only. |
| Godot collision shapes | `StaticBody3D` with the barrier faces (layer 2) | Used only by the chase camera's ray query, which keeps the camera in front of walls. |
| Recovery | `RecoverToTrack`: explicit reset at rest on the nearest centreline point, engine damage kept, lap invalid | The model has no reverse gear, so a head-on stop needs this. |

## 5. Telemetry and configuration

`TelemetryService` attaches a core `SimulationRecorder` to the loop:
- **Sampling:** every `SampleInterval / dt` physics steps (0.02 s = 4 steps by default), never at render rate.
- **Contents:** every column of DESIGN_CONTRACT §6, `# key=value` metadata (track, start time, every parameter) and
  events (shifts, impacts, laps, restarts, recoveries, parameter changes, engine failure).
- **Export:** writes `user://runs/<timestamp>_<run>.csv` with `<…>.config.json` next to it.
- **Plots:** read a separate live ring buffer of 1500 samples at the same interval, so they work without recording.
- **Configurations:** strict JSON files in `user://configs/`.

On Linux `user://` is `~/.local/share/godot/app_userdata/R56 Vehicle Dynamics Simulator/`.

## 6. Audio

`EngineSynth` is plain C#, allocation-free, and runs at about 100 µs per 256-frame chunk. `EngineAudio` streams it
through an `AudioStreamGenerator`.

| Sound | Driven by | Rule |
|---|---|---|
| Engine tone | `State.EngineRpm` | f0 = rpm/60 · 2 (inline-4, four-stroke: 2 firings per revolution), 8 harmonics. Verified to ±0.12 % at 1000/3000/6000 rpm. |
| Load / brightness | driver throttle | more upper harmonics and intake noise with throttle |
| Overrun burble | `Outputs.Branch == Coast` and rpm > 2500 | random short pops |
| Rev limiter | `Outputs.RevLimiter` | 15 Hz gate |
| Shift | gear change | 80 ms dip |
| Engine failure | `State.EngineBlown` | explosion, then silence |
| Tyre squeal | `Outputs.SlipIndicator > 1` and V > 3 m/s | band-pass noise around 1.1 kHz. The slip indicator exceeds 1 only when the traction limit (model.m §6–7) or the E1 lateral grip saturates. Exactly silent otherwise. |
| Brake scrub | Brake branch with brake input ≥ 0.95 (model.m's maximum braking force) | audio cue only. The model has no wheel lock or ABS. |
| Wind | V | ∝ (V/50)² |
| Impact | E3 impact severity | thump + noise ∝ severity |

## 7. Threading and performance

The simulator is single-threaded on purpose. The measurement below shows the physics needs no worker thread, and a
second thread would add synchronisation and an input-to-state latency of at least one frame.

Measured with `-- --perf`:
- **Physics core:** 75 µs per step including the collision, surface, lap and telemetry hooks, which is about 0.25 ms
  per frame at 60 fps.
- **Rendering:** with the 160k-triangle JCW model, 16.7 ms per frame (59.9 fps, vsync-limited) on the RTX 3050 laptop
  GPU, Circuit, 1280×720. A run on the software renderer (llvmpipe) gave 133 ms per frame with the old STL car.

## 8. Godot project map

```
VehicleDynamicsSimulator/
├── project.godot, VehicleDynamicsSimulator.csproj      (ProjectReference → src/VehicleDynamics.Core)
├── scenes/  MainMenu.tscn (main scene), Main.tscn (simulation), Vehicle.tscn (car)
├── scripts/
│   ├── simulation/  SimulationManager (hooks, reset/recover, GameSession), TelemetryService
│   ├── vehicle/     VehicleController (core owner, inputs, interpolation), VehicleVisuals, CollisionManager, InputSetup
│   ├── track/       TrackManager, TrackMeshBuilder, TrackScenery
│   ├── race/        LapTimer
│   ├── camera/      ChaseCamera (chase / bonnet / cockpit, barrier ray, shake)
│   ├── dynamics/    VehicleModelAdapter (model ↔ Godot)
│   ├── ui/          UiRoot, RaceHud, VehicleSetupPanel, ParameterCatalog, TelemetryPanel, TelemetryPlot,
│   │                RunComparisonPanel, UiTheme, UiFocus
│   ├── audio/       EngineSynth (pure C#), EngineAudio
│   ├── effects/     ImpactEffects
│   ├── menu/        MainMenu
│   └── dev/         SelfTest, CentrelineDriver, ScreenshotRig, PerfProbe
└── assets/  vehicles/mini_jcw (body, 4 wheels, steering wheel, textures, meta JSON), materials (procedural shaders)
```

## 9. Extending

- **New track:** add a factory to `TrackLibrary` (segments plus closure) and its name to `TrackManager.Available` and
  `TrackManager.Create`. Meshes, surfaces, barriers, scenery, lap timing and the self-test follow from the geometry.
  Keep inner barrier offsets below the corner radius, and keep non-adjacent parts of the track further apart than the
  sum of their barrier offsets, so projection stays unambiguous.
- **New parameter:**
  1. Add the property to `VehicleParameters`, under the right group, with a range in `Validate()`.
  2. Use it in the equations.
  3. Add a `ParameterCatalog` entry with a range inside `Validate()` and a source tag.

  JSON configs and CSV metadata pick it up by reflection.
- **New telemetry channel:** add it to `TelemetrySample`, `SimulationRecorder.Columns`/`WriteCsv` and DESIGN_CONTRACT §6.
