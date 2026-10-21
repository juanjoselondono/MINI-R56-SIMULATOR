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
- **E4 (default):** dynamic single-track model (slip angles, per-axle friction circles, yaw inertia). E1, the kinematic
  single-track model limited by a friction circle, stays selectable.
- **E2:** surface factors for curb and grass.
- **E3:** barrier impulse, the sustained-contact constraint and a 1 m/s bounce threshold.
- **E5 (default):** physical launch clutch with an engine-speed state.
- **C1:** grip limit on the tyre force, a correction of model.m §7 (default on; off for MATLAB validation).
- **Seed-game rules:** idle clamp, fuel cut, engine braking, money shift; the seed clutch launch is optional.

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
        L->>VC: BeforeStep: keyboard → InputShaper.Update(dt) / gamepad → direct, steering assist, δ = steer·δmax
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
| Response (E3, E4 model) | `RigidBodyImpact.Resolve` at the contact point: restitution + wall friction + yaw-rate change (vehicle_model §19). One bouncing impulse per contact (latch, bounce threshold on the contact-point normal speed), then the e = 0 version while latched. |
| Response (E3, E1 model) | First step of a contact: `CollisionResponse.Resolve` (restitution e, tangential loss k_t, yaw kick ≤ maxYawKick). While latched: `SustainedContact.Resolve`. Both applied via `ApplyCollision`. | One impulse per contact (`ContactLatch`); a new impulse only after separation. Speed can never increase. |
| Effects | `SimulationManager.OnImpact` → sparks, camera shake, `EngineSynth.TriggerImpact`, telemetry event `impact:<severity>`, lap invalidated | Cosmetic only. |
| Godot collision shapes | `StaticBody3D` with the barrier faces (layer 2) | Used only by the chase camera's ray query, which keeps the camera in front of walls. |
| Recovery | `RecoverToTrack`: explicit reset at rest on the nearest centreline point, engine damage kept, lap invalid | The model has no reverse gear, so a head-on stop needs this. |

### 4.1 Converted mesh tracks (Monaco)

`TrackManager.MeshTracks` maps a name to a scene (`scenes/TrackMonaco.tscn`). Its root is a `MeshTrack` with:
- a `Visual` child: the rendered GLB;
- one hidden `StaticBody3D` per surface of the physics OBJ, built at load with trimesh (`ConcavePolygonShape3D`) shapes
  and no mesh instance;
- a `StartFinish` `Area3D`.

| Concern | Implementation |
|---|---|
| Reference line | `PolylineTrack` (core) from the AC racing line in `monaco_track.json`: s = 0 at the timing gate; track limits from the per-point edge distances. `TrackManager.Line` (`ITrackLine`) drives `LapTimer`, `RecoverToTrack`, the scripted driver and telemetry s/lateral. |
| Collision layers | WALL → layer 2 (`BarrierCollisionLayer`, also used by the camera ray), TARMAC → layer 3, KERB → layer 4, vehicle trigger → layer 5. Each body has `surface` and `friction` metadata. |
| Ground, surface, grade | `BeforeStep` → `MeshTrack.Sample`: 3 wheel rays (front axle centre, rear left, rear right) and a CG ray against layers 3–4. Gives height, normal, grade (→ `RoadGrade`, E6) and surface key → `surfaces.ini` friction (→ `SurfaceGrip`). Kerb wins if it lies on or above the tarmac (−0.15 m). A wheel off the mesh takes the mean height of the others. A CG ray that misses with all wheels on ground is a mesh seam (TARMAC); otherwise OFFTRACK. |
| Walls | `CollisionManager(MeshTrack.WallContact)`: footprint box (3.76 × 1.68 × 1.0 m, 0.7 m above the ground) `CollideShape` against layer 2 → `BarrierContactResult` → the same E3 response as on procedural tracks. |
| Rendering | `VehicleController.GroundPose` lifts and tilts the body onto the ground at the interpolated pose. This is a read-only query, so the physics does not depend on the frame rate (verified bit-identical at 30/60/144 fps). |
| Teleports | `VehicleController.GroundSnap` → `MeshTrack.SnapToGround`: a full-height `RayCast3D` from above the track's bounds down to below them, against tarmac/kerb. It places the car and re-centres the per-step ground search, so restart and recover never float or sink. |
| Start/finish | `Area3D` (box across the gate, mask layer 5) detects `Vehicle.tscn`'s `TriggerVolume` → telemetry event `sf_gate:lap=N`. `LapTimer` (step-exact) is authoritative. |

The visual GLB is never queried by physics. Its import (`tools/track_post_import.gd`) strips any collision, turns off
shadows on nodes tagged `__ns` (transparent, foliage, fences, small props), applies visibility ranges (`__vr<m>`) to
small props, and assigns the converted `materials/*.tres`. Godot generates LODs and compresses the meshes. The physics
OBJ imports uncompressed (16-bit position quantisation over a 1.5 km box would be about 2 cm).

### 4.2 Racing line (driver aid)

`SimulationManager.BuildRacingLine` runs on every track load and setup change. `ReferenceLap.Compute` (core,
vehicle_model §20) produces the profile, and `RacingLine` (a `MeshInstance3D`) draws it:
- a 0.55 m ribbon, sampled every 1 m, 6 cm above the collision surface;
- unshaded, vertex-coloured, no shadows;
- zone colours blended over ±3 m.

On mesh tracks the line is the AC racing line and follows the ground: each point searches around the previous height,
so the tunnel finds the road, not the roof above it. The grade along it feeds the profile. On procedural tracks it is
the centreline at y = 0. Toggle: L / D-pad up (`GameSession.RacingLineVisible`). It is a single draw call, about 6.6k
triangles on Monaco.

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
│   ├── track/       TrackManager, TrackMeshBuilder, TrackScenery, MeshTrack (converted tracks), RacingLine
│   ├── race/        LapTimer
│   ├── camera/      ChaseCamera (chase / bonnet / cockpit, barrier ray, shake)
│   ├── dynamics/    VehicleModelAdapter (model ↔ Godot)
│   ├── ui/          UiRoot, RaceHud, VehicleSetupPanel, ParameterCatalog, TelemetryPanel, TelemetryPlot,
│   │                RunComparisonPanel, UiTheme, UiFocus
│   ├── audio/       EngineSynth (pure C#), EngineAudio
│   ├── effects/     ImpactEffects
│   ├── menu/        MainMenu
│   └── dev/         SelfTest, CentrelineDriver, ScreenshotRig, PerfProbe
├── tools/   track_post_import.gd (GLB import script), write_input_map.gd
└── assets/  vehicles/mini_jcw (body, 4 wheels, steering wheel, textures, meta JSON), materials (procedural shaders),
             tracks/monaco (monaco_visual.glb, monaco_physics.obj, monaco_track.json, textures/, materials/;
             ac_source/ = the original AC files, ignored by Godot and git)
```

## 9. Extending

- **New track:** add a factory to `TrackLibrary` (segments plus closure) and its name to `TrackManager.Available` and
  `TrackManager.Create`. Meshes, surfaces, barriers, scenery, lap timing and the self-test follow from the geometry.
  Keep inner barrier offsets below the corner radius, and keep non-adjacent parts of the track further apart than the
  sum of their barrier offsets, so projection stays unambiguous.
- **New converted track:** run `tools/asset_pipeline/kn5_convert.py` on the AC folder, set the GLB's import script to
  `res://tools/track_post_import.gd` (copy `monaco_visual.glb.import`), make a scene like `TrackMonaco.tscn`, and add it
  to `TrackManager.Available` and `MeshTracks`.
- **New parameter:**
  1. Add the property to `VehicleParameters`, under the right group, with a range in `Validate()`.
  2. Use it in the equations.
  3. Add a `ParameterCatalog` entry with a range inside `Validate()` and a source tag.

  JSON configs and CSV metadata pick it up by reflection.
- **New telemetry channel:** add it to `TelemetrySample`, `SimulationRecorder.Columns`/`WriteCsv` and DESIGN_CONTRACT §6.
