# Run instructions

All commands are run from the repository root (`game/`) on Linux unless stated otherwise.

## 1. Requirements

| Component | Version used | Required for |
|---|---|---|
| Godot Engine, **.NET edition** | 4.7.2-stable (`Godot_v4.7.2-stable_mono_linux_x86_64`) | running the simulator |
| .NET SDK | 8.0.426 (any 8.0.x SDK) | building the core, the tests and the Godot C# project |
| MATLAB | R2026a | **optional**: only to regenerate golden data or run `compare_results.m` |
| Python 3 + numpy/matplotlib | 3.x, numpy 1.26, matplotlib 3.6 | optional: asset pipeline and the convergence plot |

On this machine the SDK and Godot were installed user-locally:

```bash
# .NET 8 SDK into ~/.dotnet
curl -sL https://dot.net/v1/dotnet-install.sh -o dotnet-install.sh
bash dotnet-install.sh --channel 8.0 --install-dir ~/.dotnet

# Godot 4.7.2 .NET into ~/Applications/godot
mkdir -p ~/Applications/godot && cd ~/Applications/godot
curl -sLO https://github.com/godotengine/godot/releases/download/4.7.2-stable/Godot_v4.7.2-stable_mono_linux_x86_64.zip
unzip Godot_v4.7.2-stable_mono_linux_x86_64.zip
```

Every shell that builds or runs anything needs the SDK on its path. **Godot .NET finds the runtime through
`DOTNET_ROOT`**; without it Godot aborts with `.NET: Failed to load hostfxr`:

```bash
export PATH=$HOME/.dotnet:$PATH DOTNET_ROOT=$HOME/.dotnet
export GODOT=~/Applications/godot/Godot_v4.7.2-stable_mono_linux_x86_64/Godot_v4.7.2-stable_mono_linux.x86_64
```

Put both lines in `~/.bashrc` if you launch the editor from a desktop shortcut.

## 2. Build and test (no Godot, no MATLAB)

```bash
dotnet build VehicleDynamics.sln          # core library, both test projects, validation runner
dotnet test VehicleDynamics.sln           # 107 core tests (incl. golden MATLAB data) + 11 game-logic tests
```

- `tests/VehicleDynamics.Tests`: the physics core. The golden-data tests read `validation/golden/**`, which is
  committed, so they need no MATLAB.
- `tests/GameLogic.Tests`: compiles the Godot-free game scripts (lap timer, collision manager, engine synth, scripted
  driver) without Godot.

## 3. Validation against MATLAB

```bash
# C# outputs for every scenario of validation/scenarios.json -> validation/out/csharp/
dotnet run --project tools/ValidationRunner

# MATLAB comparison -> validation/out/comparison.{json,md}, validation/plots/*.png
(cd validation/matlab && matlab -batch "compare_results" < /dev/null > ../out/compare.log 2>&1)

# Only if model.m or scenarios.json changed: regenerate golden data from the unmodified model.m
(cd validation/matlab && matlab -batch "run_reference()" < /dev/null > ../out/run_reference.log 2>&1)
```

Always redirect MATLAB's stdin from `/dev/null` and its output to a file. Otherwise a MATLAB helper process keeps the
pipe open and the shell hangs after MATLAB has finished. Results and tolerances are explained in
`docs/validation_report.md`.

## 4. Simulator (Godot project)

### First import

`.godot/` (Godot's import cache) is not under version control. Import once after cloning:

```bash
cd VehicleDynamicsSimulator
dotnet build
$GODOT --headless --import --path .
```

### Launch

```bash
cd VehicleDynamicsSimulator
$GODOT --path .            # run the game (main menu)
$GODOT -e --path .         # open the editor, then F5 to run
```

In the editor, the C# project builds automatically on run (Build button, top right). The main menu lets you choose:
- **Track:** *Circuit* (2596 m: hairpin, sweepers, chicane), *Oval* (model.m's 2 × 400 m + R80 oval, 1302.7 m) or
  *Monaco* (converted Assetto Corsa track, 3287 m racing line, about 42 m of elevation change). On Monaco the car starts
  on the pole slot of the AC grid, behind the line.
- **Vehicle configuration:** a saved configuration, or the model.m defaults.

The car starts at rest on the start/finish line. The lap clock starts when the car first moves.

### Controls

| Key | Gamepad | Action |
|---|---|---|
| W / ↑ | RT (analog) | throttle |
| S / ↓ | LT (analog) | brake |
| A D / ← → | left stick X (analog) | steer. Stick response is \|x\|^1.5 for precision near centre. A speed-sensitive input assist divides the command by 1 + (V/25 m/s)²; the model still clamps δ to δ_max. |
| E / Q | RB / LB | shift up / down (manual) |
| T | – | toggle auto-shift (model.m gear rule: lowest gear whose 6500-rpm speed ≥ V) |
| R | Back (Select) | restart: grid, at rest, laps cleared, engine and transmission repaired |
| Backspace | B | recover to the nearest centreline point at rest. The lap becomes invalid and engine or transmission damage is kept. |
| X | X | reverse on/off (Getrag G255 / GS6-55BG reverse, 3.143). It engages up to 10 km/h against the direction of travel; above that the transmission is destroyed (no torque, "Transmission Damaged! Press R to restart"). R / Back (Select) restarts and repairs. Needs the dynamic lateral model (default). |
| P | Start | pause / resume the simulation clock (shift presses while paused are ignored) |
| C | Y | cycle camera: chase → bonnet → cockpit (driver's eye, through the glass) |
| H | – | headlights |
| L | A, D-pad up | racing line on/off: green = accelerate, yellow = coast/transition, red = heavy braking (reference lap for the current setup). The source is chosen in the setup panel (Tab): optimised minimum-curvature line (default), the track's reference line, or the best lap of a telemetry CSV. |
| F5 | – | start / stop telemetry recording (the pad's X is now reverse) |
| F6 | – | graphics preset: Ultra (native + TAA, default) → High (FSR 2.2 Quality) → Performance (FSR 2.2 Balanced) |
| Tab | – | vehicle setup panel |
| F2 | – | telemetry dashboard |
| F3 | – | run comparison |
| F1 | – | controls help |
| Esc | – | back to the main menu (unexported recordings of the session are discarded) |

While a text or number field has focus, driving keys and shortcuts are ignored. Press Enter to leave the field.

### Gamepad (Logitech F710 and other XInput pads)

- **Mode switch.** Set the F710's switch on the back to **X** (XInput). Godot then maps it to its standard layout, and a
  "Gamepad connected" message appears in the game. In D (DirectInput) mode the pad has no standard mapping, and the game
  tells you to switch.
- **Analog inputs.** Triggers and stick are passed to the model directly. The keyboard's rate limits (ramps) are applied
  only while the last input came from the keyboard; touching a key switches back.
- **Deadzones.** 0.05 on the triggers and 0.10 on the stick.
- **Remapping.** All bindings live in `project.godot`'s Input Map (Project → Project Settings → Input Map in the editor).
  To regenerate the defaults:
  `$GODOT --headless --path VehicleDynamicsSimulator --script res://tools/write_input_map.gd`.

### Panels

- **Vehicle setup (Tab).** Every exposed parameter has a slider, a number field, its unit, range, default, a reset
  button and a source tag:
  - *model.m*: from the original MATLAB model;
  - *JS seed*: a rule from the seed game;
  - *Ext. E1/E2/E3*: an extension, provisional value.

  Edits go to a working copy that is validated live; **Apply Setup** is disabled while it is invalid. Below the
  buttons the panel shows derived values (R_w, K_φ, F_x,max, gear top speeds).

  The buttons are Apply Setup, Reset Parameters, Revert Edits, Reset Vehicle (recover to track), Pause/Resume and
  Restart Simulation. Save/Load Configuration reads and writes `user://configs/*.json`. Saving stores the *applied*
  parameters, and loading fills the editor; press Apply to use what you loaded.
- **Telemetry (F2).**
  - Live values of every available quantity, with units. Extension-derived values are tagged [E1]/[E2]/[E3], and
    quantities the model does not provide are listed as unavailable.
  - Plots against time or distance: speed, rpm, accelerations, inputs, grip usage/slip, and a g-g diagram.
  - Buttons: Record/Stop (F5), Discard Recording and Export CSV.
- **Run comparison (F3).** Select two or more recorded runs to overlay speed against distance or time, measured from
  each run's start. It also shows max |Δ| and RMSE of speed, rpm and a_x against the first selected run, and lists
  the parameters that differ.

### Output files

| What | Linux path |
|---|---|
| Telemetry CSV + parameter JSON | `~/.local/share/godot/app_userdata/R56 Vehicle Dynamics Simulator/runs/` |
| Saved configurations | `~/.local/share/godot/app_userdata/R56 Vehicle Dynamics Simulator/configs/` |

The CSV columns are those of `docs/DESIGN_CONTRACT.md` §6. The `# key=value` header holds the track, start time,
sample interval and every parameter. Events (shifts, impacts, laps, restarts, recoveries, parameter changes, engine
failure) are in the `event` column.

### Command-line test and dev modes

These go after `--`. They skip the main menu.

```bash
# In-game self-test on all tracks (48 checks), deterministic; exit code 0 = all pass
$GODOT --headless --fixed-fps 60 --path . -- --selftest
# Only the Monaco suite (11 checks), e.g. after re-converting the track
$GODOT --headless --fixed-fps 60 --path . -- --selftest --selftest-only=Monaco

# Full throttle with auto-shift for N simulated seconds; prints the physical state and quits.
# The printed state is bit-identical for any --fixed-fps (frame-rate independence check).
$GODOT --headless --fixed-fps 60 --path . -- --track=Oval --autodrive=16.89

# Scripted centreline driver (attract mode); combine with the options below
$GODOT --path . -- --track=Circuit --demo

# Screenshots at given simulated times, then quit (rendering run, not headless); --bonnet / --cockpit select the camera
$GODOT --path . --resolution 1600x900 -- --track=Circuit --demo --shots=/tmp/shots --shot-times=14,52

# Graphics preset at start: ultra (native + TAA, default), high (FSR 2.2 Quality), performance (FSR 2.2 Balanced)
$GODOT --path . -- --track=Monaco --gfx=high

# Average frame time and physics cost over N frames (also GPU time and dropped 60 Hz frames;
# add --no-vsync to see the real headroom instead of the 16.7 ms vsync cap)
$GODOT --path . -- --track=Circuit --demo --perf=300

# Racing line source: optimised (default), reference, or a telemetry CSV exported by the game (fastest valid lap)
$GODOT --path . -- --track=Monaco --racing-line=$HOME/.local/share/godot/app_userdata/R56\ Vehicle\ Dynamics\ Simulator/runs/my_lap.csv

# Open panels at start (screenshots): setup, telemetry, compare, help
$GODOT --path . -- --track=Oval --show-panels=setup,telemetry
```

### Vehicle model

The car is the "2011 Mini John Cooper Works (R56) GP" OBJ package in `2011-mini-john-cooper-works-r56/`. The converted
parts in `VehicleDynamicsSimulator/assets/vehicles/mini_jcw/` are generated by:

```bash
python3 -I tools/asset_pipeline/import_jcw_obj.py 2011-mini-john-cooper-works-r56 VehicleDynamicsSimulator/assets/vehicles/mini_jcw
```

The script scales the model so its wheelbase equals model.m's 2.467 m, rotates it to Godot's frame and puts the origin
under the CG. It splits out the four wheels and the steering wheel, cleans the materials, and writes the measured hubs,
footprint and steering axis to `mini_jcw_meta.json`. Run the Godot import step again after regenerating.

### Track conversion (Monaco)

The Assetto Corsa files are in `VehicleDynamicsSimulator/assets/tracks/monaco/ac_source/`. That folder has a
`.gdignore` and is not in git. The converted assets are generated by:

```bash
M=VehicleDynamicsSimulator/assets/tracks/monaco
python3 -I tools/asset_pipeline/kn5_convert.py $M/ac_source $M monaco    # ~5 s; needs numpy and Pillow
$GODOT --headless --import --path VehicleDynamicsSimulator
```

| Output | Content |
|---|---|
| `monaco_visual.glb` | `monaco.kn5`: 741 nodes, 940k triangles (meshes merged per material and 200 m cell) |
| `textures/`, `materials/` | 273 textures (DDS; 15 with incomplete mip chains re-encoded as PNG), 436 `StandardMaterial3D` |
| `monaco_physics.obj` | `monacophysics.kn5`: TARMAC 171,704 and WALL 3,424 triangles; KERB 14,929 triangles taken from the visual kerb meshes (the physics file has no kerb surface) |
| `monaco_track.json` | grid (22 slots, from the `AC_START_n` objects), timing gates, racing line with edge distances (`fast_lane.ai`), `surfaces.ini` friction |

The converter is deterministic: re-running it reproduces the same files.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `.NET: Failed to load hostfxr`, crash on start | `DOTNET_ROOT` not set in the shell that started Godot (see §1). |
| Shell hangs after a MATLAB command | stdin/stdout not redirected (see §3). |
| Log shows `llvmpipe` as the Vulkan device | Godot fell back to software rendering. Run from a desktop session with the NVIDIA driver active. To force the GPU on hybrid laptops: `__NV_PRIME_RENDER_OFFLOAD=1 __VK_LAYER_NV_optimus=NVIDIA_only $GODOT --path .` |
| Meshes missing or pink after cloning | the import step of §4 was not run. |
