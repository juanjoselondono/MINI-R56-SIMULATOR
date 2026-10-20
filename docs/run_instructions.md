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

## 2. Build and test the physics core (no Godot, no MATLAB)

```bash
dotnet build VehicleDynamics.sln          # core library, tests, validation runner
dotnet test VehicleDynamics.sln           # unit tests + golden-data checks against MATLAB output
```

The golden-data tests read `validation/golden/**`, which is committed, so they need no MATLAB.

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
$GODOT --path .            # run the game
$GODOT -e --path .         # open the editor (then F5 to run)
```

In the editor, the C# project builds automatically on run (Build button, top right).

### Command-line test modes

```bash
# Full throttle with auto-shift for N simulated seconds; prints the physical state and quits.
$GODOT --headless --fixed-fps 60 --path . -- --autodrive=16.89
```

`--fixed-fps 60` makes the run deterministic. The number of physics steps printed equals `N / dt` regardless of the
frame rate, which is how the fixed-step clock is checked.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `.NET: Failed to load hostfxr`, crash on start | `DOTNET_ROOT` not set in the shell that started Godot (see §1). |
| Shell hangs after a MATLAB command | stdin/stdout not redirected (see §3). |
| Log shows `llvmpipe` as the Vulkan device | Godot fell back to software rendering. Run from a desktop session with the NVIDIA driver active. To force the GPU on hybrid laptops: `__NV_PRIME_RENDER_OFFLOAD=1 __VK_LAYER_NV_optimus=NVIDIA_only $GODOT --path .` |
| Meshes missing or pink after cloning | the import step of §4 was not run. |
