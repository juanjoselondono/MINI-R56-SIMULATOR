# R56 Vehicle Dynamics Simulator

A high-fidelity 3D driving simulator of a Mini Cooper R56 (N16) built in Godot 4 and C#. The core physics engine is a direct, bit-exact C# port of a rigorous MATLAB longitudinal and lateral vehicle dynamics model.

## 🧠 Architecture & Attribution

This project represents the convergence of pure mathematical modeling and AI-accelerated software engineering:

*   **Author:** Juan José Londoño David
*   **Co-Author:** Sebastian Palacios
*   **Academic Contributor:** Professor Jorge Hernán Córdoba Morales. The foundational mathematical model was developed thanks to his guidance and teachings in the *"Dinámica de vehículos y motores de automoción"* course.
*   **AI Co-Pilot:** Anthropic's Claude Code Opus 5.5 handled the heavy lifting of translating the mathematical equations into a fixed-step C# physics core, orchestrating the Godot 4 visualization pipeline, and optimizing asset toolchains.

## 🚀 Key Features

*   **Deterministic C# Physics Core:** Runs independent of the render loop at a fixed 60Hz step (0.005s). Includes engine PCHIP torque mapping, gear ratios, traction limits, aerodynamic drag, rolling resistance, and dynamic weight transfer.
*   **Dynamic Lateral Handling:** Upgraded single-track (bicycle) dynamic model utilizing precise vehicle mass (1150 kg) and yaw moment of inertia (I_zz = 1100 kg·m²) to simulate real-world understeer/oversteer characteristics and tire slip angles.
*   **High-Fidelity Track Integration:** Support for laser-scanned track data (e.g., Assetto Corsa's Monaco). Features dynamic surface friction coefficients reading directly from Godot's collision materials (Tarmac, Grass, Curbs).
*   **Real-Time Engineering Telemetry:** Live dashboard exposing 42 physical parameters, live plotting, and CSV export for post-run data analysis.
*   **XInput Controller Support:** Native analog steering, throttle, and braking integration (tested with Logitech F710).
*   **Vulkan Forward+ Rendering:** Optimized visual pipeline using Godot 4 with dynamic shadows, PBR materials, and SDFGI global illumination, while keeping the physics collision meshes lightweight.

## 📈 MATLAB Validation

The C# physics core is continually validated against the original MATLAB reference model. The simulator features an automated testing suite that executes identical scenarios in both environments, ensuring a maximum relative error of `0` across 9 distinct kinetic and kinematic scenarios.

## 🛠️ Run Instructions & Asset Disclaimer

**Note on Track Assets:** To respect copyright, high-fidelity .kn5 track models (such as Monaco from Assetto Corsa) are **not** included in this repository. Users who own Assetto Corsa can use a Python KN5 extraction tool to convert their visual and physics files into .glb and .obj formats for use in the `assets/tracks/` directory.

### Quick Start
```bash
# Ensure Godot 4.7.x and .NET 8 SDK are installed
export PATH=$HOME/.dotnet:$PATH DOTNET_ROOT=$HOME/.dotnet
export GODOT=~/Applications/godot/Godot_v4.7.2-stable_mono_linux_x86_64/Godot_v4.7.2-stable_mono_linux.x86_64

# Run the core logic validation tests
dotnet test VehicleDynamics.sln

# Build and launch with hardware acceleration
cd VehicleDynamicsSimulator 
dotnet build -c Release
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia $GODOT --path .