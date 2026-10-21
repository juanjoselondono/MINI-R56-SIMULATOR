
#!/usr/bin/env bash

set -e

cd "$HOME/Documents/vehicleDynamics/game/VehicleDynamicsSimulator"

export PATH="$HOME/.dotnet:$PATH"
export DOTNET_ROOT="$HOME/.dotnet"

export GODOT="$HOME/Applications/godot/Godot_v4.7.2-stable_mono_linux_x86_64/Godot_v4.7.2-stable_mono_linux.x86_64"

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
"$GODOT" --path .
