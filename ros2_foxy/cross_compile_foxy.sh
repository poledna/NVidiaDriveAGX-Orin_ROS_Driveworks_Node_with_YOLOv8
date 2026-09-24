#!/bin/bash
# Cross-compile the ROS 2 Foxy workspace for the DRIVE AGX Orin.
#
# Usage (from anywhere):
#   export SYSROOT=~/nvidia_ws/target_fs    # targetfs with ROS 2 Foxy installed
#   ./cross_compile_foxy.sh [extra colcon args, e.g. --packages-select drive_yolo]
#
# The result is in ros2_foxy/install - copy it to the Orin (see README.md).
set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$WORKSPACE_ROOT/build"
INSTALL_DIR="$WORKSPACE_ROOT/install"
LOG_DIR="$WORKSPACE_ROOT/log"

export SYSROOT="${SYSROOT:-$HOME/nvidia_ws/target_fs}"
ROS2_ROOT="$SYSROOT/opt/ros/foxy"

echo "=== Cross-Compiling ROS 2 Foxy workspace ==="
echo "Sysroot: $SYSROOT"

if [ ! -f "$ROS2_ROOT/setup.bash" ]; then
    echo "ERROR: ROS 2 Foxy not found in $ROS2_ROOT (see README.md, 'Install ROS 2 Foxy in the target filesystem')"
    exit 1
fi
if ! command -v aarch64-linux-gnu-g++ > /dev/null; then
    echo "ERROR: aarch64-linux-gnu-g++ not found (sudo apt install g++-aarch64-linux-gnu)"
    exit 1
fi

# Do NOT source $ROS2_ROOT/setup.bash: it would put the aarch64 binaries and
# libraries of the targetfs in PATH / LD_LIBRARY_PATH of the host. Export
# only what the build needs:
#  - ament looks up packages (message generators, type supports) in AMENT_PREFIX_PATH
#  - CMake find_package() in CMAKE_PREFIX_PATH
#  - the code generators are pure python scripts inside the targetfs
export AMENT_PREFIX_PATH="$ROS2_ROOT"
export CMAKE_PREFIX_PATH="$ROS2_ROOT"
export PYTHONPATH="$ROS2_ROOT/lib/python3.8/site-packages${PYTHONPATH:+:$PYTHONPATH}"
unset COLCON_PREFIX_PATH
# Lets the toolchain find the packages built earlier in this workspace
export CROSS_INSTALL_BASE="$INSTALL_DIR"

# Clean previous builds
echo "Cleaning previous builds..."
rm -rf "$BUILD_DIR" "$INSTALL_DIR" "$LOG_DIR"

# Cross-compile
echo "Starting cross-compilation..."
cd "$WORKSPACE_ROOT"
colcon --log-base "$LOG_DIR" build \
    --base-paths "$WORKSPACE_ROOT/src" \
    --build-base "$BUILD_DIR" \
    --install-base "$INSTALL_DIR" \
    --merge-install \
    "$@" \
    --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE="$WORKSPACE_ROOT/Toolchain-V5L-foxy.cmake" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF

echo "=== Cross-compilation complete: $INSTALL_DIR ==="
