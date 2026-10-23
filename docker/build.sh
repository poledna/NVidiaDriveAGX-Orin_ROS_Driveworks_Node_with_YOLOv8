#!/bin/bash
# Runs inside the cross container: build the workspace in the current directory
# for aarch64. Extra arguments are passed to catkin_make_isolated.
set -eo pipefail

source "$SYSROOT/opt/ros/noetic/setup.bash"

catkin_make_isolated --install \
    -DCMAKE_TOOLCHAIN_FILE="$PWD/Toolchain-V5L.cmake" \
    "$@"
