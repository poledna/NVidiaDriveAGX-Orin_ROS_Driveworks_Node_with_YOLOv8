#!/bin/bash
# Prefix the absolute /usr/... and /lib/... paths in ROS' catkin CMake configs
# with ${CMAKE_SYSROOT}, so that when cross-compiling they resolve inside the
# sysroot instead of on the build host. Natively CMAKE_SYSROOT is empty and the
# paths are unchanged.
#
# Example (tf2_geometry_msgsConfig.cmake):
#   set(_include_dirs "include;/usr/include/eigen3")
# becomes
#   set(_include_dirs "include;${CMAKE_SYSROOT}/usr/include/eigen3")
#
# Usage: rewrite_ros_cmake_paths.sh <sysroot>   (safe to run more than once)
set -euo pipefail

ROOT="${1:?usage: $0 <sysroot>}"
ROS_SHARE="${ROOT%/}/opt/ros/noetic/share"

count=0
while IFS= read -r -d '' cfg; do
    # Only the include/library lists, and only paths that start a list entry
    # (after a quote or ';'), which is also what keeps this idempotent.
    if grep -qE '^\s*set\((_include_dirs|libraries) .*[";]/(usr|lib)/' "$cfg"; then
        sed -i -E '/^\s*set\((_include_dirs|libraries) /s#([";])/(usr|lib)/#\1${CMAKE_SYSROOT}/\2/#g' "$cfg"
        count=$((count + 1))
    fi
done < <(find "$ROS_SHARE" -path '*/cmake/*Config.cmake' -print0)

echo "Rewrote absolute paths in $count ROS CMake config files"
