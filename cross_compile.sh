#!/bin/bash
# Cross-compile this workspace for DRIVE AGX Orin.
#
#   ./cross_compile.sh                    build in the docker cross container (default)
#   ./cross_compile.sh --rebuild-image    rebuild the container image first
#   ./cross_compile.sh --shell            open a shell in the cross container
#   ./cross_compile.sh --export-sysroot D copy the container's sysroot to D (for IDEs)
#   ./cross_compile.sh --host             old flow: build on this machine against $SYSROOT
#
# Any other arguments are passed to catkin_make_isolated.
set -e

WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)
BUILD_DIR="$WORKSPACE_ROOT/build_isolated"
INSTALL_DIR="$WORKSPACE_ROOT/install_isolated"
DEVEL_DIR="$WORKSPACE_ROOT/devel_isolated"

IMAGE="${IMAGE:-drive-orin-cross:6.0.6}"
MODE=build
REBUILD=0
CATKIN_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --rebuild-image)  REBUILD=1 ;;
        --shell)          MODE=shell ;;
        --host)           MODE=host ;;
        --export-sysroot) MODE=export; EXPORT_DIR="${2:?--export-sysroot needs a directory}"; shift ;;
        *)                CATKIN_ARGS+=("$1") ;;
    esac
    shift
done

if [ "$MODE" = host ]; then
    SYSROOT="${SYSROOT:-$HOME/nvidia_ws/target_fs}"
    export SYSROOT
    echo "=== Cross-Compiling ROS Package on host (sysroot: $SYSROOT) ==="
    rm -rf "$BUILD_DIR" "$INSTALL_DIR" "$DEVEL_DIR"
    source "$SYSROOT/opt/ros/noetic/setup.bash"
    cd "$WORKSPACE_ROOT"
    catkin_make_isolated -DCMAKE_TOOLCHAIN_FILE="$WORKSPACE_ROOT/Toolchain-V5L.cmake" --install "${CATKIN_ARGS[@]}"
    echo "=== Cross-compilation complete ==="
    exit 0
fi

if [ "$REBUILD" = 1 ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    # The target stage runs arm64 binaries (apt inside the Orin rootfs).
    if [ ! -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
        echo "arm64 emulation is not set up. Run once (needs root):" >&2
        echo "  docker run --privileged --rm tonistiigi/binfmt --install arm64" >&2
        exit 1
    fi
    echo "=== Building $IMAGE (first time takes a while) ==="
    DOCKER_BUILDKIT=1 docker build --target cross -t "$IMAGE" \
        -f "$WORKSPACE_ROOT/docker/Dockerfile" "$WORKSPACE_ROOT"
fi

# Mount the workspace at the same path as on the host so generated paths match.
RUN=(docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp
     -v "$WORKSPACE_ROOT:$WORKSPACE_ROOT" -w "$WORKSPACE_ROOT")

case "$MODE" in
    shell)
        exec "${RUN[@]}" -it "$IMAGE" bash
        ;;
    export)
        mkdir -p "$EXPORT_DIR"
        cid=$(docker create "$IMAGE")
        trap 'docker rm "$cid" >/dev/null' EXIT
        docker cp "$cid:/sysroot/." "$EXPORT_DIR"
        echo "Sysroot exported to $EXPORT_DIR"
        ;;
    build)
        echo "=== Cross-Compiling ROS Package in $IMAGE ==="
        rm -rf "$BUILD_DIR" "$INSTALL_DIR" "$DEVEL_DIR"
        "${RUN[@]}" "$IMAGE" drive-cross-build "${CATKIN_ARGS[@]}"
        echo "=== Cross-compilation complete: $INSTALL_DIR ==="
        ;;
esac
