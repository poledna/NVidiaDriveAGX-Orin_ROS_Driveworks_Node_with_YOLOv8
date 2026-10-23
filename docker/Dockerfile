# syntax=docker/dockerfile:1.4
#
# Reproducible cross-compilation environment for DRIVE AGX Orin (DRIVE OS 6.0.6).
#
# This replaces the manual "docker cp targetfs / chroot / aptitude / docker cp
# TensorRT / fix symlinks / patch cmake / copy libs into host /usr" steps.
#
#   sdk     - NVIDIA DRIVE OS SDK image (x86). Source of targetfs and the
#             aarch64 TensorRT / DriveWorks / NvMedia files.
#   target  - the Orin root filesystem (arm64, runs under QEMU) with ROS Noetic
#             installed. This is what used to be done by hand in the chroot.
#   cross   - x86 Ubuntu 20.04 with the aarch64 gcc 9 cross compiler and the
#             target filesystem mounted at /sysroot. Builds run here at native
#             speed.
#
# Normally driven by ./cross_compile.sh, see the README.

ARG DRIVEOS_SDK_IMAGE=nvcr.io/drive/driveos-sdk/drive-agx-orin-linux-aarch64-sdk-build-x86:6.0.6.0-0004

FROM --platform=linux/amd64 ${DRIVEOS_SDK_IMAGE} AS sdk

# ---------------------------------------------------------------------------
# target: the Orin rootfs + ROS Noetic (arm64, emulated)
# ---------------------------------------------------------------------------
FROM --platform=linux/arm64 scratch AS target
COPY --from=sdk /drive/drive-linux/filesystem/targetfs/ /

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        software-properties-common ca-certificates curl gnupg \
 && add-apt-repository -y universe \
 && apt-get update \
 && apt-get upgrade -y

# targetfs ships Mellanox builds of the RDMA libraries, which conflict with
# Ubuntu's boost. This is the "downgrade ibverbs-providers / libibverbs1 /
# librdmacm1 / perftest" solution aptitude offers, done non-interactively.
COPY docker/apt-preferences-rdma /etc/apt/preferences.d/rdma-ubuntu
RUN apt-get install -y --allow-downgrades \
        ibverbs-providers libibverbs1 librdmacm1 perftest \
 && apt-get install -y libboost-all-dev

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
        > /etc/apt/sources.list.d/ros1.list \
 && apt-get update

# Only what drive_yolo needs to build. Override with --build-arg to get more,
# e.g. ROS_PACKAGES=ros-noetic-desktop-full
ARG ROS_PACKAGES="ros-noetic-ros-base ros-noetic-cv-bridge ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs ros-noetic-message-filters ros-noetic-common-msgs"
RUN apt-get install -y ${ROS_PACKAGES} \
        build-essential cmake libopencv-dev libeigen3-dev liblog4cxx-dev \
        libtinyxml-dev libtinyxml2-dev liblz4-dev libbz2-dev libpoco-dev \
        libconsole-bridge-dev \
 && rm -rf /var/lib/apt/lists/*

# Make ROS' installed CMake configs sysroot-aware (the tf2_geometry_msgs line 96
# fix, applied to every package). Is a no-op when CMAKE_SYSROOT is empty.
COPY docker/rewrite_ros_cmake_paths.sh /tmp/
RUN /tmp/rewrite_ros_cmake_paths.sh / && rm /tmp/rewrite_ros_cmake_paths.sh

# Symlinks the toolchain expects. Created before the NVIDIA files are copied
# so nothing below runs against the libraries copied in from the SDK image.
RUN cd /usr/local/driveworks \
 && { [ -e include ] || ln -s targets/aarch64-Linux/include include; } \
 && { [ -e lib ] || ln -s targets/aarch64-Linux/lib lib; } \
 && ln -sf nvmedia_6x/nvmedia_core.h /usr/include/nvmedia_image.h

# TensorRT, DriveWorks and NvMedia from the SDK image (not part of targetfs).
COPY --from=sdk /usr/include/aarch64-linux-gnu/ /usr/include/aarch64-linux-gnu/
COPY --from=sdk /usr/lib/aarch64-linux-gnu/ /usr/lib/aarch64-linux-gnu/
COPY --from=sdk /usr/local/driveworks/targets/aarch64-Linux/ /usr/local/driveworks/targets/aarch64-Linux/
COPY --from=sdk /drive/drive-linux/include/ /usr/include/

# ---------------------------------------------------------------------------
# cross: x86 build container with the target as /sysroot
# ---------------------------------------------------------------------------
# Ubuntu 20.04 on purpose: its aarch64 cross gcc is 9.x, the same as the
# target, so the binaries don't need a newer libstdc++ than the Orin has.
FROM --platform=linux/amd64 ubuntu:20.04 AS cross

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
        cmake make python3 python3-catkin-pkg python3-empy python3-rospkg \
 && rm -rf /var/lib/apt/lists/*

COPY --from=target / /sysroot/

# Absolute symlinks (/lib/foo.so -> /lib/...) would point into this x86
# container instead of the sysroot; make them relative.
COPY fix_symlinks.py /tmp/
RUN python3 /tmp/fix_symlinks.py /sysroot && rm /tmp/fix_symlinks.py

ENV SYSROOT=/sysroot
COPY docker/build.sh /usr/local/bin/drive-cross-build
CMD ["drive-cross-build"]
