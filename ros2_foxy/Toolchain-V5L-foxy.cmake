# Cross-compilation toolchain for ROS 2 Foxy on NVIDIA DRIVE AGX Orin
# (aarch64, DRIVE OS 6.0.x targetfs = Ubuntu 20.04) with CUDA, TensorRT and DriveWorks.
#
# Used by cross_compile_foxy.sh, which also exports the environment ROS 2 needs
# (AMENT_PREFIX_PATH, PYTHONPATH, ...). The host must be Ubuntu 20.04 (the
# DRIVE OS docker is), because the ROS 2 code generators run with the host
# python3, which has to be the same version as the target's (3.8).
cmake_minimum_required(VERSION 3.10)

# Target system configuration
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

# Paths - the targetfs can be given with the SYSROOT environment variable
if(DEFINED ENV{SYSROOT})
  set(SYSROOT "$ENV{SYSROOT}")
else()
  set(SYSROOT "$ENV{HOME}/nvidia_ws/target_fs")
endif()
set(ROS2_ROOT "${SYSROOT}/opt/ros/foxy")
set(DRIVEWORKS_ROOT "${SYSROOT}/usr/local/driveworks")
set(CUDA_ROOT "${SYSROOT}/usr/local/cuda-11.4")
set(TENSORRT_ROOT "${SYSROOT}/usr")

if(NOT EXISTS "${ROS2_ROOT}/setup.bash")
  message(FATAL_ERROR "ROS 2 Foxy not found in the targetfs: ${ROS2_ROOT}")
endif()

# Cross-compilation settings
set(CMAKE_SYSROOT "${SYSROOT}")

# Search the targetfs, plus the colcon install space of this workspace (it is
# on the host) so packages built earlier in the same build are found too.
set(CMAKE_FIND_ROOT_PATH "${SYSROOT}")
if(DEFINED ENV{CROSS_INSTALL_BASE})
  list(APPEND CMAKE_FIND_ROOT_PATH "$ENV{CROSS_INSTALL_BASE}")
endif()
set(CMAKE_PREFIX_PATH "${ROS2_ROOT}")

# Compilers
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Search modes - programs from the host, everything else from the target
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Python: the interpreter runs on the host (ROS 2 code generators), but the
# message python bindings (C extensions) are built against the TARGET python.
set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Host python (same version as the target)")
set(PYTHON_INCLUDE_DIR "${SYSROOT}/usr/include/python3.8" CACHE PATH "Target python headers")
set(PYTHON_LIBRARY "${SYSROOT}/usr/lib/aarch64-linux-gnu/libpython3.8.so" CACHE FILEPATH "Target python library")
set(PYTHON_SOABI "cpython-38-aarch64-linux-gnu" CACHE STRING "Target python extension suffix")
# rosidl_generator_py asks the host numpy for its headers - put the target's
# numpy headers first so the bindings match the numpy on the Orin
include_directories(BEFORE SYSTEM "${SYSROOT}/usr/lib/python3/dist-packages/numpy/core/include")

# Library search paths - build time
set(LIB_PATHS
  "${SYSROOT}/usr/lib/aarch64-linux-gnu"
  "${SYSROOT}/lib/aarch64-linux-gnu"
  "${ROS2_ROOT}/lib"
  "${DRIVEWORKS_ROOT}/targets/aarch64-Linux/lib"
  "${CUDA_ROOT}/targets/aarch64-linux/lib"
  "${TENSORRT_ROOT}/lib/aarch64-linux-gnu"
)

# Runtime library paths (rpath) - these are the paths on the target system
set(RUNTIME_PATHS
  "/usr/lib/aarch64-linux-gnu"
  "/lib/aarch64-linux-gnu"
  "/usr/local/driveworks/targets/aarch64-Linux/lib"
  "/usr/local/cuda-11.4/targets/aarch64-linux/lib"
)

# -rpath-link: lets the linker resolve the dependencies of the shared
# libraries we link (librclcpp -> librcl -> ...) inside the targetfs
string(REPLACE ";" " -Wl,-rpath-link," RPATH_LINK_FLAGS "${LIB_PATHS}")
set(RPATH_LINK_FLAGS "-Wl,-rpath-link,${RPATH_LINK_FLAGS}")

string(REPLACE ";" " -Wl,-rpath," RPATH_FLAGS "${RUNTIME_PATHS}")
set(RPATH_FLAGS "-Wl,-rpath,${RPATH_FLAGS}")

# Linker flags - unified for all link types. --allow-shlib-undefined as in the
# ROS1 toolchain: the DriveWorks/NvMedia libraries reference driver libraries
# that only exist on the real Orin.
set(COMMON_LINK_FLAGS "${RPATH_LINK_FLAGS} ${RPATH_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${COMMON_LINK_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${COMMON_LINK_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${COMMON_LINK_FLAGS}")

# Include directories - system includes needed by DriveWorks / TensorRT
include_directories(BEFORE SYSTEM
  "${SYSROOT}/usr/include/aarch64-linux-gnu"
  "${DRIVEWORKS_ROOT}/include"
  "${SYSROOT}/usr/include/nvmedia_6x"
  "${SYSROOT}/usr/include/nvsiplclient"  # NvSIPL headers for DRIVE OS 6.x
  "${SYSROOT}/drive/drive-linux/include"  # Additional DRIVE OS headers
  "${CUDA_ROOT}/targets/aarch64-linux/include"
  "${TENSORRT_ROOT}/include/aarch64-linux-gnu"
)

set(Boost_NO_SYSTEM_PATHS ON)
set(BOOST_ROOT "${SYSROOT}/usr")
set(BOOST_INCLUDEDIR "${SYSROOT}/usr/include")
set(BOOST_LIBRARYDIR "${SYSROOT}/usr/lib/aarch64-linux-gnu")

message(STATUS "Cross-compilation toolchain (ROS 2 Foxy) loaded:")
message(STATUS "  Target: ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}")
message(STATUS "  Sysroot: ${CMAKE_SYSROOT}")
message(STATUS "  ROS 2: ${ROS2_ROOT}")
message(STATUS "  Compilers: ${CMAKE_C_COMPILER}, ${CMAKE_CXX_COMPILER}")
message(STATUS "  TensorRT root: ${TENSORRT_ROOT}")
