# ROS 2 Foxy version (cross-compiled for the DRIVE AGX Orin)

This folder is a ROS 2 **Foxy** port of the ROS1 Noetic workspace in `src/drive_yolo`.
Foxy is used because the DRIVE OS 6.0.x targetfs is Ubuntu 20.04, and Foxy is the
ROS 2 release built for 20.04.

The nodes, topics, parameters and defaults are the same as the ROS1 version, so the
rest of the main README (architecture, node descriptions, TensorRT engine notes)
still applies. Only the build, the parameter syntax and the launch files change.

```
ros2_foxy/
├── Toolchain-V5L-foxy.cmake        # CMake cross toolchain (Foxy version of Toolchain-V5L.cmake)
├── cross_compile_foxy.sh           # colcon cross build (Foxy version of cross_compile.sh)
├── tools/fix_sysroot_cmake_paths.py
└── src/
    ├── drive_yolo_msgs/            # the .msg files (ROS 2 wants them in their own package)
    └── drive_yolo/                 # nodes, python scripts, launch/*.launch.py, config/*.yaml, rviz/
        └── models -> ../../../src/drive_yolo/models   (shared with the ROS1 version)
```

## ROS1 → ROS 2, what changed

If you know the ROS1 version, most of the port is a mechanical translation. In
Python terms: in ROS1 `ros::NodeHandle` was a global "session" that every class
talked to. In ROS 2 each node *is* an object (it inherits from `rclcpp::Node`),
and publishers, timers and parameters are methods on that object.

| ROS1 | ROS 2 Foxy |
|---|---|
| `drive_yolo/Detections` msg | `drive_yolo_msgs/msg/Detections` (separate package) |
| `nh.param("x", var, default)` | `var = declare_parameter("x", default)`. Parameters must be declared, like `argparse.add_argument()` before `parse_args()` |
| `ros::Publisher` / `pub.publish(msg)` | `rclcpp::Publisher<T>::SharedPtr` / `pub->publish(msg)` |
| `ros::Time::now()` / `.toSec()` | `now()` / `.seconds()` |
| `rosrun drive_yolo X` | `ros2 run drive_yolo X` |
| `roslaunch drive_yolo X.launch` | `ros2 launch drive_yolo X.launch.py` |
| XML launch + `<rosparam>` | Python launch files + `config/*.yaml` |
| rviz | rviz2 (the `.rviz` configs are converted) |

Things to be aware of:

* **Parameters are typed.** A parameter declared as a float must get a float:
  `speed_limit_ms:=15.0`, not `speed_limit_ms:=15`. Otherwise the node exits with
  `parameter 'speed_limit_ms' has invalid type: expected [double] got [integer]`.
* **Static objects** (`static_object_detector`): ROS 2 has no list-of-dicts
  parameters, so objects are defined as a list of labels plus one group per label.
  See `config/static_objects.yaml`.
* **Parameter YAML files** use the `/**:` key instead of the node name. In Foxy,
  node-name keys always win over the parameters that launch files pass, so with the
  node name as key the launch arguments were silently ignored.
* Point cloud subscriptions use the ROS 2 "sensor data" QoS (best effort). This
  works with both best-effort and reliable LiDAR drivers.
* Executable names are unchanged (`drive_yolo_node`, `drive_yolo_lidar_fusion_node`,
  ...). The ROS1 `drive_yolo_node_node` in some launch files was a typo.
* `tracking_node.cpp` / `object_tracker.cpp` were not in the ROS1 `CMakeLists.txt`, so
  they are not ported. For the same reason `complete_system.launch` (uses
  `tracking_node`) and `3dtracking_pipeline.launch` (duplicate of
  `3d_tracking_pipeline.launch` with parameters no node reads) are not ported.
* Two small fixes that the ROS1 code doesn't have:
  * The fusion node skips a frame when no LiDAR point is left after filtering.
    In ROS1, `cv::projectPoints` throws in that case and the node dies (e.g. an
    empty road inside the ROI).
  * The projector ignores empty clouds.

### Launch files

| ROS1 (`src/drive_yolo/launch`) | ROS 2 (`ros2_foxy/src/drive_yolo/launch`) |
|---|---|
| yolo_driveworks.launch | yolo_driveworks.launch.py |
| yolo_ros_camera.launch | yolo_ros_camera.launch.py |
| lidar_camera_fusion.launch | lidar_camera_fusion.launch.py (+ `config/lidar_camera_fusion.yaml`) |
| lidar_camera_fusion_with_sync.launch | lidar_camera_fusion_with_sync.launch.py |
| M8VIL_TFs.launch | M8VIL_TFs.launch.py |
| motpy_tracking.launch | motpy_tracking.launch.py |
| LidarandCamerapipeline.launch | LidarandCamerapipeline.launch.py |
| complete_detection_pipeline.launch | complete_detection_pipeline.launch.py |
| 3d_tracking_pipeline.launch | 3d_tracking_pipeline.launch.py |
| static_object_detector.launch | static_object_detector.launch.py (+ `config/static_objects.yaml`) |
| sync_diagnostics.launch | sync_diagnostics.launch.py |
| pointcloud_delay_demo.launch | pointcloud_delay_demo.launch.py |
| pointcloud_camera_overlay.launch | pointcloud_camera_overlay.launch.py |
| modular_fusion_pipeline.launch | modular_fusion_pipeline.launch.py |
| udp_vehicle_position.launch | udp_vehicle_position.launch.py |
| osi2ros_lidar.launch | osi2ros_lidar.launch.py |
| detection_visualizer.launch | detection_visualizer.launch.py |
| tracking_with_rviz.launch | tracking_with_rviz.launch.py |
| tracking_visualizer_only.launch | tracking_visualizer_only.launch.py |

Launch arguments work like before, e.g.
```bash
ros2 launch drive_yolo lidar_camera_fusion_with_sync.launch.py time_offset_ms:=45.3
ros2 launch drive_yolo lidar_camera_fusion.launch.py lidar_topic:=/invz_summation_reflection_delayed
```

## Installation

The flow is the same as for ROS1: steps 1 and 2 of the main README (DRIVE OS docker
and `target_fs`) are unchanged. Instead of ROS Noetic, install ROS 2 Foxy in the
targetfs, then build with `cross_compile_foxy.sh`.

### 1. Install ROS 2 Foxy in the target filesystem

Enter the chroot exactly like in the main README (mount `/dev`, `/proc`, `/sys`,
`chroot ${SYSROOT} /bin/bash`, and add a DNS server to `/etc/resolv.conf`), then:

```bash
apt update
apt install -y aptitude software-properties-common curl gnupg2 lsb-release
add-apt-repository universe

# ROS 2 apt repository (Foxy is EOL but its packages are still on packages.ros.org)
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list
apt update

# ROS 2 + everything drive_yolo needs
apt install -y ros-foxy-ros-base ros-foxy-cv-bridge ros-foxy-message-filters \
  ros-foxy-tf2-ros ros-foxy-tf2-geometry-msgs ros-foxy-visualization-msgs \
  ros-foxy-nav-msgs ros-foxy-rosidl-default-generators \
  libopencv-dev python3-dev python3-numpy
```
If apt complains about `libboost-*` (pulled in by cv_bridge) and the Mellanox
`ibverbs`/`rdma` packages, use `aptitude install libboost-all-dev` and take the
"downgrade ibverbs-providers ..." solution, like in the ROS1 guide.

The python tracker also needs its pip packages on the Orin (see step 4).

Leave the chroot and unmount (`exit`, `umount ${SYSROOT}/dev ...`).

### 2. Prepare the sysroot on the host

```bash
export SYSROOT=~/nvidia_ws/target_fs
cd ~/nvidia_ws/DRIVE_AGX_ORIN_ONNX_ROS_integration

# absolute symlinks -> relative (same script as for ROS1, the sysroot is now an argument)
python3 fix_symlinks.py $SYSROOT
```
Copy TensorRT, CUDA, DriveWorks and the NvMedia headers from the docker into the
targetfs, exactly as in the ROS1 README (`docker cp cross_compile:/usr/include/aarch64-linux-gnu/ ...`
and the `ln -s` for DriveWorks and `nvmedia_image.h`).

Then fix the absolute paths in the CMake files of the targetfs:
```bash
python3 ros2_foxy/tools/fix_sysroot_cmake_paths.py $SYSROOT
```
This is the automated version of the ROS1 manual edit of `tf2_geometry_msgsConfig.cmake`.
Packages installed in the targetfs export paths like `/usr/include/eigen3` or
`/usr/lib/aarch64-linux-gnu/libtinyxml2.so` (fastrtps). When cross-compiling those
point to the host, so the script prefixes them with `${CMAKE_SYSROOT}`. The originals
are kept as `*.backup`, and `--restore` puts them back. This also replaces the ROS1
workarounds of copying `$SYSROOT/usr/lib/aarch64-linux-gnu` and `opencv4` into the
host's `/usr`. Re-run it after installing more packages in the targetfs.

### 3. Cross-compile

Host tools (Ubuntu 20.04, e.g. inside the DRIVE OS docker):
```bash
sudo apt install g++-aarch64-linux-gnu cmake python3-pip python3-numpy \
  python3-empy python3-lark-parser python3-catkin-pkg
pip3 install colcon-common-extensions
```
The host must be Ubuntu 20.04 because the ROS 2 message generators run with the
host's `python3`, which has to match the Orin's Python 3.8. It builds the message
Python bindings for the Orin.

```bash
export SYSROOT=~/nvidia_ws/target_fs
./ros2_foxy/cross_compile_foxy.sh
```
The result is in `ros2_foxy/install`. Extra arguments go to colcon, e.g.
`./ros2_foxy/cross_compile_foxy.sh --packages-select drive_yolo`.

`cross_compile_foxy.sh` does **not** source `$SYSROOT/opt/ros/foxy/setup.bash`: that
would put the Orin's aarch64 programs in the host `PATH`. It only exports what the
build needs (`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `PYTHONPATH`).

To build only the ROS side, without TensorRT (e.g. on a PC to test the fusion and
tracking), pass `-DDRIVE_YOLO_BUILD_INFERENCE=OFF` to CMake:
```bash
source /opt/ros/foxy/setup.bash
cd ros2_foxy && colcon build --cmake-args -DDRIVE_YOLO_BUILD_INFERENCE=OFF
```

### 4. Setup on the target (Orin)

```bash
scp -r ros2_foxy/install nvidia@<target-ip>:~/drive_yolo_foxy
```
On the Orin, install the ROS 2 Foxy runtime with the same `apt install` line as in
step 1 (or use the same targetfs image), and the python dependencies:
```bash
pip3 install -r ~/drive_yolo_foxy/share/drive_yolo/requirements.txt
```
Run:
```bash
source /opt/ros/foxy/setup.bash
source ~/drive_yolo_foxy/local_setup.bash
ros2 launch drive_yolo yolo_driveworks.launch.py
# or the whole thing (python tracker + YOLO + vehicle TFs + fusion)
ros2 launch drive_yolo LidarandCamerapipeline.launch.py
```
Same as ROS1: the TensorRT `.engine` is DRIVE OS specific. Rebuild it from the
`.onnx` with `trtexec` on the Orin if you use a different DRIVE OS version.

If ROS1 and ROS 2 are both installed on the Orin, don't source both in the same terminal.

## How this was tested

* All C++ nodes (except the TensorRT one, see below) and the python scripts were
  built with ROS 2 Foxy and run on x86. They were fed synthetic LiDAR + detections
  through the launch files: fusion (TF and synchronization), C++ and python 3D
  trackers (estimated 4.98 m/s for a target moving at 5 m/s), speed monitor, sync
  diagnostics, point cloud delay/filter/projector, static object detector, UDP TF,
  UDP bridge, camera info, image publisher and both visualizers.
* `cross_compile_foxy.sh` + `Toolchain-V5L-foxy.cmake` + `fix_sysroot_cmake_paths.py`
  were tested on an Ubuntu 20.04 arm64 sysroot with the official ROS 2 Foxy arm64
  release and Ubuntu's `aarch64-linux-gnu-g++-9`. `drive_yolo_msgs`, `cv_bridge` and
  `drive_yolo` cross-compile. The aarch64 nodes and the aarch64 python message
  bindings were run under qemu.
* Not tested: the YOLO node itself, because there is no TensorRT/DriveWorks/Orin here.
  Its inference and DriveWorks code is unchanged from ROS1 except for the logging
  calls. The node was compile-checked against ROS 2 Foxy.
