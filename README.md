# NVIDIA DRIVE AGX Orin ROS Integration with YOLO and LiDAR Fusion

A comprehensive ROS Noetic package for NVIDIA DRIVE AGX Orin that integrates YOLO object detection, LiDAR-camera fusion, and 3D object tracking with real-time visualization.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
  - [1. Prerequisites](#1-prerequisites)
  - [2. Cross-Compile](#2-cross-compile)
  - [3. Setup on Target Device](#3-setup-on-target-device)
- [Architecture](#architecture)
- [ROS Nodes](#ros-nodes)
- [Usage](#usage)
- [Launch Files](#launch-files)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Overview

This project provides a complete perception pipeline for autonomous driving applications on NVIDIA DRIVE AGX Orin hardware. It combines:

- **YOLO Object Detection**: Real-time object detection using TensorRT-optimized YOLO models
- **LiDAR-Camera Fusion**: Sensor fusion to enrich 2D detections with accurate 3D distance information
- **3D Object Tracking**: Multi-object tracking with Kalman filtering and motion prediction
- **RViz Visualization**: Comprehensive visualization tools for debugging and analysis

## Features

### Object Detection
- TensorRT-accelerated YOLO inference (supports YOLOv8, YOLO11, etc.)
- DriveWorks camera integration for low-latency image acquisition
- Per-class confidence thresholds
- Hood filtering for vehicle-mounted cameras
- Quiet mode for production deployments

### LiDAR-Camera Fusion
- Camera-LiDAR calibration support
- Point cloud projection onto camera images
- Temporal synchronization diagnostics
- ROI filtering and ground plane removal
- Point clustering to filter noise and rain
- Distance confidence metrics based on LiDAR point density

### 3D Tracking
- Multi-object tracking with unique IDs
- Kalman filter-based motion prediction
- Velocity and acceleration estimation
- 1-second position prediction for path planning
- Track quality metrics (stability, confidence)
- Speed monitoring with configurable thresholds

### Visualization
- RViz integration with pre-configured views
- Velocity vector arrows
- Track ID labels with class and speed
- Position markers with class-based coloring
- Predicted future positions
- Pointcloud overlay on camera images

## System Requirements

### Hardware
- **NVIDIA DRIVE AGX Orin** (or compatible DRIVE platform)
- **LiDAR sensor** (tested with InnovizOne, supports any sensor with PointCloud2 output)
- **GMSL Camera** (tested with Entron F008A120RM0AV2)

### Software
- **NVIDIA DRIVE OS 6.0.6** or later
- **DriveWorks SDK 5.10** or later
- **CUDA 11.4** or later
- **TensorRT 8.x** or later
- **ROS Noetic** (cross-compiled for ARM64)
- **Ubuntu 20.04** (for cross-compilation host)

### Python Dependencies
```
motpy>=0.0.10
numpy>=1.19.0
scipy>=1.5.0
scikit-learn>=0.23.0
filterpy>=1.4.5
```

## Installation
Note as my objective ATM is just to crosscompile the system this doesnt need an NVIDIA GPU on host, im quite sure that also normally you wouldnt need it but just in case you were curious.

The whole sysroot setup (extract the target filesystem, install ROS Noetic in it, copy in TensorRT/DriveWorks/NvMedia, fix symlinks and CMake paths) is scripted in [`docker/Dockerfile`](docker/Dockerfile). The original manual walkthrough is in [docs/MANUAL_CROSS_COMPILE.md](docs/MANUAL_CROSS_COMPILE.md) if you want to see what each step does.

### 1. Prerequisites

- Docker with BuildKit (any recent Docker) and plenty of free disk space. The NVIDIA SDK image alone is tens of GB.
- Access to the NVIDIA DRIVE OS 6.0.6 SDK image on NGC ([NVIDIA install guide](https://developer.nvidia.com/docs/drive/drive-os/6.0.6/public/drive-os-linux-installation/common/topics/installation/docker-ngc/download-install-docker-linux-devzone.html)):
  ```bash
  docker login nvcr.io
  ```
- arm64 emulation, needed only while the image is built (it runs `apt` inside the Orin root filesystem). Run this once per boot:
  ```bash
  docker run --privileged --rm tonistiigi/binfmt --install arm64
  ```

You don't need the aarch64 cross compiler, ROS, or the Python requirements on your machine. They are all in the container.

### 2. Cross-Compile

```bash
git clone git@github.com:roadview-project/NVidiaDriveAGX-Orin_ROS_Driveworks_Node_with_YOLOv8.git DRIVE_AGX_ORIN_ONNX_ROS_integration
cd DRIVE_AGX_ORIN_ONNX_ROS_integration
./cross_compile.sh
```

The first run builds the `drive-orin-cross:6.0.6` image, which takes a while because ROS is installed under emulation. After that, builds run at native speed and the result is in `install_isolated/`.

| Command | What it does |
|---|---|
| `./cross_compile.sh` | Cross-compiles the workspace into `install_isolated/` |
| `./cross_compile.sh --rebuild-image` | Rebuilds the image first, for example after changing `docker/Dockerfile` |
| `./cross_compile.sh --shell` | Opens a shell in the cross container (the sysroot is at `/sysroot`) |
| `./cross_compile.sh --export-sysroot ~/nvidia_ws/target_fs` | Copies the sysroot out of the image, e.g. for IDE code completion |
| `./cross_compile.sh --host` | The old flow: builds on your machine against `$SYSROOT` (default `~/nvidia_ws/target_fs`) |

Anything else you pass goes to `catkin_make_isolated`, e.g. `./cross_compile.sh -DCMAKE_BUILD_TYPE=Release`.

**Other DRIVE OS versions**: pass a different SDK image, e.g. for 6.0.10:
```bash
docker build --target cross -t drive-orin-cross:6.0.10 -f docker/Dockerfile \
  --build-arg DRIVEOS_SDK_IMAGE=nvcr.io/drive/driveos-sdk/<6.0.10 image> .
IMAGE=drive-orin-cross:6.0.10 ./cross_compile.sh
```
You will probably also need to update the CUDA version (`cuda-11.4`) and `VIBRANTE_PDK_DECIMAL` in `Toolchain-V5L.cmake`.

**How it works**: the Dockerfile has three stages.
1. `sdk` is NVIDIA's SDK image. It provides the target filesystem and the aarch64 TensorRT/DriveWorks/NvMedia files.
2. `target` is that filesystem as an arm64 image. ROS is installed in it with plain `apt` (Docker handles DNS, and an apt pin replaces the interactive aptitude downgrade), so it doesn't need a chroot.
3. `cross` is an x86 Ubuntu 20.04 image with gcc 9 for aarch64, the same compiler version the target uses. The `target` filesystem is copied into it at `/sysroot`. ROS's CMake configs are rewritten to use `${CMAKE_SYSROOT}`, so nothing has to be copied into your host's `/usr`.

### 3. Setup on Target Device

#### Copy Files to Target

```bash
# Copy built packages to target
scp -r ~/drive-ros/catkin_ws/install_isolated nvidia@<target-ip>:~
```

#### Install Dependencies on Target
we need to install the requirements of python AND the ros 1 in the target device
to do that follow the ROS install steps from [docs/MANUAL_CROSS_COMPILE.md](docs/MANUAL_CROSS_COMPILE.md) (the same `apt` commands are in `docker/Dockerfile`)

```bash
# Setup ROS environment
source ~/install_isolated/setup.bash"
roslaunch drive_yolo yolo_driveworks.launch
```
this will run the code with the default camera parameters i put in when i was using. I was using the Entron Camera F008.

**IMPORTANT** the ENGINE is **drive OS speficfic**. that means that if you want to use it in other version you need to recreate the engine based on the onnx file.
to do that is simple simply use **ON THE DRIVE Orin**
```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=model.onnx \
  --saveEngine=model_fp16.engine \
  --fp16 \
  --workspace=4096 \
```

Also I added the YOLOV8 engine it should work with it. but there is also the V11. the issue is that theres a difference and the detections are mangled when you use the wrong version
I will add in the future a v11 branch, its already done i just need to push it.

This runs at 30fps with yolo detections! ~faster then the nvidia rosnode (runs at 22fps) hehe cheeky no?~


there are other nodes like sensor fusion but its 2.5D
so dont get to excited


This bottom part was AI generated thanks Claude for the help linking libraries and writing this comprehensive readme!
## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Data Sources                             │
├────────────────────┬────────────────────────────────────────────┤
│ DriveWorks Camera  │  LiDAR (PointCloud2)                       │
│  (GMSL/CSI)        │                                             │
└─────────┬──────────┴────────────────┬───────────────────────────┘
          │                           │
          v                           v
┌─────────────────────┐    ┌─────────────────────────────────────┐
│  YOLO Detection     │    │  Camera-LiDAR Calibration           │
│  Node               │    │  (camera_info_publisher)            │
│  (drive_yolo_node)  │    └─────────────────────────────────────┘
└─────────┬───────────┘                           │
          │                                       │
          │ /detections                           │ /camera_info
          v                                       v
┌─────────────────────────────────────────────────────────────────┐
│                    LiDAR-Camera Fusion                          │
│              (lidar_camera_fusion_node)                         │
│  - Projects LiDAR points onto image                             │
│  - Associates points with detections                            │
│  - Computes 3D distances                                        │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      │ /detections_with_distance
                      v
┌─────────────────────────────────────────────────────────────────┐
│                    3D Object Tracking                           │
│                 (tracking_3d_node.py)                           │
│  - Kalman filtering                                             │
│  - Data association                                             │
│  - Motion prediction                                            │
└─────────────────────┬───────────────────────────────────────────┘
                      │
                      │ /tracked_objects_3d
                      v
┌─────────────────────────────────────────────────────────────────┐
│                      Visualization                              │
│         (tracking_rviz_visualizer.py + RViz)                    │
│  - Velocity arrows                                              │
│  - Track IDs and labels                                         │
│  - Position markers                                             │
│  - Predicted positions                                          │
└─────────────────────────────────────────────────────────────────┘
```

## ROS Nodes

### Core Nodes

#### 1. `drive_yolo_node` (C++)
**Package**: `drive_yolo`
**Executable**: `drive_yolo_node_node`
**Source**: `drive_yolo/src/yolo_node.cpp`

YOLO object detection node with TensorRT acceleration.

**Subscribed Topics**:
- `/camera/image_input` (sensor_msgs/Image) - Camera images when not using DriveWorks

**Published Topics**:
- `/detections` (drive_yolo/Detections) - Detected objects with bounding boxes
- `/camera/image_raw` (sensor_msgs/Image) - Raw camera images

**Parameters**:
- `model_path` (string): Path to TensorRT engine file
- `use_driveworks_camera` (bool): Enable DriveWorks camera interface
- `camera_params` (string): DriveWorks camera parameters (e.g., "camera-name=...,interface=csi-ab,link=1")
- `confidence_threshold` (double, default: 0.5): Publishing threshold
- `detection_threshold` (double, default: 0.5): Detection threshold
- `vehicle_confidence_threshold` (double): Vehicle-specific threshold
- `pedestrian_confidence_threshold` (double): Pedestrian-specific threshold
- `ignore_hood_pixels` (int, default: 400): Pixels to ignore from bottom
- `enable_hood_filtering` (bool, default: true): Enable hood filtering
- `quiet_mode` (bool, default: false): Reduce log output

**Example**:
```bash
rosrun drive_yolo drive_yolo_node_node \
  _model_path:=/models/yolo11n.engine \
  _use_driveworks_camera:=true \
  _camera_params:="camera-name=F008A120RM0AV2,interface=csi-ab,link=1,output-format=processed"
```

#### 2. `lidar_camera_fusion_node` (C++)
**Package**: `drive_yolo`
**Executable**: `drive_yolo_lidar_fusion_node`
**Source**: `drive_yolo/src/lidar_camera_fusion_node.cpp`

Fuses LiDAR point clouds with camera detections.

**Subscribed Topics**:
- `/detections` (drive_yolo/Detections) - 2D detections from YOLO
- `/lidar/points` (sensor_msgs/PointCloud2) - LiDAR point cloud
- `/camera_info` (sensor_msgs/CameraInfo) - Camera calibration

**Published Topics**:
- `/detections_with_distance` (drive_yolo/DetectionsWithDistance) - Detections enriched with 3D distance
- `/lidar_camera_fusion/debug_cloud` (sensor_msgs/PointCloud2) - Debug visualization

**Parameters**:
- `camera_matrix` (array[9]): 3x3 camera intrinsic matrix
- `distortion_coefficients` (array[5]): Distortion coefficients [k1, k2, p1, p2, k3]
- `image_width` (int): Image width in pixels
- `image_height` (int): Image height in pixels
- `use_tf_transforms` (bool, default: true): Use TF for coordinate transforms
- `lidar_frame` (string): LiDAR frame ID
- `camera_frame` (string): Camera frame ID
- `base_frame` (string): Base/vehicle frame ID
- `max_distance` (double, default: 100.0): Maximum LiDAR range (meters)
- `min_distance` (double, default: 0.5): Minimum LiDAR range (meters)
- `sync_time_slop` (double, default: 0.1): Time tolerance for synchronization (seconds)
- `sync_queue_size` (int, default: 20): Message buffer size
- `time_offset_ms` (double, default: 0.0): Manual time offset between sensors (milliseconds)
- `enable_ground_filtering` (bool, default: false): Remove ground points
- `enable_point_clustering` (bool, default: true): Filter noise with clustering
- `min_points_for_object` (int, default: 3): Minimum points to consider valid object
- `enable_roi_filtering` (bool, default: false): Enable region of interest filtering

**Example**:
```bash
roslaunch drive_yolo lidar_camera_fusion_with_sync.launch
```

#### 3. `tracking_3d_node` (Python)
**Package**: `drive_yolo`
**Executable**: `tracking_3d_node.py`
**Source**: `drive_yolo/scripts/tracking_3d_node.py`

Multi-object tracker with Kalman filtering and motion prediction.

**Subscribed Topics**:
- `/detections_with_distance` (drive_yolo/DetectionsWithDistance) - 3D detections from fusion

**Published Topics**:
- `/tracked_objects_3d` (drive_yolo/TrackedObjects3D) - Tracked objects with IDs and motion

**Parameters**:
- `distance_threshold_3d` (double, default: 2.0): Association distance threshold (meters)
- `max_missed_frames` (int, default: 8): Frames before track deletion
- `min_detections_for_tracking` (int, default: 3): Detections required to publish track
- `min_lidar_points` (int, default: 5): Minimum LiDAR points for valid track
- `min_speed_threshold` (double, default: 0.1): Speed threshold for "moving" classification (m/s)
- `max_tracking_distance` (double, default: 30.0): Maximum tracking range (meters)
- `publish_all_tracks` (bool, default: true): Publish all tracks vs. high-quality only
- `publish_high_quality_only` (bool, default: false): Only publish stable tracks
- `quiet_mode` (bool, default: false): Reduce log output

**Example**:
```bash
rosrun drive_yolo tracking_3d_node.py \
  _distance_threshold_3d:=2.0 \
  _max_missed_frames:=8
```

#### 4. `tracking_rviz_visualizer` (Python)
**Package**: `drive_yolo`
**Executable**: `tracking_rviz_visualizer.py`
**Source**: `drive_yolo/scripts/tracking_rviz_visualizer.py`

RViz visualization node for tracked objects.

**Subscribed Topics**:
- `/tracked_objects_3d` (drive_yolo/TrackedObjects3D) - Tracked objects

**Published Topics**:
- `/tracking/velocity_markers` (visualization_msgs/MarkerArray) - Velocity arrows
- `/tracking/position_markers` (visualization_msgs/MarkerArray) - Position markers
- `/tracking/label_markers` (visualization_msgs/MarkerArray) - Track ID labels
- `/tracking/prediction_markers` (visualization_msgs/MarkerArray) - Predicted positions

**Parameters**:
- `show_velocity_vectors` (bool, default: true): Display velocity arrows
- `show_track_ids` (bool, default: true): Display track ID labels
- `show_positions` (bool, default: true): Display position markers
- `show_predicted_positions` (bool, default: true): Display 1-second predictions
- `velocity_scale` (double, default: 2.0): Arrow size scale factor
- `min_velocity_display` (double, default: 0.5): Minimum speed to display arrow (m/s)
- `frame_id` (string, default: "camera"): Reference frame
- `use_class_colors` (bool, default: true): Use class-specific colors

### Color Scheme

When `use_class_colors` is enabled:
- **Car**: Cyan
- **Truck**: Orange
- **Person**: Red
- **Bicycle**: Yellow
- **Motorcycle**: Magenta
- **Bus**: Purple

**Example**:
```bash
roslaunch drive_yolo tracking_with_rviz.launch launch_rviz:=true
```

### Utility Nodes

#### 5. `camera_info_publisher` (C++)
Publishes camera calibration parameters for RViz pointcloud overlay.

**Published Topics**:
- `/camera_info` (sensor_msgs/CameraInfo)

**Parameters**:
- `camera_matrix` (array[9]): Camera intrinsics
- `distortion_coefficients` (array[5]): Distortion parameters
- `image_width`, `image_height` (int): Image dimensions
- `frame_id` (string): Camera frame ID
- `publish_rate` (double, default: 30.0): Publishing frequency (Hz)

#### 6. `sync_diagnostics_node` (C++)
Diagnoses timing synchronization between camera and LiDAR.

**Subscribed Topics**:
- `/detections` (drive_yolo/Detections)
- `/lidar/points` (sensor_msgs/PointCloud2)

**Output**: Console statistics showing time offsets and synchronization quality.

#### 7. `pointcloud_delay_node` (C++)
Adds configurable delay to pointcloud for testing synchronization.

**Parameters**:
- `delay_ms` (int, default: 0): Delay in milliseconds

#### 8. `speed_monitor_node` (C++)
Monitors object speeds and triggers warnings.

**Subscribed Topics**:
- `/tracked_objects_3d` (drive_yolo/TrackedObjects3D)

**Parameters**:
- `speed_limit_ms` (double): Speed limit (m/s)
- `warn_threshold_ms` (double): Warning threshold (m/s)
- `monitor_classes` (string): Comma-separated class IDs to monitor

## Usage

### Quick Start

#### 1. Basic YOLO Detection with DriveWorks Camera

```bash
# Start roscore
roscore &

# Launch YOLO detection
roslaunch drive_yolo yolo_driveworks.launch \
  model_path:=/models/yolo11n.engine \
  camera_params:="camera-name=F008A120RM0AV2,interface=csi-ab,link=1,output-format=processed"
```

#### 2. Complete Detection + Fusion + Tracking Pipeline

```bash
# Launch full pipeline
roslaunch drive_yolo complete_detection_pipeline.launch \
  model_path:=/models/yolo11n.engine \
  use_driveworks:=true
```

#### 3. 3D Tracking with RViz Visualization

```bash
# Launch tracking with automatic RViz
roslaunch drive_yolo tracking_with_rviz.launch launch_rviz:=true

# Or manually launch RViz with pre-configured view
rviz -d $(rospack find drive_yolo)/rviz/tracking_visualization.rviz
```

#### 4. Pointcloud Overlay in RViz

```bash
# Publish camera info and visualize pointcloud overlay
roslaunch drive_yolo pointcloud_camera_overlay.launch
```

### Advanced Usage

#### Synchronization Diagnostics

If you notice fusion issues:

```bash
# 1. Run diagnostics to measure timing offset
roslaunch drive_yolo sync_diagnostics.launch

# 2. Update time_offset_ms in lidar_camera_fusion_with_sync.launch
# 3. Re-launch fusion with corrected offset
roslaunch drive_yolo lidar_camera_fusion_with_sync.launch
```

#### Custom TF Frames

Set up TF transforms for your sensor configuration:

```bash
# Example: Publish static transforms
roslaunch drive_yolo M8VIL_TFs.launch

# Or define your own in a launch file:
<node pkg="tf2_ros" type="static_transform_publisher" name="lidar_to_base"
      args="0 0 1.383 0 0 0 car_rear_axle lidar" />
<node pkg="tf2_ros" type="static_transform_publisher" name="camera_to_base"
      args="1.5 0 1.2 0 0.1 0 car_rear_axle camera_entron" />
```

## Launch Files

### Main Pipelines

| Launch File | Description |
|------------|-------------|
| `complete_system.launch` | YOLO detection + 2D tracking |
| `complete_detection_pipeline.launch` | Full pipeline: YOLO + LiDAR fusion + 3D tracking |
| `LidarandCamerapipeline.launch` | LiDAR-camera fusion pipeline |

### Individual Components

| Launch File | Description |
|------------|-------------|
| `yolo_driveworks.launch` | YOLO detection only (DriveWorks camera) |
| `yolo_ros_camera.launch` | YOLO detection with ROS camera input |
| `lidar_camera_fusion.launch` | LiDAR-camera fusion node |
| `lidar_camera_fusion_with_sync.launch` | Fusion with synchronization parameters |
| `motpy_tracking.launch` | Python-based 3D tracking |
| `3d_tracking_pipeline.launch` | C++ 3D tracking pipeline |

### Visualization

| Launch File | Description |
|------------|-------------|
| `tracking_with_rviz.launch` | 3D tracking with RViz visualization |
| `tracking_visualizer_only.launch` | Visualizer node only |
| `detection_visualizer.launch` | 2D detection visualization |
| `pointcloud_camera_overlay.launch` | Pointcloud overlay in RViz |

### Diagnostics

| Launch File | Description |
|------------|-------------|
| `sync_diagnostics.launch` | Camera-LiDAR synchronization diagnostics |
| `pointcloud_delay_demo.launch` | Pointcloud delay testing |

### Utilities

| Launch File | Description |
|------------|-------------|
| `M8VIL_TFs.launch` | TF transforms for M8VIL vehicle configuration |
| `osi2ros_lidar.launch` | OSI to ROS LiDAR converter |

## Troubleshooting

### Common Issues

#### 1. **DriveWorks camera fails to initialize**

**Symptoms**: `DriveWorks camera init failed` error

**Solutions**:
- Verify camera is connected: `v4l2-ctl --list-devices`
- Check camera parameters match your hardware
- Ensure DriveWorks SDK is properly installed
- Try different output formats: `output-format=processed` or `output-format=raw`

#### 2. **No LiDAR points in fusion**

**Symptoms**: All detections show distance = 0

**Solutions**:
- Check LiDAR topic is publishing: `rostopic echo /lidar/points`
- Verify TF frames are correct: `rosrun tf tf_echo camera_frame lidar_frame`
- Run sync diagnostics: `roslaunch drive_yolo sync_diagnostics.launch`
- Adjust `sync_time_slop` parameter (increase if messages are being dropped)
- Check `time_offset_ms` parameter

#### 3. **Tracking IDs keep changing**

**Symptoms**: Objects get new track IDs frequently

**Solutions**:
- Increase `max_missed_frames` parameter
- Decrease `distance_threshold_3d` for tighter association
- Increase `min_detections_for_tracking` for more stable tracks
- Check detection consistency (YOLO confidence thresholds)

#### 4. **RViz markers don't appear**

**Symptoms**: No visualization in RViz

**Solutions**:
- Verify Fixed Frame matches `frame_id` parameter (default: "camera")
- Check topics are publishing: `rostopic list | grep tracking`
- Ensure MarkerArray displays are added in RViz
- Load pre-configured RViz file: `rviz -d $(rospack find drive_yolo)/rviz/tracking_visualization.rviz`

#### 5. **Cross-compilation errors**

**Symptoms**: CMake or linker errors during build

**Solutions**:
- Update paths in toolchain files to match your installation
- Verify DriveWorks libraries exist: `ls $DRIVEWORKS_ROOT/targets/aarch64-Linux/lib`
- Check CUDA version matches (11.4 in default config)
- Ensure sysroot contains ROS installation

#### 6. **Python dependencies missing on target**

**Symptoms**: `ImportError: No module named motpy`

**Solutions**:
```bash
pip3 install motpy numpy scipy scikit-learn filterpy
```

### Debugging Tools

```bash
# View all topics
rostopic list

# Monitor specific topic
rostopic echo /tracked_objects_3d

# Check message rate
rostopic hz /detections

# View TF tree
rosrun rqt_tf_tree rqt_tf_tree

# Record data for offline analysis
rosbag record /detections /lidar/points /tracked_objects_3d

# Play back recorded data
rosbag play your_recording.bag
```

## Project Structure

```
.
├── drive_yolo/              # Main package
│   ├── src/                 # C++ source files
│   │   ├── yolo_node.cpp
│   │   ├── yolo_infer.cpp
│   │   ├── lidar_camera_fusion_node.cpp
│   │   ├── tracking_3d_node.cpp
│   │   └── ...
│   ├── scripts/             # Python nodes
│   │   ├── tracking_3d_node.py
│   │   ├── tracking_rviz_visualizer.py
│   │   └── ...
│   ├── include/             # Header files
│   ├── launch/              # Launch files
│   ├── msg/                 # Custom message definitions
│   ├── rviz/                # RViz configuration files
│   ├── docs/                # Documentation
│   ├── models/              # YOLO model files (user-provided)
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── Toolchain-V5L.cmake  # Cross-compilation toolchain
├── nv_sensors/              # NVIDIA sensor drivers
│   ├── src/
│   ├── include/
│   ├── CMakeLists.txt
│   └── package.xml
└── README.md
```

## Message Definitions

### Detection.msg
```
int32 class_id
string class_name
float32 confidence
float32 x1
float32 y1
float32 x2
float32 y2
```

### DetectionWithDistance.msg
```
Detection detection
float32 distance_m
float32 distance_confidence
int32 lidar_points_count
geometry_msgs/Point position_3d
```

### TrackedObject3D.msg
```
int32 id
int32 class_id
string class_name
geometry_msgs/Point position_3d
geometry_msgs/Vector3 velocity_3d
geometry_msgs/Vector3 acceleration_3d
float32 speed_3d
float32 confidence
float32 distance_confidence
int32 lidar_points_count
float32 track_stability
float32 track_duration
int32 total_detections
bool is_moving
geometry_msgs/Point predicted_position_1s
```

## Known Limitations

- DriveWorks camera interface is DRIVE AGX-specific
- Cross-compilation requires NVIDIA DRIVE OS SDK
- LiDAR fusion assumes calibrated sensors with known TF transforms
- 3D tracking works best with consistent detections (>10 Hz)

## Future Enhancements

- [ ] Radar sensor fusion
- [ ] ROS 2 migration
- [ ] Docker containerization
- [ ] Pre-built ARM64 binaries
- [ ] Automated calibration tools
- [ ] Real-time performance benchmarking
- [ ] Integration with path planning frameworks

## References

- [NVIDIA DriveWorks Documentation](https://developer.nvidia.com/drive/driveworks)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)

## Acknowledgments

Co-funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them. Project grant no. 101069576. UK participants in this project are co-funded by Innovate UK under contract no.10045139. Swiss participants in this project are co-funded by the Swiss State Secretariat for Education, Research and Innovation (SERI) under contract no. 22.00123.

I would also like to acknoledge:
- NVIDIA Corporation for DriveWorks SDK and the Lovely NVidia people at the Forum ^^.
- Open Robotics for ROS framework
- Ultralytics for YOLO models
- Felix

# Final Words
please let me know if it works for you im curious. 

I wish this helps you to develop for the NVIDIA DRIVE AGX ORIN WITH TENSORRT 

hugs

Yuri

PS: if you use opset above 13 tensorrt will be upset
