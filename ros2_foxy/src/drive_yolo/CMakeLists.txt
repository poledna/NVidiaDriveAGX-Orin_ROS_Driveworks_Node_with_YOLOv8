cmake_minimum_required(VERSION 3.5)
project(drive_yolo)

# Foxy (Ubuntu 20.04 / gcc 9). C++17: the trackers use structured bindings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Build the TensorRT YOLO node. Turn OFF to build only the ROS side
# (fusion, tracking, point cloud tools) e.g. on a host without TensorRT.
option(DRIVE_YOLO_BUILD_INFERENCE "Build the TensorRT/CUDA YOLO node" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(drive_yolo_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d video)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Common ament dependencies for all nodes
set(ROS_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  drive_yolo_msgs
)

# ---------------------------------------------------------------------------
# TensorRT / CUDA / DriveWorks (only needed by the YOLO node)
# The cross-compilation toolchain (Toolchain-V5L-foxy.cmake) sets these roots
# inside the sysroot. When building natively on the Orin the defaults below
# point to the standard DRIVE OS 6.0.x locations.
# ---------------------------------------------------------------------------
if(DRIVE_YOLO_BUILD_INFERENCE)
  if(NOT DEFINED CUDA_ROOT)
    set(CUDA_ROOT "${CMAKE_SYSROOT}/usr/local/cuda-11.4")
  endif()
  if(NOT DEFINED TENSORRT_ROOT)
    set(TENSORRT_ROOT "${CMAKE_SYSROOT}/usr")
  endif()
  if(NOT DEFINED DRIVEWORKS_ROOT)
    set(DRIVEWORKS_ROOT "${CMAKE_SYSROOT}/usr/local/driveworks")
  endif()

  set(TENSORRT_INCLUDE_DIR "${TENSORRT_ROOT}/include/aarch64-linux-gnu")
  set(TENSORRT_LIB_DIR "${TENSORRT_ROOT}/lib/aarch64-linux-gnu")
  set(CUDA_INCLUDE_DIR "${CUDA_ROOT}/targets/aarch64-linux/include")
  set(CUDA_LIB_DIR "${CUDA_ROOT}/targets/aarch64-linux/lib")

  if(EXISTS "${TENSORRT_INCLUDE_DIR}/NvInfer.h")
    message(STATUS "Found TensorRT headers at: ${TENSORRT_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "TensorRT headers not found at: ${TENSORRT_INCLUDE_DIR} "
                        "(set TENSORRT_ROOT or -DDRIVE_YOLO_BUILD_INFERENCE=OFF)")
  endif()

  # NO_CMAKE_FIND_ROOT_PATH: the paths above already contain the sysroot
  find_library(TENSORRT_LIB nvinfer PATHS ${TENSORRT_LIB_DIR} NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  find_library(NVONNX_LIB nvonnxparser PATHS ${TENSORRT_LIB_DIR} NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  find_library(CUDART_LIB cudart PATHS ${CUDA_LIB_DIR} NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  foreach(lib TENSORRT_LIB NVONNX_LIB CUDART_LIB)
    if(NOT ${lib})
      message(FATAL_ERROR "${lib} not found (TensorRT: ${TENSORRT_LIB_DIR}, CUDA: ${CUDA_LIB_DIR})")
    endif()
    message(STATUS "Found ${lib}: ${${lib}}")
  endforeach()

  # DriveWorks is optional: without it the node subscribes to a ROS image topic
  set(DRIVEWORKS_LIB_DIR "${DRIVEWORKS_ROOT}/targets/aarch64-Linux/lib")
  set(DRIVEWORKS_INCLUDE_DIR "${DRIVEWORKS_ROOT}/targets/aarch64-Linux/include")
  set(DRIVEWORKS_LIBS
    "${DRIVEWORKS_LIB_DIR}/libdw_base.so"
    "${DRIVEWORKS_LIB_DIR}/libdw_sensors.so"
    "${DRIVEWORKS_LIB_DIR}/libdwshared.so"
  )
  set(DRIVEWORKS_FOUND TRUE)
  foreach(lib ${DRIVEWORKS_LIBS})
    if(EXISTS "${lib}")
      message(STATUS "Found DriveWorks library: ${lib}")
    else()
      message(WARNING "DriveWorks library not found: ${lib}")
      set(DRIVEWORKS_FOUND FALSE)
    endif()
  endforeach()

  # YOLO inference library (TensorRT wrapper + DriveWorks camera)
  add_library(${PROJECT_NAME} SHARED
    src/yolo_infer.cpp
    src/driveworks_camera.cpp
  )
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIR}
  )
  # VIBRANTE version definitions for DRIVE OS 6.0.6 so DriveWorks headers
  # use the correct API version
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    VIBRANTE_PDK_DECIMAL=6000600
    VIBRANTE_PDK_VERSION=6.0.6.0
    VIBRANTE
  )
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${TENSORRT_LIB}
    ${NVONNX_LIB}
    ${CUDART_LIB}
    pthread
    dl
    rt
  )
  ament_target_dependencies(${PROJECT_NAME} rclcpp)

  if(DRIVEWORKS_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC
      ${DRIVEWORKS_INCLUDE_DIR}
      "${DRIVEWORKS_ROOT}/include"
    )
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_DRIVEWORKS=1)
    target_link_libraries(${PROJECT_NAME} ${DRIVEWORKS_LIBS})
    message(STATUS "Compiling with DriveWorks support")
  else()
    message(STATUS "Compiling without DriveWorks support")
  endif()

  add_executable(${PROJECT_NAME}_node src/yolo_node.cpp)
  target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} -Wl,--no-as-needed)
  ament_target_dependencies(${PROJECT_NAME}_node ${ROS_DEPS})

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "DRIVE_YOLO_BUILD_INFERENCE=OFF - skipping the TensorRT YOLO node")
endif()

# ---------------------------------------------------------------------------
# ROS-only nodes
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_tracking_3d_node
  src/tracking_3d_node.cpp
  src/object_tracker_3d.cpp
)
target_link_libraries(${PROJECT_NAME}_tracking_3d_node ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_tracking_3d_node ${ROS_DEPS})

add_executable(${PROJECT_NAME}_speed_monitor_node src/speed_monitor_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_speed_monitor_node ${ROS_DEPS})

add_executable(${PROJECT_NAME}_image_publisher src/image_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_image_publisher ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_image_publisher ${ROS_DEPS})

add_executable(${PROJECT_NAME}_visualizer src/yolo_visualizer.cpp)
target_link_libraries(${PROJECT_NAME}_visualizer ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_visualizer ${ROS_DEPS} message_filters)

add_executable(${PROJECT_NAME}_lidar_fusion_node
  src/lidar_camera_fusion_node.cpp
  src/lidar_camera_fusion.cpp
)
target_link_libraries(${PROJECT_NAME}_lidar_fusion_node ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_lidar_fusion_node
  ${ROS_DEPS} visualization_msgs message_filters tf2 tf2_ros tf2_geometry_msgs)

add_executable(${PROJECT_NAME}_camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(${PROJECT_NAME}_camera_info_publisher ${ROS_DEPS})

add_executable(${PROJECT_NAME}_sync_diagnostics src/sync_diagnostics_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_sync_diagnostics ${ROS_DEPS})

add_executable(${PROJECT_NAME}_pointcloud_delay src/pointcloud_delay_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_pointcloud_delay ${ROS_DEPS})

add_executable(${PROJECT_NAME}_pointcloud_filter src/pointcloud_filter_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_pointcloud_filter
  ${ROS_DEPS} visualization_msgs tf2 tf2_ros)

add_executable(${PROJECT_NAME}_pointcloud_projector src/pointcloud_projector_node.cpp)
target_link_libraries(${PROJECT_NAME}_pointcloud_projector ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_pointcloud_projector
  ${ROS_DEPS} visualization_msgs cv_bridge tf2 tf2_ros tf2_geometry_msgs)

add_executable(${PROJECT_NAME}_udp_vehicle_tf src/udp_vehicle_tf_publisher.cpp)
ament_target_dependencies(${PROJECT_NAME}_udp_vehicle_tf
  ${ROS_DEPS} nav_msgs tf2 tf2_ros)

add_executable(${PROJECT_NAME}_static_object_detector src/static_object_detector_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_static_object_detector
  ${ROS_DEPS} visualization_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  ${PROJECT_NAME}_tracking_3d_node
  ${PROJECT_NAME}_speed_monitor_node
  ${PROJECT_NAME}_image_publisher
  ${PROJECT_NAME}_visualizer
  ${PROJECT_NAME}_lidar_fusion_node
  ${PROJECT_NAME}_camera_info_publisher
  ${PROJECT_NAME}_sync_diagnostics
  ${PROJECT_NAME}_pointcloud_delay
  ${PROJECT_NAME}_pointcloud_filter
  ${PROJECT_NAME}_pointcloud_projector
  ${PROJECT_NAME}_udp_vehicle_tf
  ${PROJECT_NAME}_static_object_detector
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Python nodes (rclpy) - installed next to the C++ executables so that
# `ros2 run drive_yolo <script>.py` works
install(PROGRAMS
  scripts/tracking_3d_node.py
  scripts/fused_data2ros2.py
  scripts/detection_visualizer_node.py
  scripts/osi2ros_lidar.py
  scripts/tracking_rviz_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  FILES_MATCHING
  PATTERN "*.engine"
  PATTERN "*.onnx"
  PATTERN "*.pt"
)

ament_export_include_directories(include)
ament_export_dependencies(${ROS_DEPS})
ament_package()
