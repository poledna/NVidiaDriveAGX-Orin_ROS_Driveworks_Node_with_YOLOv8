cmake_minimum_required(VERSION 3.5)
project(drive_yolo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/Detections.msg"
  "msg/DetectionWithDistance.msg"
  "msg/DetectionsWithDistance.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjects.msg"
  "msg/TrackedObject3D.msg"
  "msg/TrackedObjects3D.msg"
  "msg/ProjectedPoint.msg"
  "msg/ProjectedPoints.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
