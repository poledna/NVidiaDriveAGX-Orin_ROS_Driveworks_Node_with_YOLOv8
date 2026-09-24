#include <rclcpp/rclcpp.hpp>
#include "drive_yolo/lidar_camera_fusion.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    int ret = 0;
    try {
        auto fusion_node = std::make_shared<LidarCameraFusion>();
        RCLCPP_INFO(fusion_node->get_logger(), "LiDAR-Camera fusion node started");
        rclcpp::spin(fusion_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("lidar_camera_fusion_node"), "Fusion node failed: %s", e.what());
        ret = -1;
    }

    rclcpp::shutdown();
    return ret;
}
