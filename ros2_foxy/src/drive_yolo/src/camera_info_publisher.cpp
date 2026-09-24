/**
 * Camera Info Publisher Node
 *
 * This node publishes camera calibration information (CameraInfo) for the
 * Entron camera, enabling proper pointcloud overlay visualization in RViz.
 *
 * The calibration parameters are loaded from the fusion lidar configuration
 * and published at a fixed rate to support real-time visualization.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

class CameraInfoPublisher : public rclcpp::Node
{
public:
    CameraInfoPublisher() : rclcpp::Node("camera_info_publisher")
    {
        // Load parameters
        camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_entron");
        image_width_ = declare_parameter<int>("image_width", 3848);
        image_height_ = declare_parameter<int>("image_height", 2168);
        publish_rate_ = declare_parameter<double>("publish_rate", 30.0);

        // Load camera matrix (3x3)
        std::vector<double> camera_matrix =
            declare_parameter<std::vector<double>>("camera_matrix", std::vector<double>{});
        if (camera_matrix.size() != 9)
        {
            RCLCPP_ERROR(get_logger(), "Camera matrix must be a 9-element array (3x3 matrix in row-major order)");
            camera_matrix = {3145.24132, 0.0, 1887.68485,
                           0.0, 3166.08502, 990.62304,
                           0.0, 0.0, 1.0};
            RCLCPP_WARN(get_logger(), "Using default camera matrix from config/lidar_camera_fusion.yaml");
        }

        // Load distortion coefficients
        std::vector<double> distortion_coeffs =
            declare_parameter<std::vector<double>>("distortion_coefficients", std::vector<double>{});
        if (distortion_coeffs.size() != 5)
        {
            RCLCPP_ERROR(get_logger(), "Distortion coefficients must be a 5-element array [k1, k2, p1, p2, k3]");
            distortion_coeffs = {-0.298671, 0.087807, 0.000383, 0.000551, 0.000000};
            RCLCPP_WARN(get_logger(), "Using default distortion coefficients from config/lidar_camera_fusion.yaml");
        }

        // Initialize CameraInfo message
        initializeCameraInfo(camera_matrix, distortion_coeffs);

        // Create publisher
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);

        // Create timer for periodic publishing
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                                   std::bind(&CameraInfoPublisher::publishCameraInfo, this));

        RCLCPP_INFO(get_logger(), "Camera Info Publisher initialized");
        RCLCPP_INFO(get_logger(), "  Camera frame: %s", camera_frame_.c_str());
        RCLCPP_INFO(get_logger(), "  Image size: %dx%d", image_width_, image_height_);
        RCLCPP_INFO(get_logger(), "  Publishing rate: %.1f Hz", publish_rate_);
        RCLCPP_INFO(get_logger(), "  Topic: /camera/camera_info");
    }

private:
    void initializeCameraInfo(const std::vector<double>& camera_matrix,
                             const std::vector<double>& distortion_coeffs)
    {
        camera_info_.header.frame_id = camera_frame_;
        camera_info_.width = image_width_;
        camera_info_.height = image_height_;

        // Set distortion model (plumb_bob is standard for OpenCV calibration)
        camera_info_.distortion_model = "plumb_bob";

        // Set distortion coefficients [k1, k2, p1, p2, k3]
        camera_info_.d.resize(5);
        for (int i = 0; i < 5; i++)
        {
            camera_info_.d[i] = distortion_coeffs[i];
        }

        // Set camera matrix K (3x3)
        // K = [fx  0  cx]
        //     [ 0 fy  cy]
        //     [ 0  0   1]
        for (int i = 0; i < 9; i++)
        {
            camera_info_.k[i] = camera_matrix[i];
        }

        // Set rectification matrix R (3x3) - identity for unrectified image
        camera_info_.r[0] = 1.0; camera_info_.r[1] = 0.0; camera_info_.r[2] = 0.0;
        camera_info_.r[3] = 0.0; camera_info_.r[4] = 1.0; camera_info_.r[5] = 0.0;
        camera_info_.r[6] = 0.0; camera_info_.r[7] = 0.0; camera_info_.r[8] = 1.0;

        // Set projection matrix P (3x4)
        // P = [fx'  0  cx' Tx]
        //     [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        // For monocular camera: Tx = Ty = 0, fx' = fx, fy' = fy, cx' = cx, cy' = cy
        camera_info_.p[0]  = camera_matrix[0]; // fx
        camera_info_.p[1]  = 0.0;
        camera_info_.p[2]  = camera_matrix[2]; // cx
        camera_info_.p[3]  = 0.0;              // Tx
        camera_info_.p[4]  = 0.0;
        camera_info_.p[5]  = camera_matrix[4]; // fy
        camera_info_.p[6]  = camera_matrix[5]; // cy
        camera_info_.p[7]  = 0.0;              // Ty
        camera_info_.p[8]  = 0.0;
        camera_info_.p[9]  = 0.0;
        camera_info_.p[10] = 1.0;
        camera_info_.p[11] = 0.0;

        // Binning and ROI - not used
        camera_info_.binning_x = 0;
        camera_info_.binning_y = 0;
        camera_info_.roi.x_offset = 0;
        camera_info_.roi.y_offset = 0;
        camera_info_.roi.height = 0;
        camera_info_.roi.width = 0;
        camera_info_.roi.do_rectify = false;
    }

    void publishCameraInfo()
    {
        camera_info_.header.stamp = now();
        camera_info_pub_->publish(camera_info_);
    }

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::CameraInfo camera_info_;
    std::string camera_frame_;
    int image_width_;
    int image_height_;
    double publish_rate_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    int ret = 0;
    try {
        auto node = std::make_shared<CameraInfoPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_info_publisher"), "Node failed: %s", e.what());
        ret = -1;
    }

    rclcpp::shutdown();
    return ret;
}
