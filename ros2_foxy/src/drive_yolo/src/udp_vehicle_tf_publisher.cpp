/**
 * UDP Vehicle TF Publisher Node
 *
 * Receives vehicle position data via UDP socket:
 * - Format: roll, pitch, yaw, x, y (5 doubles)
 * - Publishes TF transform from map/world to vehicle base_link
 *
 * Usage:
 *   ros2 run drive_yolo drive_yolo_udp_vehicle_tf --ros-args -p port:=5000
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

struct VehiclePose {
    double roll, pitch, yaw;
    double x, y;
    rclcpp::Time timestamp;
};

class UDPVehicleTFPublisher : public rclcpp::Node {
public:
    UDPVehicleTFPublisher() : rclcpp::Node("udp_vehicle_tf_publisher"), running_(true) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Parameters
        udp_port_ = declare_parameter<int>("port", 5000);
        world_frame_ = declare_parameter<std::string>("world_frame", "map");
        vehicle_frame_ = declare_parameter<std::string>("vehicle_frame", "base_link");
        publish_rate_ = declare_parameter<double>("publish_rate", 50.0);  // 50 Hz
        publish_odom_ = declare_parameter<bool>("publish_odom", true);
        z_offset_ = declare_parameter<double>("z_offset", 0.0);  // Height above ground

        // Publishers
        if (publish_odom_) {
            odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vehicle/odom", 10);
            pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vehicle/pose", 10);
        }

        RCLCPP_INFO(get_logger(), "=== UDP Vehicle TF Publisher ===");
        RCLCPP_INFO(get_logger(), "Listening on UDP port: %d", udp_port_);
        RCLCPP_INFO(get_logger(), "Publishing TF: %s -> %s", world_frame_.c_str(), vehicle_frame_.c_str());
        RCLCPP_INFO(get_logger(), "Publish rate: %.1f Hz", publish_rate_);
        RCLCPP_INFO(get_logger(), "Z offset: %.2f m", z_offset_);
        RCLCPP_INFO(get_logger(), "===============================");

        // Start UDP receiver thread
        udp_thread_ = std::thread(&UDPVehicleTFPublisher::udpReceiverThread, this);

        // Start TF publisher timer
        publish_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                                           std::bind(&UDPVehicleTFPublisher::publishTF, this));
    }

    ~UDPVehicleTFPublisher() {
        running_ = false;
        if (udp_thread_.joinable()) {
            udp_thread_.join();
        }
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::thread udp_thread_;
    std::atomic<bool> running_;
    int sockfd_ = -1;

    int udp_port_;
    std::string world_frame_, vehicle_frame_;
    double publish_rate_;
    bool publish_odom_;
    double z_offset_;

    VehiclePose current_pose_;
    std::mutex pose_mutex_;

    bool has_received_data_ = false;
    size_t packets_received_ = 0;

    void udpReceiverThread() {
        // Create UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to create UDP socket");
            return;
        }

        // Set socket to non-blocking with timeout
        struct timeval tv;
        tv.tv_sec = 1;  // 1 second timeout
        tv.tv_usec = 0;
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Bind socket
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(udp_port_);

        if (bind(sockfd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to bind UDP socket to port %d", udp_port_);
            close(sockfd_);
            sockfd_ = -1;
            return;
        }

        RCLCPP_INFO(get_logger(), "UDP socket bound successfully, waiting for data...");

        // Receive loop
        while (running_ && rclcpp::ok()) {
            // Expect 5 doubles: roll, pitch, yaw, x, y
            double buffer[5];
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);

            ssize_t recv_len = recvfrom(sockfd_, buffer, sizeof(buffer), 0,
                                       (struct sockaddr*)&client_addr, &client_len);

            if (recv_len == sizeof(buffer)) {
                std::lock_guard<std::mutex> lock(pose_mutex_);

                current_pose_.roll = buffer[0];
                current_pose_.pitch = buffer[1];
                current_pose_.yaw = buffer[2];
                current_pose_.x = buffer[3];
                current_pose_.y = buffer[4];
                current_pose_.timestamp = now();

                if (!has_received_data_) {
                    RCLCPP_INFO(get_logger(), "First UDP packet received from %s:%d",
                            inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                    has_received_data_ = true;
                }

                packets_received_++;

                RCLCPP_DEBUG(get_logger(), "Received pose: x=%.2f, y=%.2f, yaw=%.2f°",
                         current_pose_.x, current_pose_.y, current_pose_.yaw * 180.0 / M_PI);

            } else if (recv_len > 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received invalid packet size: %zd bytes (expected %zu)",
                                 recv_len, sizeof(buffer));
            }
            // Timeout or error - continue loop
        }

        RCLCPP_INFO(get_logger(), "UDP receiver thread shutting down");
    }

    void publishTF() {
        if (!has_received_data_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No UDP data received yet (packets: %zu)", packets_received_);
            return;
        }

        VehiclePose pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose = current_pose_;
        }

        // Create transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now();
        transform.header.frame_id = world_frame_;
        transform.child_frame_id = vehicle_frame_;

        transform.transform.translation.x = pose.x;
        transform.transform.translation.y = pose.y;
        transform.transform.translation.z = z_offset_;

        // Convert RPY to quaternion
        tf2::Quaternion q;
        q.setRPY(pose.roll, pose.pitch, pose.yaw);

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);

        // Publish odometry if enabled
        if (publish_odom_) {
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header = transform.header;
            odom_msg.child_frame_id = vehicle_frame_;

            odom_msg.pose.pose.position.x = pose.x;
            odom_msg.pose.pose.position.y = pose.y;
            odom_msg.pose.pose.position.z = z_offset_;

            odom_msg.pose.pose.orientation = transform.transform.rotation;

            odom_pub_->publish(odom_msg);

            // Also publish PoseStamped
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = transform.header;
            pose_msg.pose = odom_msg.pose.pose;
            pose_pub_->publish(pose_msg);
        }

        double age = (now() - pose.timestamp).seconds();
        if (age > 0.5) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Stale pose data (%.2f seconds old)", age);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    int ret = 0;
    try {
        auto node = std::make_shared<UDPVehicleTFPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("udp_vehicle_tf_publisher"), "Node failed: %s", e.what());
        ret = -1;
    }

    rclcpp::shutdown();
    return ret;
}
