/**
 * Static Object Detector Node
 *
 * Detects static objects at known world positions using LiDAR data.
 * Checks if points exist within a bounding box at given X,Y world coordinates.
 *
 * Usage:
 *   - Configure static object positions via ROS parameters. ROS 2 has no
 *     array-of-struct parameters, so objects are given as a list of labels
 *     plus one parameter group per label (see config/static_objects.yaml):
 *       static_objects: ["pole_1", ...]
 *       pole_1: {x: 10.0, y: 0.0, z: 0.0, width: 0.5, height: 3.0, depth: 0.5}
 *   - Publishes bounding boxes when LiDAR points are detected inside
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "drive_yolo_msgs/msg/detection.hpp"
#include "drive_yolo_msgs/msg/detections.hpp"

#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>

struct StaticObject {
    int id;
    double x, y, z;          // Position in world frame
    double width, height, depth;  // Bounding box size
    std::string label;
    bool detected;
    int detection_count;
    rclcpp::Time last_detection;
};

class StaticObjectDetectorNode : public rclcpp::Node {
public:
    StaticObjectDetectorNode() : rclcpp::Node("static_object_detector_node") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Parameters
        world_frame_ = declare_parameter<std::string>("world_frame", "map");
        lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
        min_points_threshold_ = declare_parameter<int>("min_points_threshold", 5);
        detection_timeout_ = declare_parameter<double>("detection_timeout", 2.0);

        // Load static object definitions
        loadStaticObjects();

        // Subscribers and publishers
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", rclcpp::SensorDataQoS().keep_last(10),
            std::bind(&StaticObjectDetectorNode::cloudCallback, this, std::placeholders::_1));

        detection_pub_ = create_publisher<drive_yolo_msgs::msg::Detections>("/static_objects/detections", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/static_objects/markers", 1);

        // Timer for periodic marker publishing
        marker_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                          std::bind(&StaticObjectDetectorNode::publishMarkers, this));

        RCLCPP_INFO(get_logger(), "=== Static Object Detector Started ===");
        RCLCPP_INFO(get_logger(), "World frame: %s", world_frame_.c_str());
        RCLCPP_INFO(get_logger(), "LiDAR frame: %s", lidar_frame_.c_str());
        RCLCPP_INFO(get_logger(), "Min points threshold: %d", min_points_threshold_);
        RCLCPP_INFO(get_logger(), "Loaded %zu static objects", static_objects_.size());
        RCLCPP_INFO(get_logger(), "=====================================");

        std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for TF
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<drive_yolo_msgs::msg::Detections>::SharedPtr detection_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string world_frame_, lidar_frame_;
    int min_points_threshold_;
    double detection_timeout_;

    std::vector<StaticObject> static_objects_;

    void loadStaticObjects() {
        // Load static objects from ROS parameters
        // Format: static_objects: [label, ...] and <label>.x, y, z, width, height, depth

        const rclcpp::Time never(0, 0, RCL_ROS_TIME);
        std::vector<std::string> object_labels =
            declare_parameter<std::vector<std::string>>("static_objects", std::vector<std::string>{});
        if (object_labels.empty()) {
            RCLCPP_WARN(get_logger(), "No static_objects parameter found, using default test objects");

            // Create some test objects
            static_objects_.push_back({0, 10.0, 0.0, 0.0, 2.0, 2.0, 1.0, "pole_1", false, 0, never});
            static_objects_.push_back({1, 20.0, 5.0, 0.0, 3.0, 2.5, 2.0, "building_1", false, 0, never});
            return;
        }

        for (size_t i = 0; i < object_labels.size(); ++i) {
            const std::string& label = object_labels[i];

            StaticObject static_obj;
            static_obj.id = static_cast<int>(i);
            static_obj.label = label;

            static_obj.x = declare_parameter<double>(label + ".x", 0.0);
            static_obj.y = declare_parameter<double>(label + ".y", 0.0);
            static_obj.z = declare_parameter<double>(label + ".z", 0.0);
            static_obj.width = declare_parameter<double>(label + ".width", 1.0);
            static_obj.height = declare_parameter<double>(label + ".height", 2.0);
            static_obj.depth = declare_parameter<double>(label + ".depth", 1.0);

            static_obj.detected = false;
            static_obj.detection_count = 0;
            static_obj.last_detection = never;

            static_objects_.push_back(static_obj);

            RCLCPP_INFO(get_logger(), "Loaded object %d: '%s' at (%.1f, %.1f, %.1f), size: %.1fx%.1fx%.1f",
                    static_obj.id, static_obj.label.c_str(),
                    static_obj.x, static_obj.y, static_obj.z,
                    static_obj.width, static_obj.height, static_obj.depth);
        }
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
        // Transform point cloud to world frame
        sensor_msgs::msg::PointCloud2 cloud_world;

        try {
            const rclcpp::Time cloud_stamp(cloud_msg->header.stamp);
            if (!tf_buffer_->canTransform(world_frame_, cloud_msg->header.frame_id,
                                        cloud_stamp, rclcpp::Duration::from_seconds(0.1))) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Transform not available: %s -> %s",
                                 cloud_msg->header.frame_id.c_str(), world_frame_.c_str());
                return;
            }

            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                world_frame_, cloud_msg->header.frame_id,
                cloud_stamp, rclcpp::Duration::from_seconds(0.1));

            // Transform all points to world frame
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

            // Check each static object
            for (auto& obj : static_objects_) {
                int points_in_bbox = 0;

                // Reset iterator
                iter_x = sensor_msgs::PointCloud2ConstIterator<float>(*cloud_msg, "x");
                iter_y = sensor_msgs::PointCloud2ConstIterator<float>(*cloud_msg, "y");
                iter_z = sensor_msgs::PointCloud2ConstIterator<float>(*cloud_msg, "z");

                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                        continue;
                    }

                    // Transform point to world frame
                    geometry_msgs::msg::PointStamped pt_lidar, pt_world;
                    pt_lidar.header = cloud_msg->header;
                    pt_lidar.point.x = *iter_x;
                    pt_lidar.point.y = *iter_y;
                    pt_lidar.point.z = *iter_z;

                    tf2::doTransform(pt_lidar, pt_world, transform);

                    // Check if point is inside bounding box
                    double dx = std::abs(pt_world.point.x - obj.x);
                    double dy = std::abs(pt_world.point.y - obj.y);
                    double dz = std::abs(pt_world.point.z - obj.z);

                    if (dx <= obj.width / 2.0 &&
                        dy <= obj.depth / 2.0 &&
                        dz <= obj.height / 2.0) {
                        points_in_bbox++;
                    }
                }

                // Update detection status
                bool was_detected = obj.detected;
                obj.detected = (points_in_bbox >= min_points_threshold_);

                if (obj.detected) {
                    obj.detection_count++;
                    obj.last_detection = now();

                    if (!was_detected) {
                        RCLCPP_INFO(get_logger(), "Object '%s' (ID %d) detected! Points: %d",
                                obj.label.c_str(), obj.id, points_in_bbox);
                    }
                } else if (was_detected) {
                    RCLCPP_INFO(get_logger(), "Object '%s' (ID %d) lost", obj.label.c_str(), obj.id);
                }

                RCLCPP_DEBUG(get_logger(), "Object '%s': %d points %s",
                         obj.label.c_str(), points_in_bbox,
                         obj.detected ? "DETECTED" : "not detected");
            }

            // Publish detections
            publishDetections(cloud_msg->header);

        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Transform exception: %s", ex.what());
            return;
        }
    }

    void publishDetections(const std_msgs::msg::Header& header) {
        drive_yolo_msgs::msg::Detections detections_msg;
        detections_msg.header = header;
        detections_msg.header.frame_id = world_frame_;

        for (const auto& obj : static_objects_) {
            if (obj.detected) {
                drive_yolo_msgs::msg::Detection det;
                det.class_id = 99;  // Special ID for static objects
                det.class_name = obj.label;
                det.confidence = std::min(1.0f, obj.detection_count / 10.0f);

                // Store 3D position in unused fields (or create new message type)
                det.x = static_cast<float>(obj.x);
                det.y = static_cast<float>(obj.y);
                det.width = static_cast<float>(obj.width);

                detections_msg.detections.push_back(det);
            }
        }

        if (!detections_msg.detections.empty()) {
            detection_pub_->publish(detections_msg);
        }
    }

    void publishMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;

        for (const auto& obj : static_objects_) {
            // Bounding box marker
            visualization_msgs::msg::Marker box;
            box.header.frame_id = world_frame_;
            box.header.stamp = now();
            box.ns = "static_objects";
            box.id = obj.id;
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;

            box.pose.position.x = obj.x;
            box.pose.position.y = obj.y;
            box.pose.position.z = obj.z;
            box.pose.orientation.w = 1.0;

            box.scale.x = obj.width;
            box.scale.y = obj.depth;
            box.scale.z = obj.height;

            if (obj.detected) {
                // Green when detected
                box.color.r = 0.0;
                box.color.g = 1.0;
                box.color.b = 0.0;
                box.color.a = 0.5;
            } else {
                // Blue when not detected
                box.color.r = 0.0;
                box.color.g = 0.5;
                box.color.b = 1.0;
                box.color.a = 0.3;
            }

            marker_array.markers.push_back(box);

            // Text label
            visualization_msgs::msg::Marker text;
            text.header = box.header;
            text.ns = "static_objects_labels";
            text.id = obj.id + 1000;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;

            text.pose.position.x = obj.x;
            text.pose.position.y = obj.y;
            text.pose.position.z = obj.z + obj.height / 2.0 + 0.5;

            text.scale.z = 0.5;

            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;

            std::stringstream ss;
            ss << obj.label << "\n";
            if (obj.detected) {
                ss << "DETECTED (" << obj.detection_count << ")";
            } else {
                double time_since = (now() - obj.last_detection).seconds();
                if (time_since < detection_timeout_) {
                    ss << "Recent (" << std::fixed << std::setprecision(1) << time_since << "s ago)";
                }
            }
            text.text = ss.str();

            marker_array.markers.push_back(text);
        }

        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    int ret = 0;
    try {
        auto node = std::make_shared<StaticObjectDetectorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("static_object_detector_node"), "Node failed: %s", e.what());
        ret = -1;
    }

    rclcpp::shutdown();
    return ret;
}
