/**
 * Point Cloud Filter Node
 *
 * Applies various filters to incoming point clouds:
 * - Distance filtering (min/max range)
 * - ROI filtering (region of interest)
 * - Ground plane removal
 * - Outlier removal
 *
 * For easier debugging with visualization and configurable parameters.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <vector>
#include <cmath>
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

class PointCloudFilterNode : public rclcpp::Node {
public:
    PointCloudFilterNode() : rclcpp::Node("pointcloud_filter_node") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Distance filtering
        min_distance_ = declare_parameter<double>("min_distance", 0.5);
        max_distance_ = declare_parameter<double>("max_distance", 100.0);

        // ROI filtering
        enable_roi_filtering_ = declare_parameter<bool>("enable_roi_filtering", true);
        roi_x_min_ = declare_parameter<double>("roi_x_min", 0.0);
        roi_x_max_ = declare_parameter<double>("roi_x_max", 50.0);
        roi_y_min_ = declare_parameter<double>("roi_y_min", -10.0);
        roi_y_max_ = declare_parameter<double>("roi_y_max", 10.0);
        roi_z_min_ = declare_parameter<double>("roi_z_min", -2.0);
        roi_z_max_ = declare_parameter<double>("roi_z_max", 10.0);

        // Ground filtering
        enable_ground_filtering_ = declare_parameter<bool>("enable_ground_filtering", false);
        ground_z_threshold_ = declare_parameter<double>("ground_z_threshold", -1.0);

        // Visualization
        enable_visualization_ = declare_parameter<bool>("enable_visualization", true);
        target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

        // Topics
        std::string input_topic, output_topic;
        input_topic = declare_parameter<std::string>("input_topic", "/lidar/points");
        output_topic = declare_parameter<std::string>("output_topic", "/lidar/points_filtered");

        // Publishers and subscribers
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS().keep_last(10),
            std::bind(&PointCloudFilterNode::cloudCallback, this, std::placeholders::_1));
        filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        if (enable_visualization_) {
            roi_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/pointcloud_filter/roi_markers", 1);
            stats_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/pointcloud_filter/stats", 1);
            publishROIMarkers();
        }

        RCLCPP_INFO(get_logger(), "=== Point Cloud Filter Node Started ===");
        RCLCPP_INFO(get_logger(), "Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(get_logger(), "Output topic: %s", output_topic.c_str());
        RCLCPP_INFO(get_logger(), "Distance filter: [%.1f, %.1f] m", min_distance_, max_distance_);
        RCLCPP_INFO(get_logger(), "ROI filtering: %s", enable_roi_filtering_ ? "ENABLED" : "DISABLED");
        if (enable_roi_filtering_) {
            RCLCPP_INFO(get_logger(), "  X: [%.1f, %.1f] m", roi_x_min_, roi_x_max_);
            RCLCPP_INFO(get_logger(), "  Y: [%.1f, %.1f] m", roi_y_min_, roi_y_max_);
            RCLCPP_INFO(get_logger(), "  Z: [%.1f, %.1f] m", roi_z_min_, roi_z_max_);
        }
        RCLCPP_INFO(get_logger(), "Ground filtering: %s (Z < %.2f m)",
                 enable_ground_filtering_ ? "ENABLED" : "DISABLED", ground_z_threshold_);
        RCLCPP_INFO(get_logger(), "======================================");

        stats_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&PointCloudFilterNode::publishStats, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr roi_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr stats_marker_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    double min_distance_, max_distance_;
    bool enable_roi_filtering_;
    double roi_x_min_, roi_x_max_;
    double roi_y_min_, roi_y_max_;
    double roi_z_min_, roi_z_max_;
    bool enable_ground_filtering_;
    double ground_z_threshold_;
    bool enable_visualization_;
    std::string target_frame_;

    // Statistics
    size_t total_points_in_ = 0;
    size_t total_points_out_ = 0;
    size_t points_filtered_distance_ = 0;
    size_t points_filtered_roi_ = 0;
    size_t points_filtered_ground_ = 0;
    double last_processing_time_ms_ = 0.0;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) {
        auto start_time = now();

        sensor_msgs::msg::PointCloud2 filtered_cloud;
        filtered_cloud.header = cloud_msg->header;
        filtered_cloud.height = 1;
        filtered_cloud.is_dense = false;
        filtered_cloud.is_bigendian = cloud_msg->is_bigendian;

        // Setup fields (x, y, z, intensity if available)
        filtered_cloud.fields = cloud_msg->fields;
        filtered_cloud.point_step = cloud_msg->point_step;

        // Temporary storage for filtered points
        std::vector<uint8_t> filtered_data;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

        size_t input_count = 0;
        size_t output_count = 0;
        size_t dist_filtered = 0;
        size_t roi_filtered = 0;
        size_t ground_filtered = 0;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            input_count++;

            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // Skip invalid points
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            // Distance filter
            double distance = std::sqrt(x*x + y*y + z*z);
            if (distance < min_distance_ || distance > max_distance_) {
                dist_filtered++;
                continue;
            }

            // ROI filter
            if (enable_roi_filtering_) {
                if (x < roi_x_min_ || x > roi_x_max_ ||
                    y < roi_y_min_ || y > roi_y_max_ ||
                    z < roi_z_min_ || z > roi_z_max_) {
                    roi_filtered++;
                    continue;
                }
            }

            // Ground filter
            if (enable_ground_filtering_ && z < ground_z_threshold_) {
                ground_filtered++;
                continue;
            }

            // Point passed all filters - copy raw bytes
	    const uint8_t* point_data = reinterpret_cast<const uint8_t*>(&(*iter_x)) - 
                            cloud_msg->fields[0].offset;
            filtered_data.insert(filtered_data.end(),
                               point_data,
                               point_data + cloud_msg->point_step);
            output_count++;
        }

        // Update global statistics
        total_points_in_ = input_count;
        total_points_out_ = output_count;
        points_filtered_distance_ = dist_filtered;
        points_filtered_roi_ = roi_filtered;
        points_filtered_ground_ = ground_filtered;

        // Set filtered cloud data
        filtered_cloud.data = filtered_data;
        filtered_cloud.width = output_count;
        filtered_cloud.row_step = filtered_cloud.point_step * filtered_cloud.width;

        filtered_cloud_pub_->publish(filtered_cloud);

        last_processing_time_ms_ = (now() - start_time).seconds() * 1000.0;

        RCLCPP_DEBUG(get_logger(), "Filtered: %zu -> %zu points (%.1f%%), time: %.2f ms",
                 input_count, output_count,
                 input_count > 0 ? 100.0 * output_count / input_count : 0.0,
                 last_processing_time_ms_);
    }

    void publishROIMarkers() {
        if (!enable_roi_filtering_) return;

        visualization_msgs::msg::MarkerArray marker_array;

        // Create ROI box marker
        visualization_msgs::msg::Marker roi_box;
        roi_box.header.frame_id = target_frame_;
        roi_box.header.stamp = now();
        roi_box.ns = "roi_box";
        roi_box.id = 0;
        roi_box.type = visualization_msgs::msg::Marker::CUBE;
        roi_box.action = visualization_msgs::msg::Marker::ADD;

        roi_box.pose.position.x = (roi_x_min_ + roi_x_max_) / 2.0;
        roi_box.pose.position.y = (roi_y_min_ + roi_y_max_) / 2.0;
        roi_box.pose.position.z = (roi_z_min_ + roi_z_max_) / 2.0;
        roi_box.pose.orientation.w = 1.0;

        roi_box.scale.x = roi_x_max_ - roi_x_min_;
        roi_box.scale.y = roi_y_max_ - roi_y_min_;
        roi_box.scale.z = roi_z_max_ - roi_z_min_;

        roi_box.color.r = 0.0;
        roi_box.color.g = 1.0;
        roi_box.color.b = 0.0;
        roi_box.color.a = 0.2;

        marker_array.markers.push_back(roi_box);
        roi_marker_pub_->publish(marker_array);
    }

    void publishStats() {
        if (!enable_visualization_) return;

        // Create text marker with statistics
        visualization_msgs::msg::Marker stats_marker;
        stats_marker.header.frame_id = target_frame_;
        stats_marker.header.stamp = now();
        stats_marker.ns = "filter_stats";
        stats_marker.id = 0;
        stats_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        stats_marker.action = visualization_msgs::msg::Marker::ADD;

        stats_marker.pose.position.x = roi_x_max_ + 5.0;
        stats_marker.pose.position.y = 0.0;
        stats_marker.pose.position.z = 5.0;
        stats_marker.pose.orientation.w = 1.0;

        stats_marker.scale.z = 1.0;  // Text height

        stats_marker.color.r = 1.0;
        stats_marker.color.g = 1.0;
        stats_marker.color.b = 1.0;
        stats_marker.color.a = 1.0;

        std::stringstream ss;
        ss << "PointCloud Filter Stats\n";
        ss << "In: " << total_points_in_ << " points\n";
        ss << "Out: " << total_points_out_ << " points\n";
        if (total_points_in_ > 0) {
            ss << "Pass rate: " << std::fixed << std::setprecision(1)
               << (100.0 * total_points_out_ / total_points_in_) << "%\n";
        }
        ss << "Filtered by distance: " << points_filtered_distance_ << "\n";
        ss << "Filtered by ROI: " << points_filtered_roi_ << "\n";
        ss << "Filtered by ground: " << points_filtered_ground_ << "\n";
        ss << "Processing time: " << std::fixed << std::setprecision(2)
           << last_processing_time_ms_ << " ms";

        stats_marker.text = ss.str();

        stats_marker_pub_->publish(stats_marker);

        // Also republish ROI markers
        publishROIMarkers();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    int ret = 0;
    try {
        auto node = std::make_shared<PointCloudFilterNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pointcloud_filter_node"), "Node failed: %s", e.what());
        ret = -1;
    }

    rclcpp::shutdown();
    return ret;
}
