/**
 * Synchronization Diagnostics Node
 *
 * This node measures and reports timing differences between camera detections
 * and LiDAR pointcloud data to help diagnose synchronization issues.
 *
 * Features:
 * - Measures timestamp differences between camera and LiDAR
 * - Maintains configurable queue to buffer messages
 * - Reports statistics (mean, std dev, min, max delays)
 * - Publishes diagnostics for visualization
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <drive_yolo_msgs/msg/detections.hpp>
#include <std_msgs/msg/float64.hpp>
#include <deque>
#include <chrono>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <functional>
#include <string>
#include <vector>

class SyncDiagnostics : public rclcpp::Node
{
public:
    SyncDiagnostics()
        : rclcpp::Node("sync_diagnostics_node"),
          last_camera_stamp_(0, 0, RCL_ROS_TIME),
          last_lidar_stamp_(0, 0, RCL_ROS_TIME),
          last_camera_arrival_(0, 0, RCL_ROS_TIME),
          last_lidar_arrival_(0, 0, RCL_ROS_TIME)
    {
        // Load parameters
        queue_size_ = declare_parameter<int>("queue_size", 50);
        report_interval_ = declare_parameter<double>("report_interval", 2.0);  // Report every 2 seconds
        camera_topic_ = declare_parameter<std::string>("camera_topic", "/detections");
        lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/invz_summation_reflection");

        // Subscribers
        camera_sub_ = create_subscription<drive_yolo_msgs::msg::Detections>(
            camera_topic_, 10, std::bind(&SyncDiagnostics::cameraCallback, this, std::placeholders::_1));
        lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS().keep_last(10),
            std::bind(&SyncDiagnostics::lidarCallback, this, std::placeholders::_1));

        // Publishers for diagnostics
        delay_pub_ = create_publisher<std_msgs::msg::Float64>("/sync_diagnostics/camera_lidar_delay", 10);

        // Timer for periodic reporting
        report_timer_ = create_wall_timer(std::chrono::duration<double>(report_interval_),
                                          std::bind(&SyncDiagnostics::reportStatistics, this));

        RCLCPP_INFO(get_logger(), "=== SYNC DIAGNOSTICS NODE ===");
        RCLCPP_INFO(get_logger(), "Camera topic: %s", camera_topic_.c_str());
        RCLCPP_INFO(get_logger(), "LiDAR topic: %s", lidar_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Queue size: %d", queue_size_);
        RCLCPP_INFO(get_logger(), "Report interval: %.1f sec", report_interval_);
        RCLCPP_INFO(get_logger(), "=============================");
    }

private:
    struct TimestampPair
    {
        rclcpp::Time camera_stamp;
        rclcpp::Time lidar_stamp;
        rclcpp::Time arrival_time;
        double delay_ms;
    };

    void cameraCallback(const drive_yolo_msgs::msg::Detections::ConstSharedPtr msg)
    {
        last_camera_stamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
        last_camera_arrival_ = now();
        camera_count_++;

        // If we have a recent LiDAR message, compute delay
        if (last_lidar_stamp_.nanoseconds() != 0)
        {
            double delay = (last_camera_stamp_ - last_lidar_stamp_).seconds() * 1000.0;  // ms

            TimestampPair pair;
            pair.camera_stamp = last_camera_stamp_;
            pair.lidar_stamp = last_lidar_stamp_;
            pair.arrival_time = now();
            pair.delay_ms = delay;

            addDelay(pair);

            // Publish current delay
            std_msgs::msg::Float64 delay_msg;
            delay_msg.data = delay;
            delay_pub_->publish(delay_msg);
        }
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        last_lidar_stamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
        last_lidar_arrival_ = now();
        lidar_count_++;

        // If we have a recent camera message, compute delay
        if (last_camera_stamp_.nanoseconds() != 0)
        {
            double delay = (last_camera_stamp_ - last_lidar_stamp_).seconds() * 1000.0;  // ms

            TimestampPair pair;
            pair.camera_stamp = last_camera_stamp_;
            pair.lidar_stamp = last_lidar_stamp_;
            pair.arrival_time = now();
            pair.delay_ms = delay;

            addDelay(pair);

            // Publish current delay
            std_msgs::msg::Float64 delay_msg;
            delay_msg.data = delay;
            delay_pub_->publish(delay_msg);
        }
    }

    void addDelay(const TimestampPair& pair)
    {
        delay_queue_.push_back(pair);

        // Maintain queue size
        while (delay_queue_.size() > static_cast<size_t>(queue_size_))
        {
            delay_queue_.pop_front();
        }
    }

    void reportStatistics()
    {
        if (delay_queue_.empty())
        {
            RCLCPP_WARN(get_logger(), "No timing data collected yet. Camera msgs: %d, LiDAR msgs: %d",
                     camera_count_, lidar_count_);
            return;
        }

        // Calculate statistics
        std::vector<double> delays;
        delays.reserve(delay_queue_.size());
        for (const auto& pair : delay_queue_)
        {
            delays.push_back(pair.delay_ms);
        }

        double mean = std::accumulate(delays.begin(), delays.end(), 0.0) / delays.size();

        double variance = 0.0;
        for (double delay : delays)
        {
            variance += (delay - mean) * (delay - mean);
        }
        variance /= delays.size();
        double std_dev = std::sqrt(variance);

        auto minmax = std::minmax_element(delays.begin(), delays.end());
        double min_delay = *minmax.first;
        double max_delay = *minmax.second;

        // Calculate median
        std::vector<double> sorted_delays = delays;
        std::sort(sorted_delays.begin(), sorted_delays.end());
        double median = sorted_delays[sorted_delays.size() / 2];

        RCLCPP_INFO(get_logger(), "=== SYNC DIAGNOSTICS REPORT ===");
        RCLCPP_INFO(get_logger(), "Messages received: Camera=%d, LiDAR=%d", camera_count_, lidar_count_);
        RCLCPP_INFO(get_logger(), "Queue size: %zu/%d samples", delay_queue_.size(), queue_size_);
        RCLCPP_INFO(get_logger(), "Camera-LiDAR timestamp offset (positive = camera ahead):");
        RCLCPP_INFO(get_logger(), "  Mean:   %+7.2f ms", mean);
        RCLCPP_INFO(get_logger(), "  Median: %+7.2f ms", median);
        RCLCPP_INFO(get_logger(), "  StdDev:  %6.2f ms", std_dev);
        RCLCPP_INFO(get_logger(), "  Min:    %+7.2f ms", min_delay);
        RCLCPP_INFO(get_logger(), "  Max:    %+7.2f ms", max_delay);

        // Provide recommendation based on mean delay
        if (std::abs(mean) > 50.0)
        {
            int frames_offset = static_cast<int>(std::round(mean / 33.33));  // Assuming ~30 Hz
            RCLCPP_WARN(get_logger(), "Large synchronization offset detected!");
            RCLCPP_WARN(get_logger(), "Recommendation: Set time_offset_ms = %.1f in fusion config", mean);
            RCLCPP_WARN(get_logger(), "Or use frame_offset = %d (at 30 Hz)", frames_offset);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Synchronization looks good (offset < 50 ms)");
        }

        // Show most recent samples
        RCLCPP_INFO(get_logger(), "Recent samples (last 5):");
        int start = std::max(0, static_cast<int>(delay_queue_.size()) - 5);
        for (size_t i = start; i < delay_queue_.size(); i++)
        {
            const auto& pair = delay_queue_[i];
            RCLCPP_INFO(get_logger(), "  [%zu] Camera: %.3f, LiDAR: %.3f, Delay: %+7.2f ms",
                     i, pair.camera_stamp.seconds(), pair.lidar_stamp.seconds(), pair.delay_ms);
        }

        RCLCPP_INFO(get_logger(), "===============================");
    }

    rclcpp::Subscription<drive_yolo_msgs::msg::Detections>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr delay_pub_;
    rclcpp::TimerBase::SharedPtr report_timer_;

    std::string camera_topic_;
    std::string lidar_topic_;
    int queue_size_;
    double report_interval_;

    rclcpp::Time last_camera_stamp_;
    rclcpp::Time last_lidar_stamp_;
    rclcpp::Time last_camera_arrival_;
    rclcpp::Time last_lidar_arrival_;

    int camera_count_ = 0;
    int lidar_count_ = 0;

    std::deque<TimestampPair> delay_queue_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    int ret = 0;
    try {
        auto node = std::make_shared<SyncDiagnostics>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("sync_diagnostics_node"), "Node failed: %s", e.what());
        ret = -1;
    }

    rclcpp::shutdown();
    return ret;
}
