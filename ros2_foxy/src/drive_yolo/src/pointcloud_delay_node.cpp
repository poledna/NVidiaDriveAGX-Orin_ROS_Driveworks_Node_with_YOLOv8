/**
 * Pointcloud Delay Node
 *
 * This node adds artificial delay to pointcloud messages for testing
 * and visualization purposes. Useful for:
 * - Demonstrating synchronization issues
 * - Testing sync diagnostic and fusion configurations
 * - Creating before/after comparison videos
 *
 * Features:
 * - Configurable delay (milliseconds or frame count)
 * - Queue-based buffering
 * - Real-time statistics reporting
 * - Passthrough mode for quick enable/disable
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <deque>
#include <chrono>
#include <functional>
#include <string>

class PointCloudDelayNode : public rclcpp::Node
{
public:
    PointCloudDelayNode()
        : rclcpp::Node("pointcloud_delay_node"),
          start_time_(0, 0, RCL_ROS_TIME)
    {
        // Load parameters
        input_topic_ = declare_parameter<std::string>("input_topic", "/invz_summation_reflection");
        output_topic_ = declare_parameter<std::string>("output_topic", "/invz_summation_reflection_delayed");
        delay_ms_ = declare_parameter<double>("delay_ms", 50.0);  // 50ms default delay
        frame_delay_ = declare_parameter<int>("frame_delay", 0);  // 0 = use time delay, >0 = use frame delay
        queue_size_ = declare_parameter<int>("queue_size", 100);
        passthrough_ = declare_parameter<bool>("passthrough", false);
        report_interval_ = declare_parameter<double>("report_interval", 5.0);  // Report stats every 5 seconds
        preserve_timestamps_ = declare_parameter<bool>("preserve_timestamps", false);  // Keep original timestamps

        // Subscribers and publishers
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS().keep_last(queue_size_),
            std::bind(&PointCloudDelayNode::cloudCallback, this, std::placeholders::_1));
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        queue_size_pub_ = create_publisher<std_msgs::msg::Int32>("/pointcloud_delay/queue_size", 10);

        // Timer for publishing delayed messages
        publish_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&PointCloudDelayNode::publishCallback, this));

        // Timer for statistics reporting
        report_timer_ = create_wall_timer(std::chrono::duration<double>(report_interval_),
                                          std::bind(&PointCloudDelayNode::reportStatistics, this));

        RCLCPP_INFO(get_logger(), "=== POINTCLOUD DELAY NODE ===");
        RCLCPP_INFO(get_logger(), "Input topic:  %s", input_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Output topic: %s", output_topic_.c_str());

        if (passthrough_) {
            RCLCPP_WARN(get_logger(), "PASSTHROUGH MODE: No delay applied");
        } else if (frame_delay_ > 0) {
            RCLCPP_INFO(get_logger(), "Frame delay: %d frames (%.1f ms at 30Hz)", frame_delay_, frame_delay_ * 33.33);
        } else {
            RCLCPP_INFO(get_logger(), "Time delay: %.1f ms", delay_ms_);
        }

        RCLCPP_INFO(get_logger(), "Queue size: %d", queue_size_);
        RCLCPP_INFO(get_logger(), "Preserve timestamps: %s", preserve_timestamps_ ? "YES" : "NO (shift by delay)");
        RCLCPP_INFO(get_logger(), "============================");

        start_time_ = now();
    }

private:
    struct DelayedMessage
    {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
        rclcpp::Time received_time;
        rclcpp::Time publish_time;  // When this message should be published
    };

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        messages_received_++;

        // Passthrough mode - republish immediately
        if (passthrough_)
        {
            cloud_pub_->publish(*msg);
            messages_published_++;
            return;
        }

        // Add to queue with calculated publish time
        DelayedMessage delayed_msg;
        delayed_msg.msg = msg;
        delayed_msg.received_time = now();

        if (frame_delay_ > 0)
        {
            // Frame-based delay: wait for N more frames
            delayed_msg.publish_time = delayed_msg.received_time;  // Will be published after N frames
        }
        else
        {
            // Time-based delay
            delayed_msg.publish_time = delayed_msg.received_time + rclcpp::Duration::from_seconds(delay_ms_ / 1000.0);
        }

        message_queue_.push_back(delayed_msg);

        // Maintain queue size limit
        while (message_queue_.size() > static_cast<size_t>(queue_size_))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Queue full (%d), dropping oldest message", queue_size_);
            message_queue_.pop_front();
            messages_dropped_++;
        }

        // Publish queue size for monitoring
        std_msgs::msg::Int32 queue_size_msg;
        queue_size_msg.data = message_queue_.size();
        queue_size_pub_->publish(queue_size_msg);
    }

    void publishCallback()
    {
        if (passthrough_ || message_queue_.empty())
        {
            return;
        }

        const rclcpp::Time now = this->now();

        // Frame-based delay: always publish from front if we have enough frames queued
        if (frame_delay_ > 0)
        {
            if (message_queue_.size() >= static_cast<size_t>(frame_delay_))
            {
                publishMessage(message_queue_.front());
                message_queue_.pop_front();
            }
        }
        // Time-based delay: publish all messages that are ready
        else
        {
            while (!message_queue_.empty() && message_queue_.front().publish_time <= now)
            {
                publishMessage(message_queue_.front());
                message_queue_.pop_front();
            }
        }
    }

    void publishMessage(const DelayedMessage& delayed_msg)
    {
        // Create a copy of the message to potentially modify timestamp
        sensor_msgs::msg::PointCloud2 output_msg = *delayed_msg.msg;

        if (!preserve_timestamps_)
        {
            // Shift timestamp by the delay amount
            if (frame_delay_ > 0)
            {
                // Calculate actual delay based on frame count and average frame rate
                double avg_delay = frame_delay_ * 33.33 / 1000.0;  // Assuming ~30Hz
                output_msg.header.stamp = rclcpp::Time(output_msg.header.stamp) + rclcpp::Duration::from_seconds(avg_delay);
            }
            else
            {
                output_msg.header.stamp = rclcpp::Time(output_msg.header.stamp) + rclcpp::Duration::from_seconds(delay_ms_ / 1000.0);
            }
        }

        cloud_pub_->publish(output_msg);
        messages_published_++;

        // Track actual delay
        double actual_delay = (now() - delayed_msg.received_time).seconds() * 1000.0;
        total_delay_ += actual_delay;
    }

    void reportStatistics()
    {
        double uptime = (now() - start_time_).seconds();
        double avg_rate = messages_received_ / uptime;
        double avg_delay = messages_published_ > 0 ? total_delay_ / messages_published_ : 0.0;

        RCLCPP_INFO(get_logger(), "=== POINTCLOUD DELAY STATISTICS ===");
        RCLCPP_INFO(get_logger(), "Uptime: %.1f seconds", uptime);
        RCLCPP_INFO(get_logger(), "Messages: Received=%d, Published=%d, Dropped=%d",
                 messages_received_, messages_published_, messages_dropped_);
        RCLCPP_INFO(get_logger(), "Current queue size: %zu / %d", message_queue_.size(), queue_size_);
        RCLCPP_INFO(get_logger(), "Average rate: %.1f Hz", avg_rate);
        RCLCPP_INFO(get_logger(), "Average actual delay: %.1f ms", avg_delay);

        if (frame_delay_ > 0)
        {
            RCLCPP_INFO(get_logger(), "Frame delay: %d frames (queue needs %d to start publishing)",
                     frame_delay_, frame_delay_);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Configured delay: %.1f ms", delay_ms_);
        }

        if (message_queue_.size() >= static_cast<size_t>(queue_size_ * 0.9))
        {
            RCLCPP_WARN(get_logger(), "Queue nearly full! Consider increasing queue_size parameter.");
        }

        RCLCPP_INFO(get_logger(), "===================================");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr queue_size_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr report_timer_;

    std::string input_topic_;
    std::string output_topic_;
    double delay_ms_;
    int frame_delay_;
    int queue_size_;
    bool passthrough_;
    double report_interval_;
    bool preserve_timestamps_;

    std::deque<DelayedMessage> message_queue_;

    // Statistics
    rclcpp::Time start_time_;
    int messages_received_ = 0;
    int messages_published_ = 0;
    int messages_dropped_ = 0;
    double total_delay_ = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    int ret = 0;
    try {
        auto node = std::make_shared<PointCloudDelayNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pointcloud_delay_node"), "Node failed: %s", e.what());
        ret = -1;
    }

    rclcpp::shutdown();
    return ret;
}
