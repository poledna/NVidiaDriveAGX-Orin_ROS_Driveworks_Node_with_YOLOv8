#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "drive_yolo_msgs/msg/tracked_objects3_d.hpp"
#include "drive_yolo_msgs/msg/tracked_object3_d.hpp"
#include <vector>
#include <set>
#include <sstream>
#include <chrono>
#include <functional>
#include <iomanip>
#include <string>

class SpeedMonitorNode : public rclcpp::Node {
public:
    SpeedMonitorNode() : rclcpp::Node("speed_monitor_node") {
        // Get parameters
        speed_limit_ms_ = declare_parameter<double>("speed_limit_ms", 20.89);  
        warn_threshold_ms_ = declare_parameter<double>("warn_threshold_ms", 11.11);  // 40 km/h
        monitor_all_classes_ = declare_parameter<bool>("monitor_all_classes", false);
        quiet_mode_ = declare_parameter<bool>("quiet_mode", false);
        
        // Get list of classes to monitor
        std::string monitor_classes_str;
        monitor_classes_str = declare_parameter<std::string>("monitor_classes", "0,1,2,3,5,7");  // person, bicycle, car, motorcycle, bus, truck
        parseMonitorClasses(monitor_classes_str);
        
        // Convert speeds to km/h for display
        speed_limit_kmh_ = speed_limit_ms_ * 3.6;
        warn_threshold_kmh_ = warn_threshold_ms_ * 3.6;
        
        // Setup ROS communication
        tracked_objects_sub_ = create_subscription<drive_yolo_msgs::msg::TrackedObjects3D>(
            "/tracked_objects_3d", 10,
            std::bind(&SpeedMonitorNode::trackedObjectsCallback, this, std::placeholders::_1));
        
        speed_warnings_pub_ = create_publisher<std_msgs::msg::String>("/speed_warnings", 10);
        speed_stats_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/speed_statistics", 10);
        
        RCLCPP_INFO(get_logger(), "=== SPEED MONITOR INITIALIZED ===");
        RCLCPP_INFO(get_logger(), "Speed limit: %.1f km/h (%.2f m/s)", speed_limit_kmh_, speed_limit_ms_);
        RCLCPP_INFO(get_logger(), "Warning threshold: %.1f km/h (%.2f m/s)", warn_threshold_kmh_, warn_threshold_ms_);
        RCLCPP_INFO(get_logger(), "Monitor all classes: %s", monitor_all_classes_ ? "YES" : "NO");
        
        if (!monitor_all_classes_) {
            std::stringstream ss;
            for (auto it = monitored_classes_.begin(); it != monitored_classes_.end(); ++it) {
                if (it != monitored_classes_.begin()) ss << ", ";
                ss << *it << " (" << getClassName(*it) << ")";
            }
            RCLCPP_INFO(get_logger(), "Monitored classes: %s", ss.str().c_str());
        }
        
        RCLCPP_INFO(get_logger(), "Quiet mode: %s", quiet_mode_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(get_logger(), "Subscribing to: /tracked_objects_3d");
        RCLCPP_INFO(get_logger(), "Publishing to: /speed_warnings, /speed_statistics");
        RCLCPP_INFO(get_logger(), "===============================");
        
        // Initialize statistics
        total_objects_seen_ = 0;
        speeding_objects_count_ = 0;
        max_speed_seen_ = 0.0;
    }

private:
    rclcpp::Subscription<drive_yolo_msgs::msg::TrackedObjects3D>::SharedPtr tracked_objects_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speed_warnings_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr speed_stats_pub_;
    
    // Parameters
    double speed_limit_ms_;
    double warn_threshold_ms_;
    double speed_limit_kmh_;
    double warn_threshold_kmh_;
    bool monitor_all_classes_;
    bool quiet_mode_;
    std::set<int> monitored_classes_;
    
    // Statistics
    int total_objects_seen_;
    int speeding_objects_count_;
    double max_speed_seen_;
    
    void parseMonitorClasses(const std::string& classes_str) {
        std::stringstream ss(classes_str);
        std::string class_id_str;
        
        while (std::getline(ss, class_id_str, ',')) {
            try {
                int class_id = std::stoi(class_id_str);
                monitored_classes_.insert(class_id);
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Invalid class ID: %s", class_id_str.c_str());
            }
        }
    }
    
    void trackedObjectsCallback(const drive_yolo_msgs::msg::TrackedObjects3D::ConstSharedPtr msg) {
        std::vector<std::string> warnings;
        std::vector<std::string> info_messages;
        
        double max_speed_this_frame = 0.0;
        int moving_objects_count = 0;
        int monitored_objects_count = 0;
        int speeding_objects_this_frame = 0;
        
        for (const auto& obj : msg->objects) {
            // Skip if not a monitored class
            if (!monitor_all_classes_ && monitored_classes_.find(obj.class_id) == monitored_classes_.end()) {
                continue;
            }
            
            monitored_objects_count++;
            
            // Only consider moving objects
            if (!obj.is_moving) {
                continue;
            }
            
            moving_objects_count++;
            double speed_kmh = obj.speed_3d * 3.6;  // Convert m/s to km/h
            
            // Update statistics
            if (obj.speed_3d > max_speed_seen_) {
                max_speed_seen_ = obj.speed_3d;
            }
            
            if (obj.speed_3d > max_speed_this_frame) {
                max_speed_this_frame = obj.speed_3d;
            }
            
            // Check for speeding
            if (obj.speed_3d > speed_limit_ms_) {
                speeding_objects_count_++;
                speeding_objects_this_frame++;
                
                std::stringstream warning;
                warning << "SPEEDING: " << obj.class_name << " (ID:" << obj.id 
                       << ") at " << std::fixed << std::setprecision(1) << speed_kmh 
                       << " km/h (limit: " << speed_limit_kmh_ << " km/h)"
                       << " - Position: [" << std::setprecision(2) 
                       << obj.position_3d.x << ", " << obj.position_3d.y << ", " << obj.position_3d.z << "]m";
                warnings.push_back(warning.str());
                
            } else if (obj.speed_3d > warn_threshold_ms_) {
                std::stringstream info;
                info << "HIGH SPEED: " << obj.class_name << " (ID:" << obj.id 
                    << ") at " << std::fixed << std::setprecision(1) << speed_kmh 
                    << " km/h - Position: [" << std::setprecision(2)
                    << obj.position_3d.x << ", " << obj.position_3d.y << ", " << obj.position_3d.z << "]m";
                info_messages.push_back(info.str());
            }
            
            // Log detailed info for very fast objects (even in quiet mode)
            if (obj.speed_3d > 25.0) {  // > 90 km/h
                RCLCPP_WARN(get_logger(), "VERY HIGH SPEED DETECTED: %s (ID:%d) at %.1f km/h - Acceleration: [%.2f, %.2f, %.2f] m/s²",
                         obj.class_name.c_str(), obj.id, speed_kmh,
                         obj.acceleration_3d.x, obj.acceleration_3d.y, obj.acceleration_3d.z);
            }
        }
        
        total_objects_seen_ += monitored_objects_count;
        
        // Publish warnings
        if (!warnings.empty()) {
            std_msgs::msg::String warning_msg;
            std::stringstream combined_warnings;
            for (size_t i = 0; i < warnings.size(); ++i) {
                if (i > 0) combined_warnings << "; ";
                combined_warnings << warnings[i];
            }
            warning_msg.data = combined_warnings.str();
            speed_warnings_pub_->publish(warning_msg);
        }
        
        // Log results (respecting quiet mode)
        if (!warnings.empty()) {
            // Always show speeding warnings
            for (const auto& warning : warnings) {
                RCLCPP_WARN(get_logger(), "%s", warning.c_str());
            }
        } else if (!quiet_mode_ && !info_messages.empty()) {
            // Show high speed info only if not in quiet mode
            for (const auto& info : info_messages) {
                RCLCPP_INFO(get_logger(), "%s", info.c_str());
            }
        }
        
        // Publish statistics
        geometry_msgs::msg::Vector3 stats;
        stats.x = max_speed_this_frame * 3.6;  // Max speed this frame (km/h)
        stats.y = moving_objects_count;         // Number of moving objects
        stats.z = speeding_objects_this_frame;  // Number of speeding objects
        speed_stats_pub_->publish(stats);
        
        // Periodic summary (every 30 seconds)
        static auto last_summary = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto time_since_summary = std::chrono::duration_cast<std::chrono::seconds>(now - last_summary).count();
        
        if (time_since_summary >= 30) {
            double max_speed_kmh = max_speed_seen_ * 3.6;
            double speeding_percentage = total_objects_seen_ > 0 ? 
                (static_cast<double>(speeding_objects_count_) / total_objects_seen_ * 100.0) : 0.0;
            
            RCLCPP_INFO(get_logger(), "=== SPEED MONITORING SUMMARY ===");
            RCLCPP_INFO(get_logger(), "Objects monitored: %d", total_objects_seen_);
            RCLCPP_INFO(get_logger(), "Speeding incidents: %d (%.1f%%)", speeding_objects_count_, speeding_percentage);
            RCLCPP_INFO(get_logger(), "Max speed recorded: %.1f km/h (%.2f m/s)", max_speed_kmh, max_speed_seen_);
            RCLCPP_INFO(get_logger(), "Current frame: %d moving, %d speeding", moving_objects_count, speeding_objects_this_frame);
            RCLCPP_INFO(get_logger(), "==============================");
            
            last_summary = now;
        }
    }
    
    std::string getClassName(int class_id) {
        /*static const std::vector<std::string> class_names = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
            "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
            "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
            "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
            "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
            "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
            "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
            "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
            "toothbrush"
        };*/
        static const std::vector<std::string> class_names = {
            "vehicle", "pedestrian", "bicycle", "motorcycle"
        };
        if (class_id >= 0 && class_id < static_cast<int>(class_names.size())) {
            return class_names[class_id];
        }
        return "unknown";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    int ret = 0;
    try {
        auto node = std::make_shared<SpeedMonitorNode>();
        
        RCLCPP_INFO(node->get_logger(), "Speed Monitor Node running... Press Ctrl+C to stop");
        rclcpp::spin(node);
        
        RCLCPP_INFO(node->get_logger(), "Speed Monitor Node shutting down");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("speed_monitor_node"), "Speed Monitor node failed: %s", e.what());
        ret = -1;
    }
    
    rclcpp::shutdown();
    return ret;
}
