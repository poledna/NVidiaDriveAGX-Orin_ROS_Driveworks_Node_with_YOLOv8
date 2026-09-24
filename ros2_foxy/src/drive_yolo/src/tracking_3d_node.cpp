#include <rclcpp/rclcpp.hpp>
#include "drive_yolo/object_tracker_3d.hpp"
#include "drive_yolo_msgs/msg/detections_with_distance.hpp"
#include "drive_yolo_msgs/msg/tracked_objects3_d.hpp"    // Need to create this message
#include "drive_yolo_msgs/msg/tracked_object3_d.hpp"     // Need to create this message
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <functional>
#include <vector>

class Tracking3DNode : public rclcpp::Node {
public:
    Tracking3DNode() : rclcpp::Node("tracking_3d_node") {
        // Get parameters
        distance_threshold_3d_ = declare_parameter<double>("distance_threshold_3d", 2.0);  // 2 meters
        max_missed_frames_ = declare_parameter<int>("max_missed_frames", 8);
        min_detections_for_tracking_ = declare_parameter<int>("min_detections_for_tracking", 3);
        min_lidar_points_ = declare_parameter<int>("min_lidar_points", 5);
        min_speed_threshold_ = static_cast<float>(declare_parameter<double>("min_speed_threshold", 0.1));  // 0.1 m/s
        quiet_mode_ = declare_parameter<bool>("quiet_mode", false);
        publish_all_tracks_ = declare_parameter<bool>("publish_all_tracks", true);
        publish_high_quality_only_ = declare_parameter<bool>("publish_high_quality_only", false);
        max_tracking_distance_ = static_cast<float>(declare_parameter<double>("max_tracking_distance", 30.0));  // 30 meters
        
        // Configure tracker
        tracker_.setDistanceThreshold3D(distance_threshold_3d_);
        tracker_.setMaxMissedFrames(max_missed_frames_);
        tracker_.setMinDetectionsForTracking(min_detections_for_tracking_);
        tracker_.setMinLidarPoints(min_lidar_points_);
        tracker_.setMaxTrackingDistance(max_tracking_distance_);
        
        // Setup ROS communication
        detections_sub_ = create_subscription<drive_yolo_msgs::msg::DetectionsWithDistance>(
            "/detections_with_distance", 10,
            std::bind(&Tracking3DNode::detectionsCallback, this, std::placeholders::_1));
        
        tracked_objects_pub_ = create_publisher<drive_yolo_msgs::msg::TrackedObjects3D>("/tracked_objects_3d", 10);
        
        // Optional publishers for filtered results
        high_quality_tracks_pub_ = create_publisher<drive_yolo_msgs::msg::TrackedObjects3D>("/high_quality_tracks_3d", 10);
        moving_objects_pub_ = create_publisher<drive_yolo_msgs::msg::TrackedObjects3D>("/moving_objects_3d", 10);
        
        RCLCPP_INFO(get_logger(), "=== 3D OBJECT TRACKER INITIALIZED ===");
        RCLCPP_INFO(get_logger(), "Distance threshold (3D): %.1f meters", distance_threshold_3d_);
        RCLCPP_INFO(get_logger(), "Max missed frames: %d", max_missed_frames_);
        RCLCPP_INFO(get_logger(), "Min detections for tracking: %d", min_detections_for_tracking_);
        RCLCPP_INFO(get_logger(), "Min LiDAR points: %d", min_lidar_points_);
        RCLCPP_INFO(get_logger(), "Min speed threshold: %.2f m/s", min_speed_threshold_);
        RCLCPP_INFO(get_logger(), "Max tracking distance: %.1f meters", max_tracking_distance_);
        RCLCPP_INFO(get_logger(), "Quiet mode: %s", quiet_mode_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(get_logger(), "Publish all tracks: %s", publish_all_tracks_ ? "YES" : "NO");
        RCLCPP_INFO(get_logger(), "High quality only: %s", publish_high_quality_only_ ? "YES" : "NO");
        RCLCPP_INFO(get_logger(), "Subscribing to: /detections_with_distance");
        RCLCPP_INFO(get_logger(), "Publishing to: /tracked_objects_3d, /high_quality_tracks_3d, /moving_objects_3d");
        RCLCPP_INFO(get_logger(), "====================================");
        
        // Performance tracking
        frame_count_ = 0;
        start_time_ = std::chrono::high_resolution_clock::now();
        last_stats_report_ = start_time_;
        total_processing_time_ = 0.0;
    }

private:
    using TrackedObjects3DPublisher = rclcpp::Publisher<drive_yolo_msgs::msg::TrackedObjects3D>;

    rclcpp::Subscription<drive_yolo_msgs::msg::DetectionsWithDistance>::SharedPtr detections_sub_;
    TrackedObjects3DPublisher::SharedPtr tracked_objects_pub_;
    TrackedObjects3DPublisher::SharedPtr high_quality_tracks_pub_;
    TrackedObjects3DPublisher::SharedPtr moving_objects_pub_;
    
    MultiObjectTracker3D tracker_;
    
    // Parameters
    double distance_threshold_3d_;
    int max_missed_frames_;
    int min_detections_for_tracking_;
    int min_lidar_points_;
    float min_speed_threshold_;
    bool quiet_mode_;
    bool publish_all_tracks_;
    bool publish_high_quality_only_;
    float max_tracking_distance_;
    
    // Performance tracking
    int frame_count_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point last_stats_report_;
    double total_processing_time_;
    
    void detectionsCallback(const drive_yolo_msgs::msg::DetectionsWithDistance::ConstSharedPtr msg) {
        auto callback_start = std::chrono::high_resolution_clock::now();
        
        // Update tracker with new detections
        tracker_.updateTracks(msg);
        
        // Get different types of tracks
        auto all_tracks = tracker_.getAllTrackedObjects();
        auto high_quality_tracks = tracker_.getHighQualityTracks();
        auto moving_objects = tracker_.getMovingObjects(min_speed_threshold_);
        
        auto callback_end = std::chrono::high_resolution_clock::now();
        double processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
            callback_end - callback_start).count() / 1000.0;
        
        frame_count_++;
        total_processing_time_ += processing_time;
        
        // Publish results
        if (publish_all_tracks_) {
            publishTrackedObjects(all_tracks, msg->header, processing_time, tracked_objects_pub_);
        }
        
        if (publish_high_quality_only_) {
            publishTrackedObjects(high_quality_tracks, msg->header, processing_time, tracked_objects_pub_);
        }
        
        // Always publish filtered results
        publishTrackedObjects(high_quality_tracks, msg->header, processing_time, high_quality_tracks_pub_);
        publishTrackedObjects(moving_objects, msg->header, processing_time, moving_objects_pub_);
        
        // Log tracking results (if not in quiet mode)
        if (!quiet_mode_ && !all_tracks.empty()) {
            RCLCPP_INFO(get_logger(), "Frame %d: %.1f ms, %zu total tracks (%zu HQ, %zu moving)", 
                    frame_count_, processing_time, all_tracks.size(), 
                    high_quality_tracks.size(), moving_objects.size());
            
            // Show details for up to 3 high-quality tracks
            for (size_t i = 0; i < high_quality_tracks.size() && i < 3; ++i) {
                const auto& track = high_quality_tracks[i];
                RCLCPP_INFO(get_logger(), "  Track %d (%s): pos=[%.2f,%.2f,%.2f]m vel=[%.2f,%.2f,%.2f]m/s speed=%.2fm/s conf=%.1f%% pts=%d", 
                         track.id, track.class_name.c_str(),
                         track.position_3d.x, track.position_3d.y, track.position_3d.z,
                         track.velocity_3d.x, track.velocity_3d.y, track.velocity_3d.z, 
                         track.speed_3d, track.confidence * 100, track.lidar_points_count);
            }
            if (high_quality_tracks.size() > 3) {
                RCLCPP_INFO(get_logger(), "  ... and %zu more high-quality tracks", high_quality_tracks.size() - 3);
            }
        }
        
        // Report stats every 20 seconds
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_report = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - last_stats_report_).count();
        
        if (time_since_report >= 20) {
            auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - start_time_).count();
            double fps = static_cast<double>(frame_count_) / total_time;
            double avg_processing = total_processing_time_ / frame_count_;
            
            RCLCPP_INFO(get_logger(), "=== 3D TRACKING STATS ===");
            RCLCPP_INFO(get_logger(), "Frames processed: %d (%.1f Hz)", frame_count_, fps);
            RCLCPP_INFO(get_logger(), "Active tracks: %d (total: %d)", tracker_.getActiveTracks(), tracker_.getTotalTracks());
            RCLCPP_INFO(get_logger(), "High quality tracks: %zu", high_quality_tracks.size());
            RCLCPP_INFO(get_logger(), "Moving objects: %zu", moving_objects.size());
            RCLCPP_INFO(get_logger(), "Avg processing time: %.1f ms", avg_processing);
            RCLCPP_INFO(get_logger(), "Avg tracking distance: %.1f m", tracker_.getAverageTrackingDistance());
            RCLCPP_INFO(get_logger(), "========================");
            
            last_stats_report_ = current_time;
        }
    }
    
    void publishTrackedObjects(const std::vector<TrackedObject3D>& tracked_objects, 
                              const std_msgs::msg::Header& original_header,
                              double processing_time_ms,
                              const TrackedObjects3DPublisher::SharedPtr& publisher) {
        drive_yolo_msgs::msg::TrackedObjects3D msg;
        msg.header = original_header;
        msg.total_tracks = tracked_objects.size();
        msg.processing_time_ms = processing_time_ms;
        
        int high_quality_count = 0;
        int moving_count = 0;
        
        for (const auto& track : tracked_objects) {
            drive_yolo_msgs::msg::TrackedObject3D track_msg;
            
            // Basic info
            track_msg.id = track.id;
            track_msg.class_id = track.class_id;
            track_msg.class_name = track.class_name;
            
            // 3D position and motion
            track_msg.position_3d.x = track.position_3d.x;
            track_msg.position_3d.y = track.position_3d.y;
            track_msg.position_3d.z = track.position_3d.z;
            
            track_msg.velocity_3d.x = track.velocity_3d.x;
            track_msg.velocity_3d.y = track.velocity_3d.y;
            track_msg.velocity_3d.z = track.velocity_3d.z;
            
            track_msg.acceleration_3d.x = track.acceleration_3d.x;
            track_msg.acceleration_3d.y = track.acceleration_3d.y;
            track_msg.acceleration_3d.z = track.acceleration_3d.z;
            
            track_msg.speed_3d = track.speed_3d;
            
            // 2D bounding box (for visualization)
            track_msg.x = track.position_2d.x;
            track_msg.y = track.position_2d.y;
            track_msg.width = track.size_2d.width;
            track_msg.height = track.size_2d.height;
            
            // Quality metrics
            track_msg.confidence = track.confidence;
            track_msg.distance_confidence = track.distance_confidence;
            track_msg.lidar_points_count = track.lidar_points_count;
            track_msg.track_stability = track.getTrackStability();
            
            // Tracking info
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                track.last_seen - track.first_seen).count() / 1000.0;
            track_msg.track_duration = duration;
            track_msg.total_detections = track.total_detections;
            track_msg.is_moving = track.speed_3d > min_speed_threshold_;
            
            // Prediction
            cv::Point3f predicted = track.predictPositionAfter(1.0);  // 1 second ahead
            track_msg.predicted_position_1s.x = predicted.x;
            track_msg.predicted_position_1s.y = predicted.y;
            track_msg.predicted_position_1s.z = predicted.z;
            
            msg.objects.push_back(track_msg);
            
            // Count categories
            if (track.isHighQualityTrack()) high_quality_count++;
            if (track.speed_3d > min_speed_threshold_) moving_count++;
        }
        
        msg.high_quality_tracks = high_quality_count;
        msg.moving_objects = moving_count;
        
        publisher->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    int ret = 0;
    try {
        auto node = std::make_shared<Tracking3DNode>();
        
        RCLCPP_INFO(node->get_logger(), "3D Object Tracking Node running... Press Ctrl+C to stop");
        rclcpp::spin(node);
        
        RCLCPP_INFO(node->get_logger(), "3D Object Tracking Node shutting down");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("tracking_3d_node"), "3D Tracking node failed: %s", e.what());
        ret = -1;
    }
    
    rclcpp::shutdown();
    return ret;
}
