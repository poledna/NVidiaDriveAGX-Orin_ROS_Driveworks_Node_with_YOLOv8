#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cstring>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("image_publisher");
    
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("/camera/image_input", 1);
    
    // Get image path from parameter or use default
    std::string image_path;
    image_path = node->declare_parameter<std::string>("image_path", "image/0127.png");
    
    // Get publish rate from parameter
    double publish_rate;
    publish_rate = node->declare_parameter<double>("publish_rate", 30.0);  // 30 Hz default
    
    RCLCPP_INFO(node->get_logger(), "Loading image from: %s", image_path.c_str());
    
    // Load the actual image
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    
    if (image.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load image from: %s", image_path.c_str());
        RCLCPP_ERROR(node->get_logger(), "Make sure the file exists and is a valid image format");
        rclcpp::shutdown();
        return -1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Loaded image: %dx%d pixels, %d channels", 
             image.cols, image.rows, image.channels());
    
    // Optional: Resize if image is too large (for performance)
    cv::Mat publish_image;
    if (image.cols > 1920 || image.rows > 1080) {
        // Calculate scale to fit within 1920x1080 while maintaining aspect ratio
        double scale = std::min(1920.0 / image.cols, 1080.0 / image.rows);
        int new_width = static_cast<int>(image.cols * scale);
        int new_height = static_cast<int>(image.rows * scale);
        
        cv::resize(image, publish_image, cv::Size(new_width, new_height));
        RCLCPP_INFO(node->get_logger(), "Resized image to: %dx%d for publishing (scale: %.2f)", 
                 new_width, new_height, scale);
    } else {
        publish_image = image;
    }
    
    rclcpp::WallRate rate(publish_rate);
    
    RCLCPP_INFO(node->get_logger(), "Publishing image to /camera/image_input at %.1f Hz", publish_rate);
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to stop...");
    
    int frame_count = 0;
    auto start_time = node->now();
    
    while (rclcpp::ok()) {
        // Create ROS image message
        sensor_msgs::msg::Image img_msg;
        img_msg.header.stamp = node->now();
        img_msg.header.frame_id = "camera";
        img_msg.height = publish_image.rows;
        img_msg.width = publish_image.cols;
        img_msg.encoding = "bgr8";
        img_msg.is_bigendian = false;
        img_msg.step = publish_image.cols * publish_image.channels();
        
        // Copy image data
        size_t size = img_msg.step * img_msg.height;
        img_msg.data.resize(size);
        memcpy(img_msg.data.data(), publish_image.data, size);
        
        image_pub->publish(img_msg);
        
        frame_count++;
        
        // Print stats every 5 seconds
        if (frame_count % (static_cast<int>(publish_rate) * 5) == 0) {
            auto current_time = node->now();
            double elapsed = (current_time - start_time).seconds();
            double actual_fps = frame_count / elapsed;
            
            RCLCPP_INFO(node->get_logger(), "Published %d frames in %.1fs (actual: %.1f Hz, target: %.1f Hz)", 
                     frame_count, elapsed, actual_fps, publish_rate);
        }
        
        rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Image publisher shutting down");
    rclcpp::shutdown();
    return 0;
}