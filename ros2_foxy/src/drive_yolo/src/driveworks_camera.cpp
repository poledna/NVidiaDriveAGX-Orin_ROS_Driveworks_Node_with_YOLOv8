#ifdef USE_DRIVEWORKS

#include "drive_yolo/yolo_infer.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {
const rclcpp::Logger LOGGER = rclcpp::get_logger("driveworks_camera");
rclcpp::Clock THROTTLE_CLOCK(RCL_STEADY_TIME);
}  // namespace
#include <cuda_runtime.h>
#include <cstring>  // for memset

DriveWorksCamera::DriveWorksCamera() 
    : context_(DW_NULL_HANDLE)
    , sal_(DW_NULL_HANDLE)
    , camera_(DW_NULL_HANDLE)
    , frame_(DW_NULL_HANDLE)
    , image_(DW_NULL_HANDLE)
    , initialized_(false)
    , started_(false)
    , image_width_(0)
    , image_height_(0)
{
}

DriveWorksCamera::~DriveWorksCamera() {
    cleanup();
}

bool DriveWorksCamera::initialize(const std::string& camera_params) {
    RCLCPP_INFO(LOGGER, "=== ENHANCED DRIVEWORKS CAMERA INITIALIZATION ===");
    RCLCPP_INFO(LOGGER, "Camera params: %s", camera_params.c_str());
    
    try {
        if (!initializeDriveWorks()) {
            RCLCPP_ERROR(LOGGER, "Failed to initialize DriveWorks context/SAL");
            return false;
        }
        RCLCPP_INFO(LOGGER, "✓ DriveWorks context and SAL initialized successfully");
        
        // Parse camera parameters for validation
        RCLCPP_INFO(LOGGER, "Parsing camera parameters:");
        std::stringstream ss(camera_params);
        std::string param;
        while (std::getline(ss, param, ',')) {
            RCLCPP_INFO(LOGGER, "  - %s", param.c_str());
        }
        
        // Create sensor parameters with detailed logging
        dwSensorParams params{};
        params.protocol = "camera.gmsl";
        params.parameters = camera_params.c_str();
        
        RCLCPP_INFO(LOGGER, "Creating camera sensor...");
        RCLCPP_INFO(LOGGER, "  Protocol: %s", params.protocol);
        RCLCPP_INFO(LOGGER, "  Parameters: %s", params.parameters);
        
        // Try creating the sensor
        dwStatus result = dwSAL_createSensor(&camera_, params, sal_);
        
        // Detailed error reporting
        switch (result) {
            case DW_SUCCESS:
                RCLCPP_INFO(LOGGER, "✓ Camera sensor created successfully");
                break;
            case DW_INVALID_ARGUMENT:
                RCLCPP_ERROR(LOGGER, "✗ DW_INVALID_ARGUMENT: Invalid camera parameters");
                RCLCPP_ERROR(LOGGER, "  Check: camera name, interface, link number");
                return false;
            case DW_INVALID_HANDLE:
                RCLCPP_ERROR(LOGGER, "✗ DW_INVALID_HANDLE: Invalid DriveWorks handle");
                RCLCPP_ERROR(LOGGER, "  Check: DriveWorks initialization");
                return false;
            case DW_NOT_AVAILABLE:
                RCLCPP_ERROR(LOGGER, "✗ DW_NOT_AVAILABLE: Camera not available");
                RCLCPP_ERROR(LOGGER, "  Check: camera connection, power, permissions");
                RCLCPP_ERROR(LOGGER, "  Check: no other process using camera");
                return false;
            case DW_NOT_SUPPORTED:
                RCLCPP_ERROR(LOGGER, "✗ DW_NOT_SUPPORTED: Camera configuration not supported");
                RCLCPP_ERROR(LOGGER, "  Try: different output-format or interface");
                return false;
            case DW_INTERNAL_ERROR:
                RCLCPP_ERROR(LOGGER, "✗ DW_INTERNAL_ERROR: Internal DriveWorks error");
                RCLCPP_ERROR(LOGGER, "  This usually indicates:");
                RCLCPP_ERROR(LOGGER, "  1. Camera already in use by another process");
                RCLCPP_ERROR(LOGGER, "  2. Hardware initialization failed");
                RCLCPP_ERROR(LOGGER, "  3. Incorrect camera module connection");
                RCLCPP_ERROR(LOGGER, "  4. Power supply issues");
                RCLCPP_ERROR(LOGGER, "  5. Permission issues with /dev/video devices");
                
                // Additional diagnostics
                RCLCPP_ERROR(LOGGER, "Diagnostic suggestions:");
                RCLCPP_ERROR(LOGGER, "  - Kill any existing camera processes: pkill -f camera");
                RCLCPP_ERROR(LOGGER, "  - Check video devices: ls -la /dev/video*");
                RCLCPP_ERROR(LOGGER, "  - Try different link number (0 instead of 1)");
                RCLCPP_ERROR(LOGGER, "  - Try native output format instead of processed");
                RCLCPP_ERROR(LOGGER, "  - Verify camera module physical connection");
                return false;
            default:
                RCLCPP_ERROR(LOGGER, "✗ Unknown DriveWorks error: %s (%d)", dwGetStatusName(result), static_cast<int>(result));
                return false;
        }
        
        // Try to get image properties to verify camera is working
        RCLCPP_INFO(LOGGER, "Testing camera image properties...");
        dwImageProperties imageProperties;
        
        // Try different output formats
        std::vector<dwCameraOutputType> output_formats = {
            DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8,
            DW_CAMERA_OUTPUT_NATIVE_PROCESSED, 
            DW_CAMERA_OUTPUT_NATIVE_RAW
        };
        
        bool format_found = false;
        for (auto format : output_formats) {
            result = dwSensorCamera_getImageProperties(&imageProperties, format, camera_);
            if (result == DW_SUCCESS) {
                RCLCPP_INFO(LOGGER, "✓ Image properties retrieved with format %d", static_cast<int>(format));
                RCLCPP_INFO(LOGGER, "  Image size: %dx%d", imageProperties.width, imageProperties.height);
                format_found = true;
                break;
            } else {
                RCLCPP_WARN(LOGGER, "  Format %d failed: %s", static_cast<int>(format), dwGetStatusName(result));
            }
        }
        
        if (!format_found) {
            RCLCPP_ERROR(LOGGER, "✗ Could not get image properties with any format");
            return false;
        }
        
        image_width_ = imageProperties.width;
        image_height_ = imageProperties.height;
        
        RCLCPP_INFO(LOGGER, "✓ Camera initialized successfully: %dx%d", image_width_, image_height_);
        initialized_ = true;
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "✗ Exception during camera initialization: %s", e.what());
        return false;
    } catch (...) {
        RCLCPP_ERROR(LOGGER, "✗ Unknown exception during camera initialization");
        return false;
    }
}


bool DriveWorksCamera::initializeDriveWorks() {
    try {
        RCLCPP_INFO(LOGGER, "Initializing DriveWorks context...");
        
        // Try different initialization approaches for DRIVE OS 6.0.6
        dwStatus result;
        
        // Approach 1: Minimal initialization
        RCLCPP_INFO(LOGGER, "Trying minimal context initialization...");
        result = dwInitialize(&context_, DW_VERSION, nullptr);
        if (result == DW_SUCCESS) {
            RCLCPP_INFO(LOGGER, "Minimal DriveWorks context initialized successfully");
        } else {
            RCLCPP_WARN(LOGGER, "Minimal initialization failed: %s (%d)", dwGetStatusName(result), static_cast<int>(result));
            
            // Approach 2: With empty context parameters
            RCLCPP_INFO(LOGGER, "Trying with empty context parameters...");
            dwContextParameters contextParams;
            memset(&contextParams, 0, sizeof(contextParams));
            
            result = dwInitialize(&context_, DW_VERSION, &contextParams);
            if (result != DW_SUCCESS) {
                RCLCPP_ERROR(LOGGER, "All DriveWorks context initialization methods failed: %s (%d)", 
                         dwGetStatusName(result), static_cast<int>(result));
                return false;
            }
            RCLCPP_INFO(LOGGER, "Empty params DriveWorks context initialized successfully");
        }
        
        // Initialize SAL (Sensor Abstraction Layer)
        RCLCPP_INFO(LOGGER, "Initializing SAL...");
        result = dwSAL_initialize(&sal_, context_);
        if (result != DW_SUCCESS) {
            RCLCPP_ERROR(LOGGER, "Failed to initialize SAL: %s (%d)", dwGetStatusName(result), static_cast<int>(result));
            return false;
        }
        RCLCPP_INFO(LOGGER, "SAL initialized successfully");
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Exception during DriveWorks initialization: %s", e.what());
        return false;
    } catch (...) {
        RCLCPP_ERROR(LOGGER, "Unknown exception during DriveWorks initialization");
        return false;
    }
}

bool DriveWorksCamera::start() {
    if (!initialized_) {
        RCLCPP_ERROR(LOGGER, "Camera not initialized");
        return false;
    }
    
    dwStatus result = dwSensor_start(camera_);
    if (result != DW_SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to start camera: %s", dwGetStatusName(result));
        return false;
    }
    
    started_ = true;
    RCLCPP_INFO(LOGGER, "Camera started successfully");
    return true;
}

bool DriveWorksCamera::stop() {
    if (!started_) return true;
    
    dwStatus result = dwSensor_stop(camera_);
    if (result != DW_SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to stop camera: %s", dwGetStatusName(result));
        return false;
    }
    
    started_ = false;
    RCLCPP_INFO(LOGGER, "Camera stopped");
    return true;
}

bool DriveWorksCamera::getNextFrame(cv::Mat& image, uint64_t& timestamp) {
    if (!started_) {
        RCLCPP_ERROR(LOGGER, "Camera not started");
        return false;
    }
    
    // Get next frame with correct API for 5.10 (3 parameters only)
    dwStatus result = dwSensorCamera_readFrame(&frame_, 33000, camera_); // timeout in microseconds, then camera handle
    if (result != DW_SUCCESS) {
        if (result == DW_TIME_OUT) {
            RCLCPP_WARN_THROTTLE(LOGGER, THROTTLE_CLOCK, 1000, "Camera frame timeout");
        } else {
            RCLCPP_ERROR_THROTTLE(LOGGER, THROTTLE_CLOCK, 1000, "Failed to read camera frame: %s", dwGetStatusName(result));
        }
        return false;
    }
    
    // Get processed image from frame
    result = dwSensorCamera_getImage(&image_, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, frame_);
    if (result != DW_SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to get processed image: %s", dwGetStatusName(result));
        dwSensorCamera_returnFrame(&frame_);
        return false;
    }
    
    // Get timestamp with correct type
    dwTime_t dw_timestamp;
    result = dwSensorCamera_getTimestamp(&dw_timestamp, frame_);
    if (result != DW_SUCCESS) {
        RCLCPP_WARN_THROTTLE(LOGGER, THROTTLE_CLOCK, 5000, "Failed to get timestamp: %s", dwGetStatusName(result));
        timestamp = 0;
    } else {
        timestamp = static_cast<uint64_t>(dw_timestamp);
    }
    
    // Convert to OpenCV Mat
    image = dwImageToCvMat(image_);
    
    // Return the frame
    dwSensorCamera_returnFrame(&frame_);
    
    return !image.empty();
}


cv::Mat DriveWorksCamera::dwImageToCvMat(dwImageHandle_t dwImage) {
    dwImageCUDA* cudaImage;
    dwStatus result = dwImage_getCUDA(&cudaImage, dwImage);
    if (result != DW_SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to get CUDA image: %s", dwGetStatusName(result));
        return cv::Mat();
    }
    
    // Get image properties to understand the actual format
    dwImageProperties props;
    result = dwImage_getProperties(&props, dwImage);
    if (result != DW_SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Failed to get image properties: %s", dwGetStatusName(result));
        return cv::Mat();
    }
    
    //RCLCPP_INFO(LOGGER, "CUDA Image properties: %dx%d, pitch=%d, format=%d", 
       //     props.width, props.height, cudaImage->pitch[0], props.format);
    
    // Calculate expected stride vs actual pitch
    int expected_stride = props.width * 4; // 4 bytes for RGBA
    int actual_pitch = cudaImage->pitch[0];
    
    //RCLCPP_INFO(LOGGER, "Expected stride: %d, Actual pitch: %d", expected_stride, actual_pitch);
    
    if (actual_pitch == expected_stride) {
        // No padding, can copy directly
        cv::Mat cuda_image(props.height, props.width, CV_8UC4); // RGBA
        
        cudaError_t cuda_result = cudaMemcpy(cuda_image.data, cudaImage->dptr[0], 
                                            props.height * actual_pitch, cudaMemcpyDeviceToHost);
        
        if (cuda_result != cudaSuccess) {
            RCLCPP_ERROR(LOGGER, "CUDA memcpy failed: %s", cudaGetErrorString(cuda_result));
            return cv::Mat();
        }
        
        // Convert RGBA to BGR for OpenCV compatibility
        cv::Mat bgr_image;
        cv::cvtColor(cuda_image, bgr_image, cv::COLOR_RGBA2BGR);
        return bgr_image;
        
    } else {
        // Has padding, need to copy row by row to remove padding
        //RCLCPP_INFO(LOGGER, "Image has padding, copying row by row");
        
        // Allocate temporary buffer for padded data
        std::vector<uint8_t> padded_data(props.height * actual_pitch);
        
        cudaError_t cuda_result = cudaMemcpy(padded_data.data(), cudaImage->dptr[0], 
                                            props.height * actual_pitch, cudaMemcpyDeviceToHost);
        
        if (cuda_result != cudaSuccess) {
            RCLCPP_ERROR(LOGGER, "CUDA memcpy failed: %s", cudaGetErrorString(cuda_result));
            return cv::Mat();
        }
        
        // Create OpenCV matrix and copy row by row without padding
        cv::Mat cuda_image(props.height, props.width, CV_8UC4); // RGBA
        
        for (uint32_t row = 0; row < props.height; ++row) {
            const uint8_t* src_row = padded_data.data() + row * actual_pitch;
            uint8_t* dst_row = cuda_image.ptr<uint8_t>(row);
            memcpy(dst_row, src_row, props.width * 4); // Copy only valid pixels, skip padding
        }
        
        // Convert RGBA to BGR for OpenCV compatibility
        cv::Mat bgr_image;
        cv::cvtColor(cuda_image, bgr_image, cv::COLOR_RGBA2BGR);
        return bgr_image;
    }
}

void DriveWorksCamera::cleanup() {
    if (started_) {
        stop();
    }
    
    if (image_ != DW_NULL_HANDLE) {
        dwImage_destroy(image_);
        image_ = DW_NULL_HANDLE;
    }
    
    if (camera_ != DW_NULL_HANDLE) {
        dwSAL_releaseSensor(camera_);
        camera_ = DW_NULL_HANDLE;
    }
    
    if (sal_ != DW_NULL_HANDLE) {
        dwSAL_release(sal_);
        sal_ = DW_NULL_HANDLE;
    }
    
    if (context_ != DW_NULL_HANDLE) {
        dwRelease(context_);
        context_ = DW_NULL_HANDLE;
    }
    
    initialized_ = false;
    started_ = false;
    
    RCLCPP_INFO(LOGGER, "DriveWorks camera cleanup completed");
}

#endif // USE_DRIVEWORKS