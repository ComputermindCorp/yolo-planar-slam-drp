#include "D435iCameraImageLoading.h"

#include <cassert>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

namespace ORB_SLAM2 {

// Please refer to https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/v1.0-release/Examples/RGB-D/rgbd_realsense_D435i.cc

rs2_option get_sensor_option(const rs2::sensor& sensor) {
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n"
              << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++) {
        rs2_option option_type = static_cast<rs2_option>(i);
        // SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type)) {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            // To change the value of an option, please follow the change_sensor_option() function
        }
        else {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

D435iCameraImageLoading::D435iCameraImageLoading(const eSensor sensor, const std::string setting_path)
    : ImageLoading(sensor), align_to(rs2::align(RS2_STREAM_COLOR)) {
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0) {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        exit(EXIT_FAILURE);
    }
    else
        selected_device = devices[0];

    // Check settings file
    cv::FileStorage setting(setting_path, cv::FileStorage::READ);
    if (!setting.isOpened()) {
        fprintf(stderr, "Failed to open settings file at %s\n", setting_path.c_str());
        exit(EXIT_FAILURE);
    }

    const float fps = setting["Camera.fps"];
    const int camera_width = setting["Camera.width"];
    const int camera_height = setting["Camera.height"];
    image_size.width = camera_width;
    image_size.height = camera_height;

    // RGB stream
    cfg.enable_stream(RS2_STREAM_COLOR, camera_width, camera_height, RS2_FORMAT_RGB8, fps);

    // Depth stream
    cfg.enable_stream(RS2_STREAM_DEPTH, camera_width, camera_height, RS2_FORMAT_Z16, fps);

    // start and stop just to get necessary profile
    pipe_profile = pipe.start(cfg);

    loaded_images.store(false);
}

D435iCameraImageLoading::~D435iCameraImageLoading() {
    pipe.stop();
}

void D435iCameraImageLoading::Run() {
    mbFinished = false;

    while (true) {
        if (!loaded_images.load()) {
            MT_START(mt_image_loading);
            if (mSensor == eSensor::MONOCULAR) {
                fprintf(stderr, "D435iCameraImageLoading not support Monocular mode.\n");
                exit(EXIT_FAILURE);
            }
            else if (mSensor == eSensor::STEREO) {
                fprintf(stderr, "D435iCameraImageLoading not support Stereo mode.\n");
                exit(EXIT_FAILURE);
            }
            else if (mSensor == eSensor::RGBD) {
                // Read image from device
                MT_START(mt_capture);

                MT_START(mt_wait_for_frames);
                rs2::frameset fs = pipe.wait_for_frames();
                MT_FINISH(mt_wait_for_frames);

                MT_START(mt_align);
                rs2::frameset aligned_fs = align_to.process(fs);
                MT_FINISH(mt_align);

                MT_START(mt_get_color_frame);
                rs2::frame color_frame = aligned_fs.get_color_frame();
                MT_FINISH(mt_get_color_frame);

                MT_START(mt_get_depth_frame);
                rs2::depth_frame depth_frame = aligned_fs.get_depth_frame();
                MT_FINISH(mt_get_depth_frame);

                MT_START(mt_rs2_to_mat);
                cv::Mat rgb_img = cv::Mat(image_size, CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
                cv::Mat depth_img = cv::Mat(image_size, CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);
                MT_FINISH(mt_rs2_to_mat);

                MT_FINISH(mt_capture);

                double timestamps = fs.get_timestamp() * 1e-3;

                assert(!rgb_img.empty());
                assert(!depth_img.empty());

                {
                    std::unique_lock<std::mutex> lock(mutex_frame);
                    img_ptr_.reset(new cv::Mat(rgb_img));
                    depth_img_ptr_.reset(new cv::Mat(depth_img));
                    timestamp_ = timestamps;
                }
            }
            else {
                fprintf(stderr, "Unknown mode detected in D435iCameraImageLoading::Run.\n");
                exit(EXIT_FAILURE);
            }

            assert(!img_ptr_->empty());
            assert(mSensor != eSensor::RGBD || !depth_img_ptr_->empty());

            MT_FINISH(mt_image_loading);

            loaded_images.store(true);
        }

        if (mpImageProcessor && mpImageProcessor->wait_for_next_frame.load() && loaded_images.load()) {
            MT_START(mt_image_loading_push_result);
            PushResult();
            MT_FINISH(mt_image_loading_push_result);

            frame_id++;
        }

        if (CheckFinish())
            break;

        usleep(1000); // Sleep 1ms
    }

    SetFinish();
}

void D435iCameraImageLoading::GetFrame(
    std::unique_ptr<cv::Mat>& img_ptr,
    std::unique_ptr<cv::Mat>& depth_img_ptr,
    double& timestamp) {
    fprintf(stderr, "D435iCameraImageLoading has not been tested in RGB-D mode.\n");
    ImageLoading::GetFrame(img_ptr, depth_img_ptr, timestamp);
}

} // namespace ORB_SLAM2