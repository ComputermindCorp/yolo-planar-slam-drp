#include "MonocularCameraImageLoading.h"

#include <cassert>

#include <opencv2/imgproc.hpp>

#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

namespace ORB_SLAM2 {

MonocularCameraImageLoading::MonocularCameraImageLoading(const eSensor sensor, const std::string setting_path)
    : ImageLoading(sensor) {
    std::string src = "v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640, height=480, framerate=(fraction)10/1 ! appsink";
  //std::string src = "v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640, height=480, framerate=(fraction)30/1 ! appsink";
    cap.open(src);
    //cap.open(0);
    if (!cap.isOpened()) {
        fprintf(stderr, "Failed to open /dev/video0.\n");
        exit(EXIT_FAILURE);
    }

    // Check settings file
    cv::FileStorage setting(setting_path, cv::FileStorage::READ);
    if (!setting.isOpened()) {
        fprintf(stderr, "Failed to open settings file at %s\n", setting_path.c_str());
        exit(EXIT_FAILURE);
    }

    const float fps = setting["Camera.fps"];
    const int camera_width = setting["Camera.width"];
    const int camera_height = setting["Camera.height"];

    //cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);
    if (camera_width != cap.get(cv::CAP_PROP_FRAME_WIDTH)) {
        fprintf(stderr, "Failed to set %d to cv::CAP_PROP_FRAME_WIDTH\n", camera_width);
        exit(EXIT_FAILURE);
    }
    if (camera_height != cap.get(cv::CAP_PROP_FRAME_HEIGHT)) {
        fprintf(stderr, "Failed to set %d to cv::CAP_PROP_FRAME_HEIGHT\n", camera_height);
        exit(EXIT_FAILURE);
    }
    if (fps != cap.get(cv::CAP_PROP_FPS)) {
        fprintf(stderr, "Failed to set %3.1f to cv::CAP_PROP_FPS\n", fps);
        exit(EXIT_FAILURE);
    }

    //bool ret;
    //ret = cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    //if (!ret) {
    //    fprintf(stderr, "Failed to set cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V') to cv::CAP_PROP_FOURCC\n");
    //    exit(EXIT_FAILURE);
    //}
    //ret = cap.set(cv::CAP_PROP_FPS, fps);
    //if (!ret) {
    //    fprintf(stderr, "Failed to set %.1f to cv::CAP_PROP_FPS\n", fps);
    //    exit(EXIT_FAILURE);
    //}
    //ret = cap.set(cv::CAP_PROP_CONVERT_RGB, 0);
    //if (!ret) {
    //    fprintf(stderr, "Failed to set 0 to cv::CAP_PROP_CONVERT_RGB\n");
    //    exit(EXIT_FAILURE);
    //}

    loaded_images.store(false);
}

void MonocularCameraImageLoading::Run() {
    mbFinished = false;

    while (true) {
        if (!loaded_images.load()) {
            MT_START(mt_image_loading);
            if (mSensor == eSensor::MONOCULAR) {
                // Read image from device
                cv::Mat img, imgYuv;

                MT_START(mt_capture_read);
                cap.read(imgYuv);
                MT_FINISH(mt_capture_read);

                const double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now().time_since_epoch()).count();

                MT_START(mt_cvt_color);
                cv::cvtColor(imgYuv, img, cv::COLOR_YUV2GRAY_YUYV);
                MT_FINISH(mt_cvt_color);

                assert(!img.empty());

                {
                    std::unique_lock<std::mutex> lock(mutex_frame);
                    img_ptr_.reset(new cv::Mat(img));
                    timestamp_ = timestamp;
                }
            }
            else if (mSensor == eSensor::STEREO) {
                fprintf(stderr, "MonocularCameraImageLoading not support Stereo mode.\n");
                exit(EXIT_FAILURE);
            }
            else if (mSensor == eSensor::RGBD) {
                fprintf(stderr, "MonocularCameraImageLoading not support RGB-D mode.\n");
                exit(EXIT_FAILURE);
            }
            else {
                fprintf(stderr, "Unknown mode detected in MonocularCameraImageLoading::Run.\n");
                exit(EXIT_FAILURE);
            }

            assert(!img_ptr_->empty());

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

void MonocularCameraImageLoading::GetFrame(
    std::unique_ptr<cv::Mat>& img_ptr,
    std::unique_ptr<cv::Mat>& depth_img_ptr,
    double& timestamp) {
    fprintf(stderr, "MonocularCameraImageLoading not support RGB-D mode.\n");
    exit(EXIT_FAILURE);
}

} // namespace ORB_SLAM2
