#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include "ImageLoading.h"
#include "Enum.h"

namespace ORB_SLAM2 {

class ImageLoading;

class D435iCameraImageLoading : public ImageLoading {
public:
    D435iCameraImageLoading(const eSensor sensor, const std::string setting_path);
    ~D435iCameraImageLoading();

    void Run();

    void GetFrame(std::unique_ptr<cv::Mat>& img_ptr,
                  std::unique_ptr<cv::Mat>& depth_img_ptr,
                  double& timestamp) override;

protected:
    cv::Size image_size;

    rs2::context ctx;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // start and stop just to get necessary profile
    rs2::pipeline_profile pipe_profile;

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    // The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align_to;
};

} // namespace ORB_SLAM2
