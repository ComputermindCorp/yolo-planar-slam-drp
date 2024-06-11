#pragma once

#include <opencv2/videoio.hpp>

#include "ImageLoading.h"
#include "Enum.h"

namespace ORB_SLAM2 {

class ImageLoading;

class MonocularCameraImageLoading : public ImageLoading {
public:
    MonocularCameraImageLoading(const eSensor sensor, const std::string setting_path);

    void Run();

    void GetFrame(std::unique_ptr<cv::Mat>& img_ptr,
                  std::unique_ptr<cv::Mat>& depth_img_ptr,
                  double& timestamp) override;

protected:
    cv::VideoCapture cap;
};

} // namespace ORB_SLAM2
