#include "ImageLoading.h"

#include <cassert>

#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

namespace ORB_SLAM2 {

ImageLoading::ImageLoading(const eSensor sensor)
    : mpImageProcessor(NULL),
      timestamp_(0.0),
      frame_id(0),
      mSensor(sensor),
      mbFinishRequested(false), mbFinished(true) {
    loaded_images.store(false);
}

void ImageLoading::PushResult() {
    if (mSensor == eSensor::MONOCULAR) {
        GetFrame(mpImageProcessor->input_image_ptr_,
                 mpImageProcessor->timestamp_);
    }
    else if (mSensor == eSensor::STEREO) {
        fprintf(stderr, "ImageLoading not support Stereo mode.\n");
        exit(EXIT_FAILURE);
    }
    else if (mSensor == eSensor::RGBD) {
        GetFrame(mpImageProcessor->input_image_ptr_,
                 mpImageProcessor->depth_image_ptr_,
                 mpImageProcessor->timestamp_);
    }
    else {
        fprintf(stderr, "Unknown mode detected in ImageProcessing::Run.\n");
        exit(EXIT_FAILURE);
    }

    loaded_images.store(false);
    mpImageProcessor->wait_for_next_frame.store(false);
}

void ImageLoading::SeImageProcessor(ImageProcessing* pImageProcessor) {
    mpImageProcessor = pImageProcessor;
}

void ImageLoading::RequestFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool ImageLoading::isFinished() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

void ImageLoading::GetFrame(
    std::unique_ptr<cv::Mat>& img_ptr,
    double& timestamp) {
    std::unique_lock<std::mutex> lock(mutex_frame);
    img_ptr = std::move(img_ptr_);
    timestamp = timestamp_;
}

void ImageLoading::GetFrame(
    std::unique_ptr<cv::Mat>& img_ptr,
    std::unique_ptr<cv::Mat>& depth_img_ptr,
    double& timestamp) {
    std::unique_lock<std::mutex> lock(mutex_frame);
    img_ptr = std::move(img_ptr_);
    depth_img_ptr = std::move(depth_img_ptr_);
    timestamp = timestamp_;
}

bool ImageLoading::CheckFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ImageLoading::SetFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

} // namespace ORB_SLAM2