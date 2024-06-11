#pragma once

#include <mutex>
#include <atomic>
#include <memory>

#include <unistd.h>

#include <opencv2/core.hpp>

#include "Frame.h"
#include "Tracking.h"
#include "ImageLoading.h"
#include "ORBextractor_drp.h"
#include "Enum.h"

namespace ORB_SLAM2 {

class Tracking;
class ImageLoading;

class ImageProcessing {
public:
    ImageProcessing(const int drp_fd,
                    const bool bRGB,
                    const eSensor sensor,
#if defined(ENABLE_DRP)
                    ORBextractor_drp* extractor,
#else
                    ORBextractor* extractor,
#endif
                    ORBVocabulary* vocabulary,
                    YoloDetector* yolo_detector,
                    const cv::Mat camera_k,
                    const cv::Mat dist_coef,
                    const float bf,
                    const float th_depth,
                    const int max_features,
                    const bool use_drp,
                    const bool use_drp_ai,
                    const bool use_opencva);

    void Run();

    void PushResult();

    void SetMaxFeatures(const int max_features);
    int GetMaxFeatures();

    void SetTracker(Tracking* tracking_ptr);

    void RequestFinish();

    bool isFinished();

    std::atomic<bool> wait_for_next_frame;
    std::atomic<bool> wait_for_next_max_features;
    std::atomic<bool> changed_max_features;
    std::atomic<bool> finish_processing;

    std::mutex mutex_max_features;

protected:
    void FirstProcess();
    void SecondProcess(const size_t trial);

    Tracking* tracking_ptr_;

    std::unique_ptr<Frame> frame_ptr_;

    const int drp_fd_;
    const bool bRGB_;
    const eSensor sensor_;
    const bool use_drp_;
    const bool use_drp_ai_;
    const bool use_opencva_;

    size_t frame_id_;

    bool finish_create_frame;

public:
    double timestamp_;

    std::unique_ptr<cv::Mat> input_image_ptr_;
    std::unique_ptr<cv::Mat> depth_image_ptr_;

    cv::Mat depth_image_;
    cv::Mat gray_image_;

protected:
    std::vector<cv::Mat> blurred_images_;
    std::vector<std::vector<cv::KeyPoint>> vToDistributeKeys_;

    int max_features_;

#if defined(ENABLE_DRP)
    ORBextractor_drp* extractor_;
#else
    ORBextractor* extractor_;
#endif
    ORBVocabulary* vocabulary_;

#if defined(ENABLE_DRP_AI)
    YoloDetector* yolo_detector_;
#endif

    cv::Mat camera_k_;
    cv::Mat dist_coef_;
    const float bf_;
    const float th_depth_;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;
};

} // namespace ORB_SLAM2
