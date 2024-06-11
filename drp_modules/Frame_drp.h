/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FRAME_DRP_H
#define FRAME_DRP_H

#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "Frame.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"
#include "ORBextractor_drp.h"
#include "ImageProcessing.h"

#if defined(ENABLE_DRP_AI)
#include "YoloDetector_drp.h"
#endif

namespace ORB_SLAM2 {

class Frame_drp : public Frame {
public:
    Frame_drp();
    virtual ~Frame_drp(){};

    // Constructor for Monocular cameras(DRP).
    Frame_drp(const int drp_fd,
              const cv::Mat& imGray, const double& timeStamp,
              ORBextractor_drp* extractor,
              ORBVocabulary* voc,
              ImageProcessing* image_processing_ptr,
#if defined(ENABLE_DRP_AI)
              YoloDetector* mYoloDetector,
#endif
              cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const int sensor_type);
    // Constructor for RGB-D cameras(DRP).
    Frame_drp(const int drp_fd,
              const cv::Mat& imGray, const cv::Mat& imDepth, const double& timeStamp,
              ORBextractor_drp* extractor,
              ORBVocabulary* voc,
              ImageProcessing* image_processing_ptr,
#if defined(ENABLE_DRP_AI)
              YoloDetector* mYoloDetector,
#endif
              cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const int sensor_type);

    bool drp_setup() override;
    void drp_preprocess() override;
    bool drp_start(const size_t trial) override;
    bool drp_polling(const size_t trial) override;
    void drp_postprocess() override;
    bool drp_execute(const size_t trial) override;
    bool drp_is_finished() override;
    bool drp_retry(const size_t trial) override;

    bool done_fast() override;
    void postprocess_fast(const int max_features, const size_t trial) override;
    int get_status_algorithm() override;
    ssize_t get_status_level() override;
    void reset_to_orb_descriptors() override;

    // Last processing of the constructor
    void LastProcessingOfConstructor() override;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor_drp *mpORBextractorLeft, *mpORBextractorRight;

protected:
    void check_increment_level();
    bool done_resize();

    const int drp_fd_;
    size_t nlevels;

    std::chrono::steady_clock::time_point start;
    int status_algorithm;
    size_t status_level;
    const cv::Mat* input_image;
    cv::Mat input_K;

    size_t trial_n;

    // Input sensor
    int sensor_type_;

#if defined(ENABLE_DRP_AI)
    YoloDetector* mYoloDetector_;
#endif
};

} // namespace ORB_SLAM2

#endif // FRAME_DRP_H
