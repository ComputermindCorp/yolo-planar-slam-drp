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

#ifndef FRAME_OPENCVA_H
#define FRAME_OPENCVA_H

#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "Frame.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"
#include "ORBextractor_opencva.h"
#include "ImageProcessing.h"

#if defined(ENABLE_DRP_AI)
#include "YoloDetector_drp.h"
#endif

namespace ORB_SLAM2 {

class Frame_opencva : public Frame_drp {
public:
    Frame_opencva();
    virtual ~Frame_opencva(){};

    // Constructor for Monocular cameras(OPENCVA).
    Frame_opencva(const int drp_fd,
                  const cv::Mat& imGray, const double& timeStamp,
                  ORBextractor_drp* extractor,
                  ORBVocabulary* voc,
                  ImageProcessing* image_processing_ptr,
#if defined(ENABLE_DRP_AI)
                  YoloDetector* mYoloDetector,
#endif
                  cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const int sensor_type);
    // Constructor for RGB-D cameras(OPENCVA).
    Frame_opencva(const int drp_fd,
                  const cv::Mat& imGray, const cv::Mat& imDepth, const double& timeStamp,
                  ORBextractor_drp* extractor,
                  ORBVocabulary* voc,
                  ImageProcessing* image_processing_ptr,
#if defined(ENABLE_DRP_AI)
                  YoloDetector* mYoloDetector,
#endif
                  cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const int sensor_type);

    void drp_preprocess() override;
    bool drp_execute(const size_t trial) override;
};

} // namespace ORB_SLAM2

#endif // FRAME_OPENCVA_H
