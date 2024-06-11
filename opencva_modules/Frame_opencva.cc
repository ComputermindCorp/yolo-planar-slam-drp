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

#include "Frame_opencva.h"

#include "drp.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <unistd.h>
#include <thread>
#include <functional>

#include "Enum.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

#include "measure_time.h"

namespace ORB_SLAM2 {

Frame_opencva::Frame_opencva()
    : Frame_drp() {
}

Frame_opencva::Frame_opencva(const int drp_fd,
                             const cv::Mat& imGray, const double& timeStamp,
                             ORBextractor_drp* extractor, ORBVocabulary* voc, ImageProcessing* image_processing_ptr,
#if defined(ENABLE_DRP_AI)
                             YoloDetector* mYoloDetector,
#endif
                             cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const int sensor_type)
    : Frame_drp(drp_fd,
                imGray, timeStamp,
                extractor, voc, image_processing_ptr,
#if defined(ENABLE_DRP_AI)
                mYoloDetector,
#endif
                K, distCoef, bf, thDepth, sensor_type) {
}

Frame_opencva::Frame_opencva(const int drp_fd,
                             const cv::Mat& imGray, const cv::Mat& imDepth, const double& timeStamp,
                             ORBextractor_drp* extractor, ORBVocabulary* voc, ImageProcessing* image_processing_ptr,
#if defined(ENABLE_DRP_AI)
                             YoloDetector* mYoloDetector,
#endif
                             cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const int sensor_type)
    : Frame_drp(drp_fd,
                imGray, imDepth, timeStamp,
                extractor, voc, image_processing_ptr,
#if defined(ENABLE_DRP_AI)
                mYoloDetector,
#endif
                K, distCoef, bf, thDepth, sensor_type) {
}

void Frame_opencva::drp_preprocess() {
    if (status_algorithm == drp::STATE_ORB_DESCRIPTORS && status_level == 0)
        drp_setup();
}

bool Frame_opencva::drp_execute(const size_t trial) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame Frame_opencva::drp_execute, status_algorithm == " << status_algorithm << ", status_level == " << status_level << std::endl;
#endif

    bool finish_succeed = false;
    switch (status_algorithm) {
        case drp::STATE_NONE:
            break;
        case drp::STATE_RESIZE:
            finish_succeed = mpORBextractorLeft->OperatorResize(*input_image, status_level);
            break;
        case drp::STATE_GAUSSIAN_BLUR:
            finish_succeed = mpORBextractorLeft->OperatorGaussianBlur(status_level);
            break;
        case drp::STATE_CVFAST:
            finish_succeed = mpORBextractorLeft->OperatorCVFAST(status_level);
            break;
        case drp::STATE_ORB_DESCRIPTORS:
            finish_succeed = mpORBextractorLeft->OperatorOrbDescriptors(status_level, trial);
            break;
        case drp::STATE_FINISHED:
            break;
        default:
            std::cerr << "Invalid case of Frame_opencva::drp_execute" << std::endl;
            break;
    }

    return finish_succeed;
}

} // namespace ORB_SLAM2
