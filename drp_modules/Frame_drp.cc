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

#include "Frame_drp.h"

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

Frame_drp::Frame_drp()
    : drp_fd_(drp::FILE_DESCRIPTOR_CLOSE),
      nlevels(0)
#if defined(ENABLE_DRP_AI)
      ,
      mYoloDetector_(NULL)
#endif
{
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/Frame.cc#L61-L81
Frame_drp::Frame_drp(const int drp_fd,
                     const cv::Mat& imGray, const double& timeStamp,
                     ORBextractor_drp* extractor, ORBVocabulary* voc, ImageProcessing* image_processing_ptr,
#if defined(ENABLE_DRP_AI)
                     YoloDetector* mYoloDetector,
#endif
                     cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const int sensor_type)
    : Frame(imGray, timeStamp, extractor, voc, image_processing_ptr, K, distCoef, bf, thDepth, true),
      mpORBextractorLeft(extractor),
      mpORBextractorRight(static_cast<ORBextractor_drp*>(NULL)),
      drp_fd_(drp_fd),
      nlevels(mpORBextractorLeft->nlevels),
      input_image(&imGray),
      sensor_type_(sensor_type)
#if defined(ENABLE_DRP_AI)
      ,
      mYoloDetector_(mYoloDetector)
#endif
{
    assert(0 < drp_fd_);

    // Frame ID
    mnId = Frame::nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    mvKeyPoints.resize(nlevels);

    input_K = K.clone();

    status_algorithm = drp::STATE_RESIZE;
    status_level = 0;
    trial_n = 0;

#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "frame_idx : " << ORBextractor::frame_idx << std::endl;
#endif
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/Frame.cc#L119-L136
Frame_drp::Frame_drp(const int drp_fd,
                     const cv::Mat& imGray, const cv::Mat& imDepth, const double& timeStamp,
                     ORBextractor_drp* extractor, ORBVocabulary* voc, ImageProcessing* image_processing_ptr,
#if defined(ENABLE_DRP_AI)
                     YoloDetector* mYoloDetector,
#endif
                     cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const int sensor_type)
    : Frame(imGray, imDepth, timeStamp, extractor, voc, image_processing_ptr, K, distCoef, bf, thDepth, true),
      mpORBextractorLeft(extractor),
      mpORBextractorRight(static_cast<ORBextractor_drp*>(NULL)),
      drp_fd_(drp_fd),
      nlevels(mpORBextractorLeft->nlevels),
      input_image(&imGray),
      sensor_type_(sensor_type)
#if defined(ENABLE_DRP_AI)
      ,
      mYoloDetector_(mYoloDetector)
#endif
{
    assert(0 < drp_fd_);

    // Frame ID
    mnId = Frame::nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    mvKeyPoints.resize(nlevels);

    input_K = K.clone();

    status_algorithm = drp::STATE_RESIZE;
    status_level = 0;
    trial_n = 0;

#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "frame_idx : " << ORBextractor::frame_idx << std::endl;
#endif
}

bool Frame_drp::drp_setup() {
    bool setup_succeed = false;
    switch (status_algorithm) {
        case drp::STATE_NONE:
            break;
        case drp::STATE_RESIZE:
            setup_succeed = true; // drp::ResizeSetup(drp_fd_);
            break;
        case drp::STATE_GAUSSIAN_BLUR:
            setup_succeed = true; // drp::GaussianBlurSetup(drp_fd_);
            break;
#if defined(ENABLE_SLAMFAST)
        case drp::STATE_SLAMFAST:
            setup_succeed = true; // drp::SLAMFASTSetup(drp_fd_);
            break;
#else
        case drp::STATE_CVFAST:
            setup_succeed = drp::CVFASTSetup(drp_fd_);
            break;
#endif
        case drp::STATE_ORB_DESCRIPTORS:
            setup_succeed = true; // drp::computeOrbDescriptorsSetup(drp_fd_);
            break;
        case drp::STATE_FINISHED:
            break;
        default:
            std::cerr << "Invalid case of Frame_drp::drp_start" << std::endl;
            break;
    }

    if (!setup_succeed)
        fprintf(stderr, "Failed to drp::*Setup in Frame_drp::drp_setup. status_algorithm is %d.\n", status_algorithm);

    return setup_succeed;
}

void Frame_drp::drp_preprocess() {
    if (!drp_is_finished()) {
#if !defined(ENABLE_SLAMFAST)
        if (status_algorithm == drp::STATE_CVFAST) {
            if (status_level == 0 && mpORBextractorLeft->cell_indice.empty())
                drp_setup();
        }
        else
#endif
        {
            if (status_level == 0)
                drp_setup();
        }
    }
}

bool Frame_drp::drp_start(const size_t trial) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame Frame_drp::drp_start, status_algorithm == " << status_algorithm << ", status_level == " << status_level << std::endl;
#endif

    bool start_succeed = false;
    switch (status_algorithm) {
        case drp::STATE_NONE:
            break;
        case drp::STATE_RESIZE:
            start_succeed = mpORBextractorLeft->OperatorResizeStart(*input_image, status_level);
            break;
        case drp::STATE_GAUSSIAN_BLUR:
            start_succeed = mpORBextractorLeft->OperatorGaussianBlurStart(status_level);
            break;
#if defined(ENABLE_SLAMFAST)
        case drp::STATE_SLAMFAST:
            start_succeed = mpORBextractorLeft->OperatorSLAMFASTStart(status_level);
            break;
#else
        case drp::STATE_CVFAST:
            start_succeed = mpORBextractorLeft->OperatorCVFASTStart(status_level);
            break;
#endif
        case drp::STATE_ORB_DESCRIPTORS:
            start_succeed = mpORBextractorLeft->OperatorOrbDescriptorsStart(status_level, trial);
            break;
        case drp::STATE_FINISHED:
            break;
        default:
            std::cerr << "Invalid case of Frame_drp::drp_start" << std::endl;
            break;
    }
    start = std::chrono::steady_clock::now();

    return start_succeed;
}

bool Frame_drp::drp_polling(const size_t trial) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame Frame_drp::drp_polling" << std::endl;
#endif

    bool finish_succeed = false;
    switch (status_algorithm) {
        case drp::STATE_NONE:
            return false;
        case drp::STATE_RESIZE:
            finish_succeed = mpORBextractorLeft->OperatorResizePolling(status_level);
            break;
        case drp::STATE_GAUSSIAN_BLUR:
            finish_succeed = mpORBextractorLeft->OperatorGaussianBlurPolling(status_level);
            break;
#if defined(ENABLE_SLAMFAST)
        case drp::STATE_SLAMFAST:
            finish_succeed = mpORBextractorLeft->OperatorSLAMFASTPolling(status_level);
            break;
#else
        case drp::STATE_CVFAST:
            finish_succeed = mpORBextractorLeft->OperatorCVFASTPolling(status_level);
            break;
#endif
        case drp::STATE_ORB_DESCRIPTORS:
            finish_succeed = mpORBextractorLeft->OperatorOrbDescriptorsPolling(status_level, trial);
            break;
        case drp::STATE_FINISHED:
            return false;
        default:
            std::cerr << "Invalid case of Frame_drp::drp_polling" << std::endl;
            return false;
    }

    if (finish_succeed) {
        drp_postprocess();
        drp_preprocess();
    }
    else {
        auto now = std::chrono::steady_clock::now();
        double us = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now - start).count());
        double sec = us / 1e6;

#if defined(ENABLE_DEBUG_OUTPUT)
        std::cerr << "elapsed time : " << sec << "[s]" << std::endl;
#endif

        if (drp::WAITING_TIME < sec) {
            std::cerr << "Failed to Frame_drp::drp_polling" << std::endl;
            drp_retry(trial);
            return false;
        }
    }

    return finish_succeed;
}

void Frame_drp::drp_postprocess() {
    check_increment_level();

    trial_n = 0;

    if (done_resize() || status_level == nlevels) {
        status_algorithm++;
        status_level = 0;
    }
}

bool Frame_drp::drp_execute(const size_t trial) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame Frame_drp::drp_execute, status_algorithm == " << status_algorithm << ", status_level == " << status_level << std::endl;
#endif

    bool start_succeed = drp_start(trial);
    if (!start_succeed) {
        fprintf(stderr, "Failed to Frame_drp::drp_start in Frame_drp::drp_execute. status_algorithm is %d, status_level is %lu\n", status_algorithm, status_level);
        return false;
    }

    bool finish_succeed = false;
    while (true) {
        switch (status_algorithm) {
            case drp::STATE_NONE:
                break;
            case drp::STATE_RESIZE:
                finish_succeed = mpORBextractorLeft->OperatorResizePolling(status_level);
                break;
            case drp::STATE_GAUSSIAN_BLUR:
                finish_succeed = mpORBextractorLeft->OperatorGaussianBlurPolling(status_level);
                break;
#if defined(ENABLE_SLAMFAST)
            case drp::STATE_SLAMFAST:
                finish_succeed = mpORBextractorLeft->OperatorSLAMFASTPolling(status_level);
                break;
#else
            case drp::STATE_CVFAST:
                finish_succeed = mpORBextractorLeft->OperatorCVFASTPolling(status_level);
                break;
#endif
            case drp::STATE_ORB_DESCRIPTORS:
                finish_succeed = mpORBextractorLeft->OperatorOrbDescriptorsPolling(status_level, trial);
                break;
            case drp::STATE_FINISHED:
                break;
            default:
                std::cerr << "Invalid case of Frame_drp::drp_execute" << std::endl;
                break;
        }

        if (finish_succeed)
            break;

        {
            auto now = std::chrono::steady_clock::now();
            double us = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now - start).count());
            double sec = us / 1e6;

#if defined(ENABLE_DEBUG_OUTPUT)
            std::cerr << "elapsed time : " << sec << "[s]" << std::endl;
#endif

            if (drp::WAITING_TIME < sec) {
                std::cerr << "Failed to Frame_drp::drp_execute" << std::endl;
                return false;
            }
        }
    }

    return finish_succeed;
}

bool Frame_drp::drp_is_finished() {
    return (status_algorithm == drp::STATE_FINISHED);
}

bool Frame_drp::drp_retry(const size_t trial) {
#if defined(ENABLE_MEASURE_TIME)
    switch (status_algorithm) {
        case drp::STATE_NONE:
            std::cerr << "Invalid status in Frame_drp::drp_polling" << std::endl;
            return false;
        case drp::STATE_RESIZE:
            std::cerr << "Remove last measure time of drp::Resize" << std::endl;
            MT_REMOVE_LAST(mt_drp_resize_calc_param);
            MT_REMOVE_LAST(mt_drp_resize_u2p_param);
            MT_REMOVE_LAST(mt_drp_resize_u2p_input);
            MT_REMOVE_LAST(mt_drp_resize_activate);
            MT_REMOVE_LAST(mt_drp_resize_start);
            break;
        case drp::STATE_GAUSSIAN_BLUR:
            std::cerr << "Remove last measure time of drp::GaussianBlur" << std::endl;
            MT_REMOVE_LAST(mt_drp_gaussian_blur_calc_param[status_level]);
            MT_REMOVE_LAST(mt_drp_gaussian_blur_u2p_param[status_level]);
            MT_REMOVE_LAST(mt_drp_gaussian_blur_u2p_input[status_level]);
            MT_REMOVE_LAST(mt_drp_gaussian_blur_activate);
            MT_REMOVE_LAST(mt_drp_gaussian_blur_start[status_level]);
            break;
#if defined(ENABLE_SLAMFAST)
        case drp::STATE_SLAMFAST:
            std::cerr << "Remove last measure time of drp::SLAMFAST" << std::endl;
            MT_REMOVE_LAST(mt_drp_slamfast_calc_param[status_level]);
            MT_REMOVE_LAST(mt_drp_slamfast_u2p_param[status_level]);
            MT_REMOVE_LAST(mt_drp_slamfast_u2p_input[status_level]);
            MT_REMOVE_LAST(mt_drp_slamfast_activate);
            MT_REMOVE_LAST(mt_drp_slamfast_start[status_level]);
            break;
#else
        case drp::STATE_CVFAST:
            std::cerr << "Remove last measure time of drp::CVFAST" << std::endl;
            MT_REMOVE_LAST(mt_drp_cvfast_calc_param[status_level]);
            MT_REMOVE_LAST(mt_drp_cvfast_u2p_param[status_level]);
            MT_REMOVE_LAST(mt_drp_cvfast_u2p_input[status_level]);
            MT_REMOVE_LAST(mt_drp_cvfast_activate);
            MT_REMOVE_LAST(mt_drp_cvfast_start[status_level]);
            break;
#endif
        case drp::STATE_ORB_DESCRIPTORS:
            std::cerr << "Remove last measure time of drp::computeOrbDescriptors" << std::endl;
            MT_REMOVE_LAST(mt_drp_orb_descriptors_calc_param[trial][status_level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_cast_to_drp[trial][status_level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_u2p_param[trial][status_level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_u2p_input_image[trial][status_level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_u2p_input_keypoints[trial][status_level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_activate);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_start[trial][status_level]);
            break;
        case drp::STATE_FINISHED:
            std::cerr << "Invalid status in Frame_drp::drp_polling" << std::endl;
            return false;
        default:
            std::cerr << "Invalid status in Frame_drp::drp_polling" << std::endl;
            return false;
    }
#endif

    fprintf(stderr, "Failed to finish DRP program, status_algorithm is %d, status_level is %lu.\n", status_algorithm, status_level);
    fprintf(stderr, "V2x does not support DRP reset function.\n");
    exit(EXIT_FAILURE);

    trial_n++;
    std::cerr << "Retry Frame_drp::drp_start"
#if defined(ENABLE_DEBUG_OUTPUT)
              << ", frame_idx = " << ORBextractor::frame_idx
#endif
              << ", trial_n = " << trial_n
              << ", status_algorithm = " << status_algorithm
              << ", status_level = " << status_level << std::endl;
    std::cerr << "sleep " << (trial_n * drp::INTERVAL_RATIO_MS) << "[ms]..." << std::endl;

    usleep(trial_n * drp::INTERVAL_RATIO_MS * 1e3);
    std::cerr << "done" << std::endl;

    if (!drp_setup())
        return false;
    return drp_start(trial);
}

void Frame_drp::LastProcessingOfConstructor() {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame Frame_drp::LastProcessingOfConstructor" << std::endl;
#endif

    mvKeys.clear();
    std::copy(mpORBextractorLeft->result_keypoints.begin(), mpORBextractorLeft->result_keypoints.end(), std::back_inserter(mvKeys));
    mDescriptors = mpORBextractorLeft->result_descriptors.clone();

    mpORBextractorLeft->OperatorResultDump(mvKeys, mDescriptors);

    if (sensor_type_ == eSensor::MONOCULAR || sensor_type_ == eSensor::RGBD) {
        Frame::LastProcessingOfConstructor(*input_image, input_K);
    }
}

void Frame_drp::check_increment_level() {
#if !defined(ENABLE_SLAMFAST)
    if (status_algorithm == drp::STATE_CVFAST) {
        if (mpORBextractorLeft->cell_indice.empty())
            status_level++;
    }
    else
#endif
    {
        status_level++;
    }
}

bool Frame_drp::done_resize() {
    if (status_algorithm != drp::STATE_RESIZE)
        return false;
    if (status_level != (nlevels - 1))
        return false;

    return true;
}

bool Frame_drp::done_fast() {
    if (status_algorithm == drp::STATE_NONE)
        return false;
    if (status_algorithm == drp::STATE_RESIZE)
        return false;
    if (status_algorithm == drp::STATE_GAUSSIAN_BLUR)
        return false;
    if (status_algorithm == drp::STATE_ORB_DESCRIPTORS)
        return true;
    if (status_algorithm == drp::STATE_FINISHED)
        return true;

#if defined(ENABLE_SLAMFAST)
    assert(status_algorithm == drp::STATE_SLAMFAST);
#else
    assert(status_algorithm == drp::STATE_CVFAST);
    if (!mpORBextractorLeft->cell_indice.empty())
        return false;
#endif

    if (status_level != nlevels)
        return false;

    return true;
}

void Frame_drp::postprocess_fast(const int max_features, const size_t trial) {
    for (size_t level = 0; level < nlevels; level++) {
        mpORBextractorLeft->OperatorPrune(mpORBextractorLeft->vToDistributeKeys[level],
                                          mpORBextractorLeft->allKeypoints,
                                          max_features,
                                          level,
                                          trial);
        mpORBextractorLeft->OperatorOrientation(mpORBextractorLeft->allKeypoints, level, trial);
        mvKeyPoints[level] = mpORBextractorLeft->allKeypoints[level];
    }
}

int Frame_drp::get_status_algorithm() {
    return status_algorithm;
}

ssize_t Frame_drp::get_status_level() {
    return (ssize_t)status_level;
}

void Frame_drp::reset_to_orb_descriptors() {
    status_algorithm = drp::STATE_ORB_DESCRIPTORS;
    status_level = 0;
}

} // namespace ORB_SLAM2
