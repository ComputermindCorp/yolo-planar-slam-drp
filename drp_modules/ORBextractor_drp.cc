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

#include "ORBextractor_drp.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <algorithm>

#if defined(ENABLE_DUMP) || defined(ENABLE_LOAD)
#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#endif

#include <unistd.h>

#include <cassert>

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

#if defined(ENABLE_DRP_AI)
#include "YoloDetector_drp.h"
#endif

#include "measure_time.h"
#include "drp.h"

namespace ORB_SLAM2 {

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1046-L1053
bool ORBextractor_drp::OperatorResizeStart(const cv::InputArray zero_level_img, const int input_level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorResizeStart" << std::endl;
#endif

    if (input_level == 0) {
        DumpInputImage(zero_level_img);
    }

    if (input_level == 0) {
        offset = 0;
        nkeypoints = 0;
        result_keypoints = std::vector<cv::KeyPoint>();
        workingMat = std::vector<cv::Mat>(nlevels);
        allKeypoints = std::vector<std::vector<cv::KeyPoint>>(nlevels, std::vector<cv::KeyPoint>());

        if (zero_level_img.empty()) {
            std::cerr << "Input image is empty." << std::endl;
            return false;
        }

        in_image = zero_level_img.getMat().clone();
        assert(in_image.type() == CV_8UC1);
    }

    // Pre-compute the scale pyramid
    // Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1111-L1112
    const float input_scale = 1.0f / mvScaleFactor[input_level];
    const float output_scale = 1.0f / mvScaleFactor[input_level + 1];
    const cv::Size input_image_size(cvRound((float)in_image.cols * input_scale), cvRound((float)in_image.rows * input_scale));
    const cv::Size output_image_size(cvRound((float)in_image.cols * output_scale), cvRound((float)in_image.rows * output_scale));

    bool setup_succeed = drp::ResizeSetup(drp_fd_, input_level);
    if (!setup_succeed) {
        std::cerr << "Failed to drp::ResizeSetup" << std::endl;
        return setup_succeed;
    }

    if (input_level == 0) {
        mvImagePyramid[input_level] = in_image;

        bool start_succeed = drp::ResizeStart(drp_fd_, in_image, input_level, output_image_size);
        if (!start_succeed)
            std::cerr << "Failed to drp::ResizeStart" << std::endl;
        return start_succeed;
    }

    // Reuse input image on physical memory space.
    bool start_succeed = drp::ResizeStart(drp_fd_, input_image_size, input_level, output_image_size);
    if (!start_succeed)
        std::cerr << "Failed to drp::ResizeStart" << std::endl;
    return start_succeed;
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1052-L1053
bool ORBextractor_drp::OperatorResizePolling(const int input_level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorResizePolling" << std::endl;
#endif

    // Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1111-L1112
    const float output_scale = 1.0f / mvScaleFactor[input_level + 1];
    const cv::Size output_image_size(cvRound((float)in_image.cols * output_scale), cvRound((float)in_image.rows * output_scale));

    bool finish_succeed = drp::ResizeFinish(drp_fd_, input_level, output_image_size, mvImagePyramid[input_level + 1]);

    if (finish_succeed) {
        if (input_level == 0) {
            DumpPyramid(mvImagePyramid, input_level);
        }
        DumpPyramid(mvImagePyramid, input_level + 1);
    }

    return finish_succeed;
}

#if defined(ENABLE_SLAMFAST)

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L773-L829
bool ORBextractor_drp::OperatorSLAMFASTStart(const int level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorSLAMFASTStart" << std::endl;
#endif

    bool setup_succeed = drp::SLAMFASTSetup(drp_fd_, level);
    if (!setup_succeed) {
        std::cerr << "Failed to drp::SLAMFASTSetup" << std::endl;
        return setup_succeed;
    }

    bool start_succeed = drp::SLAMFASTStart(drp_fd_, mvImagePyramid[level].cols, mvImagePyramid[level].rows, level);
    if (!start_succeed)
        std::cerr << "Failed to drp::SLAMFASTStart" << std::endl;

    return start_succeed;
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L831-L852
bool ORBextractor_drp::OperatorSLAMFASTPolling(const int level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorSLAMFASTPolling" << std::endl;
#endif
    vToDistributeKeys[level].clear();
    vToDistributeKeys[level].reserve(mvImagePyramid[level].cols * mvImagePyramid[level].rows * MAX_RATIO_OF_KEYPOINTS_TO_PIXELS);

    bool finish_succeed = drp::SLAMFASTFinish(drp_fd_, level, vToDistributeKeys[level]);

    if (finish_succeed) {
        DumpFASTOutput(vToDistributeKeys[level], level);
    }

    return finish_succeed;
}

#else // ENABLE_SLAMFAST

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L773-L829
bool ORBextractor_drp::OperatorCVFASTStart(const int level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorCVFASTStart" << std::endl;
    std::cerr << "    cell_indice.size() : " << cell_indice.size() << std::endl;
#endif

    if (cell_indice.empty()) {
        MT_START(mt_push_cell_indice[level]);
        // Please refer to https://github.com/BZDOLiVE/YoloPlanarSLAM/blob/45510b59d41eb6bcd28e2657c39cb6ab664b5c1f/src/ORBextractor.cc#L765-L853
        maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3;
        maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3;

        const float width = (maxBorderX - minBorderX);
        const float height = (maxBorderY - minBorderY);

        nCols = width / W;
        nRows = height / W;
        wCell = ceil(width / nCols);
        hCell = ceil(height / nRows);

        for (int i = 0; i < nRows; i++) {
            const float iniY = minBorderY + i * hCell;

            if (iniY >= maxBorderY - 3)
                continue;

            for (int j = 0; j < nCols; j++) {
                const float iniX = minBorderX + j * wCell;
                if (iniX >= maxBorderX - 6)
                    continue;

                cell_indice.push({i, j});
            }
        }

        vToDistributeKeys[level].clear();
        vToDistributeKeys[level].reserve(mvImagePyramid[level].cols * mvImagePyramid[level].rows * MAX_RATIO_OF_KEYPOINTS_TO_PIXELS);
        MT_FINISH(mt_push_cell_indice[level]);
    }

    MT_SUM_START(mt_clone_cell_image[level]);
    const std::pair<size_t, size_t> p = cell_indice.front();
    const size_t cell_idx_y = p.first;
    const size_t cell_idx_x = p.second;

    const int iniY = minBorderY + cell_idx_y * hCell;
    const int maxY = std::min(maxBorderY, iniY + hCell + 6);
    const int iniX = minBorderX + cell_idx_x * wCell;
    const int maxX = std::min(maxBorderX, iniX + wCell + 6);

    const cv::Mat cell_image = mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX);
    MT_SUM_FINISH(mt_clone_cell_image[level]);

    bool start_succeed = drp::CVFASTStart(drp_fd_, cell_image, level, fast_thresholds);
    if (!start_succeed)
        std::cerr << "Failed to drp::CVFASTStart" << std::endl;

    return start_succeed;
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L831-L852
bool ORBextractor_drp::OperatorCVFASTPolling(const int level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorCVFASTPolling" << std::endl;
#endif

    std::vector<cv::KeyPoint> vKeysCell;
    bool finish_succeed = drp::CVFASTFinish(drp_fd_, level, vKeysCell);

    if (finish_succeed) {
        MT_SUM_START(mt_pop_cell_image[level]);
        // Please refer to https://github.com/BZDOLiVE/YoloPlanarSLAM/blob/45510b59d41eb6bcd28e2657c39cb6ab664b5c1f/src/ORBextractor.cc#L818-L826
        if (!vKeysCell.empty()) {
            const std::pair<size_t, size_t> p = cell_indice.front();
            const size_t cell_idx_y = p.first;
            const size_t cell_idx_x = p.second;

            for (std::vector<cv::KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++) {
                (*vit).pt.x += cell_idx_x * wCell;
                (*vit).pt.y += cell_idx_y * hCell;
                vToDistributeKeys[level].push_back(*vit);
            }
        }

        cell_indice.pop();
        MT_SUM_FINISH(mt_pop_cell_image[level]);
    }

    if (finish_succeed && cell_indice.empty()) {
        DumpFASTOutput(vToDistributeKeys[level], level);
    }

    return finish_succeed;
}

#endif // ENABLE_SLAMFAST

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1084-L1086
bool ORBextractor_drp::OperatorGaussianBlurStart(const int level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorGaussianBlurStart" << std::endl;
#endif

    DumpGaussianBlurInput(mvImagePyramid[level], level);

    bool setup_succeed = drp::GaussianBlurSetup(drp_fd_, level);
    if (!setup_succeed) {
        std::cerr << "Failed to drp::GaussianBlurSetup" << std::endl;
        return setup_succeed;
    }

    // preprocess the resized image
    bool start_succeed = drp::GaussianBlurStart(drp_fd_, mvImagePyramid[level].cols, mvImagePyramid[level].rows, level);
    if (!start_succeed)
        std::cerr << "Failed to drp::GaussianBlurStart" << std::endl;

    return start_succeed;
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1084-L1086
bool ORBextractor_drp::OperatorGaussianBlurPolling(const int level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorGaussianBlurPolling" << std::endl;
#endif

#if !defined(ENABLE_DUMP)
    bool finish_succeed = drp::GaussianBlurFinish(drp_fd_, level);
#else
    workingMat[level] = cv::Mat(mvImagePyramid[level].size(), mvImagePyramid[level].type());
    bool finish_succeed = drp::GaussianBlurFinish(drp_fd_, level, workingMat[level]);

    if (finish_succeed) {
        if (!workingMat.empty()) {
            DumpGaussianBlurOutput(workingMat[level], level);
        }

        DumpGaussianBlurOutput(workingMat[level], level);

        // output kernels
        if (!workingMat.empty() && !ORBextractor::output_kernels) {
            PrintKernels(workingMat);
            ORBextractor::output_kernels = true;
        }
    }
#endif

    return finish_succeed;
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1078-L1090
bool ORBextractor_drp::OperatorOrbDescriptorsStart(const int level, const size_t trial) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorOrbDescriptorsStart" << std::endl;
#endif

    DumpOrbDescriptorsInput(allKeypoints[level], level);

    if (level == 0) {
        for (int level = 0; level < nlevels; level++) {
            nkeypoints += (int)allKeypoints[level].size();
        }
        result_keypoints.reserve(nkeypoints);
        result_descriptors = cv::Mat(nkeypoints, 32, CV_8UC1);
    }

    std::vector<cv::KeyPoint>& keypoints = allKeypoints[level];
    int nkeypointsLevel = (int)keypoints.size();

    // If nkeypointsLevel is 0, do not execute drp::computeOrbDescriptorsStart
    if (nkeypointsLevel == 0) {
        fprintf(stderr, "Input keypoints to ORBextractor_drp::OperatorOrbDescriptorsStart(level=%d) is empty.\n", level);
        return true;
    }

    bool setup_succeed = drp::computeOrbDescriptorsSetup(drp_fd_, level, trial);
    if (!setup_succeed) {
        std::cerr << "Failed to drp::computeOrbDescriptorsSetup" << std::endl;
        return setup_succeed;
    }

    // Compute the descriptors
    cv::Mat desc = result_descriptors.rowRange(offset, offset + nkeypointsLevel);
    desc = cv::Mat::zeros((int)keypoints.size(), 32, CV_8UC1);
    bool start_succeed = drp::computeOrbDescriptorsStart(drp_fd_, mvImagePyramid[level].cols, mvImagePyramid[level].rows, keypoints, level, trial);
    if (!start_succeed)
        std::cerr << "Failed to drp::computeOrbDescriptorsStart" << std::endl;

    return start_succeed;
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1092-L1103
bool ORBextractor_drp::OperatorOrbDescriptorsPolling(const int level, const size_t trial) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_drp::OperatorOrbDescriptorsPolling" << std::endl;
#endif

    std::vector<cv::KeyPoint>& keypoints = allKeypoints[level];
    int nkeypointsLevel = (int)keypoints.size();
    cv::Mat desc = result_descriptors.rowRange(offset, offset + nkeypointsLevel);

    // If nkeypointsLevel is 0, drp::computeOrbDescriptorsStart was not executed
    bool finish_succeed = (nkeypointsLevel == 0);

    if (!finish_succeed)
        finish_succeed = drp::computeOrbDescriptorsFinish(drp_fd_, keypoints, level, trial, desc);

#if defined(ENABLE_DUMP)
    if (finish_succeed) {
        DumpOrbDescriptorsOutput(keypoints, desc, level);
    }
#endif

    if (finish_succeed) {
        MT_START(mt_drp_orb_descriptors_postprocess[trial][level]);

        // Scale keypoint coordinates
        if (level != 0) {
            float scale = mvScaleFactor[level]; // getScale(level, firstLevel, scaleFactor);
            for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
                                                     keypointEnd = keypoints.end();
                 keypoint != keypointEnd; ++keypoint)
                keypoint->pt *= scale;
        }

        offset += nkeypointsLevel;

        // And add the keypoints to the output
        result_keypoints.insert(result_keypoints.end(), keypoints.begin(), keypoints.end());

        MT_FINISH(mt_drp_orb_descriptors_postprocess[trial][level]);
    }

    return finish_succeed;
}

} // namespace ORB_SLAM2
