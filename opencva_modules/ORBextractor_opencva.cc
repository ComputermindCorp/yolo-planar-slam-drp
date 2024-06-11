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

#include "ORBextractor_opencva.h"

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

#include "drp.h"
#include "opencva.h"

#include "measure_time.h"

namespace ORB_SLAM2 {

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1046-L1053
bool ORBextractor_opencva::OperatorResize(const cv::InputArray zero_level_img, const int input_level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_opencva::OperatorResize" << std::endl;
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
    const float output_scale = 1.0f / mvScaleFactor[input_level + 1];
    const cv::Size output_image_size(cvRound((float)in_image.cols * output_scale), cvRound((float)in_image.rows * output_scale));

    if (input_level == 0) {
        mvImagePyramid[input_level] = in_image;
    }

    MT_START(mt_resize[input_level]);
    bool opencva_succeed = opencva::resize(mvImagePyramid[input_level], input_level, output_image_size, mvImagePyramid[input_level + 1]);
    MT_FINISH(mt_resize[input_level]);

    if (!opencva_succeed) {
        std::cerr << "Failed to opencva::resize" << std::endl;
        return false;
    }

    if (input_level == 0) {
        DumpPyramid(mvImagePyramid, input_level);
    }
    DumpPyramid(mvImagePyramid, input_level + 1);

    return opencva_succeed;
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L773-L829
bool ORBextractor_opencva::OperatorCVFAST(const int level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_opencva::OperatorCVFAST" << std::endl;
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

    std::vector<cv::KeyPoint> vKeysCell;

    MT_SUM_START(mt_cvfast[level]);
    bool opencva_succeed = opencva::cvfast(cell_image, level, fast_thresholds[0], vKeysCell);
    if (!opencva_succeed) {
        std::cerr << "Failed to opencva::cvfast" << std::endl;
        return false;
    }

    if (vKeysCell.empty()) {
        bool opencva_succeed = opencva::cvfast(cell_image, level, fast_thresholds[1], vKeysCell);
        if (!opencva_succeed) {
            std::cerr << "Failed to opencva::cvfast" << std::endl;
            return false;
        }
    }
    MT_SUM_FINISH(mt_cvfast[level]);

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

    if (cell_indice.empty()) {
        DumpFASTOutput(vToDistributeKeys[level], level);
    }

    return true;
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1084-L1086
bool ORBextractor_opencva::OperatorGaussianBlur(const int level) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_opencva::OperatorGaussianBlur" << std::endl;
#endif

    DumpGaussianBlurInput(mvImagePyramid[level], level);

    // preprocess the resized image
    MT_START(mt_gaussianblur[level]);
    bool opencva_succeed = opencva::gaussian_blur(mvImagePyramid[level], level, workingMat[level]);
    MT_FINISH(mt_gaussianblur[level]);

    if (!opencva_succeed) {
        std::cerr << "Failed to opencva::gaussian_blur" << std::endl;
        return false;
    }

    if (!workingMat.empty()) {
        DumpGaussianBlurOutput(workingMat[level], level);
    }

    DumpGaussianBlurOutput(workingMat[level], level);

    // output kernels
    if (!workingMat.empty() && !ORBextractor::output_kernels) {
        PrintKernels(workingMat);
        ORBextractor::output_kernels = true;
    }

    return true;
}

// Please refer to https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/ORBextractor.cc#L1078-L1090
bool ORBextractor_opencva::OperatorOrbDescriptors(const int level, const size_t trial) {
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "start " << ORBextractor::frame_idx << " frame ORBextractor_opencva::OperatorOrbDescriptors" << std::endl;
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
        fprintf(stderr, "Input keypoints to ORBextractor_opencva::OperatorOrbDescriptorsStart(level=%d) is empty.\n", level);
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
    bool start_succeed = drp::computeOrbDescriptorsStart(drp_fd_, workingMat[level], keypoints, level, trial);
    if (!start_succeed) {
        std::cerr << "Failed to drp::computeOrbDescriptorsStart" << std::endl;
        return false;
    }

    // If nkeypointsLevel is 0, drp::computeOrbDescriptorsStart was not executed
    bool finish_succeed = (nkeypointsLevel == 0);

    while (!finish_succeed) {
        finish_succeed = drp::computeOrbDescriptorsFinish(drp_fd_, keypoints, level, trial, desc);
    }

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

    return true;
}

} // namespace ORB_SLAM2
