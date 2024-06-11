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

#ifndef ORBEXTRACTOR_DRP_H
#define ORBEXTRACTOR_DRP_H

#include "ORBextractor.h"

#include <sys/stat.h>

namespace ORB_SLAM2 {

constexpr int EDGE_THRESHOLD = 19;

class ORBextractor_drp : public ORBextractor {
public:
    ORBextractor_drp(
        int drp_fd,
        float scaleFactor, int nlevels,
        int iniThFAST, int minThFAST, const bool use_drp_ai)
        : ORBextractor(
            scaleFactor, nlevels,
            iniThFAST, minThFAST, use_drp_ai),
          drp_fd_(drp_fd) {
        nkeypoints = 0;
        offset = 0;
        result_keypoints.clear();
        workingMat = std::vector<cv::Mat>(nlevels);
        vToDistributeKeys.resize(nlevels);
    }

    ~ORBextractor_drp() {}

    virtual bool OperatorResize(const cv::InputArray zero_level_img, const int input_level) {
        fprintf(stderr, "ORBextractor_drp::OperatorResize is not implemented.\n");
        return false;
    }
    virtual bool OperatorCVFAST(const int level) {
        fprintf(stderr, "ORBextractor_drp::OperatorCVFAST is not implemented.\n");
        return false;
    }
    virtual bool OperatorGaussianBlur(const int level) {
        fprintf(stderr, "ORBextractor_drp::OperatorGaussianBlur is not implemented.\n");
        return false;
    }
    virtual bool OperatorOrbDescriptors(const int level, const size_t trial) {
        fprintf(stderr, "ORBextractor_drp::OperatorOrbDescriptors is not implemented.\n");
        return false;
    }

    bool OperatorResizeStart(const cv::InputArray zero_level_img, const int input_level);
    bool OperatorResizePolling(const int input_level);
#if defined(ENABLE_SLAMFAST)
    bool OperatorSLAMFASTStart(const int level);
    bool OperatorSLAMFASTPolling(const int level);
#else
    bool OperatorCVFASTStart(const int level);
    bool OperatorCVFASTPolling(const int level);
#endif
    bool OperatorGaussianBlurStart(const int level);
    bool OperatorGaussianBlurPolling(const int level);
    bool OperatorOrbDescriptorsStart(const int level, const size_t trial);
    bool OperatorOrbDescriptorsPolling(const int level, const size_t trial);

    std::vector<cv::KeyPoint> result_keypoints;
    cv::Mat result_descriptors;

    std::vector<std::vector<cv::KeyPoint>> allKeypoints;

    std::vector<std::vector<cv::KeyPoint>> vToDistributeKeys;
    std::queue<std::pair<size_t, size_t>> cell_indice;

protected:
    int drp_fd_;

    cv::Mat in_image;
    int nkeypoints;
    int offset;
    std::vector<cv::Mat> workingMat;

    // Please refer to https://github.com/BZDOLiVE/YoloPlanarSLAM/blob/45510b59d41eb6bcd28e2657c39cb6ab664b5c1f/src/ORBextractor.cc#L765-L853
    const float W = 30;
    const int minBorderX = EDGE_THRESHOLD - 3;
    const int minBorderY = minBorderX;
    int maxBorderX;
    int maxBorderY;

    int nCols;
    int nRows;
    int wCell;
    int hCell;

    const int fast_thresholds[2] = {20, 7};

    struct stat statBuf;
};

} // namespace ORB_SLAM2

#endif
