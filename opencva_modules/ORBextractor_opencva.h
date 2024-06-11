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

#ifndef ORBEXTRACTOR_OPENCVA_H
#define ORBEXTRACTOR_OPENCVA_H

#include "ORBextractor.h"
#include "ORBextractor_drp.h"
#include "drp.h"

#include <sys/stat.h>

namespace ORB_SLAM2 {

// constexpr int EDGE_THRESHOLD = 19;

class ORBextractor_opencva : public ORBextractor_drp {
public:
    ORBextractor_opencva(const int drp_fd, float scaleFactor, int nlevels,
                         int iniThFAST, int minThFAST, const bool use_drp_ai)
        : ORBextractor_drp(drp_fd, scaleFactor, nlevels,
                           iniThFAST, minThFAST, use_drp_ai) {
    }

    ~ORBextractor_opencva() {}

    bool OperatorResize(const cv::InputArray zero_level_img, const int input_level) override;
    bool OperatorCVFAST(const int level) override;
    bool OperatorGaussianBlur(const int level) override;
    bool OperatorOrbDescriptors(const int level, const size_t trial) override;
};

} // namespace ORB_SLAM2

#endif // ORBEXTRACTOR_OPENCVA_H
