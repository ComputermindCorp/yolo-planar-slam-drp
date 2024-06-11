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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

#if defined(ENABLE_DUMP)
#include <sys/stat.h>
#endif

#include "YoloDetector.h"

#if defined(ENABLE_DRP_AI)
#include "command/object_detection.h"
#endif

constexpr double MAX_RATIO_OF_KEYPOINTS_TO_PIXELS = 0.1; // 10%

namespace ORB_SLAM2 {

const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;

float IC_Angle(const cv::Mat& image, cv::Point2f pt, const std::vector<int>& u_max);
void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax);

class ExtractorNode {
public:
    ExtractorNode()
        : bNoMore(false) {}

    void DivideNode(ExtractorNode& n1, ExtractorNode& n2, ExtractorNode& n3, ExtractorNode& n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor {
public:
    enum { HARRIS_SCORE = 0,
           FAST_SCORE = 1 };

    ORBextractor(
        float scaleFactor, int nlevels,
        int iniThFAST, int minThFAST, const bool use_drp_ai);

    ~ORBextractor() {}

    void OperatorImagePyramid(const cv::InputArray _image);
    void OperatorSLAMFAST(std::vector<cv::KeyPoint>& vToDistributeKeys, const int level);
    void OperatorOrientation(std::vector<std::vector<cv::KeyPoint>>& allKeypoints, const int level, const size_t trial);
    void OperatorGaussianBlur(cv::Mat& workingMat, const int level);
    void OperatorOrbDescriptors(const cv::Mat& workingMat, std::vector<std::vector<cv::KeyPoint>>& allKeypoints, const int offset, const int level, const size_t trial, cv::Mat& descriptors);
    void OperatorPrune(const std::vector<cv::KeyPoint>& input_keypoints, std::vector<std::vector<cv::KeyPoint>>& allKeypoints, const int nfeatures, const int level, const size_t trial);
    void OperatorResultDump(const std::vector<cv::KeyPoint>& _keypoints, const cv::Mat& _descriptors);

    void ReceiveYoloOutput();

    // Remove dynamic keypoints
    void RemoveMovingKeyPoints(std::vector<cv::KeyPoint>& tempKeypoints, const float scale);
    void RemoveMovingKeyPoints(std::vector<std::vector<cv::KeyPoint>>& mvKeyPoints);

    int inline GetLevels() {
        return nlevels;
    }

    float inline GetScaleFactor() {
        return scaleFactor;
    }

    std::vector<float> inline GetScaleFactors() {
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors() {
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares() {
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares() {
        return mvInvLevelSigma2;
    }

#if defined(ENABLE_DRP_AI)
    void SetYoloBoundingBoxList(std::shared_ptr<ObjectDetection>& result_object_detection);
#endif

    void SetYoloBoundingBoxList(const std::vector<YoloBoundingBox>& yoloBoundingBoxList) {
        yoloBoundingBoxList_ = yoloBoundingBoxList;
    }

    std::vector<YoloBoundingBox> GetYoloBoundingBoxList() const {
        return yoloBoundingBoxList_;
    }

    void SetYoloDetector(YoloDetector* mYoloDetector) {
        mYoloDetector_ = mYoloDetector;
    }

    std::vector<cv::Mat> mvImagePyramid;
    int nlevels;

#if defined(ENABLE_DUMP) || defined(ENABLE_DEBUG_OUTPUT)
    static int frame_idx;
#endif

    void ComputePyramid(cv::Mat image);
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int& minX,
                                                const int& maxX, const int& minY, const int& maxY,
                                                const int& nFeaturesPerLevel, const int nfeatures, const int& level);

protected:
    void DumpInputImage(const cv::InputArray input_image);
    void DumpPyramid(const std::vector<cv::Mat>& image_pyramid, const size_t level);
    void DumpFASTOutput(const std::vector<cv::KeyPoint>& keypoints, const size_t level);
    void DumpGaussianBlurInput(const cv::InputArray input_image, const size_t level);
    void DumpGaussianBlurOutput(const cv::InputArray image, const size_t level);
    void DumpOrbDescriptorsInput(const std::vector<cv::KeyPoint>& keypoints, const size_t level);
    void DumpOrbDescriptorsOutput(const std::vector<cv::KeyPoint>& keypoints,
                                  const cv::Mat& descriptors,
                                  const size_t level);
    void DumpYoloOutput();
    void PrintKernels(const cv::InputArray image);

    std::vector<cv::Point> pattern;

    double scaleFactor;
    int iniThFAST;
    int minThFAST;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

#if defined(ENABLE_DUMP)
    struct stat statBuf;
#endif

    YoloDetector* mYoloDetector_;

#if defined(ENABLE_DRP_AI)
    const bool use_drp_ai_;
#endif

    static bool output_kernels;

private:
    // Save the result from yolo object detection
    std::vector<YoloBoundingBox> yoloBoundingBoxList_;
};

} // namespace ORB_SLAM2

#endif
