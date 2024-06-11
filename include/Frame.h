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

#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "YoloDetector.h"
#include "CAPE.h"
#include "MapPlane.h"

#include "drp.h"

namespace ORB_SLAM2 {
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class MapPlane;
class ImageProcessing;

// class BoundingBox
// {
//     public:
//     cv::Rect2f rect;
//     std::string label;

//     BoundingBox(cv::Rect2f input_rect, std::string input_label){
//         this->rect = input_rect;
//         this->label = input_label;
//     }

//     cv::Rect2f GetRect(){return this->rect;}
//     std::string GetLabel(){return this->label;}
// };

class Frame {
public:
    Frame();

    // Copy constructor.
    Frame(const Frame& frame);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat& imGray, const cv::Mat& imDepth, const double& timeStamp, ORBextractor* extractor, ORBVocabulary* voc,
          ImageProcessing* image_processing_ptr,
          cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const bool use_drp);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat& imGray, const double& timeStamp, ORBextractor* extractor, ORBVocabulary* voc,
          ImageProcessing* image_processing_ptr,
          cv::Mat& K, cv::Mat& distCoef, const float& bf, const float& thDepth, const bool use_drp);

    ~Frame(void);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter() {
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse() {
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY);

    vector<size_t> GetFeaturesInArea(const float& x, const float& y, const float& r, const int minLevel = -1, const int maxLevel = -1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat& imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int& i);

    // I adjust the construct function of Frame in original orbslam
    // and I move something in original construct to this function
    // after remove dynamic keypoints and plane extraction, use this function
    void MyCalculateAfterRemoveDynamicPointsAndPlaneExtraction();
    void MyCalculateAfterRemoveDynamicPointsAndPlaneExtraction(const cv::Mat& imDepth);

    // Remove dynamic keypoints
    void RemoveMovingKeyPoints();

    cv::Mat ComputePlaneWorldCoeff(const int& idx);

    // build point cloud for every MapPlane in this keyframe from segment image
    void BuildMapPlanePointCloud(const cv::Mat& imDepth, const cv::Mat& segmentImage, int cloudStep = 5);

    // Last processing of the constructor
    void LastProcessingOfConstructor(const cv::Mat& imGray, const cv::Mat& K);

    void SetYoloBoundingBoxList(const std::vector<YoloBoundingBox>& yoloBoundingBoxList);
    std::vector<YoloBoundingBox> GetYoloBoundingBoxList() const;

    void SetYoloDetector(YoloDetector* mYoloDetector);

    void ExtractOrbDescriptors(const std::vector<cv::Mat>& blurred_images, const size_t trial);

    // Dummy function
    virtual bool drp_setup() {
        fprintf(stderr, "Frame::drp_setup is not implemented.\n");
        return false;
    };
    virtual void drp_preprocess() {
        fprintf(stderr, "Frame::drp_preprocess is not implemented.\n");
    };
    virtual bool drp_start(const size_t trial) {
        fprintf(stderr, "Frame::drp_start is not implemented.\n");
        return false;
    };
    virtual bool drp_polling(const size_t trial) {
        fprintf(stderr, "Frame::drp_polling is not implemented.\n");
        return false;
    };
    virtual void drp_postprocess() {
        fprintf(stderr, "Frame::drp_postprocess is not implemented.\n");
    };
    virtual bool drp_execute(const size_t trial) {
        fprintf(stderr, "Frame::drp_execute is not implemented.\n");
        return false;
    };
    virtual bool drp_is_finished() {
        fprintf(stderr, "Frame::drp_is_finished is not implemented.\n");
        return false;
    };
    virtual bool drp_retry(const size_t trial) {
        fprintf(stderr, "Frame::drp_retry is not implemented.\n");
        return false;
    };
    virtual bool done_fast() {
        fprintf(stderr, "Frame::done_fast is not implemented.\n");
        return false;
    };
    virtual void postprocess_fast(const int max_features, const size_t trial) {
        fprintf(stderr, "Frame::postprocess_fast is not implemented.\n");
    };
    virtual int get_status_algorithm() {
        fprintf(stderr, "Frame::get_status_algorithm is not implemented.\n");
        return drp::STATE_NONE;
    };
    virtual ssize_t get_status_level() {
        fprintf(stderr, "Frame::get_status_level is not implemented.\n");
        return -1;
    };
    virtual void reset_to_orb_descriptors() {
        fprintf(stderr, "Frame::reset_to_orb_descriptors is not implemented.\n");
    };
    virtual void LastProcessingOfConstructor(){};

public:
    // Save the feature points from ORB algorithm
    // After feature extraction, delete dynamic points from this vector
    std::vector<std::vector<cv::KeyPoint>> mvKeyPoints;

    // Save the result of plane coefficient from CAPE
    std::vector<CAPE::PlaneSeg> mvPlaneCameraCoefficients;
    // Save the result of cylinder coefficient from CAPE
    std::vector<CAPE::CylinderSeg> mvCylinderCameraCoefficients;
    // Size of mvPlaneCameraCoefficients
    int mnPlaneNum;

    // MapPlanes associated to plane in this frame, NULL pointer if no association.
    std::vector<MapPlane*> mvpMapPlanes;
    // Flag to identify outlier planes new planes.
    std::vector<bool> mvbPlaneOutlier;

    // used to determine a keyframe
    bool mbNewPlane;

    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

    ImageProcessing* image_processing_ptr_;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

    bool use_drp_;

private:
    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat& imLeft);

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

} // namespace ORB_SLAM2

#endif // FRAME_H
