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

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "Enum.h"

#include <unistd.h>
#include <mutex>

#include "PlaneMatcher.h"
#include "YoloDetector.h"
#include "CAPE.h"

#if defined(ENABLE_DRP)
#include "ORBextractor_drp.h"
#include "Frame_drp.h"
#endif

#if defined(ENABLE_DRP_AI)
#include "YoloDetector_drp.h"
#endif

namespace ORB_SLAM2 {

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class PlaneMatcher;

class SocketViewer;

class Tracking {
public:
    Tracking(
#if defined(ENABLE_DRP)
        const int drp_fd,
#endif
        System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
        KeyFrameDatabase* pKFDB, const string& strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageRGBD();
    cv::Mat GrabImageMonocular();

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetSocketViewer(SocketViewer* pSocketViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string& strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool& flag);

public:
    YoloDetector* mYoloDetector;

    // ORB
#if defined(ENABLE_DRP)
    ORBextractor_drp *mpORBextractorLeft, *mpORBextractorRight;
#else
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
#endif

    // BoW
    ORBVocabulary* mpORBVocabulary;

    CAPE::PlaneDetector mPlaneDetector;
    PlaneMatcher mPlaneMatcher;
    // When tracking, system need enough keypoint matching.
    // If system can not get enough keypoint matching, we set tracking status to lost.
    // This number represents the effect of plane.
    // For example if this number is x, one plane matching have same effect as x point matching.
    const int mPlaneEffect;

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    std::unique_ptr<Frame> mCurrentFrame;
    cv::Mat mImGray;
    cv::Mat mImDepth;
    double timestamp;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    // Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    void Reset();

    bool use_drp;
    bool use_drp_ai;
    bool use_opencva;

    int nFeatures;

    std::atomic<bool> wait_for_next_frame;

protected:
    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    // Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    // Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    // System
    System* mpSystem;

    // Drawers
    Viewer* mpViewer;
    SocketViewer* mpSocketViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // Map
    Map* mpMap;

protected:
    // New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Current matches in frame
    int mnMatchesInliers;

    // Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    // Motion Model
    cv::Mat mVelocity;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

private:
#if defined(ENABLE_DRP)
    const int drp_fd_;
#endif
};

} // namespace ORB_SLAM2

#endif // TRACKING_H
