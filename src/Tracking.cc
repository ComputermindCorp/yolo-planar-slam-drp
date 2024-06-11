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

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"
#include "ConfigFile.h"

#include "Enum.h"

#if !defined(ENABLE_SLAMFAST)
#include "ORBextractor_opencva.h"
#endif

#include <iostream>

#include <mutex>

#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

using namespace std;

namespace ORB_SLAM2 {

Tracking::Tracking(
#if defined(ENABLE_DRP)
    const int drp_fd,
#endif
    System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap, KeyFrameDatabase* pKFDB, const string& strSettingPath, const int sensor)
    : mpORBVocabulary(pVoc), mPlaneDetector(), mPlaneMatcher(), mPlaneEffect(3),
      mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false),
      mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL), mpSocketViewer(NULL),
      mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),
#if defined(ENABLE_DRP)
      drp_fd_(drp_fd)
#endif
{

    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    mPlaneDetector = CAPE::PlaneDetector(fSettings["Camera.height"], fSettings["Camera.width"], false, ConfigFile::planeMinSize, ConfigFile::planeCellWidth, ConfigFile::planeCellHeight);
    mPlaneDetector.loadCalibParameters(fx, fy, cx, cy);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl
         << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if (DistCoef.rows == 5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if (mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    use_drp = (((std::string)fSettings["UseDrp"]) == "true");
    use_drp_ai = (((std::string)fSettings["UseDrpAI"]) == "true");
    use_opencva = (((std::string)fSettings["UseOpenCVA"]) == "true");

    assert(use_drp_ai == true && "Cannot use ncnn. use_drp_ai must be true.");
    assert(!use_drp || (use_drp && 0 < drp_fd_));
    assert((!use_opencva || use_drp) && "If use_opencva is true, use_drp must be true.");
#if defined(ENABLE_SLAMFAST)
    assert(!use_opencva && "SLAMFAST is not available in OpenCVA.");
#endif

#if !defined(ENABLE_SLAMFAST)
    if (use_opencva) {
        mpORBextractorLeft = new ORBextractor_opencva(drp_fd_, fScaleFactor, nLevels, fIniThFAST, fMinThFAST, use_drp_ai);

        if (sensor == eSensor::STEREO) {
            mpORBextractorRight = new ORBextractor_opencva(drp_fd_, fScaleFactor, nLevels, fIniThFAST, fMinThFAST, use_drp_ai);
        }
    }
    else
#endif
    {
        mpORBextractorLeft =
#if defined(ENABLE_DRP)
            new ORBextractor_drp(
                drp_fd_,
#else
            new ORBextractor(
#endif
                fScaleFactor, nLevels, fIniThFAST, fMinThFAST, use_drp_ai);

        if (sensor == eSensor::STEREO) {
            mpORBextractorRight =
#if defined(ENABLE_DRP)
                new ORBextractor_drp(
                    drp_fd_,
#else
                new ORBextractor(
#endif
                    fScaleFactor, nLevels, fIniThFAST, fMinThFAST, use_drp_ai);
        }
    }

    cout << endl
         << "ORB Extractor Parameters: " << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if (sensor == eSensor::STEREO || sensor == eSensor::RGBD) {
        mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
        cout << endl
             << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if (sensor == eSensor::RGBD) {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if (fabs(mDepthMapFactor) < 1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }
    cout << "build tracking thread success" << endl;

#if defined(ENABLE_DRP_AI)
    if (use_drp_ai) {
        mYoloDetector = new YoloDetector_drp();
    }
    else
#endif
        mYoloDetector = new YoloDetector();

    wait_for_next_frame.store(true);
}

void Tracking::SetLocalMapper(LocalMapping* pLocalMapper) {
    mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing* pLoopClosing) {
    mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(Viewer* pViewer) {
    mpViewer = pViewer;
}

void Tracking::SetSocketViewer(SocketViewer* pSocketViewer) {
    mpSocketViewer = pSocketViewer;
}

cv::Mat Tracking::GrabImageRGBD() {
    MT_START(mt_plane_detector_process);
    mPlaneDetector.Process(mImDepth, mCurrentFrame->mvPlaneCameraCoefficients, mCurrentFrame->mvCylinderCameraCoefficients);
    MT_FINISH(mt_plane_detector_process);

    MT_START(mt_my_calculate_after);
    mCurrentFrame->MyCalculateAfterRemoveDynamicPointsAndPlaneExtraction(mImDepth);
    MT_FINISH(mt_my_calculate_after);

    MT_START(mt_track);
    Track();
    MT_FINISH(mt_track);

    mPlaneDetector.mpCurrentDepthImage = nullptr;

    return mCurrentFrame->mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocular() {
    MT_START(mt_my_calculate_after);
    mCurrentFrame->MyCalculateAfterRemoveDynamicPointsAndPlaneExtraction();
    MT_FINISH(mt_my_calculate_after);

    MT_START(mt_track);
    Track();
    MT_FINISH(mt_track);

    mPlaneDetector.mpCurrentDepthImage = nullptr;

    return mCurrentFrame->mTcw.clone();
}

void Tracking::Track() {
    if (mState == NO_IMAGES_YET) {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;

    MT_START(mt_lock_map_update);
    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    MT_FINISH(mt_lock_map_update);

    if (mState == NOT_INITIALIZED) {
        MT_START(mt_monocular_initialization);
        if (mSensor == eSensor::STEREO || mSensor == eSensor::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();
        MT_FINISH(mt_monocular_initialization);

        mpFrameDrawer->Update(this);

        if (mState != OK)
            return;
    }
    else {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if (!mbOnlyTracking) {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if (mState == OK) {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if (mVelocity.empty() || mCurrentFrame->mnId < mnLastRelocFrameId + 2) {
                    MT_START(mt_track_reference_keyframe);
                    bOK = TrackReferenceKeyFrame();
                    MT_FINISH(mt_track_reference_keyframe);
                }
                else {
                    MT_START(mt_track_current_frame);
                    bOK = TrackWithMotionModel();
                    MT_FINISH(mt_track_current_frame);
                    if (!bOK) {
                        MT_START(mt_track_reference_keyframe);
                        bOK = TrackReferenceKeyFrame();
                        MT_FINISH(mt_track_reference_keyframe);
                    }
                }
            }
            else {
                bOK = Relocalization();
            }
        }
        else {
            // Localization Mode: Local Mapping is deactivated

            if (mState == LOST) {
                bOK = Relocalization();
            }
            else {
                if (!mbVO) {
                    // In last frame we tracked enough MapPoints in the map

                    if (!mVelocity.empty()) {
                        MT_START(mt_track_current_frame);
                        bOK = TrackWithMotionModel();
                        MT_FINISH(mt_track_current_frame);
                    }
                    else {
                        MT_START(mt_track_reference_keyframe);
                        bOK = TrackReferenceKeyFrame();
                        MT_FINISH(mt_track_reference_keyframe);
                    }
                }
                else {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if (!mVelocity.empty()) {
                        MT_START(mt_track_current_frame);
                        bOKMM = TrackWithMotionModel();
                        MT_FINISH(mt_track_current_frame);
                        vpMPsMM = mCurrentFrame->mvpMapPoints;
                        vbOutMM = mCurrentFrame->mvbOutlier;
                        TcwMM = mCurrentFrame->mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if (bOKMM && !bOKReloc) {
                        mCurrentFrame->SetPose(TcwMM);
                        mCurrentFrame->mvpMapPoints = vpMPsMM;
                        mCurrentFrame->mvbOutlier = vbOutMM;

                        if (mbVO) {
                            for (int i = 0; i < mCurrentFrame->N; i++) {
                                if (mCurrentFrame->mvpMapPoints[i] && !mCurrentFrame->mvbOutlier[i]) {
                                    mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if (bOKReloc) {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame->mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if (!mbOnlyTracking) {
            if (bOK) {
                MT_START(mt_track_local_map);
                bOK = TrackLocalMap();
                MT_FINISH(mt_track_local_map);
            }
        }
        else {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if (bOK && !mbVO) {
                MT_START(mt_track_local_map);
                bOK = TrackLocalMap();
                MT_FINISH(mt_track_local_map);
            }
        }

        if (bOK)
            mState = OK;
        else
            mState = LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if (bOK) {
            // Update motion model
            if (!mLastFrame.mTcw.empty()) {
                cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                mVelocity = mCurrentFrame->mTcw * LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);

            // Clean VO matches
            for (int i = 0; i < mCurrentFrame->N; i++) {
                MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];
                if (pMP)
                    if (pMP->Observations() < 1) {
                        mCurrentFrame->mvbOutlier[i] = false;
                        mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for (list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++) {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if (NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for (int i = 0; i < mCurrentFrame->N; i++) {
                if (mCurrentFrame->mvpMapPoints[i] && mCurrentFrame->mvbOutlier[i])
                    mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (mState == LOST) {
            if (mpMap->KeyFramesInMap() <= 5) {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if (!mCurrentFrame->mpReferenceKF)
            mCurrentFrame->mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(*mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if (!mCurrentFrame->mTcw.empty()) {
        cv::Mat Tcr = mCurrentFrame->mTcw * mCurrentFrame->mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame->mTimeStamp);
        mlbLost.push_back(mState == LOST);
    }
    else {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState == LOST);
    }
}

void Tracking::StereoInitialization() {
    // NEED TEST
    if (mCurrentFrame->N > 500 || (mCurrentFrame->mnPlaneNum >= 3 && mCurrentFrame->N > 50)) {
        // Set Frame pose to the origin
        mCurrentFrame->SetPose(cv::Mat::eye(4, 4, CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for (int i = 0; i < mCurrentFrame->N; i++) {
            float z = mCurrentFrame->mvDepth[i];
            if (z > 0) {
                cv::Mat x3D = mCurrentFrame->UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpMap);
                pNewMP->AddObservation(pKFini, i);
                pKFini->AddMapPoint(pNewMP, i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame->mvpMapPoints[i] = pNewMP;
            }
        }

        // Add planes
        for (int i = 0; i < mCurrentFrame->mnPlaneNum; i++) {
            if (mCurrentFrame->mvbPlaneOutlier[i]) {
                continue;
            }
            cv::Mat p3D = mCurrentFrame->ComputePlaneWorldCoeff(i);
            MapPlane* pNewMP = new MapPlane(p3D, pKFini, mpMap);
            pNewMP->AddObservation(pKFini, i);
            pKFini->AddMapPlane(pNewMP, i);
            mpMap->AddMapPlane(pNewMP);
            mCurrentFrame->mvpMapPlanes[i] = pNewMP;
            pNewMP->mnFirstFrameId = mCurrentFrame->mnId;
        }
        pKFini->BuildMapPlanePointCloud(*(mPlaneDetector.mpCurrentDepthImage), mPlaneDetector.mSegmentImageOutput, ConfigFile::cloudStep);

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(*mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame->mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame->mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame->mTcw);

        mState = OK;
    }
}

void Tracking::MonocularInitialization() {
    if (!mpInitializer) {
        // Set Reference Frame
        if (mCurrentFrame->mvKeys.size() > 100) {
            mInitialFrame = Frame(*mCurrentFrame);
            mLastFrame = Frame(*mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame->mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame->mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame->mvKeysUn[i].pt;

            if (mpInitializer)
                delete mpInitializer;

            mpInitializer = new Initializer(*mCurrentFrame, 1.0, 200);

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            return;
        }
    }
    else {
        // Try to initialize
        if ((int)mCurrentFrame->mvKeys.size() <= 100) {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        MT_START(mt_search_for_initialization);
        int nmatches = matcher.SearchForInitialization(mInitialFrame, *mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);
        MT_FINISH(mt_search_for_initialization);

        // Check if there are enough correspondences
        if (nmatches < 100) {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw;                 // Current Camera Rotation
        cv::Mat tcw;                 // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if (mpInitializer->Initialize(*mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
            Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
            tcw.copyTo(Tcw.rowRange(0, 3).col(3));
            mCurrentFrame->SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular() {
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for (size_t i = 0; i < mvIniMatches.size(); i++) {
        if (mvIniMatches[i] < 0)
            continue;

        // Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        // Fill Current Frame structure
        mCurrentFrame->mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame->mvbOutlier[mvIniMatches[i]] = false;

        // Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
        if (vpAllMapPoints[iMP]) {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame->SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame->mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame->mpReferenceKF = pKFcur;

    mLastFrame = Frame(*mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;
}

void Tracking::CheckReplacedInLastFrame() {
    for (int i = 0; i < mLastFrame.N; i++) {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if (pMP) {
            MapPoint* pRep = pMP->GetReplaced();
            if (pRep) {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

bool Tracking::TrackReferenceKeyFrame() {
    // Compute Bag of Words vector
    mCurrentFrame->ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<MapPoint*> vpMapPointMatches;

    MT_START(mt_search_by_bow_in_track_reference_keyframe);
    int nmatches = matcher.SearchByBoW(mpReferenceKF, *mCurrentFrame, vpMapPointMatches);
    MT_FINISH(mt_search_by_bow_in_track_reference_keyframe);

    if (nmatches < 15)
        return false;

    mCurrentFrame->mvpMapPoints = vpMapPointMatches;
    mCurrentFrame->SetPose(mLastFrame.mTcw);

    // Plane Matching
    int planeMatchNum = mPlaneMatcher.AssociatePlanesFromMap(*mCurrentFrame, mpMap->GetAllMapPlanes());

    Optimizer::PoseOptimization(mCurrentFrame.get());

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame->N; i++) {
        if (mCurrentFrame->mvpMapPoints[i]) {
            if (mCurrentFrame->mvbOutlier[i]) {
                MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];

                mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                mCurrentFrame->mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame->mnId;
                nmatches--;
            }
            else if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    int nDisgardPlane = 0;
    for (int i = 0; i < mCurrentFrame->mnPlaneNum; i++) {
        if (mCurrentFrame->mvpMapPlanes[i]) {
            if (mCurrentFrame->mvpMapPlanes[i] != nullptr && mCurrentFrame->mvbPlaneOutlier[i]) {
                mCurrentFrame->mvpMapPlanes[i] = static_cast<MapPlane*>(NULL);
                planeMatchNum--;
                nDisgardPlane++;
            }
            else
                nmatchesMap = nmatchesMap + mPlaneEffect;
        }
    }

    // mCurrentFrame->BuildMapPlanePointCloud(*(mPlaneDetector.mpCurrentDepthImage), mPlaneDetector.mSegmentImageOutput, ConfigFile::cloudStep);

    return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame() {
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == eSensor::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float, int>> vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for (int i = 0; i < mLastFrame.N; i++) {
        float z = mLastFrame.mvDepth[i];
        if (z > 0) {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if (!pMP)
            bCreateNew = true;
        else if (pMP->Observations() < 1) {
            bCreateNew = true;
        }

        if (bCreateNew) {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

            mLastFrame.mvpMapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else {
            nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > 100)
            break;
    }
}

bool Tracking::TrackWithMotionModel() {
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame->SetPose(mVelocity * mLastFrame.mTcw);

    fill(mCurrentFrame->mvpMapPoints.begin(), mCurrentFrame->mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if (mSensor != eSensor::STEREO)
        th = 15;
    else
        th = 7;

    MT_START(mt_search_for_projection_in_track_with_motion_model_1);
    int nmatches = matcher.SearchByProjection(*mCurrentFrame, mLastFrame, th, mSensor == eSensor::MONOCULAR);
    MT_FINISH(mt_search_for_projection_in_track_with_motion_model_1);

    // If few matches, uses a wider window search
    if (nmatches < 20) {
        fill(mCurrentFrame->mvpMapPoints.begin(), mCurrentFrame->mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
        MT_START(mt_search_for_projection_in_track_with_motion_model_2);
        nmatches = matcher.SearchByProjection(*mCurrentFrame, mLastFrame, 2 * th, mSensor == eSensor::MONOCULAR);
        MT_FINISH(mt_search_for_projection_in_track_with_motion_model_2);
    }

    // Plane matching
    int planeMatchNum = mPlaneMatcher.AssociatePlanesFromMap(*mCurrentFrame, mpMap->GetAllMapPlanes());

    nmatches = nmatches + (mPlaneEffect * planeMatchNum);
    if (nmatches < 20) {
        return false;
    }

    // Optimize frame pose with all matches
    MT_START(mt_pose_optimization_in_track_with_motion_model);
    Optimizer::PoseOptimization(mCurrentFrame.get());
    MT_FINISH(mt_pose_optimization_in_track_with_motion_model);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame->N; i++) {
        if (mCurrentFrame->mvpMapPoints[i]) {
            if (mCurrentFrame->mvbOutlier[i]) {
                MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];

                mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                mCurrentFrame->mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame->mnId;
                nmatches--;
            }
            else if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }
    int nDisgardPlane = 0;
    for (int i = 0; i < mCurrentFrame->mnPlaneNum; i++) {
        if (mCurrentFrame->mvpMapPlanes[i]) {
            if (mCurrentFrame->mvpMapPlanes[i] != nullptr && mCurrentFrame->mvbPlaneOutlier[i]) {
                mCurrentFrame->mvpMapPlanes[i] = static_cast<MapPlane*>(NULL);
                planeMatchNum--;
                nDisgardPlane++;
            }
            else
                nmatchesMap = nmatchesMap + mPlaneEffect;
        }
    }
    // mCurrentFrame->BuildMapPlanePointCloud(*(mPlaneDetector.mpCurrentDepthImage), mPlaneDetector.mSegmentImageOutput, ConfigFile::cloudStep);

    if (mbOnlyTracking) {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    return nmatchesMap >= 10;
}

bool Tracking::TrackLocalMap() {
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    MT_START(mt_update_local_map);
    UpdateLocalMap();
    MT_FINISH(mt_update_local_map);

    MT_START(mt_search_local_points);
    SearchLocalPoints();
    MT_FINISH(mt_search_local_points);

    // Plane matching
    int planeMatchNum = mPlaneMatcher.AssociatePlanesFromMap(*mCurrentFrame, mpMap->GetAllMapPlanes());

    // Optimize Pose
    MT_START(mt_pose_optimization);
    Optimizer::PoseOptimization(mCurrentFrame.get());
    MT_FINISH(mt_pose_optimization);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    MT_START(mt_update_mappoints);
    for (int i = 0; i < mCurrentFrame->N; i++) {
        if (mCurrentFrame->mvpMapPoints[i]) {
            if (!mCurrentFrame->mvbOutlier[i]) {
                mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
                if (!mbOnlyTracking) {
                    if (mCurrentFrame->mvpMapPoints[i]->Observations() > 0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if (mSensor == eSensor::STEREO)
                mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }
    MT_FINISH(mt_update_mappoints);

    int nDisgardPlane = 0;
    for (int i = 0; i < mCurrentFrame->mnPlaneNum; i++) {
        if (mCurrentFrame->mvpMapPlanes[i]) {
            if (mCurrentFrame->mvpMapPlanes[i] != nullptr && mCurrentFrame->mvbPlaneOutlier[i]) {
                mCurrentFrame->mvpMapPlanes[i] = static_cast<MapPlane*>(NULL);
                planeMatchNum--;
                nDisgardPlane++;
            }
            else
                mnMatchesInliers = mnMatchesInliers + mPlaneEffect;
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
        return false;

    if (mnMatchesInliers < 30)
        return false;
    else
        return true;
}

bool Tracking::NeedNewKeyFrame() {
    if (mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if (mCurrentFrame->mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check the current frame whether has a new plane
    for (size_t i = 0; i < mCurrentFrame->mvpMapPlanes.size(); i++) {
        if (mCurrentFrame->mvpMapPlanes[i] == nullptr && mCurrentFrame->mvbPlaneOutlier[i] == false) {
            mCurrentFrame->mbNewPlane = true;
            break;
        }
    }
    if (mCurrentFrame->mbNewPlane) {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle) {
            return true;
        }
        else {
            mpLocalMapper->InterruptBA();
            if (mSensor != eSensor::MONOCULAR) {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;
    if (mSensor != eSensor::MONOCULAR) {
        for (int i = 0; i < mCurrentFrame->N; i++) {
            if (mCurrentFrame->mvDepth[i] > 0 && mCurrentFrame->mvDepth[i] < mThDepth) {
                if (mCurrentFrame->mvpMapPoints[i] && !mCurrentFrame->mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
        thRefRatio = 0.4f;

    if (mSensor == eSensor::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame->mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame->mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
    // Condition 1c: tracking is weak
    const bool c1c = mSensor != eSensor::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

    if ((c1a || c1b || c1c) && c2) {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle) {
            return true;
        }
        else {
            mpLocalMapper->InterruptBA();
            if (mSensor != eSensor::MONOCULAR) {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;

    return false;
}

void Tracking::CreateNewKeyFrame() {
    if (!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(*mCurrentFrame, mpMap, mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame->mpReferenceKF = pKF;

    if (mSensor != eSensor::MONOCULAR) {
        mCurrentFrame->UpdatePoseMatrices();

        // Add Planes
        for (int i = 0; i < mCurrentFrame->mnPlaneNum; i++) {
            if (mCurrentFrame->mvbPlaneOutlier[i]) {
                // mCurrentFrame->mvbPlaneOutlier[i] = false;
                continue;
            }
            if (mCurrentFrame->mvpMapPlanes[i] != nullptr) {
                mCurrentFrame->mvpMapPlanes[i]->AddObservation(pKF, i);
                continue;
            }

            cv::Mat p3D = mCurrentFrame->ComputePlaneWorldCoeff(i);
            MapPlane* pNewMP = new MapPlane(p3D, pKF, mpMap);
            pNewMP->AddObservation(pKF, i);
            pKF->AddMapPlane(pNewMP, i);
            mpMap->AddMapPlane(pNewMP);
            pNewMP->mnFirstFrameId = mCurrentFrame->mnId;
        }
        pKF->BuildMapPlanePointCloud(*(mPlaneDetector.mpCurrentDepthImage), mPlaneDetector.mSegmentImageOutput, ConfigFile::cloudStep);

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float, int>> vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame->N);
        for (int i = 0; i < mCurrentFrame->N; i++) {
            float z = mCurrentFrame->mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty()) {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++) {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];
                if (!pMP)
                    bCreateNew = true;
                else if (pMP->Observations() < 1) {
                    bCreateNew = true;
                    mCurrentFrame->mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if (bCreateNew) {
                    cv::Mat x3D = mCurrentFrame->UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D, pKF, mpMap);
                    pNewMP->AddObservation(pKF, i);
                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame->mvpMapPoints[i] = pNewMP;
                    nPoints++;
                }
                else {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame->mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
    // Do not search map points already matched
    MT_START(mt_already_matched);
    for (vector<MapPoint*>::iterator vit = mCurrentFrame->mvpMapPoints.begin(), vend = mCurrentFrame->mvpMapPoints.end(); vit != vend; vit++) {
        MapPoint* pMP = *vit;
        if (pMP) {
            if (pMP->isBad()) {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame->mnId;
                pMP->mbTrackInView = false;
            }
        }
    }
    MT_FINISH(mt_already_matched);

    int nToMatch = 0;

    // Project points in frame and check its visibility
    MT_START(mt_project_points);
    for (vector<MapPoint*>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++) {
        MapPoint* pMP = *vit;
        if (pMP->mnLastFrameSeen == mCurrentFrame->mnId)
            continue;
        if (pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame->isInFrustum(pMP, 0.5)) {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }
    MT_FINISH(mt_project_points);

    if (nToMatch > 0) {
        ORBmatcher matcher(0.8);
        int th = 1;
        if (mSensor == eSensor::RGBD)
            th = 3;
        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame->mnId < mnLastRelocFrameId + 2)
            th = 5;

        MT_START(mt_search_for_projection_in_search_local_points);
        matcher.SearchByProjection(*mCurrentFrame, mvpLocalMapPoints, th);
        MT_FINISH(mt_search_for_projection_in_search_local_points);
    }
}

void Tracking::UpdateLocalMap() {
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
    mvpLocalMapPoints.clear();

    for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++) {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for (vector<MapPoint*>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++) {
            MapPoint* pMP = *itMP;
            if (!pMP)
                continue;
            if (pMP->mnTrackReferenceForFrame == mCurrentFrame->mnId)
                continue;
            if (!pMP->isBad()) {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame->mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames() {
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*, int> keyframeCounter;
    for (int i = 0; i < mCurrentFrame->N; i++) {
        if (mCurrentFrame->mvpMapPoints[i]) {
            MapPoint* pMP = mCurrentFrame->mvpMapPoints[i];
            if (!pMP->isBad()) {
                const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                for (map<KeyFrame*, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                    keyframeCounter[it->first]++;
            }
            else {
                mCurrentFrame->mvpMapPoints[i] = NULL;
            }
        }
    }

    if (keyframeCounter.empty())
        return;

    int max = 0;
    KeyFrame* pKFmax = static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for (map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++) {
        KeyFrame* pKF = it->first;

        if (pKF->isBad())
            continue;

        if (it->second > max) {
            max = it->second;
            pKFmax = pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame->mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++) {
        // Limit the number of keyframes
        if (mvpLocalKeyFrames.size() > 80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame*>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++) {
            KeyFrame* pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad()) {
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame->mnId) {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrame->mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for (set<KeyFrame*>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
            KeyFrame* pChildKF = *sit;
            if (!pChildKF->isBad()) {
                if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame->mnId) {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = mCurrentFrame->mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if (pParent) {
            if (pParent->mnTrackReferenceForFrame != mCurrentFrame->mnId) {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame = mCurrentFrame->mnId;
                break;
            }
        }
    }

    if (pKFmax) {
        mpReferenceKF = pKFmax;
        mCurrentFrame->mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization() {
    // Compute Bag of Words Vector
    mCurrentFrame->ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(mCurrentFrame.get());

    if (vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++) {
        KeyFrame* pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else {
            MT_START(mt_search_by_bow_in_relocalization);
            int nmatches = matcher.SearchByBoW(pKF, *mCurrentFrame, vvpMapPointMatches[i]);
            MT_FINISH(mt_search_by_bow_in_relocalization);
            if (nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            }
            else {
                PnPsolver* pSolver = new PnPsolver(*mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch) {
        for (int i = 0; i < nKFs; i++) {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (!Tcw.empty()) {
                Tcw.copyTo(mCurrentFrame->mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++) {
                    if (vbInliers[j]) {
                        mCurrentFrame->mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame->mvpMapPoints[j] = NULL;
                }

                int nGood = Optimizer::PoseOptimization(mCurrentFrame.get());

                if (nGood < 10)
                    continue;

                for (int io = 0; io < mCurrentFrame->N; io++)
                    if (mCurrentFrame->mvbOutlier[io])
                        mCurrentFrame->mvpMapPoints[io] = static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if (nGood < 50) {
                    MT_START(mt_search_for_projection_in_relocalization_1);
                    int nadditional = matcher2.SearchByProjection(*mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);
                    MT_FINISH(mt_search_for_projection_in_relocalization_1);

                    if (nadditional + nGood >= 50) {
                        nGood = Optimizer::PoseOptimization(mCurrentFrame.get());

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if (nGood > 30 && nGood < 50) {
                            sFound.clear();
                            for (int ip = 0; ip < mCurrentFrame->N; ip++)
                                if (mCurrentFrame->mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame->mvpMapPoints[ip]);
                            MT_START(mt_search_for_projection_in_relocalization_2);
                            nadditional = matcher2.SearchByProjection(*mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);
                            MT_FINISH(mt_search_for_projection_in_relocalization_2);

                            // Final optimization
                            if (nGood + nadditional >= 50) {
                                nGood = Optimizer::PoseOptimization(mCurrentFrame.get());

                                for (int io = 0; io < mCurrentFrame->N; io++)
                                    if (mCurrentFrame->mvbOutlier[io])
                                        mCurrentFrame->mvpMapPoints[io] = NULL;
                            }
                        }
                    }
                }

                // If the pose is supported by enough inliers stop ransacs and continue
                if (nGood >= 50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch) {
        return false;
    }
    else {
        mnLastRelocFrameId = mCurrentFrame->mnId;
        return true;
    }
}

void Tracking::Reset() {
    cout << "System Reseting" << endl;
    if (mpViewer) {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
            usleep(3000);
    }
    if (mpSocketViewer) {
        mpSocketViewer->RequestStop();
        while (!mpSocketViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if (mpInitializer) {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if (mpViewer)
        mpViewer->Release();
    if (mpSocketViewer)
        mpSocketViewer->Release();
}

void Tracking::ChangeCalibration(const string& strSettingPath) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool& flag) {
    mbOnlyTracking = flag;
}

} // namespace ORB_SLAM2
