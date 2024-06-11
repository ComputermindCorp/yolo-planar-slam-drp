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

#include "System.h"
#include "Converter.h"
#include "Enum.h"
#include "DatasetImageLoading.h"
#include "MonocularCameraImageLoading.h"

#if defined(ENABLE_REALSENSE2)
#include "D435iCameraImageLoading.h"
#endif

#include <thread>
// #include <pangolin/pangolin.h>
#include <iomanip>
#include <fcntl.h>

#if defined(ENABLE_DRP)
#include "drp.h"
#include "opencva.h"
#endif

#define MEASURE_TIME_DECLARE_BODY
#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

namespace ORB_SLAM2 {

System::System(const string& strVocFile, const string& strSettingsFile,
               const InputType input_type, const eSensor sensor,
               const std::string image_dir,
               const std::vector<std::string> image_filenames,
               const std::vector<std::string> depth_filenames,
               const std::vector<double> timestamp,
               const bool bUsePangolinViewer, const bool bUseSocketViewer,
               const unsigned int drp_dev_num, const unsigned int drp_ai_dev_num)
    : input_type_(input_type), mSensor(sensor),
      image_dir_(image_dir), image_filenames_(image_filenames), depth_filenames_(depth_filenames), timestamp_(timestamp),
      mpViewer(static_cast<Viewer*>(NULL)), mpSocketViewer(static_cast<SocketViewer*>(NULL)), mbReset(false), mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false),
      drp_dev_num_(drp_dev_num), drp_ai_dev_num_(drp_ai_dev_num),
      drp_fd(drp::FILE_DESCRIPTOR_CLOSE) {
    // Output welcome message
    cout << endl
         << "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl
         << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
         << "This is free software, and you are welcome to redistribute it" << endl
         << "under certain conditions. See LICENSE.txt." << endl
         << endl;

    cout << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
        cout << "Monocular" << endl;
    else if (mSensor == STEREO)
        cout << "Stereo" << endl;
    else if (mSensor == RGBD)
        cout << "RGB-D" << endl;

    OutputDRPAISLAMVersion();
    OutputBuildSwitch();
    OutputExecuteSettings(strSettingsFile);

    // Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    // Load ORB Vocabulary
    cout << endl
         << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl
         << endl;

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // Create the Map
    mpMap = new Map();

    // Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

#if defined(ENABLE_DRP)
    use_drp_ = (((std::string)fsSettings["UseDrp"]) == "true");
    use_opencva_ = (((std::string)fsSettings["UseOpenCVA"]) == "true");

    assert((!use_opencva_ || use_drp_) && "If use_opencva_ is true, use_drp_ must be true.");
#if defined(ENABLE_SLAMFAST)
    assert(!use_opencva_ && "SLAMFAST is not available in OpenCVA.");
#endif

    if (use_drp_) {
        if (!DRP_Initialize()) {
            fprintf(stderr, "Failed to System::DRP_Initialize.\n");
            exit(EXIT_FAILURE);
        }
        const bool use_cvfast = (use_drp_ && !use_opencva_);
        drp::LoadBinConfigurations(GetDrpFileDescriptor(), use_cvfast);
    }

#if !defined(ENABLE_SLAMFAST)
    if (use_opencva_) {
        if (!opencva::initialize()) {
            fprintf(stderr, "Failed to opencva::initialize.\n");
            exit(EXIT_FAILURE);
        }
    }
    else
#endif
    {
        if (!opencva::disable()) {
            fprintf(stderr, "Failed to opencva::disable.\n");
            exit(EXIT_FAILURE);
        }
    }
#endif

#if defined(ENABLE_DRP_AI)
    use_drp_ai_ = (((std::string)fsSettings["UseDrpAI"]) == "true");

    assert(use_drp_ai_ == true && "Cannot use ncnn. use_drp_ai_ must be true.");

    if (use_drp_ai_) {
        if (!DRP_AI_Initialize()) {
            fprintf(stderr, "Failed to System::DRP_AI_Initialize.\n");
            exit(EXIT_FAILURE);
        }
    }
#endif

    // Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(
#if defined(ENABLE_DRP)
        drp_fd,
#endif
        this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
        mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

    // Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    if (input_type == InputType::DATASET) {
        mpImageLoader = new DatasetImageLoading(mSensor, image_dir_, image_filenames_, depth_filenames_, timestamp_);
        mptImageLoading = new thread(&ORB_SLAM2::ImageLoading::Run, mpImageLoader);
    }
    else if (input_type == InputType::MONO_CAMERA) {
        mpImageLoader = new MonocularCameraImageLoading(mSensor, strSettingsFile);
        mptImageLoading = new thread(&ORB_SLAM2::ImageLoading::Run, mpImageLoader);
    }
#if defined(ENABLE_REALSENSE2)
    else if (input_type == InputType::D435i_CAMERA) {
        fprintf(stderr, "WARNING : The operation has not been verified when D435i is used.\n");
        mpImageLoader = new D435iCameraImageLoading(mSensor, strSettingsFile);
        mptImageLoading = new thread(&ORB_SLAM2::ImageLoading::Run, mpImageLoader);
    }
#endif
    else {
        fprintf(stderr, "Invalid InputType : %d in System::System\n", input_type);
    }

    int nRGB = fsSettings["Camera.RGB"];
    mpImageProcessor = new ImageProcessing(
        drp_fd, (bool)nRGB, mSensor,
        mpTracker->mpORBextractorLeft,
        mpVocabulary,
        mpTracker->mYoloDetector,
        mpTracker->mK,
        mpTracker->mDistCoef,
        mpTracker->mbf,
        mpTracker->mThDepth,
        (mSensor == MONOCULAR)
            ? (2 * mpTracker->nFeatures)
            : mpTracker->nFeatures,
        use_drp_, use_drp_ai_,
        use_opencva_);
    mptImageProcessing = new thread(&ORB_SLAM2::ImageProcessing::Run, mpImageProcessor);

    // Initialize the Viewer thread and launch
    if (bUsePangolinViewer) {
        fprintf(stderr, "Cannot use Pangolin Viewer.\n");
        exit(EXIT_FAILURE);
        // mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
        // mptViewer = new thread(&Viewer::Run, mpViewer);
        // mpTracker->SetViewer(mpViewer);
    }
    if (bUseSocketViewer) {
        mpSocketViewer = new SocketViewer(this, mpFrameDrawer, mpMapDrawer, strSettingsFile);
        mptSocketViewer = new thread(&SocketViewer::Run, mpSocketViewer);
        mpTracker->SetSocketViewer(mpSocketViewer);
    }

    // Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    mpImageLoader->SeImageProcessor(mpImageProcessor);

    mpImageProcessor->SetTracker(mpTracker);
}

System::System(const string& strVocFile, const string& strSettingsFile,
               const InputType input_type, const eSensor sensor,
               const bool bUsePangolinViewer, const bool bUseSocketViewer,
               const unsigned int drp_dev_num, const unsigned int drp_ai_dev_num)
    : System(strVocFile, strSettingsFile,
             input_type, sensor,
             "", std::vector<std::string>(), std::vector<std::string>(), std::vector<double>(), // Dummy
             bUsePangolinViewer, bUseSocketViewer,
             drp_dev_num, drp_ai_dev_num) {
    assert(input_type == InputType::MONO_CAMERA || input_type == InputType::D435i_CAMERA);
}

cv::Mat System::TrackRGBD() {
    if (mSensor != RGBD) {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped()) {
                usleep(1000); // Sleep 1ms
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset) {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    MT_START(mt_tracker_wait_for_next_frame);
    while (mpTracker->wait_for_next_frame.load()) {
        usleep(1000); // Sleep 1ms
    }
    MT_FINISH(mt_tracker_wait_for_next_frame);

    MT_START(mt_grab);
    cv::Mat Tcw = mpTracker->GrabImageRGBD();
    MT_FINISH(mt_grab);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;

    {
        MT_START(mt_wait_for_next_max_features_to_true);
        while (!mpImageProcessor->wait_for_next_max_features.load()) {
            usleep(1000); // Sleep 1ms
        }
        MT_FINISH(mt_wait_for_next_max_features_to_true);

        mpImageProcessor->SetMaxFeatures(mpTracker->nFeatures);
    }

    mpTracker->wait_for_next_frame.store(true);

    return Tcw;
}

cv::Mat System::TrackMonocular() {
    if (mSensor != MONOCULAR) {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode) {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped()) {
                usleep(1000); // Sleep 1ms
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset) {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    MT_START(mt_tracker_wait_for_next_frame);
    while (mpTracker->wait_for_next_frame.load()) {
        usleep(1000); // Sleep 1ms
    }
    MT_FINISH(mt_tracker_wait_for_next_frame);

    MT_START(mt_grab);
    cv::Mat Tcw = mpTracker->GrabImageMonocular();
    MT_FINISH(mt_grab);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;

    trajectory.push_back({mpTracker->timestamp, Tcw});

    {
        MT_START(mt_wait_for_next_max_features_to_true);
        while (!mpImageProcessor->wait_for_next_max_features.load()) {
            usleep(1000); // Sleep 1ms
        }
        MT_FINISH(mt_wait_for_next_max_features_to_true);

        if (mTrackingState == eTrackingState::NOT_INITIALIZED
            || mTrackingState == eTrackingState::NO_IMAGES_YET)
            mpImageProcessor->SetMaxFeatures(2 * mpTracker->nFeatures);
        else
            mpImageProcessor->SetMaxFeatures(mpTracker->nFeatures);
    }

    mpTracker->wait_for_next_frame.store(true);

    return Tcw;
}

void System::ActivateLocalizationMode() {
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged() {
    static int n = 0;
    int curn = mpMap->GetLastBigChangeIdx();
    if (n < curn) {
        n = curn;
        return true;
    }
    else
        return false;
}

void System::Reset() {
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown() {
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpImageLoader->RequestFinish();
    mpImageProcessor->RequestFinish();
    if (mpViewer) {
        mpViewer->RequestFinish();
        while (!mpViewer->isFinished())
            usleep(5000);
    }

    if (mpSocketViewer) {
        mpSocketViewer->RequestFinish();
        while (!mpSocketViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while (!mpLocalMapper->isFinished()
           || !mpLoopCloser->isFinished()
           || mpLoopCloser->isRunningGBA()
           || !mpImageLoader->isFinished()
           || !mpImageProcessor->isFinished()) {
        usleep(5000);
    }

    // if (mpViewer)
    //     pangolin::BindToContext("YOLO-Planar-SLAM: Map Viewer");

#if defined(ENABLE_DRP)
    if (use_drp_) {
        if (!DRP_Finalize()) {
            fprintf(stderr, "Failed to System::DRP_Finalize.\n");
            exit(EXIT_FAILURE);
        }
    }

#if !defined(ENABLE_SLAMFAST)
    if (use_opencva_) {
        if (!opencva::finalize()) {
            fprintf(stderr, "Failed to opencva::finalize.\n");
            exit(EXIT_FAILURE);
        }
    }
#endif
#endif

#if defined(ENABLE_DRP_AI)
    if (use_drp_ai_) {
        if (!DRP_AI_Finalize()) {
            fprintf(stderr, "Failed to System::DRP_AI_Finalize.\n");
            exit(EXIT_FAILURE);
        }
    }
#endif
}

void System::SaveTrajectoryTUM(const string& filename) {
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR) {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    if (vpKFs.empty()) {
        fprintf(stderr, "No output key frames\n");
        return;
    }
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                 lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++, lbL++) {
        if (*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while (pKF->isBad()) {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        cv::Mat Tcw = (*lit) * Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl
         << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string& filename) {
    cout << endl
         << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    if (vpKFs.empty()) {
        fprintf(stderr, "No output key frames\n");
        return;
    }
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    // cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if (pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
    cout << endl
         << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string& filename) {
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR) {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    if (vpKFs.empty()) {
        fprintf(stderr, "No output key frames\n");
        return;
    }
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++) {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        while (pKF->isBad()) {
            //  cout << "bad parent" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        cv::Mat Tcw = (*lit) * Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

        f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " << Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " << Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl
         << "trajectory saved!" << endl;
}

void System::SaveTrajectory(const string& filename) {
    std::cout << "trajectory.size() : " << trajectory.size() << std::endl;
    ofstream f;
    f.open("frameTrajectory.txt");
    f << fixed;

    for (auto p : trajectory) {
        double stamp = p.first;
        cv::Mat Tcw = p.second;
        if (Tcw.rows == 4 && Tcw.cols == 4) {
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            f << setprecision(15) << stamp << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    }
    f.close();
    cout << endl
         << "trajectory saved!" << endl;
}

void System::SavePlanePointCloud(const string& filename) {
    cout << "Saving plane pointcloud to file to " << filename << " ..." << endl;
    const vector<MapPlane*>& vpMPs = mpMap->GetAllMapPlanes();
    if (vpMPs.empty()) {
        cout << "Do not have any plane in map, save fail" << endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto pMP : vpMPs) {
        for (auto& p : pMP->GetPlanePointCloud()->points) {
            planePointCloud->points.push_back(p);
        }
    }
    planePointCloud->height = 1;
    planePointCloud->width = planePointCloud->points.size();
    pcl::PLYWriter plyWriter;
    plyWriter.write(filename, *planePointCloud, true);

    cout << "plane point cloud saved!" << endl;
    return;
}

int System::GetTrackingState() {
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints() {
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

void System::SetTrackTime(const double track_time) {
    if (mpViewer) {
        mpViewer->SetTrackTime(track_time);
    }
    if (mpSocketViewer) {
        mpSocketViewer->SetTrackTime(track_time);
    }
}

int System::GetDrpFileDescriptor() {
    assert(drp_fd != drp::FILE_DESCRIPTOR_CLOSE);

    return drp_fd;
}

#if defined(ENABLE_DRP)
bool System::DRP_Initialize() {
    assert(drp_dev_num_ == 1 && "drp_dev_num_ must be 1.");

    char device_path[MAX_DEVICE_PATH];
    sprintf(device_path, "/dev/drp%u", drp_dev_num_);

    if (drp_fd != drp::FILE_DESCRIPTOR_CLOSE) {
        fprintf(stderr, "%s is already open.\n", device_path);
        return true;
    }

    errno = 0;
    drp_fd = open(device_path, O_RDWR);
    bool open_succeed = (0 < drp_fd);
    if (!open_succeed) {
        fprintf(stderr, "Failed to open(%s). Return code is %d, errno is %d\n", device_path, drp_fd, errno);
        return false;
    }

    return true;
}

bool System::DRP_Finalize() {
    assert(drp_dev_num_ == 1 && "drp_dev_num_ must be 1.");

    char device_path[MAX_DEVICE_PATH];
    sprintf(device_path, "/dev/drp%u", drp_dev_num_);

    if (drp_fd == drp::FILE_DESCRIPTOR_CLOSE) {
        fprintf(stderr, "%s is already closed.\n", device_path);
        return true;
    }

    errno = 0;
    int drpai_ret = close(drp_fd);
    bool close_succeed = (drpai_ret == 0);

    if (!close_succeed) {
        fprintf(stderr, "Failed to close(%s). Return code is %d, errno is %d\n", device_path, drpai_ret, errno);
        return false;
    }

    drp_fd = drp::FILE_DESCRIPTOR_CLOSE;

    return true;
}
#endif

#if defined(ENABLE_DRP_AI)
bool System::DRP_AI_Initialize() {
    assert(drp_ai_dev_num_ == 0 && "drp_ai_dev_num_ must be 0.");

    // Replaced by RecognizeBase::recognize_start

    return true;
}

bool System::DRP_AI_Finalize() {
    assert(drp_ai_dev_num_ == 0 && "drp_ai_dev_num_ must be 0.");

    // Replaced by RecognizeBase::recognize_end

    return true;
}
#endif

void System::OutputDRPAISLAMVersion() {
    const std::string version = "2.9.0";
    std::cerr << std::endl;
    std::cerr << "The version of DRP-AI-SLAM : " << version << std::endl;
    std::cerr << std::endl;
}

void System::OutputBuildSwitch() {
    std::cerr << std::endl;
#if defined(ENABLE_MEASURE_TIME)
    std::cerr << "defined ENABLE_MEASURE_TIME" << std::endl;
#endif
#if defined(ENABLE_DUMP)
    std::cerr << "defined ENABLE_DUMP" << std::endl;
#endif
#if defined(ENABLE_DEBUG_OUTPUT)
    std::cerr << "defined ENABLE_DEBUG_OUTPUT" << std::endl;
#endif
#if defined(ENABLE_LOAD_YOLO)
    std::cerr << "defined ENABLE_LOAD_YOLO" << std::endl;
#endif
#if defined(ENABLE_DRP)
    std::cerr << "defined ENABLE_DRP" << std::endl;
#endif
#if defined(ENABLE_DRP_AI)
    std::cerr << "defined ENABLE_DRP_AI" << std::endl;
#endif
#if defined(ENABLE_REALSENSE2)
    std::cerr << "defined ENABLE_REALSENSE2" << std::endl;
#endif
#if defined(ENABLE_GOOGLE_PERF)
    std::cerr << "defined ENABLE_GOOGLE_PERF" << std::endl;
#endif
#if defined(ENABLE_YOCTO)
    std::cerr << "defined ENABLE_YOCTO" << std::endl;
#endif
#if defined(ENABLE_SLAMFAST)
    std::cerr << "defined ENABLE_SLAMFAST" << std::endl;
#endif
    std::cerr << std::endl;
}

void System::OutputExecuteSettings(const string& strSettingsFile) {
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    const float fps = fsSettings["Camera.fps"];
    const int width = fsSettings["Camera.width"];
    const int height = fsSettings["Camera.height"];
    const int RGB = fsSettings["Camera.RGB"];
    const int nFeatures = fsSettings["ORBextractor.nFeatures"];
    const bool use_drp = (((std::string)fsSettings["UseDrp"]) == "true");
    const bool use_drp_ai = (((std::string)fsSettings["UseDrpAI"]) == "true");
    const bool use_opencva = (((std::string)fsSettings["UseOpenCVA"]) == "true");
    const int sleep_ms = (fsSettings["Sleep"].empty()) ? 0 : fsSettings["Sleep"];
    const std::string viewer_type = (std::string)fsSettings["Viewer.Type"];

    std::cerr << std::endl;

    std::cerr << "Camera.fps : " << fps << std::endl;
    std::cerr << "Camera.width : " << width << std::endl;
    std::cerr << "Camera.height : " << height << std::endl;
    std::cerr << "Camera.RGB : " << RGB << std::endl;
    std::cerr << "ORBextractor.nFeatures : " << nFeatures << std::endl;
    std::cerr << "UseDrp : " << use_drp << std::endl;
    std::cerr << "UseDrpAI : " << use_drp_ai << std::endl;
    std::cerr << "UseOpenCVA : " << use_opencva << std::endl;
    std::cerr << "Sleep : " << sleep_ms << std::endl;
    std::cerr << "Viewer.Type : " << viewer_type << std::endl;

    if (viewer_type == "SocketViewer") {
        if (!fsSettings["SocketPublisher.emitting_interval"].empty()) {
            unsigned int emitting_interval = (unsigned int)(float)fsSettings["SocketPublisher.emitting_interval"];
            std::cerr << "SocketPublisher.emitting_interval : " << emitting_interval << std::endl;
        }
        if (!fsSettings["SocketPublisher.image_quality"].empty()) {
            unsigned int image_quality = (unsigned int)(float)fsSettings["SocketPublisher.image_quality"];
            std::cerr << "SocketPublisher.image_quality : " << image_quality << std::endl;
        }
        if (!fsSettings["SocketPublisher.server_uri"].empty()) {
            std::string server_uri = fsSettings["SocketPublisher.server_uri"];
            std::cerr << "SocketPublisher.server_uri : " << server_uri << std::endl;
        }
    }

    std::cerr << std::endl;
}

} // namespace ORB_SLAM2
