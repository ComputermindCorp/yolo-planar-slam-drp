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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ImageLoading.h"
#include "ImageProcessing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "Enum.h"

#include "SocketViewer.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#define MAX_DEVICE_PATH 128

namespace ORB_SLAM2 {

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;
class ImageLoading;
class ImageProcessing;

class SocketViewer;

class System {
public:
public:
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string& strVocFile, const string& strSettingsFile,
           const InputType input_type, const eSensor sensor,
           const std::string image_dir,
           const std::vector<std::string> image_filenames,
           const std::vector<std::string> depth_filenames,
           const std::vector<double> timestamp,
           const bool bUsePangolinViewer = false, const bool bUseSocketViewer = true,
           const unsigned int drp_dev_num = 0, const unsigned int drp_ai_dev_num = 0);
    System(const string& strVocFile, const string& strSettingsFile,
           const InputType input_type, const eSensor sensor,
           const bool bUsePangolinViewer = false, const bool bUseSocketViewer = true,
           const unsigned int drp_dev_num = 0, const unsigned int drp_ai_dev_num = 0);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD();

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular();

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string& filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string& filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string& filename);

    void SaveTrajectory(const string& filename);

    void SavePlanePointCloud(const string& filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    void SetTrackTime(const double track_time);

    int GetDrpFileDescriptor();

#if defined(ENABLE_DRP)
    bool DRP_Initialize();
    bool DRP_Finalize();
#endif

#if defined(ENABLE_DRP_AI)
    bool DRP_AI_Initialize();
    bool DRP_AI_Finalize();
#endif

    std::vector<pair<double, cv::Mat>> trajectory;

private:
    void OutputDRPAISLAMVersion();
    void OutputBuildSwitch();
    void OutputExecuteSettings(const string& strSettingsFile);

    const InputType input_type_;

    // Input sensor
    eSensor mSensor;

    const std::string image_dir_;
    const std::vector<std::string> image_filenames_;
    const std::vector<std::string> depth_filenames_;
    const std::vector<double> timestamp_;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

public:
    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

private:
    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    ImageLoading* mpImageLoader;

    ImageProcessing* mpImageProcessor;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;
    SocketViewer* mpSocketViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptImageLoading;
    std::thread* mptImageProcessing;
    std::thread* mptViewer;
    std::thread* mptSocketViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    const unsigned int drp_dev_num_;
    const unsigned int drp_ai_dev_num_;
    int drp_fd;
    bool use_drp_;
    bool use_drp_ai_;
    bool use_opencva_;
};

} // namespace ORB_SLAM2

#endif // SYSTEM_H
