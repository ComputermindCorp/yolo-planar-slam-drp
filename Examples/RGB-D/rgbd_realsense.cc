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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <errno.h>
#include <signal.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Enum.h"

#include "measure_time.h"

#if defined(ENABLE_DRP)
#include "drp.h"
#endif

#if defined(ENABLE_GOOGLE_PERF)
#include <gperftools/profiler.h>
#endif

using namespace std;

bool interrupt = false;
void signal_handler(const int in) {
    if (in == SIGINT)
        interrupt = true;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        cerr << endl
             << "Usage: ./rgbd_realsense path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    fprintf(stderr, "WARNING : The case of using D435i in RGB-D mode is not supported.\n");

    // Check settings file
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        cerr << "Failed to open settings file at: " << argv[2] << endl;
        exit(-1);
    }

    const int sleep_ms = (fsSettings["Sleep"].empty()) ? 0 : fsSettings["Sleep"];
    if (0 < sleep_ms)
        std::cout << "Sleep " << sleep_ms << "ms every frame." << std::endl;

    MT_INIT();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const bool bUsePangolinViewer = (((std::string)fsSettings["Viewer.Type"]) == "PangolinViewer");
    const bool bUseSocketViewer = (((std::string)fsSettings["Viewer.Type"]) == "SocketViewer");
    const unsigned int drp_dev_num = 1;
    const unsigned int drp_ai_dev_num = 0;
    ORB_SLAM2::System SLAM(argv[1], argv[2],
                           InputType::D435i_CAMERA, eSensor::RGBD,
                           bUsePangolinViewer, bUseSocketViewer, drp_dev_num, drp_ai_dev_num);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    printf("Press Ctrl+C to exit.\n");
    if (signal(SIGINT, signal_handler) == SIG_ERR) {
        fprintf(stderr, "Failed to setup signal_handler.\n");
        return 1;
    }

#if defined(ENABLE_GOOGLE_PERF)
    ProfilerStart("slam.prof");
#endif

    // Main loop
    MT_START(mt_main_loop);
    int nImages = 0;
    while (!interrupt) {
        // #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // #else
        //         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        // #endif

        // Pass the image to the SLAM system
        MT_START(mt_main_track);
        SLAM.TrackRGBD();
        MT_FINISH(mt_main_track);

        MT_START(mt_option_usleep);
        if (0 < sleep_ms)
            usleep(sleep_ms * 1e3);
        MT_FINISH(mt_option_usleep);

        // #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // #else
        //         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        // #endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        SLAM.SetTrackTime(ttrack);

        vTimesTrack.push_back(ttrack);

#if defined(ENABLE_MEASURE_TIME)
        std::string state = (SLAM.mpTracker->mState == eTrackingState::SYSTEM_NOT_READY)  ? "SYSTEM_NOT_READY"
                            : (SLAM.mpTracker->mState == eTrackingState::NO_IMAGES_YET)   ? "NO_IMAGES_YET"
                            : (SLAM.mpTracker->mState == eTrackingState::NOT_INITIALIZED) ? "NOT_INITIALIZED"
                            : (SLAM.mpTracker->mState == eTrackingState::OK)              ? "OK"
                            : (SLAM.mpTracker->mState == eTrackingState::LOST)            ? "LOST"
                                                                                          : "Unknown";

        printf("Measurement result of %4d frame\n", nImages);
        printf("    System::TrackRGBD            : %8.3lf[ms]\n", mt_main_track.last * 1e3);
        printf("    Sleep specified at execution : %8.3lf[ms]\n", mt_option_usleep.last * 1e3);
        printf("    Tracking state               : %s\n", state.c_str());
#endif

        nImages++;
    }

    MT_FINISH(mt_main_loop);

#if defined(ENABLE_GOOGLE_PERF)
    ProfilerStop();
#endif

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SavePlanePointCloud("PlanePointCloud.ply");

    MT_PRINT();

    return 0;
}
