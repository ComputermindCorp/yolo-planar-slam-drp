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

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Converter.h"
#include "ConfigFile.h"
#include "Enum.h"

#include "measure_time.h"

#if defined(ENABLE_DRP)
#include "drp.h"
#endif

using namespace std;

void LoadImages(const string& strImagePath, const string& strPathTimes,
                vector<string>& vstrImages, vector<double>& vTimeStamps);

int main(int argc, char** argv) {
    if (argc != 5) {
        cerr << endl
             << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file" << endl;
        return 1;
    }

    fprintf(stderr, "WARNING : The case of using EuRoC in Monocular mode is not supported.\n");

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    if (nImages <= 0) {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

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
    ORB_SLAM2::ConfigFile configFile(argv[2]);
    ORB_SLAM2::System SLAM(argv[1], argv[2],
                           InputType::DATASET, eSensor::MONOCULAR,
                           "",
                           vstrImageFilenames,
                           std::vector<std::string>(), // Depth is empty
                           vTimestamps,
                           bUsePangolinViewer, bUseSocketViewer, drp_dev_num, drp_ai_dev_num);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    MT_START(mt_main_loop);
    for (int ni = 0; ni < nImages; ni++) {
        double tframe = vTimestamps[ni];

        // #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // #else
        //         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        // #endif

        // Pass the image to the SLAM system
        MT_START(mt_main_track);
        SLAM.TrackMonocular();
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

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        MT_START(mt_main_usleep);
        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
        MT_FINISH(mt_main_usleep);

#if defined(ENABLE_MEASURE_TIME)
        std::string state = (SLAM.mpTracker->mState == eTrackingState::SYSTEM_NOT_READY)  ? "SYSTEM_NOT_READY"
                            : (SLAM.mpTracker->mState == eTrackingState::NO_IMAGES_YET)   ? "NO_IMAGES_YET"
                            : (SLAM.mpTracker->mState == eTrackingState::NOT_INITIALIZED) ? "NOT_INITIALIZED"
                            : (SLAM.mpTracker->mState == eTrackingState::OK)              ? "OK"
                            : (SLAM.mpTracker->mState == eTrackingState::LOST)            ? "LOST"
                                                                                          : "Unknown";

        printf("Measurement result of %4d frame\n", ni);
        printf("    System::TrackMonocular                 : %8.3lf[ms]\n", mt_main_track.last * 1e3);
        printf("    Sleep specified at execution           : %8.3lf[ms]\n", mt_option_usleep.last * 1e3);
        printf("    Sleep while waiting for the next frame : %8.3lf[ms]\n", mt_main_usleep.last * 1e3);
        printf("    Tracking state                         : %s\n", state.c_str());
#endif
    }

    MT_FINISH(mt_main_loop);

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
    SLAM.SaveTrajectory("frameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    MT_PRINT();

    return 0;
}

void LoadImages(const string& strImagePath, const string& strPathTimes,
                vector<string>& vstrImages, vector<double>& vTimeStamps) {
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t / 1e9);
        }
    }
}
