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

#include "SocketViewer.h"

#include <mutex>

namespace ORB_SLAM2 {

SocketViewer::SocketViewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, const string& strSettingPath)
    : mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer),
      mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false),
      emitting_interval_(15000),
      image_quality_(20),
      client_(new socket_publisher::SocketClient("http://127.0.0.1:3000")),
      track_time_(0.0) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if (mImageWidth < 1 || mImageHeight < 1) {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    if (!fSettings["SocketPublisher.emitting_interval"].empty())
        emitting_interval_ = (unsigned int)(float)fSettings["SocketPublisher.emitting_interval"];
    if (!fSettings["SocketPublisher.image_quality"].empty())
        image_quality_ = (unsigned int)(float)fSettings["SocketPublisher.image_quality"];
    if (!fSettings["SocketPublisher.server_uri"].empty()) {
        std::string server_uri = fSettings["SocketPublisher.server_uri"];
        client_ = std::unique_ptr<socket_publisher::SocketClient>(new socket_publisher::SocketClient(server_uri));
    }

    data_serializer_ = std::unique_ptr<socket_publisher::DataSerializer>(
        new socket_publisher::DataSerializer(mpFrameDrawer, mpMapDrawer, mImageWidth, mImageHeight));

    client_->set_signal_callback(std::bind(&SocketViewer::callback, this, std::placeholders::_1));
}

void SocketViewer::Run() {
    mbFinished = false;
    mbStopped = false;

    const auto serialized_reset_signal = socket_publisher::DataSerializer::serialized_reset_signal_;
    client_->emit("map_publish", serialized_reset_signal);

    while (1) {
        const auto t0 = std::chrono::system_clock::now();

        const auto serialized_map_data = data_serializer_->serialize_map_diff();
        if (!serialized_map_data.empty()) {
            client_->emit("map_publish", serialized_map_data);
        }

        const auto serialized_frame_data = data_serializer_->serialize_latest_frame(image_quality_, track_time_);
        if (!serialized_frame_data.empty()) {
            client_->emit("frame_publish", serialized_frame_data);
        }

        // sleep until emitting interval time is past
        const auto t1 = std::chrono::system_clock::now();
        const auto elapse_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        if (elapse_us < emitting_interval_) {
            const auto sleep_us = emitting_interval_ - elapse_us;
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }

        if (Stop()) {
            while (isStopped()) {
                usleep(1000); // Sleep 1ms
            }
        }

        if (CheckFinish())
            break;
    }

    SetFinish();
}

void SocketViewer::callback(const std::string& message) {
    if (message == "disable_mapping_mode") {
        std::cerr << "disable_mapping_mode is not implemented." << std::endl;
    }
    else if (message == "enable_mapping_mode") {
        std::cerr << "enable_mapping_mode is not implemented." << std::endl;
    }
    else if (message == "reset") {
        mpSystem->Reset();
    }
    else if (message == "terminate") {
        RequestFinish();
    }
}

void SocketViewer::RequestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool SocketViewer::CheckFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void SocketViewer::SetFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool SocketViewer::isFinished() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void SocketViewer::RequestStop() {
    unique_lock<mutex> lock(mMutexStop);
    if (!mbStopped)
        mbStopRequested = true;
}

bool SocketViewer::isStopped() {
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool SocketViewer::Stop() {
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if (mbFinishRequested)
        return false;
    else if (mbStopRequested) {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;
}

void SocketViewer::Release() {
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void SocketViewer::SetTrackTime(const double track_time) {
    unique_lock<mutex> lock(lock_track_time_);
    track_time_ = track_time;
}

} // namespace ORB_SLAM2
