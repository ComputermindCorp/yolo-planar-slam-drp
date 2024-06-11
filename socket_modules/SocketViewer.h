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

#ifndef SOCKET_VIEWER_H
#define SOCKET_VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "System.h"

#include "SocketClient.h"
#include "DataSerializer.h"

#include <mutex>

namespace socket_publisher {
class DataSerializer;
}

namespace ORB_SLAM2 {

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class SocketViewer {
public:
    SocketViewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, const string& strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    void SetTrackTime(const double track_time);

private:
    bool Stop();

    System* mpSystem;
    std::shared_ptr<FrameDrawer> mpFrameDrawer;
    std::shared_ptr<MapDrawer> mpMapDrawer;

    float mImageWidth, mImageHeight;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    void callback(const std::string& message);

    unsigned int emitting_interval_;
    unsigned int image_quality_;

    std::unique_ptr<socket_publisher::SocketClient> client_;
    std::unique_ptr<socket_publisher::DataSerializer> data_serializer_;

    double track_time_;
    std::mutex lock_track_time_;
};

} // namespace ORB_SLAM2

#endif // SOCKET_VIEWER_H
