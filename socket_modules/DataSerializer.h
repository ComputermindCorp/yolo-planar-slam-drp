#ifndef SOCKET_VIEWER_DATASERIALIZER_H
#define SOCKET_VIEWER_DATASERIALIZER_H

#include <memory>

#include <Eigen/Core>
#include <sioclient/sio_client.h>
#include <opencv2/core.hpp>

#include "KeyFrame.h"
#include "MapPoint.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "System.h"

using Vec3_t = Eigen::Vector3d;
using Mat44_t = Eigen::Matrix4d;

namespace ORB_SLAM2 {

class KeyFrame;
class MapPoint;
class FrameDrawer;
class MapDrawer;
class System;

} // namespace ORB_SLAM2

namespace socket_publisher {

class DataSerializer {
public:
    DataSerializer(
        const std::shared_ptr<ORB_SLAM2::FrameDrawer>& pFrameDrawer,
        const std::shared_ptr<ORB_SLAM2::MapDrawer>& pMapDrawer,
        const unsigned int image_width,
        const unsigned int image_height);

    std::string serialize_messages(const std::vector<std::string>& tags, const std::vector<std::string>& messages);

    std::string serialize_map_diff();

    std::string serialize_latest_frame(const unsigned int image_quality, const double track_time);

    static std::string serialized_reset_signal_;

private:
    const std::shared_ptr<ORB_SLAM2::FrameDrawer> pFrameDrawer_;
    const std::shared_ptr<ORB_SLAM2::MapDrawer> pMapDrawer_;
    const unsigned int image_width_;
    const unsigned int image_height_;
    std::unique_ptr<std::unordered_map<unsigned int, double>> keyframe_hash_map_;
    std::unique_ptr<std::unordered_map<unsigned int, double>> point_hash_map_;

    double current_pose_hash_ = 0;
    int frame_hash_ = 0;

    inline double get_vec_hash(const cv::Mat& point) {
        return point.at<double>(0) + point.at<double>(1) + point.at<double>(2);
    }

    inline double get_mat_hash(const cv::Mat& pose) {
        if (pose.empty())
            return 0.0;
        return pose.at<double>(0, 3) + pose.at<double>(1, 3) + pose.at<double>(2, 3);
    }

    std::string serialize_as_protobuf(
        const std::vector<ORB_SLAM2::KeyFrame*>& keyfrms,
        const std::vector<ORB_SLAM2::MapPoint*>& all_landmarks,
        const std::vector<ORB_SLAM2::MapPoint*>& local_landmarks,
        const cv::Mat& current_camera_pose);

    std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);
};

} // namespace socket_publisher

#endif // SOCKET_VIEWER_DATASERIALIZER_H
