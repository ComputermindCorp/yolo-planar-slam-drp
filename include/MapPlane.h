#pragma once
#include <iostream>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/exceptions.h>

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"
#include "ConfigFile.h"

namespace ORB_SLAM2 {

class KeyFrame;
class Frame;
class Map;

class MapPlane {
public:
    MapPlane(const cv::Mat& Pos, KeyFrame* pRefKF, Map* pMap);

    void SetWorldPos(const cv::Mat& Pos);
    cv::Mat GetWorldPos();

    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*, size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF, size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    // future work
    void Replace(MapPlane* pMP);
    MapPlane* GetReplaced();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPlanePointCloud();
    // Extend old point cloud from new point cloud and use pcl::VoexlGrid to down sampling the point cloud
    // Grow point cloud and filter the point cloud
    void UpdatePointCloud(pcl::PointCloud<pcl::PointXYZRGB> inputPointCloud);

    // Transform the pointcloud after local BA update the pose of plane
    bool TransformPointCloud(const Eigen::Vector4d coefBefore, const Eigen::Vector4d coefAfter);

public:
    long unsigned int mnId; ///< Global ID for MapPlane;
    static long unsigned int nLastId;
    int nObs;

    static std::mutex mGlobalMutex;

    long int mnFirstKFid;
    long unsigned int mnBALocalForKF; // used in local BA

    int mnFirstFrameId;
    int mMatchCount;

    int mRed;
    int mGreen;
    int mBlue;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    std::mutex mMutexPos;
    std::mutex mMutexFeatures;

    std::mutex mMutexPointCloud;

    cv::Mat mWorldPlaneCoefficient;

    std::map<KeyFrame*, size_t> mObservations;

    KeyFrame* mpRefKF;

    bool mbBad;
    MapPlane* mpReplaced;

    Map* mpMap;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mpPlanePointCloud;
};

} // namespace ORB_SLAM2