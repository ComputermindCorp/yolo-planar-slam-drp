#pragma once
#include <iostream>

#include "KeyFrame.h"
#include "Frame.h"
#include "ConfigFile.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <mutex>

namespace ORB_SLAM2 {

class KeyFrame;
class Frame;

class PlaneMatcher {
public:
    PlaneMatcher();

    int AssociatePlanesFromMap(Frame& pF, const std::vector<MapPlane*>& vpMapPlanes, bool out = false);
    double PointDistanceFromPlane(const cv::Mat& currentPlaneCoefficient, pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPlanePointCloud, bool out = false);

public:
    float mDisTh;
    float mAngleTh;
    float mDoubleFPS;
};

} // namespace ORB_SLAM2