#include "PlaneMatcher.h"

namespace ORB_SLAM2 {

PlaneMatcher::PlaneMatcher() {
    mAngleTh = ConfigFile::planeAssociationAngleThreshold;
    mDisTh = ConfigFile::planeAssociationDistanceThreshold;
    mDoubleFPS = ConfigFile::FPS * 2;
}

int PlaneMatcher::AssociatePlanesFromMap(Frame& currentFrame, const std::vector<MapPlane*>& vpMapPlanes, bool out) {
    currentFrame.mbNewPlane = false;

    int nmatches = 0;

    for (int i = 0; i < currentFrame.mnPlaneNum; ++i) {
        if (currentFrame.mvbPlaneOutlier[i]) {
            currentFrame.mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
            continue;
        }

        cv::Mat pM = currentFrame.ComputePlaneWorldCoeff(i);

        float ldTh = mDisTh;

        bool found = false;
        for (auto vpMapPlane : vpMapPlanes) {
            if (vpMapPlane->isBad()) {
                continue;
            }

            if ((currentFrame.mnId - vpMapPlane->mnFirstFrameId) > mDoubleFPS && vpMapPlane->mMatchCount < (mDoubleFPS / 5)) {
                vpMapPlane->SetBadFlag();
                continue;
            }

            cv::Mat pW = vpMapPlane->GetWorldPos();

            float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) + pM.at<float>(1, 0) * pW.at<float>(1, 0) + pM.at<float>(2, 0) * pW.at<float>(2, 0);

            // associate plane
            if ((angle > mAngleTh || angle < -mAngleTh)) {
                double dis = PointDistanceFromPlane(pM, vpMapPlane->GetPlanePointCloud());
                if (dis < ldTh) {
                    ldTh = dis;
                    currentFrame.mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
                    currentFrame.mvpMapPlanes[i] = vpMapPlane;
                    vpMapPlane->mMatchCount = vpMapPlane->mMatchCount + 1;
                    found = true;
                    continue;
                }
            }
        }

        if (found) {
            nmatches++;
        }
    }
    // cout << "Plane num: " << currentFrame.mnPlaneNum << ", Plane match: " << nmatches << endl;
    return nmatches;
}

double PlaneMatcher::PointDistanceFromPlane(const cv::Mat& currentPlaneCoefficient, pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPlanePointCloud, bool out) {
    if (out) {
        cout << " compute dis: " << endl;
    }
    double result = 9999999999;
    double totalDis = 0;
    int count = 0;
    pcl::PointXYZRGB p;
    for (size_t i = 0; i < mapPlanePointCloud->points.size(); i += 2) {
        p = mapPlanePointCloud->points[i];
        double dis = abs(currentPlaneCoefficient.at<float>(0, 0) * p.x + currentPlaneCoefficient.at<float>(1, 0) * p.y + currentPlaneCoefficient.at<float>(2, 0) * p.z + currentPlaneCoefficient.at<float>(3, 0));
        // if(dis < result){
        //     result = dis;
        // }
        totalDis = totalDis + dis;
        count = count + 1;
    }
    result = totalDis / count;
    if (out) {
        cout << endl
             << "ave : " << result << endl;
    }
    return result;
}

} // namespace ORB_SLAM2