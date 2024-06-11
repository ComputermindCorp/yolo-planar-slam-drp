#include "MapPlane.h"

namespace ORB_SLAM2 {
long unsigned int MapPlane::nLastId = 0;
mutex MapPlane::mGlobalMutex;

MapPlane::MapPlane(const cv::Mat& Pos, KeyFrame* pRefKF, Map* pMap)
    : mnFirstKFid(pRefKF->mnId), mnFirstFrameId(0), mMatchCount(0),
      mpRefKF(pRefKF),
      mbBad(false), mpReplaced(static_cast<MapPlane*>(NULL)),
      mpMap(pMap),
      mpPlanePointCloud(new pcl::PointCloud<pcl::PointXYZRGB>()) {
    mnId = nLastId++;
    Pos.copyTo(mWorldPlaneCoefficient);
    srand(mnId * 100);
    mRed = rand() % 255;
    mBlue = rand() % 255;
    mGreen = rand() % 255;
    return;
}

void MapPlane::SetWorldPos(const cv::Mat& Pos) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPlaneCoefficient);
}

cv::Mat MapPlane::GetWorldPos() {
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPlaneCoefficient.clone();
}

KeyFrame* MapPlane::GetReferenceKeyFrame() {
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

map<KeyFrame*, size_t> MapPlane::GetObservations() {
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPlane::Observations() {
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPlane::AddObservation(KeyFrame* pKF, size_t idx) {
    unique_lock<mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF))
        return;
    mObservations[pKF] = idx;
    nObs++;
    return;
}

void MapPlane::EraseObservation(KeyFrame* pKF) {
    bool bBad = false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF)) {
            mObservations.erase(pKF);
            nObs--;

            if (mpRefKF == pKF)
                mpRefKF = mObservations.begin()->first;

            if (nObs <= 2)
                bBad = true;
        }
    }
    if (bBad) {
        SetBadFlag();
    }
    return;
}

int MapPlane::GetIndexInKeyFrame(KeyFrame* pKF) {
    unique_lock<mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPlane::IsInKeyFrame(KeyFrame* pKF) {
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPlane::SetBadFlag() {
    map<KeyFrame*, size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad = true;
        obs = mObservations;
        mObservations.clear();
    }
    for (auto& ob : obs) {
        KeyFrame* pKF = ob.first;
        pKF->EraseMapPlaneMatch(ob.second);
    }

    mpMap->EraseMapPlane(this);
    return;
}

bool MapPlane::isBad() {
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPlane::Replace(MapPlane* pMP) {
    // future work
    return;
}

MapPlane* MapPlane::GetReplaced() {
    // future work
    return nullptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MapPlane::GetPlanePointCloud() {
    unique_lock<mutex> lock(mMutexPointCloud);
    return mpPlanePointCloud;
}

void MapPlane::UpdatePointCloud(pcl::PointCloud<pcl::PointXYZRGB> inputPointCloud) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);

    *mpPlanePointCloud += inputPointCloud;

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    float leafSize = ConfigFile::planeVoxelLeafSize;
    voxel.setLeafSize(leafSize, leafSize, leafSize);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coarseCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel.setInputCloud(mpPlanePointCloud);
    voxel.filter(*coarseCloud);

    mpPlanePointCloud = coarseCloud;
    return;
}

bool MapPlane::TransformPointCloud(const Eigen::Vector4d coefBefore, const Eigen::Vector4d coefAfter) {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);

    Eigen::Vector3d normalBefore = coefBefore.head<3>();
    Eigen::Vector3d normalAfter = coefAfter.head<3>();
    normalBefore = normalBefore.normalized();
    normalAfter = normalAfter.normalized();

    Eigen::Vector3d axisVector = normalBefore.cross(normalAfter);
    double a = normalBefore.dot(normalAfter);
    double b = normalBefore.norm() * normalAfter.norm();
    double angle = acos(a / b);
    Eigen::AngleAxisd rotationVector(angle, axisVector.normalized());

    double d1 = coefBefore(3);
    double d2 = coefAfter(3);

    if (angle > 0 && angle < 0.1 && abs(d1 - d2) > 0 && abs(d1 - d2) < 0.1) {
        double xaxisTransBefore = -d1 * (normalBefore(0) / sqrt(normalBefore.dot(normalBefore)));
        double yaxisTransBefore = -d1 * (normalBefore(1) / sqrt(normalBefore.dot(normalBefore)));
        double zaxisTransBefore = -d1 * (normalBefore(2) / sqrt(normalBefore.dot(normalBefore)));

        double xaxisTransAfter = d2 * (normalAfter(0) / sqrt(normalAfter.dot(normalAfter)));
        double yaxisTransAfter = d2 * (normalAfter(1) / sqrt(normalAfter.dot(normalAfter)));
        double zaxisTransAfter = d2 * (normalAfter(2) / sqrt(normalAfter.dot(normalAfter)));

        Eigen::Affine3d t1 = Eigen::Affine3d::Identity();
        Eigen::Affine3d t2 = Eigen::Affine3d::Identity();
        Eigen::Affine3d t3 = Eigen::Affine3d::Identity();
        Eigen::Affine3d transformMatrix = Eigen::Affine3d::Identity();
        t1.translation() << xaxisTransBefore, yaxisTransBefore, zaxisTransBefore;
        t2.rotate(rotationVector);
        t3.translation() << xaxisTransAfter, yaxisTransAfter, zaxisTransAfter;
        transformMatrix = t3 * t2 * t1;

        pcl::transformPointCloud(*mpPlanePointCloud, *mpPlanePointCloud, transformMatrix);
        return true;
    }
    else {
        return false;
    }
}

} // namespace ORB_SLAM2