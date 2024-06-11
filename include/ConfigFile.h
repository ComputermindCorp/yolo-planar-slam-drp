//
// Created by fishmarch on 19-5-24.
//

// Use this file to read parameters about plane from the config file

#ifndef CONFIGFILE_H
#define CONFIGFILE_H
#include <opencv2/core/core.hpp>
#include <memory>
#include <iostream>

using namespace std;
namespace ORB_SLAM2 {

class ConfigFile {
public:
    ConfigFile(const string& filename);
    ~ConfigFile();

public:
    static int planeMinSize;
    static int planeCellHeight;
    static int planeCellWidth;

    static int cloudStep;
    static float planeVoxelLeafSize;

    static float planeAssociationAngleThreshold;
    static float planeAssociationDistanceThreshold;

    static float planeAngleInfo;
    static float planeDistanceInfo;
    static float planeChi;

    static float FPS;

private:
    cv::FileStorage mFile;
};

} // namespace ORB_SLAM2
#endif