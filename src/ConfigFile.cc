//
// Created by fishmarch on 19-5-24.
//

#include "ConfigFile.h"

namespace ORB_SLAM2 {

int ConfigFile::planeMinSize = 3000;
int ConfigFile::planeCellWidth = 20;
int ConfigFile::planeCellHeight = 20;

int ConfigFile::cloudStep = 5;
float ConfigFile::planeVoxelLeafSize = 0.1;

float ConfigFile::planeAssociationAngleThreshold = 0.9;
float ConfigFile::planeAssociationDistanceThreshold = 0.1;

float ConfigFile::planeAngleInfo = 1;
float ConfigFile::planeDistanceInfo = 100;
float ConfigFile::planeChi = 300;

float ConfigFile::FPS = 30;

ConfigFile::ConfigFile(const string& filename) {
    try {
        mFile.open(filename, cv::FileStorage::READ);
    }
    catch (cv::Exception ex) {
        fprintf(stderr, "Failed to read %s\n", filename.c_str());
        exit(EXIT_FAILURE);
    }

    if (mFile.isOpened()) {
        planeMinSize = mFile["Plane.MinSize"];
        planeCellWidth = mFile["Plane.CellWidth"];
        planeCellHeight = mFile["Plane.CellHeight"];

        cloudStep = mFile["Cloud.Step"];
        planeVoxelLeafSize = mFile["Plane.LeafSize"];

        planeAssociationAngleThreshold = mFile["Plane.AssociationAngleThreshold"];
        planeAssociationDistanceThreshold = mFile["Plane.AssociationDistanceThreshold"];

        planeAngleInfo = mFile["Plane.AngleInfo"];
        planeDistanceInfo = mFile["Plane.DistanceInfo"];
        planeChi = mFile["Plane.Chi"];

        FPS = mFile["Camera.fps"];
    }
    else {
        std::cerr << "parameter file does not exist." << std::endl;
        mFile.release();
    }
    return;
}

ConfigFile::~ConfigFile() {
    if (mFile.isOpened()) {
        mFile.release();
    }
}

} // namespace ORB_SLAM2