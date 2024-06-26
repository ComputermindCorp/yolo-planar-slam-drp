/*
 * Copyright 2018 Pedro Proenza <p.proenca@surrey.ac.uk> (University of Surrey)
 *
 */

#pragma once
#include <iostream>
#include "Params.h"
#include <Eigen/Dense>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

namespace CAPE {

class PlaneSeg {
public:
    int nr_pts, min_nr_pts;
    double x_acc, y_acc, z_acc,
        xx_acc, yy_acc, zz_acc,
        xy_acc, xz_acc, yz_acc;
    float score;
    float MSE;
    bool planar;

    // Plane params
    double mean[3];
    double normal[3];
    double d;

    // bool isSmall;

    cv::Mat mPlaneCameraCoefficient;

public:
    PlaneSeg(Eigen::MatrixXf& cloud_array, int cell_id, int nr_pts_per_cell, int cell_width);
    void fitPlane();
    void expandSegment(PlaneSeg* plane_seg);
    void clearPoints();
    ~PlaneSeg(void);

    // Convert normal & d to cv::Mat
    void SetPlaneCameraCoefficient();
};

} // namespace CAPE
