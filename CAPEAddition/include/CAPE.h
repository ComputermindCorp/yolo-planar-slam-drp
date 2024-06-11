/*
 * Copyright 2018 Pedro Proenza <p.proenca@surrey.ac.uk> (University of Surrey)
 *
 */
#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include "PlaneSeg.h"
#include "CylinderSeg.h"
#include "Histogram.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace CAPE {

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;

class PlaneDetector {
private:
    int cell_width;
    int cell_height;
    int depth_height;
    int depth_width;
    float max_merge_dist;
    float min_cos_angle_4_merge;
    bool cylinder_detection;
    std::vector<PlaneSeg*> Grid;
    cv::Mat_<int> grid_plane_seg_map;
    cv::Mat_<uchar> grid_plane_seg_map_eroded;
    cv::Mat_<int> grid_cylinder_seg_map;
    cv::Mat_<uchar> grid_cylinder_seg_map_eroded;
    cv::Mat mask;
    cv::Mat mask_eroded;
    cv::Mat mask_square_eroded;
    cv::Mat mask_dilated;
    cv::Mat mask_diff;
    cv::Mat mask_square_kernel;
    cv::Mat mask_cross_kernel;
    float* distances_stacked;
    Eigen::ArrayXf distances_cell_stacked;
    unsigned char* seg_map_stacked;
    bool* activation_map;
    bool* unassigned_mask;

    // RGB camera parameters
    float fx_rgb, fy_rgb, cx_rgb, cy_rgb;
    // Depth camera parameters
    float fx_ir, fy_ir, cx_ir, cy_ir;

    // Parameters for backprojection point cloud
    cv::Mat_<float> X_pre;
    cv::Mat_<float> Y_pre;
    cv::Mat_<int> cell_map;

    cv::Mat_<float> U;
    cv::Mat_<float> V;
    cv::Mat_<float> X;
    cv::Mat_<float> Y;

    Eigen::MatrixXf cloud_array;
    Eigen::MatrixXf cloud_array_organized;

    int min_plane_size;

public:
    cv::Mat_<uchar> mSegmentImageOutput;
    const cv::Mat* mpCurrentDepthImage;

public:
    // this is original default PlaneDetector() parameter, I dont know why, just do not use it
    // float COS_ANGLE_MAX = 0.97814;
    // float MAX_MERGE_DIST = 900;
    PlaneDetector();
    PlaneDetector(int depth_height, int depth_width, bool cylinder_detection, int min_plane_size = 3000, int cell_width = 20, int cell_height = 20, float min_cos_angle_4_merge = cos(M_PI / 12), float max_merge_dist = 50.0f);

    void Process(cv::Mat& d_img, vector<PlaneSeg>& plane_segments_final, vector<CylinderSeg>& cylinder_segments_final);
    void RegionGrowing(unsigned short width, unsigned short height, bool* input, bool* output, vector<PlaneSeg*>& Grid, vector<float>& cell_dist_tols, unsigned short x, unsigned short y, double* normal, double d);
    void getConnectedComponents(cv::Mat& segment_map, MatrixXb& planes_association_matrix);
    ~PlaneDetector(void);

    // Usually SLAM RGBD dataset provide us images after preprocessing
    // So the rgb image have already aligned to the depth image
    // If your dataset do not have preprocessing
    // You can modify this function to read rgb parameter and depth parameter seperately
    void loadCalibParameters(float fx, float fy, float cx, float cy);

    void projectPointCloud(cv::Mat& d_img);
    void organizePointCloudByCell();
};

} // namespace CAPE
