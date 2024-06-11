#pragma once

#include <list>
#include <sys/time.h>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <chrono>

#if defined(MEASURE_TIME_DECLARE_BODY)
#define EXTERN
#else /* MEASURE_TIME_DECLARE_BODY */
#define EXTERN extern
#endif /* MEASURE_TIME_DECLARE_BODY */

#define OUTPUT_CSVNAME "frame_times.csv"

EXTERN struct measure_time_t {
    std::chrono::steady_clock::time_point start, end;
    double sum = 0.0;
    uint32_t cnt;
    std::vector<double> elapsed_time;
    double last;
}
// ImageLoading::Run
mt_image_loading,
    mt_imread,
    mt_capture_read,
    mt_cvt_color,
    mt_capture,
    // in Capture image
    mt_wait_for_frames,
    mt_align,
    mt_get_color_frame,
    mt_get_depth_frame,
    mt_rs2_to_mat,
    // ImageLoading::Run
    mt_image_loading_push_result,
    // ImageProcessing::Run
    mt_image_processing,
    mt_image_processing_first,
    mt_image_processing_second[2],
    // in Image processing
    mt_input_to_gray,
    mt_convert_depth,
    mt_copy_to_tracking,
    mt_create_frame,
    mt_input_to_bgr,
    mt_yolo_object_detect,
    // in YoloObjectDetect
    mt_drp_ai_yolo_convert_color_format,
    mt_drp_ai_yolo_padding,
    mt_drp_ai_yolo_u2p_input,
    mt_drp_ai_yolo_polling,
    mt_drp_ai_yolo_get_object_detection,
    // in Image processing
    mt_extract_orb_keypoints,
    // in Frame::ExtractORBKeyPoints
    // in ORBextractor::ExtractORBKeyPoints
    mt_create_pyramid,
    // in ComputePyramid
    mt_copy_make_border[8],
    mt_resize[8],
    // in Resize
    mt_drp_resize_calc_param,
    mt_drp_resize_u2p_param,
    mt_drp_resize_u2p_input,
    mt_drp_resize_activate,
    mt_drp_resize_start,
    mt_drp_resize_get_status,
    mt_drp_resize_p2u[8],
    mt_drp_resize_clone_output_mat[8],
    // in Frame::ExtractORBKeyPoints
    // in ORBextractor::ExtractORBKeyPoints
    mt_compute_key_points_oct_tree,
    // in ORBextractor::ComputeKeyPointsOctTree
    mt_remove_moving_key_points,
    mt_slamfast[8],
    // in ComputeKeyPointsOctTree[0-7]
    mt_push_cell_indice[8],
    mt_clone_cell_image[8],
    mt_receive_yolo_output,
    mt_pop_cell_image[8],
    mt_cell_postprocess[2][8],
    mt_drp_slamfast_calc_param[8],
    mt_drp_slamfast_u2p_param[8],
    mt_drp_slamfast_u2p_input[8],
    mt_drp_slamfast_activate,
    mt_drp_slamfast_start[8],
    mt_drp_slamfast_get_status[8],
    mt_drp_slamfast_p2u_keypoints_size[8],
    mt_drp_slamfast_p2u_keypoints[8],
    mt_drp_slamfast_cast_to_cv[8],
    mt_cvfast[8],
    // in FAST[0-7]
    mt_drp_cvfast_calc_param[8],
    mt_drp_cvfast_u2p_param[8],
    mt_drp_cvfast_u2p_input[8],
    mt_drp_cvfast_activate,
    mt_drp_cvfast_start[8],
    mt_drp_cvfast_get_status[8],
    mt_drp_cvfast_p2u_keypoints_size[8],
    mt_drp_cvfast_p2u_keypoints[8],
    mt_drp_cvfast_cast_to_cv[8],
    // in ORBextractor::ComputeKeyPointsOctTree
    mt_wait_for_next_max_features_to_false,
    mt_distribute_keypoints_via_tree[2][8],
    mt_compute_orientation[2][8],
    // in Image processing
    mt_extract_orb_desc[2],
    mt_gaussianblur[8],
    // in GaussianBlur[0-7]
    mt_drp_gaussian_blur_calc_param[8],
    mt_drp_gaussian_blur_u2p_param[8],
    mt_drp_gaussian_blur_u2p_input[8],
    mt_drp_gaussian_blur_activate,
    mt_drp_gaussian_blur_start[8],
    mt_drp_gaussian_blur_get_status[8],
    mt_drp_gaussian_blur_p2u[8],
    // in Frame::ExtractOrbDescriptors
    mt_compute_orb[2][8],
    // in computeOrbDescriptors[0-7]
    mt_drp_orb_descriptors_calc_param[2][8],
    mt_drp_orb_descriptors_cast_to_drp[2][8],
    mt_drp_orb_descriptors_u2p_param[2][8],
    mt_drp_orb_descriptors_u2p_input_image[2][8],
    mt_drp_orb_descriptors_u2p_input_keypoints[2][8],
    mt_drp_orb_descriptors_activate,
    mt_drp_orb_descriptors_start[2][8],
    mt_drp_orb_descriptors_get_status[2][8],
    mt_drp_orb_descriptors_p2u[2][8],
    mt_drp_orb_descriptors_postprocess[2][8],
    // in Image processing
    mt_last_processing_of_constructor[2],
    // ImageProcessing::Run
    mt_image_processing_push_result,
    // in drp::*
    mt_drp_time[2][6][8],
    // tracking
    mt_main_loop,
    // in Main loop
    mt_option_usleep,
    mt_main_usleep,
    mt_main_track,
    // in System::Track*
    mt_tracker_wait_for_next_frame,
    mt_wait_for_next_max_features_to_true,
    mt_grab,
    // in Tracking::GrabImage*
    mt_plane_detector_process,
    // in PlaneDetector::Process
    mt_organize_pointcloud_by_cell,
    mt_planar_cell_fitting,
    mt_project_pointcloud,
    // in PlaneDetector::projectPointCloud
    mt_mul_x,
    mt_mul_y,
    mt_divide_x,
    mt_divide_y,
    mt_calc_u,
    mt_calc_v,
    mt_set_cloud_array,
    // in Tracking::GrabImage*
    mt_my_calculate_after,
    mt_track,
    // in Tracking::Track
    mt_lock_map_update,
    mt_monocular_initialization,
    // in Tracking::MonocularInitialization
    mt_search_for_initialization,
    // in Tracking::Track
    mt_track_reference_keyframe,
    // in Tracking::TrackReferenceKeyFrame
    mt_search_by_bow_in_track_reference_keyframe,
    // in Tracking::Track
    mt_track_current_frame,
    // in Tracking::TrackWithMotionModel
    mt_pose_optimization_in_track_with_motion_model,
    mt_search_for_projection_in_track_with_motion_model_1,
    mt_search_for_projection_in_track_with_motion_model_2,
    // in Tracking::Track
    mt_track_local_map,
    // in Tracking::TrackLocalMap
    mt_update_local_map,
    mt_pose_optimization,
    mt_update_mappoints,
    mt_search_local_points,
    // in Tracking::SearchLocalPoints
    mt_already_matched,
    mt_project_points,
    mt_search_for_projection_in_search_local_points,
    // in Tracking::Relocalization
    mt_search_by_bow_in_relocalization,
    mt_search_for_projection_in_relocalization_1,
    mt_search_for_projection_in_relocalization_2,
    // RecognizeBase::inference_thread
    mt_recognize_thread,
    mt_recognize_polling,
    // LocalMapping::Run
    mt_mapping,
    mt_process_new_keyframe,
    mt_create_new_map_points,
    mt_search_in_neighbors,
    mt_local_bundle_adjustment,
    mt_keyframe_culling,
    // LoopClosing::Run
    mt_loop_closing;

EXTERN struct measure_num_t {
    std::vector<int32_t> num;
    uint32_t cnt;
} mt_num_keypts_to_distribute[8],
    mt_num_keypts_at_level[8];

#if defined(MEASURE_TIME_DECLARE_BODY)
struct measure_time_with_name_t {
    struct measure_time_t* mt;
    const char* name;
    const char* func;
} measure_time_list[] = {
    // ImageLoading::Run
    {&mt_image_loading, "Image loading", "ImageLoading::Run"},
    {&mt_imread, "cv::imread", "Image loading"},
    {&mt_capture_read, "cv::VideoCapture::read", "Image loading"},
    {&mt_cvt_color, "cv::cvtColor", "Image loading"},
    {&mt_capture, "Capture image", "Image loading"},
    // in Capture image
    {&mt_wait_for_frames, "rs2::pipeline::wait_for_frames", "Capture image"},
    {&mt_align, "rs2::align::process", "Capture image"},
    {&mt_get_color_frame, "rs2::frameset::get_color_frame", "Capture image"},
    {&mt_get_depth_frame, "rs2::frameset::get_depth_frame", "Capture image"},
    {&mt_rs2_to_mat, "create cv::Mat", "Capture image"},
    // ImageLoading::Run
    {&mt_image_loading_push_result, "ImageLoading::PushResult", "ImageLoading::Run"},
    // ImageProcessing::Run
    {&mt_image_processing, "Image processing", "ImageProcessing::Run"},
    {&mt_image_processing_first, "Image processing(first)", "ImageProcessing::Run"},
    {&mt_image_processing_second[0], "Image processing(second)[0]", "ImageProcessing::Run"},
    {&mt_image_processing_second[1], "Image processing(second)[1]", "ImageProcessing::Run"},
    // in Image processing
    {&mt_input_to_gray, "cv::cvtColor to gray image", "Image processing(first)"},
    {&mt_convert_depth, "cv::Mat::convertTo", "Image processing(first)"},
    {&mt_copy_to_tracking, "Copy to Tracking member variables", "Image processing(first)"},
    {&mt_create_frame, "Frame::Frame", "Image processing(first)"},
    {&mt_input_to_bgr, "cv::cvtColor to yolo input image", "Image processing(first)"},
    {&mt_yolo_object_detect, "YoloObjectDetect", "Image processing(first)"},
    // in YoloObjectDetect
    {&mt_drp_ai_yolo_convert_color_format, "convert to RGB or YUYV", "YoloObjectDetect"},
    {&mt_drp_ai_yolo_padding, "bottom padding", "YoloObjectDetect"},
    {&mt_drp_ai_yolo_u2p_input, "copy input to physical memory", "YoloObjectDetect"},
    {&mt_drp_ai_yolo_polling, "polling", "YoloObjectDetect"},
    {&mt_drp_ai_yolo_get_object_detection, "Yolo*Model::get_object_detection", "YoloObjectDetect"},
    // in Image processing(first)
    {&mt_extract_orb_keypoints, "Frame::ExtractORBKeyPoints", "Image processing"},
    // in Frame::ExtractORBKeyPoints
    // in ORBextractor::ExtractORBKeyPoints
    {&mt_create_pyramid, "ComputePyramid", "Frame::ExtractORBKeyPoints"},
    // in ComputePyramid
    {&mt_copy_make_border[0], "cv::copyMakeBorder[0]", "ComputePyramid"},
    {&mt_copy_make_border[1], "cv::copyMakeBorder[1]", "ComputePyramid"},
    {&mt_copy_make_border[2], "cv::copyMakeBorder[2]", "ComputePyramid"},
    {&mt_copy_make_border[3], "cv::copyMakeBorder[3]", "ComputePyramid"},
    {&mt_copy_make_border[4], "cv::copyMakeBorder[4]", "ComputePyramid"},
    {&mt_copy_make_border[5], "cv::copyMakeBorder[5]", "ComputePyramid"},
    {&mt_copy_make_border[6], "cv::copyMakeBorder[6]", "ComputePyramid"},
    {&mt_copy_make_border[7], "cv::copyMakeBorder[7]", "ComputePyramid"},
    {&mt_resize[0], "Resize[0]", "ComputePyramid"},
    {&mt_resize[1], "Resize[1]", "ComputePyramid"},
    {&mt_resize[2], "Resize[2]", "ComputePyramid"},
    {&mt_resize[3], "Resize[3]", "ComputePyramid"},
    {&mt_resize[4], "Resize[4]", "ComputePyramid"},
    {&mt_resize[5], "Resize[5]", "ComputePyramid"},
    {&mt_resize[6], "Resize[6]", "ComputePyramid"},
    {&mt_resize[7], "Resize[7]", "ComputePyramid"},
    // in Resize
    {&mt_drp_resize_calc_param, "calculate parameter", "Resize"},
    {&mt_drp_resize_u2p_param, "copy parameter using MemcpyU2P", "Resize"},
    {&mt_drp_resize_u2p_input, "copy input using MemcpyU2P", "Resize"},
    {&mt_drp_resize_activate, "Activate", "Resize"},
    {&mt_drp_resize_start, "Start", "Resize"},
    {&mt_drp_resize_get_status, "GetStatus", "Resize"},
    {&mt_drp_resize_p2u[0], "MemcpyP2U[0]", "Resize"},
    {&mt_drp_resize_p2u[1], "MemcpyP2U[1]", "Resize"},
    {&mt_drp_resize_p2u[2], "MemcpyP2U[2]", "Resize"},
    {&mt_drp_resize_p2u[3], "MemcpyP2U[3]", "Resize"},
    {&mt_drp_resize_p2u[4], "MemcpyP2U[4]", "Resize"},
    {&mt_drp_resize_p2u[5], "MemcpyP2U[5]", "Resize"},
    {&mt_drp_resize_p2u[6], "MemcpyP2U[6]", "Resize"},
    {&mt_drp_resize_p2u[7], "MemcpyP2U[7]", "Resize"},
    {&mt_drp_resize_clone_output_mat[0], "cv::Mat::clone[0]", "Resize"},
    {&mt_drp_resize_clone_output_mat[1], "cv::Mat::clone[1]", "Resize"},
    {&mt_drp_resize_clone_output_mat[2], "cv::Mat::clone[2]", "Resize"},
    {&mt_drp_resize_clone_output_mat[3], "cv::Mat::clone[3]", "Resize"},
    {&mt_drp_resize_clone_output_mat[4], "cv::Mat::clone[4]", "Resize"},
    {&mt_drp_resize_clone_output_mat[5], "cv::Mat::clone[5]", "Resize"},
    {&mt_drp_resize_clone_output_mat[6], "cv::Mat::clone[6]", "Resize"},
    {&mt_drp_resize_clone_output_mat[7], "cv::Mat::clone[7]", "Resize"},
    // in Frame::ExtractORBKeyPoints
    // in ORBextractor::ExtractORBKeyPoints
    {&mt_compute_key_points_oct_tree, "ORBextractor::ComputeKeyPointsOctTree", "Frame::ExtractORBKeyPoints"},
    // in ORBextractor::ComputeKeyPointsOctTree
    {&mt_remove_moving_key_points, "Frame::RemoveMovingKeyPoints", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_slamfast[0], "ComputeKeyPointsOctTree[0]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_slamfast[1], "ComputeKeyPointsOctTree[1]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_slamfast[2], "ComputeKeyPointsOctTree[2]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_slamfast[3], "ComputeKeyPointsOctTree[3]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_slamfast[4], "ComputeKeyPointsOctTree[4]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_slamfast[5], "ComputeKeyPointsOctTree[5]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_slamfast[6], "ComputeKeyPointsOctTree[6]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_slamfast[7], "ComputeKeyPointsOctTree[7]", "ORBextractor::ComputeKeyPointsOctTree"},
    // in ComputeKeyPointsOctTree[0-7]
    {&mt_push_cell_indice[0], "push cell_indice[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_push_cell_indice[1], "push cell_indice[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_push_cell_indice[2], "push cell_indice[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_push_cell_indice[3], "push cell_indice[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_push_cell_indice[4], "push cell_indice[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_push_cell_indice[5], "push cell_indice[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_push_cell_indice[6], "push cell_indice[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_push_cell_indice[7], "push cell_indice[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_clone_cell_image[0], "clone cell_image[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_clone_cell_image[1], "clone cell_image[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_clone_cell_image[2], "clone cell_image[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_clone_cell_image[3], "clone cell_image[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_clone_cell_image[4], "clone cell_image[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_clone_cell_image[5], "clone cell_image[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_clone_cell_image[6], "clone cell_image[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_clone_cell_image[7], "clone cell_image[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_receive_yolo_output, "receive output of DRP-AI", "ComputeKeyPointsOctTree"},
    {&mt_pop_cell_image[0], "pop cell_image[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_pop_cell_image[1], "pop cell_image[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_pop_cell_image[2], "pop cell_image[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_pop_cell_image[3], "pop cell_image[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_pop_cell_image[4], "pop cell_image[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_pop_cell_image[5], "pop cell_image[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_pop_cell_image[6], "pop cell_image[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_pop_cell_image[7], "pop cell_image[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_cell_postprocess[0][0], "postprocess CV FAST[0][0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_cell_postprocess[0][1], "postprocess CV FAST[0][1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_cell_postprocess[0][2], "postprocess CV FAST[0][2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_cell_postprocess[0][3], "postprocess CV FAST[0][3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_cell_postprocess[0][4], "postprocess CV FAST[0][4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_cell_postprocess[0][5], "postprocess CV FAST[0][5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_cell_postprocess[0][6], "postprocess CV FAST[0][6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_cell_postprocess[0][7], "postprocess CV FAST[0][7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_cell_postprocess[1][0], "postprocess CV FAST[1][0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_cell_postprocess[1][1], "postprocess CV FAST[1][1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_cell_postprocess[1][2], "postprocess CV FAST[1][2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_cell_postprocess[1][3], "postprocess CV FAST[1][3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_cell_postprocess[1][4], "postprocess CV FAST[1][4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_cell_postprocess[1][5], "postprocess CV FAST[1][5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_cell_postprocess[1][6], "postprocess CV FAST[1][6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_cell_postprocess[1][7], "postprocess CV FAST[1][7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_drp_slamfast_calc_param[0], "calculate parameter[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_drp_slamfast_calc_param[1], "calculate parameter[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_drp_slamfast_calc_param[2], "calculate parameter[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_drp_slamfast_calc_param[3], "calculate parameter[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_drp_slamfast_calc_param[4], "calculate parameter[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_drp_slamfast_calc_param[5], "calculate parameter[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_drp_slamfast_calc_param[6], "calculate parameter[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_drp_slamfast_calc_param[7], "calculate parameter[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_drp_slamfast_u2p_param[0], "copy parameter using MemcpyU2P[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_drp_slamfast_u2p_param[1], "copy parameter using MemcpyU2P[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_drp_slamfast_u2p_param[2], "copy parameter using MemcpyU2P[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_drp_slamfast_u2p_param[3], "copy parameter using MemcpyU2P[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_drp_slamfast_u2p_param[4], "copy parameter using MemcpyU2P[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_drp_slamfast_u2p_param[5], "copy parameter using MemcpyU2P[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_drp_slamfast_u2p_param[6], "copy parameter using MemcpyU2P[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_drp_slamfast_u2p_param[7], "copy parameter using MemcpyU2P[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_drp_slamfast_u2p_input[0], "copy input using MemcpyU2P[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_drp_slamfast_u2p_input[1], "copy input using MemcpyU2P[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_drp_slamfast_u2p_input[2], "copy input using MemcpyU2P[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_drp_slamfast_u2p_input[3], "copy input using MemcpyU2P[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_drp_slamfast_u2p_input[4], "copy input using MemcpyU2P[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_drp_slamfast_u2p_input[5], "copy input using MemcpyU2P[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_drp_slamfast_u2p_input[6], "copy input using MemcpyU2P[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_drp_slamfast_u2p_input[7], "copy input using MemcpyU2P[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_drp_slamfast_activate, "Activate", "ComputeKeyPointsOctTree[0-7]"},
    {&mt_drp_slamfast_start[0], "Start[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_drp_slamfast_start[1], "Start[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_drp_slamfast_start[2], "Start[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_drp_slamfast_start[3], "Start[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_drp_slamfast_start[4], "Start[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_drp_slamfast_start[5], "Start[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_drp_slamfast_start[6], "Start[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_drp_slamfast_start[7], "Start[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_drp_slamfast_get_status[0], "GetStatus[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_drp_slamfast_get_status[1], "GetStatus[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_drp_slamfast_get_status[2], "GetStatus[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_drp_slamfast_get_status[3], "GetStatus[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_drp_slamfast_get_status[4], "GetStatus[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_drp_slamfast_get_status[5], "GetStatus[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_drp_slamfast_get_status[6], "GetStatus[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_drp_slamfast_get_status[7], "GetStatus[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_drp_slamfast_p2u_keypoints_size[0], "copy keypoints size using MemcpyP2U[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_drp_slamfast_p2u_keypoints_size[1], "copy keypoints size using MemcpyP2U[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_drp_slamfast_p2u_keypoints_size[2], "copy keypoints size using MemcpyP2U[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_drp_slamfast_p2u_keypoints_size[3], "copy keypoints size using MemcpyP2U[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_drp_slamfast_p2u_keypoints_size[4], "copy keypoints size using MemcpyP2U[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_drp_slamfast_p2u_keypoints_size[5], "copy keypoints size using MemcpyP2U[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_drp_slamfast_p2u_keypoints_size[6], "copy keypoints size using MemcpyP2U[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_drp_slamfast_p2u_keypoints_size[7], "copy keypoints size using MemcpyP2U[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_drp_slamfast_p2u_keypoints[0], "copy keypoints using MemcpyP2U[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_drp_slamfast_p2u_keypoints[1], "copy keypoints using MemcpyP2U[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_drp_slamfast_p2u_keypoints[2], "copy keypoints using MemcpyP2U[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_drp_slamfast_p2u_keypoints[3], "copy keypoints using MemcpyP2U[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_drp_slamfast_p2u_keypoints[4], "copy keypoints using MemcpyP2U[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_drp_slamfast_p2u_keypoints[5], "copy keypoints using MemcpyP2U[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_drp_slamfast_p2u_keypoints[6], "copy keypoints using MemcpyP2U[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_drp_slamfast_p2u_keypoints[7], "copy keypoints using MemcpyP2U[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_drp_slamfast_cast_to_cv[0], "cv::KeyPoint to drp::KeyPoint_FAST[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_drp_slamfast_cast_to_cv[1], "cv::KeyPoint to drp::KeyPoint_FAST[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_drp_slamfast_cast_to_cv[2], "cv::KeyPoint to drp::KeyPoint_FAST[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_drp_slamfast_cast_to_cv[3], "cv::KeyPoint to drp::KeyPoint_FAST[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_drp_slamfast_cast_to_cv[4], "cv::KeyPoint to drp::KeyPoint_FAST[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_drp_slamfast_cast_to_cv[5], "cv::KeyPoint to drp::KeyPoint_FAST[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_drp_slamfast_cast_to_cv[6], "cv::KeyPoint to drp::KeyPoint_FAST[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_drp_slamfast_cast_to_cv[7], "cv::KeyPoint to drp::KeyPoint_FAST[7]", "ComputeKeyPointsOctTree[7]"},
    {&mt_cvfast[0], "FAST[0]", "ComputeKeyPointsOctTree[0]"},
    {&mt_cvfast[1], "FAST[1]", "ComputeKeyPointsOctTree[1]"},
    {&mt_cvfast[2], "FAST[2]", "ComputeKeyPointsOctTree[2]"},
    {&mt_cvfast[3], "FAST[3]", "ComputeKeyPointsOctTree[3]"},
    {&mt_cvfast[4], "FAST[4]", "ComputeKeyPointsOctTree[4]"},
    {&mt_cvfast[5], "FAST[5]", "ComputeKeyPointsOctTree[5]"},
    {&mt_cvfast[6], "FAST[6]", "ComputeKeyPointsOctTree[6]"},
    {&mt_cvfast[7], "FAST[7]", "ComputeKeyPointsOctTree[7]"},
    // in FAST[0-7]
    {&mt_drp_cvfast_calc_param[0], "calculate parameter[0]", "FAST[0]"},
    {&mt_drp_cvfast_calc_param[1], "calculate parameter[1]", "FAST[1]"},
    {&mt_drp_cvfast_calc_param[2], "calculate parameter[2]", "FAST[2]"},
    {&mt_drp_cvfast_calc_param[3], "calculate parameter[3]", "FAST[3]"},
    {&mt_drp_cvfast_calc_param[4], "calculate parameter[4]", "FAST[4]"},
    {&mt_drp_cvfast_calc_param[5], "calculate parameter[5]", "FAST[5]"},
    {&mt_drp_cvfast_calc_param[6], "calculate parameter[6]", "FAST[6]"},
    {&mt_drp_cvfast_calc_param[7], "calculate parameter[7]", "FAST[7]"},
    {&mt_drp_cvfast_u2p_param[0], "copy parameter using MemcpyU2P[0]", "FAST[0]"},
    {&mt_drp_cvfast_u2p_param[1], "copy parameter using MemcpyU2P[1]", "FAST[1]"},
    {&mt_drp_cvfast_u2p_param[2], "copy parameter using MemcpyU2P[2]", "FAST[2]"},
    {&mt_drp_cvfast_u2p_param[3], "copy parameter using MemcpyU2P[3]", "FAST[3]"},
    {&mt_drp_cvfast_u2p_param[4], "copy parameter using MemcpyU2P[4]", "FAST[4]"},
    {&mt_drp_cvfast_u2p_param[5], "copy parameter using MemcpyU2P[5]", "FAST[5]"},
    {&mt_drp_cvfast_u2p_param[6], "copy parameter using MemcpyU2P[6]", "FAST[6]"},
    {&mt_drp_cvfast_u2p_param[7], "copy parameter using MemcpyU2P[7]", "FAST[7]"},
    {&mt_drp_cvfast_u2p_input[0], "copy input using MemcpyU2P[0]", "FAST[0]"},
    {&mt_drp_cvfast_u2p_input[1], "copy input using MemcpyU2P[1]", "FAST[1]"},
    {&mt_drp_cvfast_u2p_input[2], "copy input using MemcpyU2P[2]", "FAST[2]"},
    {&mt_drp_cvfast_u2p_input[3], "copy input using MemcpyU2P[3]", "FAST[3]"},
    {&mt_drp_cvfast_u2p_input[4], "copy input using MemcpyU2P[4]", "FAST[4]"},
    {&mt_drp_cvfast_u2p_input[5], "copy input using MemcpyU2P[5]", "FAST[5]"},
    {&mt_drp_cvfast_u2p_input[6], "copy input using MemcpyU2P[6]", "FAST[6]"},
    {&mt_drp_cvfast_u2p_input[7], "copy input using MemcpyU2P[7]", "FAST[7]"},
    {&mt_drp_cvfast_activate, "Activate", "FAST[0-7]"},
    {&mt_drp_cvfast_start[0], "Start[0]", "FAST[0]"},
    {&mt_drp_cvfast_start[1], "Start[1]", "FAST[1]"},
    {&mt_drp_cvfast_start[2], "Start[2]", "FAST[2]"},
    {&mt_drp_cvfast_start[3], "Start[3]", "FAST[3]"},
    {&mt_drp_cvfast_start[4], "Start[4]", "FAST[4]"},
    {&mt_drp_cvfast_start[5], "Start[5]", "FAST[5]"},
    {&mt_drp_cvfast_start[6], "Start[6]", "FAST[6]"},
    {&mt_drp_cvfast_start[7], "Start[7]", "FAST[7]"},
    {&mt_drp_cvfast_get_status[0], "GetStatus[0]", "FAST[0]"},
    {&mt_drp_cvfast_get_status[1], "GetStatus[1]", "FAST[1]"},
    {&mt_drp_cvfast_get_status[2], "GetStatus[2]", "FAST[2]"},
    {&mt_drp_cvfast_get_status[3], "GetStatus[3]", "FAST[3]"},
    {&mt_drp_cvfast_get_status[4], "GetStatus[4]", "FAST[4]"},
    {&mt_drp_cvfast_get_status[5], "GetStatus[5]", "FAST[5]"},
    {&mt_drp_cvfast_get_status[6], "GetStatus[6]", "FAST[6]"},
    {&mt_drp_cvfast_get_status[7], "GetStatus[7]", "FAST[7]"},
    {&mt_drp_cvfast_p2u_keypoints_size[0], "copy keypoints size using MemcpyP2U[0]", "FAST[0]"},
    {&mt_drp_cvfast_p2u_keypoints_size[1], "copy keypoints size using MemcpyP2U[1]", "FAST[1]"},
    {&mt_drp_cvfast_p2u_keypoints_size[2], "copy keypoints size using MemcpyP2U[2]", "FAST[2]"},
    {&mt_drp_cvfast_p2u_keypoints_size[3], "copy keypoints size using MemcpyP2U[3]", "FAST[3]"},
    {&mt_drp_cvfast_p2u_keypoints_size[4], "copy keypoints size using MemcpyP2U[4]", "FAST[4]"},
    {&mt_drp_cvfast_p2u_keypoints_size[5], "copy keypoints size using MemcpyP2U[5]", "FAST[5]"},
    {&mt_drp_cvfast_p2u_keypoints_size[6], "copy keypoints size using MemcpyP2U[6]", "FAST[6]"},
    {&mt_drp_cvfast_p2u_keypoints_size[7], "copy keypoints size using MemcpyP2U[7]", "FAST[7]"},
    {&mt_drp_cvfast_p2u_keypoints[0], "copy keypoints using MemcpyP2U[0]", "FAST[0]"},
    {&mt_drp_cvfast_p2u_keypoints[1], "copy keypoints using MemcpyP2U[1]", "FAST[1]"},
    {&mt_drp_cvfast_p2u_keypoints[2], "copy keypoints using MemcpyP2U[2]", "FAST[2]"},
    {&mt_drp_cvfast_p2u_keypoints[3], "copy keypoints using MemcpyP2U[3]", "FAST[3]"},
    {&mt_drp_cvfast_p2u_keypoints[4], "copy keypoints using MemcpyP2U[4]", "FAST[4]"},
    {&mt_drp_cvfast_p2u_keypoints[5], "copy keypoints using MemcpyP2U[5]", "FAST[5]"},
    {&mt_drp_cvfast_p2u_keypoints[6], "copy keypoints using MemcpyP2U[6]", "FAST[6]"},
    {&mt_drp_cvfast_p2u_keypoints[7], "copy keypoints using MemcpyP2U[7]", "FAST[7]"},
    {&mt_drp_cvfast_cast_to_cv[0], "cv::KeyPoint to drp::KeyPoint_FAST[0]", "FAST[0]"},
    {&mt_drp_cvfast_cast_to_cv[1], "cv::KeyPoint to drp::KeyPoint_FAST[1]", "FAST[1]"},
    {&mt_drp_cvfast_cast_to_cv[2], "cv::KeyPoint to drp::KeyPoint_FAST[2]", "FAST[2]"},
    {&mt_drp_cvfast_cast_to_cv[3], "cv::KeyPoint to drp::KeyPoint_FAST[3]", "FAST[3]"},
    {&mt_drp_cvfast_cast_to_cv[4], "cv::KeyPoint to drp::KeyPoint_FAST[4]", "FAST[4]"},
    {&mt_drp_cvfast_cast_to_cv[5], "cv::KeyPoint to drp::KeyPoint_FAST[5]", "FAST[5]"},
    {&mt_drp_cvfast_cast_to_cv[6], "cv::KeyPoint to drp::KeyPoint_FAST[6]", "FAST[6]"},
    {&mt_drp_cvfast_cast_to_cv[7], "cv::KeyPoint to drp::KeyPoint_FAST[7]", "FAST[7]"},
    // in ORBextractor::ComputeKeyPointsOctTree
    {&mt_wait_for_next_max_features_to_false, "Wait for ImageProcessing::wait_for_next_max_features to false", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[0][0], "ORBextractor::DistributeOctTree[0][0]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[0][1], "ORBextractor::DistributeOctTree[0][1]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[0][2], "ORBextractor::DistributeOctTree[0][2]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[0][3], "ORBextractor::DistributeOctTree[0][3]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[0][4], "ORBextractor::DistributeOctTree[0][4]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[0][5], "ORBextractor::DistributeOctTree[0][5]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[0][6], "ORBextractor::DistributeOctTree[0][6]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[0][7], "ORBextractor::DistributeOctTree[0][7]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[1][0], "ORBextractor::DistributeOctTree[1][0]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[1][1], "ORBextractor::DistributeOctTree[1][1]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[1][2], "ORBextractor::DistributeOctTree[1][2]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[1][3], "ORBextractor::DistributeOctTree[1][3]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[1][4], "ORBextractor::DistributeOctTree[1][4]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[1][5], "ORBextractor::DistributeOctTree[1][5]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[1][6], "ORBextractor::DistributeOctTree[1][6]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_distribute_keypoints_via_tree[1][7], "ORBextractor::DistributeOctTree[1][7]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[0][0], "computeOrientation[0][0]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[0][1], "computeOrientation[0][1]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[0][2], "computeOrientation[0][2]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[0][3], "computeOrientation[0][3]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[0][4], "computeOrientation[0][4]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[0][5], "computeOrientation[0][5]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[0][6], "computeOrientation[0][6]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[0][7], "computeOrientation[0][7]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[1][0], "computeOrientation[1][0]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[1][1], "computeOrientation[1][1]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[1][2], "computeOrientation[1][2]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[1][3], "computeOrientation[1][3]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[1][4], "computeOrientation[1][4]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[1][5], "computeOrientation[1][5]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[1][6], "computeOrientation[1][6]", "ORBextractor::ComputeKeyPointsOctTree"},
    {&mt_compute_orientation[1][7], "computeOrientation[1][7]", "ORBextractor::ComputeKeyPointsOctTree"},
    // in Image processing
    {&mt_extract_orb_desc[0], "Frame::ExtractOrbDescriptors[0]", "Image processing"},
    {&mt_extract_orb_desc[1], "Frame::ExtractOrbDescriptors[1]", "Image processing"},
    {&mt_gaussianblur[0], "GaussianBlur[0]", "Image processing"},
    {&mt_gaussianblur[1], "GaussianBlur[1]", "Image processing"},
    {&mt_gaussianblur[2], "GaussianBlur[2]", "Image processing"},
    {&mt_gaussianblur[3], "GaussianBlur[3]", "Image processing"},
    {&mt_gaussianblur[4], "GaussianBlur[4]", "Image processing"},
    {&mt_gaussianblur[5], "GaussianBlur[5]", "Image processing"},
    {&mt_gaussianblur[6], "GaussianBlur[6]", "Image processing"},
    {&mt_gaussianblur[7], "GaussianBlur[7]", "Image processing"},
    // in GaussianBlur[0-7]
    {&mt_drp_gaussian_blur_calc_param[0], "calculate parameter[0]", "GaussianBlur[0]"},
    {&mt_drp_gaussian_blur_calc_param[1], "calculate parameter[1]", "GaussianBlur[1]"},
    {&mt_drp_gaussian_blur_calc_param[2], "calculate parameter[2]", "GaussianBlur[2]"},
    {&mt_drp_gaussian_blur_calc_param[3], "calculate parameter[3]", "GaussianBlur[3]"},
    {&mt_drp_gaussian_blur_calc_param[4], "calculate parameter[4]", "GaussianBlur[4]"},
    {&mt_drp_gaussian_blur_calc_param[5], "calculate parameter[5]", "GaussianBlur[5]"},
    {&mt_drp_gaussian_blur_calc_param[6], "calculate parameter[6]", "GaussianBlur[6]"},
    {&mt_drp_gaussian_blur_calc_param[7], "calculate parameter[7]", "GaussianBlur[7]"},
    {&mt_drp_gaussian_blur_u2p_param[0], "copy parameter using MemcpyU2P[0]", "GaussianBlur[0]"},
    {&mt_drp_gaussian_blur_u2p_param[1], "copy parameter using MemcpyU2P[1]", "GaussianBlur[1]"},
    {&mt_drp_gaussian_blur_u2p_param[2], "copy parameter using MemcpyU2P[2]", "GaussianBlur[2]"},
    {&mt_drp_gaussian_blur_u2p_param[3], "copy parameter using MemcpyU2P[3]", "GaussianBlur[3]"},
    {&mt_drp_gaussian_blur_u2p_param[4], "copy parameter using MemcpyU2P[4]", "GaussianBlur[4]"},
    {&mt_drp_gaussian_blur_u2p_param[5], "copy parameter using MemcpyU2P[5]", "GaussianBlur[5]"},
    {&mt_drp_gaussian_blur_u2p_param[6], "copy parameter using MemcpyU2P[6]", "GaussianBlur[6]"},
    {&mt_drp_gaussian_blur_u2p_param[7], "copy parameter using MemcpyU2P[7]", "GaussianBlur[7]"},
    {&mt_drp_gaussian_blur_u2p_input[0], "copy input using MemcpyU2P[0]", "GaussianBlur[0]"},
    {&mt_drp_gaussian_blur_u2p_input[1], "copy input using MemcpyU2P[1]", "GaussianBlur[1]"},
    {&mt_drp_gaussian_blur_u2p_input[2], "copy input using MemcpyU2P[2]", "GaussianBlur[2]"},
    {&mt_drp_gaussian_blur_u2p_input[3], "copy input using MemcpyU2P[3]", "GaussianBlur[3]"},
    {&mt_drp_gaussian_blur_u2p_input[4], "copy input using MemcpyU2P[4]", "GaussianBlur[4]"},
    {&mt_drp_gaussian_blur_u2p_input[5], "copy input using MemcpyU2P[5]", "GaussianBlur[5]"},
    {&mt_drp_gaussian_blur_u2p_input[6], "copy input using MemcpyU2P[6]", "GaussianBlur[6]"},
    {&mt_drp_gaussian_blur_u2p_input[7], "copy input using MemcpyU2P[7]", "GaussianBlur[7]"},
    {&mt_drp_gaussian_blur_activate, "Activate", "GaussianBlur"},
    {&mt_drp_gaussian_blur_start[0], "Start[0]", "GaussianBlur[0]"},
    {&mt_drp_gaussian_blur_start[1], "Start[1]", "GaussianBlur[1]"},
    {&mt_drp_gaussian_blur_start[2], "Start[2]", "GaussianBlur[2]"},
    {&mt_drp_gaussian_blur_start[3], "Start[3]", "GaussianBlur[3]"},
    {&mt_drp_gaussian_blur_start[4], "Start[4]", "GaussianBlur[4]"},
    {&mt_drp_gaussian_blur_start[5], "Start[5]", "GaussianBlur[5]"},
    {&mt_drp_gaussian_blur_start[6], "Start[6]", "GaussianBlur[6]"},
    {&mt_drp_gaussian_blur_start[7], "Start[7]", "GaussianBlur[7]"},
    {&mt_drp_gaussian_blur_get_status[0], "GetStatus[0]", "GaussianBlur[0]"},
    {&mt_drp_gaussian_blur_get_status[1], "GetStatus[1]", "GaussianBlur[1]"},
    {&mt_drp_gaussian_blur_get_status[2], "GetStatus[2]", "GaussianBlur[2]"},
    {&mt_drp_gaussian_blur_get_status[3], "GetStatus[3]", "GaussianBlur[3]"},
    {&mt_drp_gaussian_blur_get_status[4], "GetStatus[4]", "GaussianBlur[4]"},
    {&mt_drp_gaussian_blur_get_status[5], "GetStatus[5]", "GaussianBlur[5]"},
    {&mt_drp_gaussian_blur_get_status[6], "GetStatus[6]", "GaussianBlur[6]"},
    {&mt_drp_gaussian_blur_get_status[7], "GetStatus[7]", "GaussianBlur[7]"},
    {&mt_drp_gaussian_blur_p2u[0], "MemcpyP2U[0]", "GaussianBlur[0]"},
    {&mt_drp_gaussian_blur_p2u[1], "MemcpyP2U[1]", "GaussianBlur[1]"},
    {&mt_drp_gaussian_blur_p2u[2], "MemcpyP2U[2]", "GaussianBlur[2]"},
    {&mt_drp_gaussian_blur_p2u[3], "MemcpyP2U[3]", "GaussianBlur[3]"},
    {&mt_drp_gaussian_blur_p2u[4], "MemcpyP2U[4]", "GaussianBlur[4]"},
    {&mt_drp_gaussian_blur_p2u[5], "MemcpyP2U[5]", "GaussianBlur[5]"},
    {&mt_drp_gaussian_blur_p2u[6], "MemcpyP2U[6]", "GaussianBlur[6]"},
    {&mt_drp_gaussian_blur_p2u[7], "MemcpyP2U[7]", "GaussianBlur[7]"},
    // in Frame::ExtractOrbDescriptors
    {&mt_compute_orb[0][0], "computeDescriptors[0][0]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[0][1], "computeDescriptors[0][1]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[0][2], "computeDescriptors[0][2]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[0][3], "computeDescriptors[0][3]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[0][4], "computeDescriptors[0][4]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[0][5], "computeDescriptors[0][5]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[0][6], "computeDescriptors[0][6]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[0][7], "computeDescriptors[0][7]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[1][0], "computeDescriptors[1][0]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[1][1], "computeDescriptors[1][1]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[1][2], "computeDescriptors[1][2]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[1][3], "computeDescriptors[1][3]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[1][4], "computeDescriptors[1][4]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[1][5], "computeDescriptors[1][5]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[1][6], "computeDescriptors[1][6]", "Frame::ExtractOrbDescriptors"},
    {&mt_compute_orb[1][7], "computeDescriptors[1][7]", "Frame::ExtractOrbDescriptors"},
    // in computeOrbDescriptors[0-7]
    {&mt_drp_orb_descriptors_calc_param[0][0], "calculate parameter[0][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_calc_param[0][1], "calculate parameter[0][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_calc_param[0][2], "calculate parameter[0][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_calc_param[0][3], "calculate parameter[0][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_calc_param[0][4], "calculate parameter[0][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_calc_param[0][5], "calculate parameter[0][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_calc_param[0][6], "calculate parameter[0][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_calc_param[0][7], "calculate parameter[0][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_calc_param[1][0], "calculate parameter[1][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_calc_param[1][1], "calculate parameter[1][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_calc_param[1][2], "calculate parameter[1][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_calc_param[1][3], "calculate parameter[1][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_calc_param[1][4], "calculate parameter[1][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_calc_param[1][5], "calculate parameter[1][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_calc_param[1][6], "calculate parameter[1][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_calc_param[1][7], "calculate parameter[1][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_cast_to_drp[0][0], "cv::KeyPoint to drp::KeyPoint_ORB[0][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_cast_to_drp[0][1], "cv::KeyPoint to drp::KeyPoint_ORB[0][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_cast_to_drp[0][2], "cv::KeyPoint to drp::KeyPoint_ORB[0][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_cast_to_drp[0][3], "cv::KeyPoint to drp::KeyPoint_ORB[0][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_cast_to_drp[0][4], "cv::KeyPoint to drp::KeyPoint_ORB[0][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_cast_to_drp[0][5], "cv::KeyPoint to drp::KeyPoint_ORB[0][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_cast_to_drp[0][6], "cv::KeyPoint to drp::KeyPoint_ORB[0][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_cast_to_drp[0][7], "cv::KeyPoint to drp::KeyPoint_ORB[0][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_cast_to_drp[1][0], "cv::KeyPoint to drp::KeyPoint_ORB[1][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_cast_to_drp[1][1], "cv::KeyPoint to drp::KeyPoint_ORB[1][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_cast_to_drp[1][2], "cv::KeyPoint to drp::KeyPoint_ORB[1][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_cast_to_drp[1][3], "cv::KeyPoint to drp::KeyPoint_ORB[1][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_cast_to_drp[1][4], "cv::KeyPoint to drp::KeyPoint_ORB[1][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_cast_to_drp[1][5], "cv::KeyPoint to drp::KeyPoint_ORB[1][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_cast_to_drp[1][6], "cv::KeyPoint to drp::KeyPoint_ORB[1][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_cast_to_drp[1][7], "cv::KeyPoint to drp::KeyPoint_ORB[1][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_u2p_param[0][0], "copy parameter using MemcpyU2P[0][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_u2p_param[0][1], "copy parameter using MemcpyU2P[0][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_u2p_param[0][2], "copy parameter using MemcpyU2P[0][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_u2p_param[0][3], "copy parameter using MemcpyU2P[0][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_u2p_param[0][4], "copy parameter using MemcpyU2P[0][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_u2p_param[0][5], "copy parameter using MemcpyU2P[0][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_u2p_param[0][6], "copy parameter using MemcpyU2P[0][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_u2p_param[0][7], "copy parameter using MemcpyU2P[0][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_u2p_param[1][0], "copy parameter using MemcpyU2P[1][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_u2p_param[1][1], "copy parameter using MemcpyU2P[1][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_u2p_param[1][2], "copy parameter using MemcpyU2P[1][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_u2p_param[1][3], "copy parameter using MemcpyU2P[1][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_u2p_param[1][4], "copy parameter using MemcpyU2P[1][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_u2p_param[1][5], "copy parameter using MemcpyU2P[1][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_u2p_param[1][6], "copy parameter using MemcpyU2P[1][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_u2p_param[1][7], "copy parameter using MemcpyU2P[1][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_u2p_input_image[0][0], "copy input image using MemcpyU2P[0][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_u2p_input_image[0][1], "copy input image using MemcpyU2P[0][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_u2p_input_image[0][2], "copy input image using MemcpyU2P[0][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_u2p_input_image[0][3], "copy input image using MemcpyU2P[0][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_u2p_input_image[0][4], "copy input image using MemcpyU2P[0][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_u2p_input_image[0][5], "copy input image using MemcpyU2P[0][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_u2p_input_image[0][6], "copy input image using MemcpyU2P[0][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_u2p_input_image[0][7], "copy input image using MemcpyU2P[0][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_u2p_input_image[1][0], "copy input image using MemcpyU2P[1][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_u2p_input_image[1][1], "copy input image using MemcpyU2P[1][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_u2p_input_image[1][2], "copy input image using MemcpyU2P[1][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_u2p_input_image[1][3], "copy input image using MemcpyU2P[1][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_u2p_input_image[1][4], "copy input image using MemcpyU2P[1][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_u2p_input_image[1][5], "copy input image using MemcpyU2P[1][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_u2p_input_image[1][6], "copy input image using MemcpyU2P[1][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_u2p_input_image[1][7], "copy input image using MemcpyU2P[1][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[0][0], "copy input keypoints using MemcpyU2P[0][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[0][1], "copy input keypoints using MemcpyU2P[0][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[0][2], "copy input keypoints using MemcpyU2P[0][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[0][3], "copy input keypoints using MemcpyU2P[0][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[0][4], "copy input keypoints using MemcpyU2P[0][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[0][5], "copy input keypoints using MemcpyU2P[0][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[0][6], "copy input keypoints using MemcpyU2P[0][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[0][7], "copy input keypoints using MemcpyU2P[0][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[1][0], "copy input keypoints using MemcpyU2P[1][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[1][1], "copy input keypoints using MemcpyU2P[1][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[1][2], "copy input keypoints using MemcpyU2P[1][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[1][3], "copy input keypoints using MemcpyU2P[1][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[1][4], "copy input keypoints using MemcpyU2P[1][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[1][5], "copy input keypoints using MemcpyU2P[1][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[1][6], "copy input keypoints using MemcpyU2P[1][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_u2p_input_keypoints[1][7], "copy input keypoints using MemcpyU2P[1][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_activate, "Activate", "computeOrbDescriptors"},
    {&mt_drp_orb_descriptors_start[0][0], "Start[0][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_start[0][1], "Start[0][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_start[0][2], "Start[0][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_start[0][3], "Start[0][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_start[0][4], "Start[0][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_start[0][5], "Start[0][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_start[0][6], "Start[0][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_start[0][7], "Start[0][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_start[1][0], "Start[1][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_start[1][1], "Start[1][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_start[1][2], "Start[1][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_start[1][3], "Start[1][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_start[1][4], "Start[1][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_start[1][5], "Start[1][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_start[1][6], "Start[1][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_start[1][7], "Start[1][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_get_status[0][0], "GetStatus[0][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_get_status[0][1], "GetStatus[0][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_get_status[0][2], "GetStatus[0][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_get_status[0][3], "GetStatus[0][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_get_status[0][4], "GetStatus[0][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_get_status[0][5], "GetStatus[0][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_get_status[0][6], "GetStatus[0][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_get_status[0][7], "GetStatus[0][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_get_status[1][0], "GetStatus[1][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_get_status[1][1], "GetStatus[1][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_get_status[1][2], "GetStatus[1][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_get_status[1][3], "GetStatus[1][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_get_status[1][4], "GetStatus[1][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_get_status[1][5], "GetStatus[1][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_get_status[1][6], "GetStatus[1][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_get_status[1][7], "GetStatus[1][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_p2u[0][0], "MemcpyP2U[0][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_p2u[0][1], "MemcpyP2U[0][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_p2u[0][2], "MemcpyP2U[0][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_p2u[0][3], "MemcpyP2U[0][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_p2u[0][4], "MemcpyP2U[0][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_p2u[0][5], "MemcpyP2U[0][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_p2u[0][6], "MemcpyP2U[0][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_p2u[0][7], "MemcpyP2U[0][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_p2u[1][0], "MemcpyP2U[1][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_p2u[1][1], "MemcpyP2U[1][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_p2u[1][2], "MemcpyP2U[1][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_p2u[1][3], "MemcpyP2U[1][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_p2u[1][4], "MemcpyP2U[1][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_p2u[1][5], "MemcpyP2U[1][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_p2u[1][6], "MemcpyP2U[1][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_p2u[1][7], "MemcpyP2U[1][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_postprocess[0][0], "postprocess[0][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_postprocess[0][1], "postprocess[0][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_postprocess[0][2], "postprocess[0][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_postprocess[0][3], "postprocess[0][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_postprocess[0][4], "postprocess[0][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_postprocess[0][5], "postprocess[0][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_postprocess[0][6], "postprocess[0][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_postprocess[0][7], "postprocess[0][7]", "computeOrbDescriptors[7]"},
    {&mt_drp_orb_descriptors_postprocess[1][0], "postprocess[1][0]", "computeOrbDescriptors[0]"},
    {&mt_drp_orb_descriptors_postprocess[1][1], "postprocess[1][1]", "computeOrbDescriptors[1]"},
    {&mt_drp_orb_descriptors_postprocess[1][2], "postprocess[1][2]", "computeOrbDescriptors[2]"},
    {&mt_drp_orb_descriptors_postprocess[1][3], "postprocess[1][3]", "computeOrbDescriptors[3]"},
    {&mt_drp_orb_descriptors_postprocess[1][4], "postprocess[1][4]", "computeOrbDescriptors[4]"},
    {&mt_drp_orb_descriptors_postprocess[1][5], "postprocess[1][5]", "computeOrbDescriptors[5]"},
    {&mt_drp_orb_descriptors_postprocess[1][6], "postprocess[1][6]", "computeOrbDescriptors[6]"},
    {&mt_drp_orb_descriptors_postprocess[1][7], "postprocess[1][7]", "computeOrbDescriptors[7]"},
    // in Image processing
    {&mt_last_processing_of_constructor[0], "Frame_drp::LastProcessingOfConstructor[0]", "Image processing"},
    {&mt_last_processing_of_constructor[1], "Frame_drp::LastProcessingOfConstructor[1]", "Image processing"},
    // ImageProcessing::Run
    {&mt_image_processing_push_result, "ImageProcessing::PushResult", "ImageProcessing::Run"},
    // in drp::*
    {&mt_drp_time[0][1][0], "resize DRP time[0]", "drp::Resize*"},
    {&mt_drp_time[0][1][1], "resize DRP time[1]", "drp::Resize*"},
    {&mt_drp_time[0][1][2], "resize DRP time[2]", "drp::Resize*"},
    {&mt_drp_time[0][1][3], "resize DRP time[3]", "drp::Resize*"},
    {&mt_drp_time[0][1][4], "resize DRP time[4]", "drp::Resize*"},
    {&mt_drp_time[0][1][5], "resize DRP time[5]", "drp::Resize*"},
    {&mt_drp_time[0][1][6], "resize DRP time[6]", "drp::Resize*"},
    {&mt_drp_time[0][1][7], "resize DRP time[7]", "drp::Resize*"},
    {&mt_drp_time[0][2][0], "GaussianBlur DRP time[0]", "drp::GaussianBlur*"},
    {&mt_drp_time[0][2][1], "GaussianBlur DRP time[1]", "drp::GaussianBlur*"},
    {&mt_drp_time[0][2][2], "GaussianBlur DRP time[2]", "drp::GaussianBlur*"},
    {&mt_drp_time[0][2][3], "GaussianBlur DRP time[3]", "drp::GaussianBlur*"},
    {&mt_drp_time[0][2][4], "GaussianBlur DRP time[4]", "drp::GaussianBlur*"},
    {&mt_drp_time[0][2][5], "GaussianBlur DRP time[5]", "drp::GaussianBlur*"},
    {&mt_drp_time[0][2][6], "GaussianBlur DRP time[6]", "drp::GaussianBlur*"},
    {&mt_drp_time[0][2][7], "GaussianBlur DRP time[7]", "drp::GaussianBlur*"},
    {&mt_drp_time[0][3][0], "fast DRP time[0]", "drp::*FAST*"},
    {&mt_drp_time[0][3][1], "fast DRP time[1]", "drp::*FAST*"},
    {&mt_drp_time[0][3][2], "fast DRP time[2]", "drp::*FAST*"},
    {&mt_drp_time[0][3][3], "fast DRP time[3]", "drp::*FAST*"},
    {&mt_drp_time[0][3][4], "fast DRP time[4]", "drp::*FAST*"},
    {&mt_drp_time[0][3][5], "fast DRP time[5]", "drp::*FAST*"},
    {&mt_drp_time[0][3][6], "fast DRP time[6]", "drp::*FAST*"},
    {&mt_drp_time[0][3][7], "fast DRP time[7]", "drp::*FAST*"},
    {&mt_drp_time[0][4][0], "computeOrbDescriptors DRP time[0][0]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[0][4][1], "computeOrbDescriptors DRP time[0][1]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[0][4][2], "computeOrbDescriptors DRP time[0][2]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[0][4][3], "computeOrbDescriptors DRP time[0][3]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[0][4][4], "computeOrbDescriptors DRP time[0][4]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[0][4][5], "computeOrbDescriptors DRP time[0][5]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[0][4][6], "computeOrbDescriptors DRP time[0][6]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[0][4][7], "computeOrbDescriptors DRP time[0][7]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[1][4][0], "computeOrbDescriptors DRP time[1][0]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[1][4][1], "computeOrbDescriptors DRP time[1][1]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[1][4][2], "computeOrbDescriptors DRP time[1][2]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[1][4][3], "computeOrbDescriptors DRP time[1][3]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[1][4][4], "computeOrbDescriptors DRP time[1][4]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[1][4][5], "computeOrbDescriptors DRP time[1][5]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[1][4][6], "computeOrbDescriptors DRP time[1][6]", "drp::computeOrbDescriptors*"},
    {&mt_drp_time[1][4][7], "computeOrbDescriptors DRP time[1][7]", "drp::computeOrbDescriptors*"},
    // tracking
    {&mt_main_loop, "Main loop", "Executable file"},
    // in Main loop
    {&mt_option_usleep, "Sleep specified at execution", "Main loop"},
    {&mt_main_usleep, "Sleep while waiting for the next frame", "Main loop"},
    {&mt_main_track, "System::Track*", "Main loop"},
    // in System::Track*
    {&mt_tracker_wait_for_next_frame, "Wait for Tracking::wait_for_next_frame to false", "System::Track*"},
    {&mt_wait_for_next_max_features_to_true, "Wait for ImageProcessing::wait_for_next_max_features to true", "System::Track*"},
    {&mt_grab, "Tracking::GrabImage*", "System::Track*"},
    // in Tracking::GrabImage*
    {&mt_plane_detector_process, "PlaneDetector::Process", "Tracking::GrabImage*"},
    // in PlaneDetector::Process
    {&mt_organize_pointcloud_by_cell, "PlaneDetector::organizePointCloudByCell", "PlaneDetector::Process"},
    {&mt_planar_cell_fitting, "Planar cell fitting", "PlaneDetector::Process"},
    {&mt_project_pointcloud, "PlaneDetector::projectPointCloud", "PlaneDetector::Process"},
    // in PlaneDetector::projectPointCloud
    {&mt_mul_x, "X of cv::Mat::mul", "PlaneDetector::projectPointCloud"},
    {&mt_mul_y, "Y of cv::Mat::mul", "PlaneDetector::projectPointCloud"},
    {&mt_divide_x, "X of cv::divide", "PlaneDetector::projectPointCloud"},
    {&mt_divide_y, "Y of cv::divide", "PlaneDetector::projectPointCloud"},
    {&mt_calc_u, "calculate U", "PlaneDetector::projectPointCloud"},
    {&mt_calc_v, "calculate V", "PlaneDetector::projectPointCloud"},
    {&mt_set_cloud_array, "set cloud_array", "PlaneDetector::projectPointCloud"},
    // in Tracking::GrabImage*
    {&mt_my_calculate_after, "Frame::MyCalculateAfterRemoveDynamicPointsAndPlaneExtraction", "Tracking::GrabImage*"},
    {&mt_track, "Tracking::Track", "Tracking::GrabImage*"},
    // in Tracking::Track
    {&mt_lock_map_update, "Map::mMutexMapUpdate", "Tracking::Track"},
    {&mt_monocular_initialization, "Tracking::MonocularInitialization", "Tracking::Track"},
    // in Tracking::MonocularInitialization
    {&mt_search_for_initialization, "ORBmatcher::SearchForInitialization", "Tracking::MonocularInitialization"},
    // in Tracking::Track
    {&mt_track_reference_keyframe, "Tracking::TrackReferenceKeyFrame", "Tracking::Track"},
    // in Tracking::TrackReferenceKeyFrame
    {&mt_search_by_bow_in_track_reference_keyframe, "ORBmatcher::SearchByBoW", "Tracking::TrackReferenceKeyFrame"},
    // in Tracking::Track
    {&mt_track_current_frame, "Tracking::TrackWithMotionModel", "Tracking::Track"},
    // in Tracking::TrackWithMotionModel
    {&mt_pose_optimization_in_track_with_motion_model, "Optimizer::PoseOptimization", "Tracking::TrackWithMotionModel"},
    {&mt_search_for_projection_in_track_with_motion_model_1, "first ORBmatcher::SearchByProjection", "Tracking::TrackWithMotionModel"},
    {&mt_search_for_projection_in_track_with_motion_model_2, "second ORBmatcher::SearchByProjection", "Tracking::TrackWithMotionModel"},
    // in Tracking::Track
    {&mt_track_local_map, "Tracking::TrackLocalMap", "Tracking::Track"},
    // in Tracking::TrackLocalMap
    {&mt_update_local_map, "Tracking::UpdateLocalMap", "Tracking::TrackLocalMap"},
    {&mt_pose_optimization, "Optimizer::PoseOptimization", "Tracking::TrackLocalMap"},
    {&mt_update_mappoints, "Update MapPoints Statistics", "Tracking::TrackLocalMap"},
    {&mt_search_local_points, "Tracking::SearchLocalPoints", "Tracking::TrackLocalMap"},
    // in Tracking::SearchLocalPoints
    {&mt_already_matched, "Do not search map points already matched", "Tracking::SearchLocalPoints"},
    {&mt_project_points, "Project points in frame and check its visibility", "Tracking::SearchLocalPoints"},
    {&mt_search_for_projection_in_search_local_points, "ORBmatcher::SearchByProjection", "Tracking::SearchLocalPoints"},
    // in Tracking::Relocalization
    {&mt_search_by_bow_in_relocalization, "ORBmatcher::SearchByBoW", "Tracking::Relocalization"},
    {&mt_search_for_projection_in_relocalization_1, "ORBmatcher::SearchByProjection", "Tracking::Relocalization"},
    {&mt_search_for_projection_in_relocalization_2, "ORBmatcher::SearchByProjection", "Tracking::Relocalization"},
    // RecognizeBase::inference_thread
    {&mt_recognize_thread, "RecognizeBase::inference_thread", "Inference"},
    {&mt_recognize_polling, "polling", "RecognizeBase::inference_thread"},
    // LocalMapping::Run
    {&mt_mapping, "Local mapping", "LocalMapping::Run"},
    {&mt_process_new_keyframe, "LocalMapping::ProcessNewKeyFrame", "Local mapping"},
    {&mt_create_new_map_points, "LocalMapping::CreateNewMapPoints", "Local mapping"},
    {&mt_search_in_neighbors, "LocalMapping::SearchInNeighbors", "Local mapping"},
    {&mt_local_bundle_adjustment, "Optimizer::LocalBundleAdjustment", "Local mapping"},
    {&mt_keyframe_culling, "LocalMapping::KeyFrameCulling", "Local mapping"},
    // LoopClosing::Run
    {&mt_loop_closing, "Loop closing", "LoopClosing::Run"}};

struct measure_num_with_name_t {
    struct measure_num_t* mn;
    const char* name;
} measure_num_list[] = {
    {&mt_num_keypts_to_distribute[0], "keypts_to_distribute[0]"},
    {&mt_num_keypts_to_distribute[1], "keypts_to_distribute[1]"},
    {&mt_num_keypts_to_distribute[2], "keypts_to_distribute[2]"},
    {&mt_num_keypts_to_distribute[3], "keypts_to_distribute[3]"},
    {&mt_num_keypts_to_distribute[4], "keypts_to_distribute[4]"},
    {&mt_num_keypts_to_distribute[5], "keypts_to_distribute[5]"},
    {&mt_num_keypts_to_distribute[6], "keypts_to_distribute[6]"},
    {&mt_num_keypts_to_distribute[7], "keypts_to_distribute[7]"},
    {&mt_num_keypts_at_level[0], "keypts_at_level[0]"},
    {&mt_num_keypts_at_level[1], "keypts_at_level[1]"},
    {&mt_num_keypts_at_level[2], "keypts_at_level[2]"},
    {&mt_num_keypts_at_level[3], "keypts_at_level[3]"},
    {&mt_num_keypts_at_level[4], "keypts_at_level[4]"},
    {&mt_num_keypts_at_level[5], "keypts_at_level[5]"},
    {&mt_num_keypts_at_level[6], "keypts_at_level[6]"},
    {&mt_num_keypts_at_level[7], "keypts_at_level[7]"}};
#endif /* MEASURE_TIME_DECLARE_BODY */

#if defined(MEASURE_TIME_DECLARE_BODY)
#ifndef ENABLE_VTUNE_PROF
void measure_time_reset(void) {
    for (size_t i = 0; i < sizeof(measure_time_list) / sizeof(measure_time_list[0]); i++) {
        measure_time_list[i].mt->elapsed_time.clear();
        measure_time_list[i].mt->cnt = 0;
        measure_time_list[i].mt->last = 0.0;
    }

    for (size_t i = 0; i < sizeof(measure_num_list) / sizeof(measure_num_list[0]); i++) {
        measure_time_list[i].mt->cnt = 0;
        measure_time_list[i].mt->last = 0.0;
    }
}
void measure_time_init(void) {
    measure_time_reset();
}
void measure_time_start(struct measure_time_t& mt) {
    mt.start = std::chrono::steady_clock::now();
}
void measure_time_end(struct measure_time_t& mt) {
    mt.end = std::chrono::steady_clock::now();

    double sec = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(mt.end - mt.start).count()) / 1e9;
    mt.elapsed_time.push_back(sec);
    mt.sum += sec;
    mt.cnt++;
    mt.last = sec;
}
void measure_time_sum_start(struct measure_time_t& mt) {
    measure_time_start(mt);
}
void measure_time_sum_end(struct measure_time_t& mt) {
    mt.end = std::chrono::steady_clock::now();

    double sec = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(mt.end - mt.start).count()) / 1e9;
    mt.sum += sec;
    mt.cnt++;
    mt.last = sec;
}

void measure_time_push_back_zero(struct measure_time_t& mt) {
    double sec = 0.0;
    mt.elapsed_time.push_back(sec);
    mt.cnt++;
    mt.last = sec;
}

void measure_time_remove_last(struct measure_time_t& mt) {
    if (mt.elapsed_time.empty()) {
        std::cerr << "Failed to remove last of elapsed_time." << std::endl;
        return;
    }
    mt.elapsed_time.pop_back();
    mt.cnt--;
    mt.last = -1.0; // Dummy value
}

void measure_counts(struct measure_num_t& mn, uint32_t size) {
    mn.num.push_back(size);
    mn.cnt++;
}
void measure_time_print() {
    // ,,                                                 ..,run_euroc_slam, ,...
    // FRAMENO,keypts_to_distribute[0],...keypts_at_level[7],cv::imread,SLAM.feed_monocular_frame,...
    // 0, 2050,100,                                       ..,0.0002,0.0003
    std::ofstream output_csvfile(OUTPUT_CSVNAME, std::ios::out);

    const int frame_no_size = 1;
    const int num_list_size = sizeof(measure_num_list) / sizeof(measure_num_list[0]);
    const int time_list_size = sizeof(measure_time_list) / sizeof(measure_time_list[0]);

    // create 1st header
    for (int i = 0; i < frame_no_size + num_list_size; i++) {
        output_csvfile << ",";
    }
    for (int i = 0; i < time_list_size; i++) {
        output_csvfile << measure_time_list[i].func << ",";
    }
    output_csvfile << std::endl;

    // create 2nd header
    output_csvfile << "NAME"
                   << ",";
    for (int i = 0; i < num_list_size; i++) {
        output_csvfile << measure_num_list[i].name << ",";
    }
    for (int i = 0; i < time_list_size; i++) {
        output_csvfile << measure_time_list[i].name << ",";
    }
    output_csvfile << std::endl;

    // calculate max_cnt
    uint32_t max_cnt = 0;
    for (int i = 0; i < num_list_size; i++) {
        uint32_t ref_cnt = measure_num_list[i].mn->num.size();
        if (ref_cnt > max_cnt) {
            max_cnt = ref_cnt;
        }
    }
    for (int i = 0; i < time_list_size; i++) {
        uint32_t ref_cnt = measure_time_list[i].mt->elapsed_time.size();
        if (ref_cnt > max_cnt) {
            max_cnt = ref_cnt;
        }
    }

    // create count col
    output_csvfile << "COUNT"
                   << ",";
    for (int i = 0; i < num_list_size; i++) {
        output_csvfile << measure_num_list[i].mn->cnt << ",";
    }
    for (int i = 0; i < time_list_size; i++) {
        output_csvfile << measure_time_list[i].mt->cnt << ",";
    }
    output_csvfile << std::endl;

    // create sum
    output_csvfile << "SUM"
                   << ",";
    uint32_t num_sums[num_list_size];
    for (int i = 0; i < num_list_size; i++) {
        num_sums[i] = 0;
        for (size_t j = 0; j < measure_num_list[i].mn->cnt; j++) {
            num_sums[i] += measure_num_list[i].mn->num[j];
        }
        output_csvfile << num_sums[i] << ",";
    }
    for (int i = 0; i < time_list_size; i++) {
        output_csvfile << measure_time_list[i].mt->sum << ",";
    }
    output_csvfile << std::endl;

    // create average
    output_csvfile << "AVERAGE"
                   << ",";
    for (int i = 0; i < num_list_size; i++) {
        uint32_t num_ave = 0;
        if (0 < measure_num_list[i].mn->cnt)
            num_ave = num_sums[i] / measure_num_list[i].mn->cnt;
        output_csvfile << num_ave << ",";
    }

    for (int i = 0; i < time_list_size; i++) {
        double time_ave = 0.0;
        if (0 < measure_time_list[i].mt->cnt)
            time_ave = measure_time_list[i].mt->sum / measure_time_list[i].mt->cnt;
        output_csvfile << time_ave << ",";
    }
    output_csvfile << std::endl;

    // create frame cols
    for (size_t c = 0; c < max_cnt; c++) {
        output_csvfile << c << ",";
        for (int i = 0; i < num_list_size; i++) {
            if (c < measure_num_list[i].mn->cnt) {
                output_csvfile << measure_num_list[i].mn->num[c] << ",";
            }
            else {
                output_csvfile << ",";
            }
        }
        for (int i = 0; i < time_list_size; i++) {
            if (c < measure_time_list[i].mt->cnt && c < measure_time_list[i].mt->elapsed_time.size()) {
                output_csvfile << measure_time_list[i].mt->elapsed_time[c] << ",";
            }
            else {
                output_csvfile << ",";
            }
        }
        output_csvfile << std::endl;
    }

    output_csvfile.close();
}
#else  /* !ENABLE_VTUNE_PROF */
void profile_time_reset(void) {
    // todo
}
void profile_time_init(void) {
    // todo
}
void profile_time_start(struct measure_time_t& mt) {
    // todo
}
void profile_time_end(struct measure_time_t& mt) {
    // todo
}
void profile_time_sum_start(struct measure_time_t& mt) {
    // todo
}
void profile_time_sum_end(struct measure_time_t& mt) {
    // todo
}
void profile_time_remove_last(struct measure_time_t& mt) {
    // todo
}
void profile_time_print(void) {
    // todo
}
#endif /* !ENABLE_VTUNE_PROF */

#else /* MEASURE_TIME_DECLARE_BODY*/
void measure_time_reset(void);
void measure_time_init(void);
void measure_time_start(struct measure_time_t& mt);
void measure_time_end(struct measure_time_t& mt);
void measure_time_sum_start(struct measure_time_t& mt);
void measure_time_sum_end(struct measure_time_t& mt);
void measure_time_push_back_zero(struct measure_time_t& mt);
void measure_time_remove_last(struct measure_time_t& mt);
void measure_counts(struct measure_num_t& mn, uint32_t size);
void measure_time_print(void);
void profile_time_reset(void);
void profile_time_init(void);
void profile_time_start(struct measure_time_t& mt);
void profile_time_end(struct measure_time_t& mt);
void profile_time_sum_start(struct measure_time_t& mt);
void profile_time_sum_end(struct measure_time_t& mt);
void profile_time_remove_last(struct measure_time_t& mt);
void profile_time_print(void);

#endif /* MEASURE_TIME_DECLARE_BODY*/

#if defined(ENABLE_MEASURE_TIME)
#ifndef ENABLE_VTUNE_PROF
#define MT_RESET() measure_time_reset()
#define MT_INIT() measure_time_init()
#define MT_START(mt) measure_time_start(mt)
#define MT_FINISH(mt) measure_time_end(mt)
#define MT_SUM_START(mt) measure_time_sum_start(mt)
#define MT_SUM_FINISH(mt) measure_time_sum_end(mt)
#define MT_PUSH_BACK_ZERO(mt) measure_time_push_back_zero(mt)
#define MT_REMOVE_LAST(mt) measure_time_remove_last(mt)
#define MT_COUNT(mn, size) measure_counts(mn, size)
#define MT_PRINT() measure_time_print()
#else /* !ENABLE_VTUNE_PROF */
#define MT_RESET() profile_time_reset()
#define MT_INIT() profile_time_init()
#define MT_START(mt) profile_time_start(mt)
#define MT_FINISH(mt) profile_time_end(mt)
#define MT_SUM_START(mt) profile_time_sum_start(mt)
#define MT_SUM_FINISH(mt) profile_time_sum_end(mt)
#define MT_PUSH_BACK_ZERO(mt) // To be defined in the future
#define MT_REMOVE_LAST(mt) profile_time_remove_last(mt)
#define MT_COUNT(mn, size) profile_counts(mn, size)
#define MT_PRINT() profile_time_print()
#endif /* !ENABLE_VTUNE_PROF */

#else /* ENABLE_MEASURE_TIME */
#define MT_RESET()
#define MT_INIT()
#define MT_START(mt)
#define MT_FINISH(mt)
#define MT_SUM_START(mt)
#define MT_SUM_FINISH(mt)
#define MT_PUSH_BACK_ZERO(mt)
#define MT_REMOVE_LAST(mt)
#define MT_COUNT(mn, size)
#define MT_PRINT()
#endif /* ENABLE_MEASURE_TIME */
