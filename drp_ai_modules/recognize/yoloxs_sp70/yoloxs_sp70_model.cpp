/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2023 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : yoloxs_sp70_model.cpp
 * Version      : -
 * Description  : RZ/V DRP-AI Sample Application for USB Camera version
 ***********************************************************************************************************************/

/*****************************************
 * Includes
 ******************************************/
#include "yoloxs_sp70_model.h"

YoloXSSp70Model::YoloXSSp70Model()
    : IRecognizeModel(MODEL_DIR.data(), MODEL_NAME.data(), YOLOXS_SP70_DRPAI_IN_WIDTH, YOLOXS_SP70_DRPAI_IN_HEIGHT, 3) {
    std::cout << "Yolo model" << std::endl;

    num_grids = {80, 40, 20};
    anchors = {
        1.08, 1.19,
        3.42, 4.41,
        6.63, 11.38,
        9.42, 5.11,
        16.62, 10.52};
    label_file_map = {"aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};
    num_inf_out = (label_file_map.size() + 5) * YOLOXS_SP70_NUM_BB * num_grids[0] * num_grids[0]
                  + (label_file_map.size() + 5) * YOLOXS_SP70_NUM_BB * num_grids[1] * num_grids[1]
                  + (label_file_map.size() + 5) * YOLOXS_SP70_NUM_BB * num_grids[2] * num_grids[2];
    outBuffSize = num_inf_out;
}

/**
 * @brief inf_post_process
 * @details implementation post process
 * @param arg
 * @return int32_t
 */
int32_t YoloXSSp70Model::inf_post_process(float* arg) {
    /*CPU Post-Processing for YOLOv2*/
    postproc_data.clear();
    post_proc(arg, postproc_data);
    return 0;
}

/**
 * @brief get_object_detection
 * @details implementation object detection
 * @return shared_ptr<ObjectDetection>
 */
shared_ptr<ObjectDetection> YoloXSSp70Model::get_object_detection() {
    ObjectDetection* ret = new ObjectDetection();
    for (detection det : postproc_data) {
        if (det.prob == 0) {
            continue;
        }
        else {
            bbox_t dat;
            dat.name = label_file_map[det.c].c_str();
            dat.X = (int32_t)(det.bbox.x - (det.bbox.w / 2));
            dat.Y = (int32_t)(det.bbox.y - (det.bbox.h / 2));
            dat.W = (int32_t)det.bbox.w;
            dat.H = (int32_t)det.bbox.h;
            dat.pred = det.prob * 100.0;

            ret->predict.push_back(dat);
        }
    }
    return std::shared_ptr<ObjectDetection>(move(ret));
}

/**
 * @brief get_command
 * @details implementation create command
 * @return shared_ptr<PredictNotifyBase>
 */
shared_ptr<PredictNotifyBase> YoloXSSp70Model::get_command() {
    return shared_ptr<PredictNotifyBase>(move(get_object_detection()));
}

/**
 * @brief print_result
 * @details implementation print postprocess result
 * @return int32_t
 */
int32_t YoloXSSp70Model::print_result() {
    /*Displays AI Inference results & Processing Time on console*/
    YoloCommon::print_boxes(postproc_data, label_file_map);
    return 0;
}

/**
 * @brief post_proc
 * @details NN output to bouding box
 * @param floatarr DRP-AI result
 * @param det detected boundig box list
 */
void YoloXSSp70Model::post_proc(float* floatarr, std::vector<detection>& det) {
    // Please refer to R_Post_Proc implemented in app_yolox_cam/src/main.cpp

    /* Following variables are required for correct_region_boxes in Darknet implementation*/
    /* Note: This implementation refers to the "darknet detector test" */
    float new_w, new_h;
    float correct_w = 1.;
    float correct_h = 1.;
    if ((float)(YOLOXS_SP70_MODEL_IN_W / correct_w) < (float)(YOLOXS_SP70_MODEL_IN_H / correct_h)) {
        new_w = (float)YOLOXS_SP70_MODEL_IN_W;
        new_h = correct_h * YOLOXS_SP70_MODEL_IN_W / correct_w;
    }
    else {
        new_w = correct_w * YOLOXS_SP70_MODEL_IN_H / correct_h;
        new_h = YOLOXS_SP70_MODEL_IN_H;
    }

    uint32_t label_num = label_file_map.size();

    // YOLOX-S
    int stride = 0;
    std::vector<int> strides = {8, 16, 32};

    /*Post Processing Start*/
    for (uint32_t n = 0; n < YOLOXS_SP70_NUM_INF_OUT_LAYER; n++) {
        uint8_t num_grid = num_grids[n];

        for (uint32_t b = 0; b < YOLOXS_SP70_NUM_BB; b++) {
            stride = strides[n];
            for (uint32_t y = 0; y < num_grid; y++) {
                for (uint32_t x = 0; x < num_grid; x++) {
                    uint32_t offs = YoloCommon::yolo_offset(n, b, y, x, num_grids.data(), YOLOXS_SP70_NUM_BB, label_file_map.size());
                    double tc = floatarr[YoloCommon::yolo_index(num_grid, offs, 4)];

                    double objectness = tc;

                    float max_pred = 0;
                    int8_t pred_class = -1;
                    if (objectness > YOLOXS_SP70_TH_PROB) {
                        /* Get the class prediction */
                        float classes[label_num];
                        for (uint32_t i = 0; i < label_num; i++) {
                            classes[i] = floatarr[YoloCommon::yolo_index(num_grid, offs, 5 + i)];
                        }

                        for (uint32_t i = 0; i < label_num; i++) {
                            if (classes[i] > max_pred) {
                                pred_class = i;
                                max_pred = classes[i];
                            }
                        }
                    }

                    /* Store the result into the list if the probability is more than the threshold */
                    float probability = max_pred * objectness;
                    if (probability > YOLOXS_SP70_TH_PROB) {
                        float tx = floatarr[offs];
                        float ty = floatarr[YoloCommon::yolo_index(num_grid, offs, 1)];
                        float tw = floatarr[YoloCommon::yolo_index(num_grid, offs, 2)];
                        float th = floatarr[YoloCommon::yolo_index(num_grid, offs, 3)];

                        /* Compute the bounding box */
                        /*get_yolo_box/get_region_box in paper implementation*/
                        float center_x = (tx + float(x)) * stride;
                        float center_y = (ty + float(y)) * stride;
                        center_x = center_x / (float)YOLOXS_SP70_MODEL_IN_W;
                        center_y = center_y / (float)YOLOXS_SP70_MODEL_IN_H;
                        float box_w = exp(tw) * stride;
                        float box_h = exp(th) * stride;
                        box_w = box_w / (float)YOLOXS_SP70_MODEL_IN_W;
                        box_h = box_h / (float)YOLOXS_SP70_MODEL_IN_H;

                        /* Adjustment for size */
                        /* correct_yolo/region_boxes */
                        center_x = (center_x - (YOLOXS_SP70_MODEL_IN_W - new_w) / 2. / YOLOXS_SP70_MODEL_IN_W) / ((float)new_w / YOLOXS_SP70_MODEL_IN_W);
                        center_y = (center_y - (YOLOXS_SP70_MODEL_IN_H - new_h) / 2. / YOLOXS_SP70_MODEL_IN_H) / ((float)new_h / YOLOXS_SP70_MODEL_IN_H);
                        box_w *= (float)(YOLOXS_SP70_MODEL_IN_W / new_w);
                        box_h *= (float)(YOLOXS_SP70_MODEL_IN_H / new_h);

                        center_x = round(center_x * YOLOXS_SP70_DRPAI_IN_WIDTH);
                        center_y = round(center_y * YOLOXS_SP70_DRPAI_IN_HEIGHT);
                        box_w = round(box_w * YOLOXS_SP70_DRPAI_IN_WIDTH);
                        box_h = round(box_h * YOLOXS_SP70_DRPAI_IN_HEIGHT);

                        Box bb = {center_x, center_y, box_w, box_h};
                        detection d = {bb, pred_class, probability};
                        det.push_back(d);
                    }
                }
            }
        }
    }
    /* Non-Maximum Supression filter */
    filter_boxes_nms(det, det.size(), YOLOXS_SP70_TH_NMS);
}
