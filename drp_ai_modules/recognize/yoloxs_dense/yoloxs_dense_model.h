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
 * File Name    : yoloxs_dense_model.h
 * Version      : -
 * Description  : RZ/V DRP-AI Sample Application for USB Camera version
 ***********************************************************************************************************************/

#pragma once
#ifndef YOLOXS_DENSE_MODEL_H
#define YOLOXS_DENSE_MODEL_H

/*****************************************
 * Includes
 ******************************************/
#include <linux/drpai.h>
#include "../irecognize_model.h"
#include "../../includes.h"
#include "../command/object_detection.h"
#include "../common/box.h"
#include "../common/functions.h"
#include "../common/yolo_common.h"

class YoloXSDenseModel : public IRecognizeModel {
private:
    constexpr static string_view MODEL_NAME = "YOLOX_S_dense_640x640_RGB_10271351";
    constexpr static int32_t YOLOXS_DENSE_NUM_BB = 1;
    /* Number of output layers. This value MUST match with the length of num_grids[] below */
    constexpr static int32_t YOLOXS_DENSE_NUM_INF_OUT_LAYER = 3;
    /* Thresholds */
    constexpr static float YOLOXS_DENSE_TH_PROB = 0.5f;
    constexpr static float YOLOXS_DENSE_TH_NMS = 0.5f;
    /* Size of input image to the model */
    constexpr static int32_t YOLOXS_DENSE_MODEL_IN_W = 640;
    constexpr static int32_t YOLOXS_DENSE_MODEL_IN_H = 640;

    constexpr static string_view MODEL_DIR = "YOLOX_S_dense_640x640_RGB_10271351";
    constexpr static int32_t YOLOXS_DENSE_DRPAI_IN_WIDTH = 640;
    constexpr static int32_t YOLOXS_DENSE_DRPAI_IN_HEIGHT = 640;

public:
    YoloXSDenseModel();
    virtual int32_t inf_post_process(float* arg);
    virtual shared_ptr<PredictNotifyBase> get_command();
    virtual shared_ptr<ObjectDetection> get_object_detection();
    virtual int32_t print_result();

private:
    void post_proc(float* floatarr, std::vector<detection>& det);
    /*****************************************
     * YOLOv2
     ******************************************/
    /* Empty since labels will be loaded from label_list file */
    std::vector<std::string> label_file_map = {};

    /* Number of grids in the image. The length of this array MUST match with the NUM_INF_OUT_LAYER */
    vector<uint8_t> num_grids;
    /* Number of DRP-AI output */
    uint32_t num_inf_out;
    /* Anchor box information */
    vector<double> anchors;
    vector<detection> postproc_data;
};

#endif // !YOLOXS_DENSE_MODEL_H
