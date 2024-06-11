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
 * File Name    : recognize_base.h
 * Version      : 7.30
 * Description  : RZ/V DRP-AI Sample Application for USB Camera version
 ***********************************************************************************************************************/

#pragma once
#ifndef RECOGNIZE_BASE_H
#define RECOGNIZE_BASE_H

/*****************************************
 * Includes
 ******************************************/
#include <condition_variable>
#include <mutex>
#include <queue>

#include "../includes.h"
#include "../drp/drp_proc.h"
#include "common/recognize_define.h"
#include "irecognize_model.h"
#include "recognize_data.h"

#define WAIT_TIME (1000) /* microseconds */
/*Timer Related*/
#define CAPTURE_TIMEOUT (20)          /* seconds */
#define AI_THREAD_TIMEOUT (20)        /* seconds */
#define FRAMERATE_THREAD_TIMEOUT (20) /* seconds */

class RecognizeBase {
public:
    RecognizeBase(const int32_t drp_max_freq,
                  const int32_t drpai_freq);
    ~RecognizeBase() {}

    int32_t initialize(IRecognizeModel* model);
    virtual int32_t recognize_start();
    virtual void recognize_end();

    uint64_t get_data_in_addr();
    uint32_t get_data_in_size();

public:
    // variants
    shared_ptr<DRPProc> _drp;

    std::atomic<bool> set_input;
    std::atomic<bool> inference_running;

    shared_ptr<IRecognizeModel> _model;

private:
    static void* inference_thread(void* arg);
    void inference_postprocess(void* arg, recognizeData_t& data);
    int32_t end_all_threads();
    int8_t get_result(shared_ptr<DRPProc> drp, shared_ptr<float> drpOutBuf, uint64_t output_addr, uint32_t output_size);
    int8_t wait_join(pthread_t* p_join_thread, uint32_t join_time);
    double timedifference_msec(struct timespec t0, struct timespec t1);
    int32_t get_time(timespec& time_t);
    string get_send_image(uint8_t* image);

private:
    pthread_t _pthread_ai_inf;

    /**
     * @brief
     *
     */
    int32_t _outBuffSize;
    // for threads
    bool inference_thread_running_;

    std::atomic<bool> inference_start;

    mutex mtx_;
    condition_variable cv_;

    // for capture
    volatile uint64_t capture_address;
    vector<uint8_t> capture_data;
    uint8_t* input_data;

    /*AI Inference for DRPAI*/
    st_addr_t drpai_address;
    std::string dir;
    std::string address_file;

    std::string drpai_address_file;
    std::string drpai_file_path[8];

    const int32_t drp_max_freq_;
    const int32_t drpai_freq_;
};

#endif
