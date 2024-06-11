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
 * File Name    : recognize_base.cpp
 * Version      : 7.30
 * Description  : RZ/V DRP-AI Sample Application for USB Camera version
 ***********************************************************************************************************************/

/*****************************************
 * Includes
 ******************************************/
#include "recognize_base.h"
#include "../command/object_detection.h"
#include "common/recognize_define.h"

#include "measure_time.h"

/**
 * @brief RecognizeBase
 * @details  Construct a new Recognize Base:: Recognize Base object
 */
RecognizeBase::RecognizeBase(const int32_t drp_max_freq,
                             const int32_t drpai_freq)
    : drp_max_freq_(drp_max_freq), drpai_freq_(drpai_freq) {
    _drp = make_shared<DRPProc>();

    _pthread_ai_inf = 0;
}

/**
 * @brief initialize
 * @details Initialization for recognize process.
 * @param model DRP-AI Model
 * @return int32_t success:0 error: != 0
 */
int32_t RecognizeBase::initialize(IRecognizeModel* model) {
    std::cout << "############ INIT ############" << std::endl;
    _model = shared_ptr<IRecognizeModel>(move(model));

    std::cout << "DRP PREFIX:" << _model->model_prefix << std::endl;
    std::cout << "outbuff :" << _model->outBuffSize << std::endl;

    _outBuffSize = _model->outBuffSize;
    dir = _model->model_dir + "/";
    address_file = dir + "addr_map.txt";

    return 0;
}

/**
 * @brief recognize_start
 * @details load drp data / start threads
 * @return int32_t success:0 error: != 0
 */
int32_t RecognizeBase::recognize_start() {
    /*DRP-AI Driver Open*/
    int8_t ret = _drp->drp_open();

    // Get DRP-AI Memory Area Address via DRP-AI Driver
    drpai_data_t drp_ai_area;
    errno = 0;
    ret = ioctl(_drp->_drpai_fd, DRPAI_GET_DRPAI_AREA, &drp_ai_area);
    if (0 != ret) {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_GET_DRPAI_AREA: errno=%d\n", errno);
        return ret;
    }

    printf("RZ/V DRP-AI Sample Application\n");
    printf("Model :  %s\n", _model->model_prefix.c_str());

    std::cout << "file:" << address_file.c_str() << std::endl;
    ret = _drp->read_addrmap_txt(address_file, &drpai_address, drp_ai_area.address);

    /* Set frequency case of sparse model */
    if (0 <= drp_max_freq_) {
        /* freq = 1260 / (mindiv + 1) [MHz]                                             */
        /* default: mindiv = 2 (420MHz)                                                 */
        /*  2:420MHz,  3:315MHz,  4:252MHz,  5:210MHz,  6:180MHz,  7:158MHz,  8:140MHz  */
        /*  9:126MHz, 10:115MHz, 11:105MHz, 12: 97MHz, 13: 90MHz, 14: 84MHz, 15: 79MHz  */
        /* 16: 74MHz, 17: 70MHz, 18: 66MHz, 19: 63MHz, 20: 60MHz, 21: 57MHz, 22: 55MHz  */
        /* 23: 53MHz, 24: 50MHz, 25: 48MHz, 26: 47MHz, 27: 45MHz, 28: 43MHz, 29: 42MHz  */

        uint32_t mindiv = (uint32_t)drp_max_freq_;
        ret = _drp->drp_set_drp_max_freq(mindiv);
        if (0 <= ret) {
            printf("Setting DRP Max freq. mindiv=%d\n", mindiv);
        }
        else {
            fprintf(stderr, "[ERROR] Failed to set DRP Max freq.\n");
        }
    }

    if (0 <= drpai_freq_) {
        /* divfix = 1, 2: freq = 1000MHz, freq = 1260 / (divfix - 1) [MHz] (divfix > 2) */
        /* default: divfix = 1 (1000MHz)                                                */
        /*  3:630MHz,  4:420MHz,  5:315MHz,  6:252MHz,  7:210MHz,  8:180MHz,  9:158MHz  */
        /* 10:140MHz, 11:126MHz, 12:115MHz, 13:105MHz, 14: 97MHz, 15: 90MHz, 16: 84MHz  */
        /* uint32_t divfix = 6; */ /* 252MHz original/4 */

        uint32_t divfix = (uint32_t)drpai_freq_;

        ret = _drp->drp_set_drpai_freq(divfix);
        if (0 <= ret) {
            printf("Setting DRP-AI freq. divfix=%d\n", divfix);
        }
        else {
            fprintf(stderr, "[ERROR] Failed to set DRP-AI freq.\n");
        }
    }

    /* Load DRP-AI Data from Filesystem to Memory via DRP-AI Driver */
    drpai_file_path[0] = dir + "/drp_desc.bin";
    drpai_file_path[1] = dir + "/drp_config.mem";
    drpai_file_path[2] = dir + "/drp_param.bin";
    drpai_file_path[3] = dir + "/aimac_desc.bin";
    drpai_file_path[4] = dir + "/weight.bin";
    drpai_file_path[5] = dir + "/aimac_cmd.bin";
    drpai_file_path[6] = dir + "/aimac_param_desc.bin";
    drpai_file_path[7] = dir + "/aimac_param_cmd.bin";

    ret = _drp->load_drpai_data(drpai_address, drpai_file_path);

    if (ret != 0) {
        fprintf(stderr, "[ERROR] Failed to Load DRP-AI Data.\n");
        return -1;
    }

    set_input.store(false);
    inference_running.store(false);

    /* inference thread */
    inference_thread_running_ = true;
    int32_t create_thread_ai = pthread_create(&_pthread_ai_inf, NULL, inference_thread, this);
    if (0 != create_thread_ai) {
        fprintf(stderr, "[ERROR] Failed to create AI Inference Thread.\n");
        return -1;
    }

    return 0;
}
/**
 * @brief recognize_end
 * @details Recognize end proc
 */
void RecognizeBase::recognize_end() {
    end_all_threads();
    if (_drp)
        _drp->drp_close();

    std::cout << "********************** END *********************" << std::endl;
}

/**
 * @brief get_data_in_addr
 * @details return drpai_address.data_in_addr
 */
uint64_t RecognizeBase::get_data_in_addr() {
    return drpai_address.data_in_addr;
}

/**
 * @brief get_data_in_size
 * @details return drpai_address.data_in_size
 */
uint32_t RecognizeBase::get_data_in_size() {
    return drpai_address.data_in_size;
}

/**
 * @brief inference thread
 * @details inference and send results
 * @param arg pointer to itself
 * @return void*
 */
void* RecognizeBase::inference_thread(void* arg) {
    RecognizeBase* me = (RecognizeBase*)arg;

    /*Variable for getting Inference output data*/
    drpai_data_t drpai_data;
    /*Inference Variables*/
    int8_t inf_status = 0;
    drpai_data_t proc[DRPAI_INDEX_NUM];
    int32_t inf_cnt = -1;
    drpai_status_t drpai_status;
    /*Variable for checking return value*/
    int8_t ret = 0;

    printf("Inference Thread Starting\n");

    uint64_t udmabuf_address = 0;
    // Please refer to recognize_base.cpp provided October 13, 2023
    {
        /* Obtain udmabuf memory area starting address */
        int8_t fd = 0;
        char addr[1024];
        int32_t read_ret = 0;

        errno = 0;
        fd = open("/sys/class/u-dma-buf/udmabuf0/phys_addr", O_RDONLY);
        if (0 > fd) {
            fprintf(stderr, "[ERROR] Failed to open udmabuf0/phys_addr : errno=%d\n", errno);
            return NULL;
        }
        read_ret = read(fd, addr, 1024);
        if (0 > read_ret) {
            fprintf(stderr, "[ERROR] Failed to read udmabuf0/phys_addr : errno=%d\n", errno);
            close(fd);
            return NULL;
        }
        sscanf(addr, "%lx", &udmabuf_address);
        close(fd);

        udmabuf_address &= 0xFFFFFFFF;
    }

    proc[DRPAI_INDEX_INPUT].address = udmabuf_address;
    proc[DRPAI_INDEX_INPUT].size = me->drpai_address.data_in_size;
    proc[DRPAI_INDEX_DRP_CFG].address = me->drpai_address.drp_config_addr;
    proc[DRPAI_INDEX_DRP_CFG].size = me->drpai_address.drp_config_size;
    proc[DRPAI_INDEX_DRP_PARAM].address = me->drpai_address.drp_param_addr;
    proc[DRPAI_INDEX_DRP_PARAM].size = me->drpai_address.drp_param_size;
    proc[DRPAI_INDEX_AIMAC_DESC].address = me->drpai_address.desc_aimac_addr;
    proc[DRPAI_INDEX_AIMAC_DESC].size = me->drpai_address.desc_aimac_size;
    proc[DRPAI_INDEX_DRP_DESC].address = me->drpai_address.desc_drp_addr;
    proc[DRPAI_INDEX_DRP_DESC].size = me->drpai_address.desc_drp_size;
    proc[DRPAI_INDEX_WEIGHT].address = me->drpai_address.weight_addr;
    proc[DRPAI_INDEX_WEIGHT].size = me->drpai_address.weight_size;
    proc[DRPAI_INDEX_OUTPUT].address = me->drpai_address.data_out_addr;
    proc[DRPAI_INDEX_OUTPUT].size = me->drpai_address.data_out_size;
    proc[DRPAI_INDEX_AIMAC_CMD].address = me->drpai_address.aimac_cmd_addr;
    proc[DRPAI_INDEX_AIMAC_CMD].size = me->drpai_address.aimac_cmd_size;
    proc[DRPAI_INDEX_AIMAC_PARAM_DESC].address = me->drpai_address.aimac_param_desc_addr;
    proc[DRPAI_INDEX_AIMAC_PARAM_DESC].size = me->drpai_address.aimac_param_desc_size;
    proc[DRPAI_INDEX_AIMAC_PARAM_CMD].address = me->drpai_address.aimac_param_cmd_addr;
    proc[DRPAI_INDEX_AIMAC_PARAM_CMD].size = me->drpai_address.aimac_param_cmd_size;

    fprintf(stderr, "proc[DRPAI_INDEX_INPUT].address            : 0x%10lx\n", proc[DRPAI_INDEX_INPUT].address);
    fprintf(stderr, "proc[DRPAI_INDEX_INPUT].size               : %lu\n", proc[DRPAI_INDEX_INPUT].size);
    fprintf(stderr, "proc[DRPAI_INDEX_DRP_CFG].address          : 0x%10lx\n", proc[DRPAI_INDEX_DRP_CFG].address);
    fprintf(stderr, "proc[DRPAI_INDEX_DRP_CFG].size             : %lu\n", proc[DRPAI_INDEX_DRP_CFG].size);
    fprintf(stderr, "proc[DRPAI_INDEX_DRP_PARAM].address        : 0x%10lx\n", proc[DRPAI_INDEX_DRP_PARAM].address);
    fprintf(stderr, "proc[DRPAI_INDEX_DRP_PARAM].size           : %lu\n", proc[DRPAI_INDEX_DRP_PARAM].size);
    fprintf(stderr, "proc[DRPAI_INDEX_AIMAC_DESC].address       : 0x%10lx\n", proc[DRPAI_INDEX_AIMAC_DESC].address);
    fprintf(stderr, "proc[DRPAI_INDEX_AIMAC_DESC].size          : %lu\n", proc[DRPAI_INDEX_AIMAC_DESC].size);
    fprintf(stderr, "proc[DRPAI_INDEX_DRP_DESC].address         : 0x%10lx\n", proc[DRPAI_INDEX_DRP_DESC].address);
    fprintf(stderr, "proc[DRPAI_INDEX_DRP_DESC].size            : %lu\n", proc[DRPAI_INDEX_DRP_DESC].size);
    fprintf(stderr, "proc[DRPAI_INDEX_WEIGHT].address           : 0x%10lx\n", proc[DRPAI_INDEX_WEIGHT].address);
    fprintf(stderr, "proc[DRPAI_INDEX_WEIGHT].size              : %lu\n", proc[DRPAI_INDEX_WEIGHT].size);
    fprintf(stderr, "proc[DRPAI_INDEX_OUTPUT].address           : 0x%10lx\n", proc[DRPAI_INDEX_OUTPUT].address);
    fprintf(stderr, "proc[DRPAI_INDEX_OUTPUT].size              : %lu\n", proc[DRPAI_INDEX_OUTPUT].size);
    fprintf(stderr, "proc[DRPAI_INDEX_AIMAC_CMD].address        : 0x%10lx\n", proc[DRPAI_INDEX_AIMAC_CMD].address);
    fprintf(stderr, "proc[DRPAI_INDEX_AIMAC_CMD].size           : %lu\n", proc[DRPAI_INDEX_AIMAC_CMD].size);
    fprintf(stderr, "proc[DRPAI_INDEX_AIMAC_PARAM_DESC].address : 0x%10lx\n", proc[DRPAI_INDEX_AIMAC_PARAM_DESC].address);
    fprintf(stderr, "proc[DRPAI_INDEX_AIMAC_PARAM_DESC].size    : %lu\n", proc[DRPAI_INDEX_AIMAC_PARAM_DESC].size);
    fprintf(stderr, "proc[DRPAI_INDEX_AIMAC_PARAM_CMD].address  : 0x%10lx\n", proc[DRPAI_INDEX_AIMAC_PARAM_CMD].address);
    fprintf(stderr, "proc[DRPAI_INDEX_AIMAC_PARAM_CMD].size     : %lu\n", proc[DRPAI_INDEX_AIMAC_PARAM_CMD].size);

    /*DRP-AI Output Memory Preparation*/
    drpai_data.address = me->drpai_address.data_out_addr;
    drpai_data.size = me->drpai_address.data_out_size;

    printf("Inference Loop Starting\n");

    vector<uint8_t> inputimage;
    recognizeData_t data;

    errno = 0;
    ret = me->_drp->drp_adrconv(&proc[0]);
    if (0 != ret) {
        fprintf(stderr, "[ERROR] Failed to drp_adrconv: errno=%d\n", errno);
        return NULL;
    }

    /*Inference Loop Start*/
    while (me->inference_thread_running_) {
        /*Start DRP-AI Driver*/

        while (!me->set_input.load()) {
            usleep(1000); // sleep 1ms

            if (!me->inference_thread_running_)
                break;
        }

        MT_START(mt_recognize_thread);

        if (!me->inference_thread_running_)
            break;

        me->inference_running.store(true);

        errno = 0;
        ret = me->_drp->drp_start(&proc[0]);
        if (0 != ret) {
            fprintf(stderr, "[ERROR] Failed to drp_start: errno=%d\n", errno);
            break;
        }

        inf_cnt++;

        MT_START(mt_recognize_polling);

        ret = me->_drp->drp_pselect(NULL);
        if (0 == ret) {
            fprintf(stderr, "[ERROR] DRP-AI Inference pselect() Timeout: errno=%d\n", errno);
            break;
        }
        else if (0 > ret) {
            /*Checks if DRPAI pselect ended without issue*/
            fprintf(stderr, "[ERROR] DRP-AI Inference pselect() Error: errno=%d\n", errno);
            // ret = ioctl(me->drpai_fd, DRPAI_GET_STATUS, &drpai_status);
            ret = me->_drp->drp_getStatus(&drpai_status);
            if (-1 == ret) {
                fprintf(stderr, "[ERROR] Failed to drp_getStatus : errno=%d\n", errno);
                break;
            }
        }
        else {
            /*Do nothing*/
        }

        /*Checks if DRPAI Inference ended without issue*/
        // inf_status = ioctl(me->drpai_fd, DRPAI_GET_STATUS, &drpai_status);
        inf_status = me->_drp->drp_getStatus(&drpai_status);

        MT_FINISH(mt_recognize_polling);

        if (0 == inf_status) {
            shared_ptr<float> drpai_output_buf;

            {
                /*Process to read the DRPAI output data.*/
                drpai_output_buf.reset(new float[me->_outBuffSize], std::default_delete<float[]>());
                ret = me->get_result(me->_drp, drpai_output_buf, drpai_data.address, drpai_data.size);
                if (0 != ret) {
                    fprintf(stderr, "[ERROR] Failed to get_result.\n");
                    break;
                }
            }

            data.predict_image = me->input_data;
            data.predict_result = move(drpai_output_buf);

            me->inference_postprocess(arg, data);

            me->set_input.store(false);
            me->inference_running.store(false);
        }
        else {
            /* inf_status != 0 */
            fprintf(stderr, "[ERROR] DRPAI Internal Error: errno=%d\n", errno);
            break;
        }

        MT_FINISH(mt_recognize_thread);
    }
    /*End of Inference Loop*/

    /*To terminate the loop in _capture Thread.*/
    me->set_input.store(false);
    me->inference_running.store(false);

    cout << "<<<<<<<<<<<<<<<<<<<<< AI Inference Thread Terminated >>>>>>>>>>>>>>>>>>" << endl;
    pthread_exit(NULL);
    me->_pthread_ai_inf = 0;
    return NULL;
}

/**
 * @brief inference_postprocess
 * @details postprocess and send command
 * @param arg pointer to itself
 * @param data inference result data
 */
void RecognizeBase::inference_postprocess(void* arg, recognizeData_t& data) {
    _model->inf_post_process(data.predict_result.get());

    string b64;
    shared_ptr<PredictNotifyBase> notify;

#ifdef COUT_INFERENCE_RESULT_ON
    _model->print_result();
#endif
    {
        notify = _model->get_command();
    }

    {
        /* Create websocket command*/
        notify->img = b64;
        notify->img_org_w = _model->_capture_w;
        notify->img_org_h = _model->_capture_h;
        notify->drp_time = data.drp_time_ms;
    }

    data.predict_result.reset();
}

/**
 * @brief Get DRP-AI inference result
 * @details description
 * @param drp pointer to DRP class
 * @param[out] drpOutBuf pointer to store result
 * @param output_addr DRP output address
 * @param output_size DRP output sizze
 * @return int8_t success:0
 */
int8_t RecognizeBase::get_result(shared_ptr<DRPProc> drp, shared_ptr<float> drpOutBuf, uint64_t output_addr, uint32_t output_size) {
    drpai_data_t drpai_data;
    float drpai_buf[BUF_SIZE];
    drpai_data.address = output_addr;
    drpai_data.size = output_size;
    int32_t i = 0;
    int8_t ret = 0;

    errno = 0;
    /* Assign the memory address and size to be read */
    ret = drp->drp_assign(&drpai_data);
    if (-1 == ret) {
        fprintf(stderr, "[ERROR] Failed to drp_assign: errno=%d\n", errno);
        return -1;
    }

    float outsize_bufsize = drpai_data.size / BUF_SIZE;

    /* Read the memory via DRP-AI Driver and store the output to buffer */
    for (i = 0; i < outsize_bufsize; i++) {
        errno = 0;
        ret = drp->drp_read(drpai_buf, BUF_SIZE);
        if (-1 == ret) {
            fprintf(stderr, "[ERROR] Failed to drp_read: errno=%d\n", errno);
            return -1;
        }
        memcpy(&(drpOutBuf.get()[BUF_SIZE / sizeof(float) * i]), drpai_buf, BUF_SIZE);
    }

    if (0 != (drpai_data.size % BUF_SIZE)) {
        errno = 0;
        ret = drp->drp_read(drpai_buf, (drpai_data.size % BUF_SIZE));
        if (-1 == ret) {
            fprintf(stderr, "[ERROR] Failed to drp_read: errno=%d\n", errno);
            return -1;
        }
        memcpy(&(drpOutBuf.get()[(drpai_data.size - (drpai_data.size % BUF_SIZE)) / sizeof(float)]), drpai_buf, (drpai_data.size % BUF_SIZE));
    }
    return 0;
}

/**
 * @brief end_all_threads
 * @details terminate all threads.
 * @return int32_t success :0
 */
int32_t RecognizeBase::end_all_threads() {
    int32_t ret;
    int32_t ret_main;
    set_input.store(false);
    inference_running.store(false);

    inference_thread_running_ = false;

    if (0 != _pthread_ai_inf) {
        ret = wait_join(&_pthread_ai_inf, AI_THREAD_TIMEOUT);
        if (0 != ret) {
            fprintf(stderr, "[ERROR] Failed to exit AI Inference Thread on time.[%d]\n", ret);
            ret_main = -1;
        }
    }

    std::cout << "********************** ALL THREAD END *********************" << std::endl;

    return ret_main;
}

/**
 * @brief wait_join
 * @details wait for thread
 * @param p_join_thread target thread.
 * @param join_time timeout time[sec]
 * @return int8_t success:0
 */
int8_t RecognizeBase::wait_join(pthread_t* p_join_thread, uint32_t join_time) {
    int8_t ret_err;
    struct timespec join_timeout;
    ret_err = clock_gettime(CLOCK_REALTIME, &join_timeout);
    if (0 == ret_err) {
        join_timeout.tv_sec += join_time;
        ret_err = pthread_timedjoin_np(*p_join_thread, NULL, &join_timeout);
    }
    return ret_err;
}
/**
 * @brief timedifference_msec
 * @details calc time diffrence t0 and t1
 * @param t0 time (start)
 * @param t1 time (end)
 * @return double diff time[ms]
 */
double RecognizeBase::timedifference_msec(timespec t0, timespec t1) {
    return (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0;
}

/**
 * @brief get current time
 * @details description
 * @param[out] time_t reference to store current time
 * @return int32_t success:0
 */
int32_t RecognizeBase::get_time(timespec& time_t) {
    int32_t ret = timespec_get(&time_t, TIME_UTC);
    if (0 == ret) {
        fprintf(stderr, "[ERROR] Failed to get Inference Start Time\n");
    }
    return ret;
}
