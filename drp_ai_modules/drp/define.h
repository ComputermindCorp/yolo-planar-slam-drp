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
 * File Name    : define.h
 * Version      : 7.30
 * Description  : RZ/V DRP-AI Sample Application for PyTorch ResNet USB Camera version
 ***********************************************************************************************************************/

#ifndef DRP_DEFINE_MACRO_H
#define DRP_DEFINE_MACRO_H

/*****************************************
 * includes
 ******************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <errno.h>
#include <vector>
#include <map>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <atomic>
#include <semaphore.h>

/*****************************************
 * Macro for Application
 ******************************************/
/*Maximum DRP-AI Timeout threshold*/
#define DRPAI_TIMEOUT (5)

/*Buffer size for writing data to memory via DRP-AI Driver.*/
#define BUF_SIZE (1024)
/*Number of bytes for single FP32 number in DRP-AI.*/
#define NUM_BYTE (4)

/*Index to access drpai_file_path[]*/
#define INDEX_D (0)
#define INDEX_C (1)
#define INDEX_P (2)
#define INDEX_A (3)
#define INDEX_W (4)
#define INDEX_AC (5)
#define INDEX_AP (6)
#define INDEX_APC (7)

/*****************************************
 * Typedef
 ******************************************/
/* For DRP-AI Address List */
typedef struct
{
    uint64_t desc_aimac_addr;
    uint64_t desc_aimac_size;
    uint64_t desc_drp_addr;
    uint64_t desc_drp_size;
    uint64_t drp_param_addr;
    uint64_t drp_param_size;
    uint64_t data_in_addr;
    uint64_t data_in_size;
    uint64_t data_addr;
    uint64_t data_size;
    uint64_t work_addr;
    uint64_t work_size;
    uint64_t data_out_addr;
    uint64_t data_out_size;
    uint64_t drp_config_addr;
    uint64_t drp_config_size;
    uint64_t weight_addr;
    uint64_t weight_size;
    uint64_t aimac_cmd_addr;
    uint64_t aimac_cmd_size;
    uint64_t aimac_param_desc_addr;
    uint64_t aimac_param_desc_size;
    uint64_t aimac_param_cmd_addr;
    uint64_t aimac_param_cmd_size;
} st_addr_t;
#endif
