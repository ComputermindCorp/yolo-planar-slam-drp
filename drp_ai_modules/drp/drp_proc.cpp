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
 * File Name    : drp_proc.cpp
 * Version      : 7.30
 * Description  : RZ/V DRP-AI Sample Application for USB Camera version
 ***********************************************************************************************************************/

/*****************************************
 * Includes
 ******************************************/
#include "drp_proc.h"
#include <iostream>
#include <signal.h>

#include <cassert>

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

/**
 * @brief drp_open
 * @details Open DRP-AI driver and store handle.
 * @return int8_t
 */
int8_t DRPProc::drp_open() {
    /*DRP-AI Driver Open*/
    errno = 0;
    _drpai_fd = open("/dev/drpai0", O_RDWR);
    if (0 > _drpai_fd) {
        fprintf(stderr, "[ERROR] Failed to open DRP-AI Driver: errno=%d\n", errno);
        return -1;
    }
    else {
        std::cout << "[OK] DRP-AI Driver Open OK" << std::endl;
    }
    return 0;
}

/**
 * @brief drp_close
 * @details Close DRP-AI driver
 * @return int8_t
 */
int8_t DRPProc::drp_close() {
    /*Close DRP-AI Driver.*/
    if (0 < _drpai_fd) {
        errno = 0;
        int32_t ret = close(_drpai_fd);
        if (0 != ret) {
            fprintf(stderr, "[ERROR] Failed to close DRP-AI Driver: errno=%d\n", errno);
        }
        else {
            _drpai_fd = 0;
        }
        return ret;
    }
    return 0;
}

/**
 * @brief read_addrmap_txt
 * @details Loads address and size of DRP-AI Object files into struct addr.
 * @param addr_file filename of addressmap file (from DRP-AI Object files)
 * @param drpai_address pointer to store address
 * @return int8_t 0 if succeeded
 *                 not 0 otherwise
 */
int8_t DRPProc::read_addrmap_txt(std::string addr_file, st_addr_t* drpai_address, const uint64_t alpha2_first_address) {
    std::string str;
    uint64_t l_addr;
    uint32_t l_size;
    std::string element, a, s;

    std::ifstream ifs(addr_file);
    if (ifs.fail()) {
        fprintf(stderr, "[ERROR] Failed to open address map list : %s\n", addr_file.c_str());
        return -1;
    }

    while (getline(ifs, str)) {
        std::istringstream iss(str);
        iss >> element >> a >> s;
#if 0
        l_addr = strtol(a.c_str(), NULL, 10);
        l_size = strtol(s.c_str(), NULL, 10);
#else
        l_addr = strtol(a.c_str(), NULL, 16);
        l_size = strtol(s.c_str(), NULL, 16);
#endif

        if ("drp_config" == element) {
            drpai_address->drp_config_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->drp_config_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if (("desc_aimac" == element) || ("aimac_desc" == element)) {
            drpai_address->desc_aimac_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->desc_aimac_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if (("desc_drp" == element) || ("drp_desc" == element)) {
            drpai_address->desc_drp_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->desc_drp_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if ("drp_param" == element) {
            drpai_address->drp_param_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->drp_param_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if ("weight" == element) {
            drpai_address->weight_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->weight_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if ("data_in" == element) {
            assert(DRPAI_ALPHA1_FIRST_ADDRESS == l_addr && "The address of data_in must be 0x8000_0000.\n");

            drpai_address->data_in_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->data_in_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if ("data" == element) {
            drpai_address->data_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->data_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if ("data_out" == element) {
            drpai_address->data_out_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->data_out_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if ("work" == element) {
            drpai_address->work_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->work_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if ("aimac_param_cmd" == element) {
            drpai_address->aimac_param_cmd_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->aimac_param_cmd_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if ("aimac_param_desc" == element) {
            drpai_address->aimac_param_desc_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->aimac_param_desc_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else if ("aimac_cmd" == element) {
            drpai_address->aimac_cmd_addr = (l_addr - DRPAI_ALPHA1_FIRST_ADDRESS) + alpha2_first_address;
            drpai_address->aimac_cmd_size = l_size;
            printf("addr:0x%010lX size:0x%08X(%u) addrmap:%s\n", l_addr, l_size, l_size, element.c_str());
        }
        else {
            /*Ignore other space*/
        }
    }

    return 0;
}

/**
 * @brief load_data_to_mem
 * @details Load DRP-AI Data from file.
 * @param data DRP-AI data file path
 * @param from DRP-AI data addrerss
 * @param size Data size
 * @return int8_t
 */
int8_t DRPProc::load_data_to_mem(std::string data, uint64_t from, uint32_t size) {
    int8_t ret_load_data = 0;
    int8_t obj_fd;
    uint8_t drpai_buf[BUF_SIZE];
    drpai_data_t drpai_data;
    uint8_t assign_ret = 0;
    uint8_t rw_ret = 0;
    int32_t i = 0;

    printf("Loading : %s\n", data.c_str());
    errno = 0;
    obj_fd = open(data.c_str(), O_RDONLY);
    if (0 > obj_fd) {
        fprintf(stderr, "[ERROR] Failed to open: %s errno=%d\n", data.c_str(), errno);
        ret_load_data = -1;
        goto end;
    }

    drpai_data.address = from;
    drpai_data.size = size;

    errno = 0;
    assign_ret = drp_assign(&drpai_data);
    if (-1 == assign_ret) {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_ASSIGN: errno=%d\n", errno);
        ret_load_data = -1;
        goto end;
    }

    for (i = 0; i < (int32_t)(drpai_data.size / BUF_SIZE); i++) {
        errno = 0;
        rw_ret = read(obj_fd, drpai_buf, BUF_SIZE);
        if (0 > rw_ret) {
            fprintf(stderr, "[ERROR] Failed to read: %s errno=%d\n", data.c_str(), errno);
            ret_load_data = -1;
            goto end;
        }
        rw_ret = write(_drpai_fd, drpai_buf, BUF_SIZE);
        if (-1 == rw_ret) {
            fprintf(stderr, "[ERROR] Failed to write via DRP-AI Driver: errno=%d\n", errno);
            ret_load_data = -1;
            goto end;
        }
    }
    if (0 != (drpai_data.size % BUF_SIZE)) {
        errno = 0;
        rw_ret = read(obj_fd, drpai_buf, (drpai_data.size % BUF_SIZE));
        if (0 > rw_ret) {
            fprintf(stderr, "[ERROR] Failed to read: %s errno=%d\n", data.c_str(), errno);
            ret_load_data = -1;
            goto end;
        }
        rw_ret = write(_drpai_fd, drpai_buf, (drpai_data.size % BUF_SIZE));
        if (-1 == rw_ret) {
            fprintf(stderr, "[ERROR] Failed to write via DRP-AI Driver: errno=%d\n", errno);
            ret_load_data = -1;
            goto end;
        }
    }
    goto end;

end:
    if (0 < obj_fd) {
        close(obj_fd);
    }
    return ret_load_data;
}

/**
 * @brief load_drpai_data
 * @details Loads DRP-AI Object files to memory via DRP-AI Driver.
 * @param drpai_address
 * @param drpai_file_path
 * @return int8_t
 */
int8_t DRPProc::load_drpai_data(st_addr_t& drpai_address, std::string drpai_file_path[]) {
    std::cout << "load_drpai_data" << std::endl;

    uint64_t addr = 0;
    uint32_t size = 0;
    uint8_t i = 0;
    int8_t ret = 0;
    double diff = 0;

    struct timespec start_time, stop_time;
    /*Start load time*/
    ret = timespec_get(&start_time, TIME_UTC);
    if (0 == ret) {
        fprintf(stderr, "[ERROR] Failed to run timespect_get().\n");
        return -1;
    }
    printf("[START] Loading DRP-AI Data...\n");
    for (i = 0; i < 8; i++) {
        switch (i) {
            case (INDEX_W):
                addr = drpai_address.weight_addr;
                size = drpai_address.weight_size;
                break;
            case (INDEX_C):
                addr = drpai_address.drp_config_addr;
                size = drpai_address.drp_config_size;
                break;
            case (INDEX_P):
                addr = drpai_address.drp_param_addr;
                size = drpai_address.drp_param_size;
                break;
            case (INDEX_A):
                addr = drpai_address.desc_aimac_addr;
                size = drpai_address.desc_aimac_size;
                break;
            case (INDEX_D):
                addr = drpai_address.desc_drp_addr;
                size = drpai_address.desc_drp_size;
                break;
            case (INDEX_AC):
                addr = drpai_address.aimac_cmd_addr;
                size = drpai_address.aimac_cmd_size;
                break;
            case (INDEX_AP):
                addr = drpai_address.aimac_param_desc_addr;
                size = drpai_address.aimac_param_desc_size;
                break;
            case (INDEX_APC):
                addr = drpai_address.aimac_param_cmd_addr;
                size = drpai_address.aimac_param_cmd_size;
                break;
            default:
                break;
        }

        printf("addr = 0x%010lX\n", addr);
        printf("size = 0x%08X(%u)\n", size, size);

        ret = load_data_to_mem(drpai_file_path[i], addr, size);
        if (0 > ret) {
            fprintf(stderr, "[ERROR] Failed to load data from memory: %s\n", drpai_file_path[i].c_str());
            return -1;
        }
    }

    /*Stop load time*/
    ret = timespec_get(&stop_time, TIME_UTC);
    if (0 == ret) {
        fprintf(stderr, "[ERROR] Failed to run timespect_get().\n");
        return -1;
    }
    diff = timedifference_msec(start_time, stop_time);
    printf("[END] Loading DRP-AI Data : Total loading time %f s\n", diff * 0.001);
    return 0;
}

/**
 * @brief drp_pselect
 *
 * @param sigset
 * @return int8_t
 */
int8_t DRPProc::drp_pselect(sigset_t* sigset) {
    /*Setup pselect settings*/
    sigset_t sigsett;
    struct timespec tv;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(_drpai_fd, &rfds);
    tv.tv_sec = DRPAI_TIMEOUT;
    tv.tv_nsec = 0;
    sigemptyset(&sigsett);
    sigaddset(&sigsett, SIGUSR1);
    /*Wait Till The DRP-AI Ends*/

    return pselect(_drpai_fd + 1, &rfds, NULL, NULL, &tv, &sigsett);
}

/**
 * @brief drp_adrconv
 * @details Convert memory address
 * @param drpData
 * @return int8_t
 */
int8_t DRPProc::drp_adrconv(drpai_data_t drpData[]) {
    int32_t ret;

    // Get DRP-AI Memory Area Address via DRP-AI Driver
    drpai_data_t drp_ai_area;
    errno = 0;
    ret = ioctl(_drpai_fd, DRPAI_GET_DRPAI_AREA, &drp_ai_area);
    if (0 != ret) {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_GET_DRPAI_AREA: errno=%d\n", errno);
        return ret;
    }

    // Search first and last address in DRP-AI Memory Area
    const uint64_t alpha2_first_address = drp_ai_area.address;
    uint64_t alpha2_last_address = drpData[0].address;
    uint64_t alpha2_last_size = drpData[0].size;
    for (size_t i = 0; i < DRPAI_INDEX_NUM; i++) {
        if (alpha2_last_address < drpData[i].address) {
            alpha2_last_address = drpData[i].address;
            alpha2_last_size = drpData[i].size;
        }
    }

    // Convert memory address
    drpai_adrconv_t addr_info;
    addr_info.org_address = DRPAI_ALPHA1_FIRST_ADDRESS;
    addr_info.size = (alpha2_last_address + alpha2_last_size) - alpha2_first_address;
    addr_info.conv_address = alpha2_first_address;
    addr_info.mode = DRPAI_ADRCONV_MODE_REPLACE;

    errno = 0;
    ret = ioctl(_drpai_fd, DRPAI_SET_ADRCONV, &addr_info);
    if (0 != ret) {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_SET_ADRCONV: errno=%d\n", errno);
        return ret;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief drp_start
 * @details Start DRP proc
 * @param drpData
 * @return int8_t
 */
int8_t DRPProc::drp_start(drpai_data_t drpData[]) {
    /*Start DRP-AI Driver*/
    errno = 0;
    int32_t ret = ioctl(_drpai_fd, DRPAI_START, &drpData[0]);
    if (0 != ret) {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_START: errno=%d\n", errno);
    }
    return ret;
}

/**
 * @brief drp_getStatus
 * @details get drp status
 * @param drpai_status
 * @return int8_t
 */
int8_t DRPProc::drp_getStatus(drpai_status_t* drpai_status) {
    return ioctl(_drpai_fd, DRPAI_GET_STATUS, drpai_status);
}

/**
 * @brief drp_assign
 * @details Assign drp data
 * @param drpai_data
 * @return int8_t
 */
int8_t DRPProc::drp_assign(drpai_data_t* drpai_data) {
    return ioctl(_drpai_fd, DRPAI_ASSIGN, drpai_data);
}

/**
 * @brief drp_read
 * @details Read DRP data
 * @param outDataBuff
 * @param buffSize
 * @return int8_t
 */
int8_t DRPProc::drp_read(void* outDataBuff, int32_t buffSize) {
    return read(_drpai_fd, outDataBuff, buffSize);
}

/**
 * @brief timedifference_msec
 * @details  computes the time difference in ms between two moments
 * @param t0 start time
 * @param t1 stop time
 * @return double the time difference in ms
 */
double DRPProc::timedifference_msec(struct timespec t0, struct timespec t1) {
    return (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0;
}

/**
 * @brief drp_set_drp_max_freq
 * @details Set DRP max frequency
 * @param mindiv division ratio
 * @return int8_t
 */
int8_t DRPProc::drp_set_drp_max_freq(uint32_t mindiv) {
    return ioctl(_drpai_fd, DRPAI_SET_DRP_MAX_FREQ, &mindiv);
}

/**
 * @brief drp_set_drpai_freq
 * @details Set DRP-AI frequency
 * @param divfix division ratio
 * @return int8_t
 */
int8_t DRPProc::drp_set_drpai_freq(uint32_t divfix) {
    return ioctl(_drpai_fd, DRPAI_SET_DRPAI_FREQ, &divfix);
}

#define DUMP_BUF_SIZE (1024 * 64)

/**
 * @brief drp_reg_dump
 * @details DRP-AI register dump
 * @param stp_file STP reg file name
 * @param aimac_file AIMAC reg file name
 * @return int8_t
 */
#if 0 /* CPG dump */
int8_t DRPProc::drp_reg_dump(const char* stp_file, const char* aimac_file, const char* cpg_file)
#else
int8_t DRPProc::drp_reg_dump(const char* stp_file, const char* aimac_file)
#endif
{
    int8_t ret;
    int8_t stp_fd = 0;
    int8_t aimac_fd = 0;
#if 0 /* CPG dump */
    int8_t cpg_fd = 0;
#endif
    uint32_t stp_size = 0x1000000;
    uint32_t aimac_size = 0x400000;
#if 0 /* CPG dump */
    uint32_t cpg_size = 0x10000;
#endif
    int32_t i;
    size_t bytes;
    static uint8_t dump_buf[DUMP_BUF_SIZE];

    errno = 0;
    ret = ioctl(_drpai_fd, DRPAI_REG_DUMP, NULL);
    if (-1 == ret) {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_REG_DUMP errno=%d\n", errno);
        ret = -1;
        goto end;
    }

    errno = 0;
    stp_fd = open(stp_file, O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (0 > stp_fd) {
        fprintf(stderr, "[ERROR] Failed to open: %s errno=%d\n", stp_file, errno);
        ret = -1;
        goto end;
    }

    for (i = 0; i < (int32_t)(stp_size / DUMP_BUF_SIZE); i++) {
        errno = 0;
        bytes = read(_drpai_fd, dump_buf, DUMP_BUF_SIZE);
        if (-1 == (int32_t)bytes) {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            ret = -1;
            goto end;
        }
        bytes = write(stp_fd, dump_buf, DUMP_BUF_SIZE);
        if (0 > ret) {
            fprintf(stderr, "[ERROR] Failed to write: %s errno=%d\n", stp_file, errno);
            ret = -1;
            goto end;
        }
    }
    if (0 != (stp_size % DUMP_BUF_SIZE)) {
        errno = 0;
        bytes = read(_drpai_fd, dump_buf, (stp_size % DUMP_BUF_SIZE));
        if (-1 == (int32_t)bytes) {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            ret = -1;
            goto end;
        }
        bytes = write(stp_fd, dump_buf, (stp_size % DUMP_BUF_SIZE));
        if (0 > bytes) {
            fprintf(stderr, "[ERROR] Failed to read: %s errno=%d\n", stp_file, errno);
            ret = -1;
            goto end;
        }
    }

    errno = 0;
    aimac_fd = open(aimac_file, O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (0 > aimac_fd) {
        fprintf(stderr, "[ERROR] Failed to open: %s errno=%d\n", aimac_file, errno);
        ret = -1;
        goto end;
    }

    for (i = 0; i < (int32_t)(aimac_size / DUMP_BUF_SIZE); i++) {
        errno = 0;
        bytes = read(_drpai_fd, dump_buf, DUMP_BUF_SIZE);
        if (-1 == (int32_t)bytes) {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            ret = -1;
            goto end;
        }
        bytes = write(aimac_fd, dump_buf, DUMP_BUF_SIZE);
        if (0 > ret) {
            fprintf(stderr, "[ERROR] Failed to write: %s errno=%d\n", aimac_file, errno);
            ret = -1;
            goto end;
        }
    }
    if (0 != (aimac_size % DUMP_BUF_SIZE)) {
        errno = 0;
        bytes = read(_drpai_fd, dump_buf, (aimac_size % DUMP_BUF_SIZE));
        if (-1 == (int32_t)bytes) {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            ret = -1;
            goto end;
        }
        bytes = write(aimac_fd, dump_buf, (stp_size % DUMP_BUF_SIZE));
        if (0 > bytes) {
            fprintf(stderr, "[ERROR] Failed to read: %s errno=%d\n", aimac_file, errno);
            ret = -1;
            goto end;
        }
    }

#if 0 /* CPG dump */
    errno = 0;
    cpg_fd = open(cpg_file, O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (0 > cpg_fd)
    {
        fprintf(stderr, "[ERROR] Failed to open: %s errno=%d\n", cpg_file, errno);
        ret = -1;
        goto end;
    }

    for (i = 0; i < (cpg_size / DUMP_BUF_SIZE); i++)
    {
        errno = 0;
        bytes = read(_drpai_fd, dump_buf, DUMP_BUF_SIZE);
        if ( -1 == bytes )
        {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            ret = -1;
            goto end;
        }
        bytes = write(cpg_fd, dump_buf, DUMP_BUF_SIZE);
        if ( 0 > ret )
        {
            fprintf(stderr, "[ERROR] Failed to write: %s errno=%d\n", cpg_file, errno);
            ret = -1;
            goto end;
        }
    }
    if ( 0 != (cpg_size % DUMP_BUF_SIZE))
    {
        errno = 0;
        bytes = read(_drpai_fd, dump_buf, (cpg_size % DUMP_BUF_SIZE));
        if ( -1 == bytes )
        {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            ret = -1;
            goto end;
        }
        bytes = write(cpg_fd, dump_buf, (cpg_size % DUMP_BUF_SIZE));
        if ( 0 > bytes )
        {
            fprintf(stderr, "[ERROR] Failed to read: %s errno=%d\n", cpg_file, errno);
            ret = -1;
            goto end;
        }
    }
#endif
    goto end;

end:
    if (0 < stp_fd) {
        close(stp_fd);
    }
    if (0 < aimac_fd) {
        close(aimac_fd);
    }
#if 0 /* CPG dump */
    if (0 < cpg_fd)
    {
        close(cpg_fd);
    }
#endif
    return ret;
}

/**
 * @brief write_data_from_mem
 * @details Write DRP-AI Data to file.
 * @param data DRP-AI data file path
 * @param from DRP-AI data addrerss
 * @param size Data size
 * @return int8_t
 */
int8_t DRPProc::write_data_from_mem(std::string data, uint32_t from, uint32_t size) {
    int8_t ret_write_data = 0;
    int8_t obj_fd;
    static uint8_t drpai_buf[BUF_SIZE];
    drpai_data_t drpai_data;
    uint8_t assign_ret = 0;
    uint8_t rw_ret = 0;
    int32_t i = 0;

    errno = 0;
    obj_fd = open(data.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (0 > obj_fd) {
        fprintf(stderr, "[ERROR] Failed to open: %s errno=%d\n", data.c_str(), errno);
        ret_write_data = -1;
        goto end;
    }

    drpai_data.address = from;
    drpai_data.size = size;

    errno = 0;
    assign_ret = drp_assign(&drpai_data);
    if (-1 == assign_ret) {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_ASSIGN: errno=%d\n", errno);
        ret_write_data = -1;
        goto end;
    }

    for (i = 0; i < (int32_t)(drpai_data.size / BUF_SIZE); i++) {
        errno = 0;
        rw_ret = read(_drpai_fd, drpai_buf, BUF_SIZE);
        if (0 > rw_ret) {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            ret_write_data = -1;
            goto end;
        }
        rw_ret = write(obj_fd, drpai_buf, BUF_SIZE);
        if (-1 == rw_ret) {
            fprintf(stderr, "[ERROR] Failed to write: %s errno=%d\n", data.c_str(), errno);
            ret_write_data = -1;
            goto end;
        }
    }
    if (0 != (drpai_data.size % BUF_SIZE)) {
        errno = 0;
        rw_ret = read(_drpai_fd, drpai_buf, (drpai_data.size % BUF_SIZE));
        if (0 > rw_ret) {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            ret_write_data = -1;
            goto end;
        }
        rw_ret = write(obj_fd, drpai_buf, (drpai_data.size % BUF_SIZE));
        if (-1 == rw_ret) {
            fprintf(stderr, "[ERROR] Failed to write: %s errno=%d\n", data.c_str(), errno);
            ret_write_data = -1;
            goto end;
        }
    }
    goto end;

end:
    if (0 < obj_fd) {
        close(obj_fd);
    }
    return ret_write_data;
}

/**
 * @brief write_drpai_data
 * @details Write DRP-AI Object files from memory via DRP-AI Driver.
 * @param drpai_address
 * @param drpai_file_path
 * @return int8_t
 */
int8_t DRPProc::write_drpai_data(st_addr_t& drpai_address, std::string drpai_file_path[]) {
    uint64_t addr = 0;
    uint32_t size = 0;
    uint8_t i = 0;
    int8_t ret = 0;

    for (i = 0; i < 8; i++) {
        switch (i) {
            case (INDEX_W):
                addr = 0;
                size = 0;
                break;
            case (INDEX_C):
                addr = 0;
                size = 0;
                break;
            case (INDEX_P):
                addr = drpai_address.drp_param_addr;
                size = drpai_address.drp_param_size;
                break;
            case (INDEX_A):
                addr = drpai_address.desc_aimac_addr;
                size = drpai_address.desc_aimac_size;
                break;
            case (INDEX_D):
                addr = drpai_address.desc_drp_addr;
                size = drpai_address.desc_drp_size;
                break;
            case (INDEX_AC):
                addr = drpai_address.aimac_cmd_addr;
                size = drpai_address.aimac_cmd_size;
                break;
            case (INDEX_AP):
                addr = drpai_address.aimac_param_desc_addr;
                size = drpai_address.aimac_param_desc_size;
                break;
            case (INDEX_APC):
                addr = drpai_address.aimac_param_cmd_addr;
                size = drpai_address.aimac_param_cmd_size;
                break;
            default:
                break;
        }

        if ((0 != addr) && (0 != size)) {
            ret = write_data_from_mem(drpai_file_path[i], addr, size);
            if (0 > ret) {
                fprintf(stderr, "[ERROR] Failed to write data from memory: %s\n", drpai_file_path[i].c_str());
                return -1;
            }
        }
    }

    return 0;
}
