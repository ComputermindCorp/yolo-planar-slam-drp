
#include "drp.h"

#include <cmath>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

#include "linux/drp.h"
#include "linux/drpai.h"

namespace drp {

constexpr uint16_t MIN_WIDTH = 16;
constexpr uint16_t MIN_HEIGHT = 16;
constexpr uint16_t MAX_WIDTH = 800;
constexpr uint16_t MAX_HEIGHT = 600;

uint32_t gaussian_blur_config_size;   /* size of gaussian_blur_drp_out.bin */
uint32_t orb_descriptors_config_size; /* size of orb_descriptors_drp_out.bin */
uint32_t resize_config_size;          /* size of resize_drp_out.bin */
uint32_t cvfast_config_size;          /* size of cvfast_drp_out.bin */
uint32_t slamfast_config_size;        /* size of slamfast_drp_out.bin */

uint32_t drp_gaussian_blur_config_address;
uint32_t drp_orb_descriptors_config_address;
uint32_t drp_resize_config_address;
uint32_t drp_cvfast_config_address;
uint32_t drp_slamfast_config_address;

drp_data_t proc[8];

#define PROC_GAUSSIAN_BLUR_CONFIG 0
#define PROC_GAUSSIAN_BLUR_PARAM 1
#define PROC_ORB_DESCRIPTORS_CONFIG 2
#define PROC_ORB_DESCRIPTORS_PARAM 3
#define PROC_RESIZE_CONFIG 4
#define PROC_RESIZE_PARAM 5
#define PROC_CVFAST_CONFIG 6
#define PROC_CVFAST_PARAM 7
#define PROC_SLAMFAST_CONFIG PROC_CVFAST_CONFIG
#define PROC_SLAMFAST_PARAM PROC_CVFAST_PARAM

#define BUF_SIZE 128

#define DRPCFG_NUM 1

bool WriteConfigurationCode(const int drp_fd, const char* file_path, const drp_data_t* data) {
    assert(DRP_FIRST_ADDRESS <= data->address);
    assert(data->address <= DRP_LAST_ADDRESS);
    assert(data->address % DRP_ALIGN_BYTE == 0);

    int file_fd = open(file_path, O_RDONLY);
    if (file_fd < 0) {
        fprintf(stderr, "Failed to open %s\n", file_path);
        close(file_fd);
        return false;
    }

    int drpai_ret = ioctl(drp_fd, DRP_ASSIGN, data);
    bool ioctl_succeed = ((drpai_ret == 0) && (errno == 0));
    if (!ioctl_succeed) {
        fprintf(stderr, "Failed to ioctl(DRP_ASSIGN) in WriteConfigurationCode. Return code is %d, errno is %d\n", drpai_ret, errno);
        close(file_fd);
        return false;
    }

    const size_t loop_count = data->size / BUF_SIZE;
    const ssize_t loop_remain = data->size % BUF_SIZE;
    for (size_t i = 0; i < loop_count; i++) {
        ssize_t rw_ret;
        uint8_t buf[BUF_SIZE];

        /* sd -> user buf */
        errno = 0;
        rw_ret = read(file_fd, buf, BUF_SIZE);
        if (BUF_SIZE != rw_ret) {
            fprintf(stderr, "Failed to read in WriteConfigurationCode. Return code is %ld, errno is %d\n", rw_ret, errno);
            close(file_fd);
            return false;
        }

        /* user buf -> kernel buf -> DRP for CMA */
        errno = 0;
        rw_ret = write(drp_fd, buf, BUF_SIZE);
        if (BUF_SIZE != rw_ret) {
            fprintf(stderr, "Failed to write in WriteConfigurationCode. Return code is %ld, errno is %d\n", rw_ret, errno);
            close(file_fd);
            return false;
        }
    }

    if (0 != loop_remain) {
        ssize_t rw_ret;
        uint8_t buf[BUF_SIZE];

        /* sd -> user buf */
        errno = 0;
        rw_ret = read(file_fd, buf, loop_remain);
        if (loop_remain != rw_ret) {
            fprintf(stderr, "Failed to read in WriteConfigurationCode. Return code is %ld, errno is %d\n", rw_ret, errno);
            close(file_fd);
            return false;
        }

        /* user buf -> kernel buf -> DRP for CMA */
        errno = 0;
        rw_ret = write(drp_fd, buf, loop_remain);
        if (loop_remain != rw_ret) {
            fprintf(stderr, "Failed to write in WriteConfigurationCode. Return code is %ld, errno is %d\n", rw_ret, errno);
            close(file_fd);
            return false;
        }
    }

    close(file_fd);
    return true;
}

bool MemcpyU2P(const int drp_fd, const void* src, const drp_data_t* data) {
    assert(DRP_FIRST_ADDRESS <= data->address);
    assert(data->address <= DRP_LAST_ADDRESS);
    assert(data->address % DRP_ALIGN_BYTE == 0);

    errno = 0;
    int drpai_ret = ioctl(drp_fd, DRP_ASSIGN, data);
    bool ioctl_succeed = ((drpai_ret == 0) && (errno == 0));
    if (!ioctl_succeed) {
        fprintf(stderr, "Failed to ioctl(DRP_ASSIGN) in MemcpyU2P. Return code is %d, errno is %d\n", drpai_ret, errno);
        return false;
    }

    errno = 0;
    ssize_t rw_ret = write(drp_fd, src, data->size);
    if (data->size != rw_ret) {
        fprintf(stderr, "Failed to write in MemcpyU2P. Return code is %ld, errno is %d\n", rw_ret, errno);
        return false;
    }

    return true;
}

bool MemcpyP2U(const int drp_fd, void* dst, const drp_data_t* data) {
    assert(DRP_FIRST_ADDRESS <= data->address);
    assert(data->address <= DRP_LAST_ADDRESS);
    assert(data->address % DRP_ALIGN_BYTE == 0);

    int drpai_ret = ioctl(drp_fd, DRP_ASSIGN, data);
    bool ioctl_succeed = ((drpai_ret == 0) && (errno == 0));
    if (!ioctl_succeed) {
        fprintf(stderr, "Failed to ioctl(DRP_ASSIGN) in MemcpyP2U. Return code is %d, errno is %d\n", drpai_ret, errno);
        return false;
    }

    errno = 0;
    ssize_t rw_ret = read(drp_fd, dst, data->size);
    if (data->size != rw_ret) {
        fprintf(stderr, "Failed to read in MemcpyP2U. Return code is %ld, errno is %d\n", rw_ret, errno);
        return false;
    }

    return true;
}

bool Activate(const int drp_fd,
              const iodata_info_st* iodata,
              const uint32_t iodata_num) {
    assert(0 < drp_fd);

    drp_seq_t seq;
    seq.num = DRPCFG_NUM;
    seq.order[0] = DRP_EXE_DRP_40BIT;
    seq.address = DRP_SEQ_ADDRESS;

    seq.iodata_num = iodata_num;
    for (size_t i = 0; i < seq.iodata_num; i++) {
        seq.iodata[i].address = iodata[i].address;
        seq.iodata[i].size = iodata[i].size;
        seq.iodata[i].pos = iodata[i].pos;
    }

    errno = 0;
    int drpai_ret = ioctl(drp_fd, DRP_SET_SEQ, &seq);
    bool ioctl_succeed = ((drpai_ret == 0) && (errno == 0));

    if (!ioctl_succeed) {
        fprintf(stderr, "Failed to ioctl(DRP_SET_SEQ). Return code is %d, errno is %d\n", drpai_ret, errno);
        return false;
    }

    return ioctl_succeed;
}

bool Start(const int drp_fd, const drp_data_t* proc) {
    assert(DRP_FIRST_ADDRESS <= proc[0].address);
    assert(proc[0].address <= DRP_LAST_ADDRESS);
    assert(proc[0].address % DRP_ALIGN_BYTE == 0);
    assert(DRP_FIRST_ADDRESS <= proc[1].address);
    assert(proc[1].address <= DRP_LAST_ADDRESS);
    assert(proc[1].address % DRP_ALIGN_BYTE == 0);

    errno = 0;
    int drpai_ret = ioctl(drp_fd, DRP_START, proc);
    bool ioctl_succeed = ((drpai_ret == 0) && (errno == 0));

    if (!ioctl_succeed) {
        fprintf(stderr, "Failed to Start(DRP_START). Return code is %d, errno is %d\n", drpai_ret, errno);
        return false;
    }

    return ioctl_succeed;
}

bool GetStatus(const int drp_fd) {
    drp_status_t drpai_status;

    errno = 0;
    int drpai_ret = ioctl(drp_fd, DRP_GET_STATUS, &drpai_status);
    bool ioctl_succeed = ((drpai_ret == 0) && (errno == 0));
    bool status_ok = (drpai_status.status == DRP_STATUS_IDLE);
    bool err_is_success = (drpai_status.err == DRP_ERRINFO_SUCCESS);

    bool finished = (ioctl_succeed && status_ok && err_is_success);

    return finished;
}

uint32_t GetConfigurationSize(const char* file_path) {
    struct stat statBuf;

    if (stat(file_path, &statBuf) != 0) {
        fprintf(stderr, "Failed to get the size of %s\n", file_path);
        exit(EXIT_FAILURE);
    }

    return statBuf.st_size;
}

bool LoadBinConfigurations(const int drp_fd, const bool use_cvfast) {
    bool succeed;

    gaussian_blur_config_size = GetConfigurationSize(GAUSSIAN_BLUR_CONFIG_PATH);
    orb_descriptors_config_size = GetConfigurationSize(ORB_DESCRIPTORS_CONFIG_PATH);
    resize_config_size = GetConfigurationSize(RESIZE_CONFIG_PATH);
#if defined(ENABLE_SLAMFAST)
    slamfast_config_size = GetConfigurationSize(SLAMFAST_CONFIG_PATH);
#else
    if (use_cvfast) {
        cvfast_config_size = GetConfigurationSize(CVFAST_CONFIG_PATH);
    }
#endif

    // Workaround #126
    drp_gaussian_blur_config_address = DRP_CONFIG_ADDRESS;
    drp_gaussian_blur_config_address += (DRP_ALIGN_BYTE - (drp_gaussian_blur_config_address % DRP_ALIGN_BYTE));
    drp_orb_descriptors_config_address = drp_gaussian_blur_config_address + gaussian_blur_config_size;
    drp_orb_descriptors_config_address += (DRP_ALIGN_BYTE - (drp_orb_descriptors_config_address % DRP_ALIGN_BYTE));
    drp_resize_config_address = drp_orb_descriptors_config_address + orb_descriptors_config_size;
    drp_resize_config_address += (DRP_ALIGN_BYTE - (drp_resize_config_address % DRP_ALIGN_BYTE));
#if defined(ENABLE_SLAMFAST)
    drp_slamfast_config_address = drp_resize_config_address + resize_config_size;
    drp_slamfast_config_address += (DRP_ALIGN_BYTE - (drp_slamfast_config_address % DRP_ALIGN_BYTE));
#else
    drp_cvfast_config_address = drp_resize_config_address + resize_config_size;
    drp_cvfast_config_address += (DRP_ALIGN_BYTE - (drp_cvfast_config_address % DRP_ALIGN_BYTE));
#endif

    assert(drp_gaussian_blur_config_address < drp_orb_descriptors_config_address);
    assert(drp_orb_descriptors_config_address < drp_resize_config_address);
#if defined(ENABLE_SLAMFAST)
    assert(drp_resize_config_address < drp_slamfast_config_address);
#else
    if (use_cvfast) {
        assert(drp_resize_config_address < drp_cvfast_config_address);
    }
#endif

    assert(DRP_FIRST_ADDRESS <= drp_gaussian_blur_config_address);
    assert(drp_gaussian_blur_config_address <= DRP_LAST_ADDRESS);
    assert(drp_gaussian_blur_config_address % DRP_ALIGN_BYTE == 0);

    assert(DRP_FIRST_ADDRESS <= drp_orb_descriptors_config_address);
    assert(drp_orb_descriptors_config_address <= DRP_LAST_ADDRESS);
    assert(drp_orb_descriptors_config_address % DRP_ALIGN_BYTE == 0);

    assert(DRP_FIRST_ADDRESS <= drp_resize_config_address);
    assert(drp_resize_config_address <= DRP_LAST_ADDRESS);
    assert(drp_resize_config_address % DRP_ALIGN_BYTE == 0);

#if defined(ENABLE_SLAMFAST)
    assert(DRP_FIRST_ADDRESS <= drp_slamfast_config_address);
    assert(drp_slamfast_config_address <= DRP_LAST_ADDRESS);
    assert(drp_slamfast_config_address % DRP_ALIGN_BYTE == 0);
#else
    assert(DRP_FIRST_ADDRESS <= drp_cvfast_config_address);
    assert(drp_cvfast_config_address <= DRP_LAST_ADDRESS);
    assert(drp_cvfast_config_address % DRP_ALIGN_BYTE == 0);
#endif

    proc[PROC_GAUSSIAN_BLUR_CONFIG].address = drp_gaussian_blur_config_address;
    proc[PROC_GAUSSIAN_BLUR_CONFIG].size = gaussian_blur_config_size;

    succeed = WriteConfigurationCode(drp_fd, GAUSSIAN_BLUR_CONFIG_PATH, &proc[PROC_GAUSSIAN_BLUR_CONFIG]);
    if (!succeed) {
        fprintf(stderr, "Failed to WriteConfigurationCode of GaussianBlur.\n");
        return false;
    }

    proc[PROC_ORB_DESCRIPTORS_CONFIG].address = drp_orb_descriptors_config_address;
    proc[PROC_ORB_DESCRIPTORS_CONFIG].size = orb_descriptors_config_size;

    succeed = WriteConfigurationCode(drp_fd, ORB_DESCRIPTORS_CONFIG_PATH, &proc[PROC_ORB_DESCRIPTORS_CONFIG]);
    if (!succeed) {
        fprintf(stderr, "Failed to WriteConfigurationCode of computeOrbDescriptors.\n");
        return false;
    }

    proc[PROC_RESIZE_CONFIG].address = drp_resize_config_address;
    proc[PROC_RESIZE_CONFIG].size = resize_config_size;

    succeed = WriteConfigurationCode(drp_fd, RESIZE_CONFIG_PATH, &proc[PROC_RESIZE_CONFIG]);
    if (!succeed) {
        fprintf(stderr, "Failed to WriteConfigurationCode of Resize.\n");
        return false;
    }

#if defined(ENABLE_SLAMFAST)
    proc[PROC_SLAMFAST_CONFIG].address = drp_slamfast_config_address;
    proc[PROC_SLAMFAST_CONFIG].size = slamfast_config_size;

    succeed = WriteConfigurationCode(drp_fd, SLAMFAST_CONFIG_PATH, &proc[PROC_SLAMFAST_CONFIG]);
    if (!succeed) {
        fprintf(stderr, "Failed to WriteConfigurationCode of SLAMFAST.\n");
        return false;
    }
#else
    if (use_cvfast) {
        proc[PROC_CVFAST_CONFIG].address = drp_cvfast_config_address;
        proc[PROC_CVFAST_CONFIG].size = cvfast_config_size;

        succeed = WriteConfigurationCode(drp_fd, CVFAST_CONFIG_PATH, &proc[PROC_CVFAST_CONFIG]);
        if (!succeed) {
            fprintf(stderr, "Failed to WriteConfigurationCode of CVFAST.\n");
            return false;
        }
    }
#endif

    return true;
}

bool GaussianBlurSetup(const int drp_fd, const size_t level) {
    const uint32_t input_address = (level == 0) ? DRP_RESIZE_INPUT_ADDRESS : (DRP_GAUSSIAN_BLUR_INPUT_ADDRESS + image_data_stride[level]);
    const uint32_t output_address = DRP_GAUSSIAN_BLUR_OUTPUT_ADDRESS + image_data_stride[level];

    const uint32_t iodata_num = 2;
    iodata_info_st iodata[iodata_num];

    // Input
    iodata[0].address = input_address;
    iodata[0].size = MAX_WIDTH * MAX_HEIGHT;
    iodata[0].pos = 0;

    // Output
    iodata[1].address = output_address;
    iodata[1].size = MAX_WIDTH * MAX_HEIGHT;
    iodata[1].pos = 4;

    MT_START(mt_drp_gaussian_blur_activate);
    bool activate_succeed = Activate(drp_fd, iodata, iodata_num);
    MT_FINISH(mt_drp_gaussian_blur_activate);

    if (!activate_succeed) {
        fprintf(stderr, "Failed to Activate in GaussianBlurSetup.\n");
        return false;
    }
    return true;
}

bool GaussianBlurStart(const int drp_fd, const cv::Mat& src, const size_t level) {
    assert(src.type() == CV_8UC1);
    assert(src.isContinuous());

    bool succeed;

    const uint32_t image_bytes = sizeof(uint8_t) * src.rows * src.cols;
    const uint32_t input_address = (level == 0) ? DRP_RESIZE_INPUT_ADDRESS : (DRP_GAUSSIAN_BLUR_INPUT_ADDRESS + image_data_stride[level]);

    drp_data_t input_data;
    input_data.address = input_address;
    input_data.size = image_bytes;

    MT_START(mt_drp_gaussian_blur_u2p_input[level]);
    succeed = MemcpyU2P(drp_fd, src.data, &input_data);
    MT_FINISH(mt_drp_gaussian_blur_u2p_input[level]);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of input data in GaussianBlurStart.\n");
        return false;
    }

    return GaussianBlurStart(drp_fd, src.cols, src.rows, level);
}

bool GaussianBlurStart(const int drp_fd, const uint16_t cols, const uint16_t rows, const size_t level) {
    assert(0 < drp_fd);

    bool succeed;

    assert(MIN_WIDTH <= cols);
    assert(MIN_HEIGHT <= rows);
    assert(cols <= MAX_WIDTH);
    assert(rows <= MAX_HEIGHT);
    assert(0 <= level && level < 8);

    uint32_t parameter[4];

    const uint32_t input_address = (level == 0) ? DRP_RESIZE_INPUT_ADDRESS : (DRP_GAUSSIAN_BLUR_INPUT_ADDRESS + image_data_stride[level]);
    const uint32_t output_address = DRP_GAUSSIAN_BLUR_OUTPUT_ADDRESS + image_data_stride[level];

    MT_START(mt_drp_gaussian_blur_calc_param[level]);
    parameter[0] = input_address;
    parameter[1] = output_address;
    parameter[2] = cols + (rows << 16);
    parameter[3] = 0;
    MT_FINISH(mt_drp_gaussian_blur_calc_param[level]);

    proc[PROC_GAUSSIAN_BLUR_PARAM].address = DRP_PARAM_ADDRESS;
    proc[PROC_GAUSSIAN_BLUR_PARAM].size = 4 * 4;

    MT_START(mt_drp_gaussian_blur_u2p_param[level]);
    succeed = MemcpyU2P(drp_fd, parameter, &proc[PROC_GAUSSIAN_BLUR_PARAM]);
    MT_FINISH(mt_drp_gaussian_blur_u2p_param[level]);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of parameter in GaussianBlurStart.\n");
        return false;
    }

    proc[PROC_GAUSSIAN_BLUR_CONFIG].address = drp_gaussian_blur_config_address;
    proc[PROC_GAUSSIAN_BLUR_CONFIG].size = gaussian_blur_config_size;

    MT_START(mt_drp_gaussian_blur_start[level]);
    bool start_succeed = Start(drp_fd, &proc[PROC_GAUSSIAN_BLUR_CONFIG]);
    MT_FINISH(mt_drp_gaussian_blur_start[level]);

    if (!start_succeed) {
        fprintf(stderr, "Failed to Start in GaussianBlurStart.\n");
        return false;
    }

    MT_SUM_START(mt_drp_time[0][drp::STATE_GAUSSIAN_BLUR][level]);

    return true;
}

bool GaussianBlurFinish(const int drp_fd, const size_t level) {
    assert(0 < drp_fd);

    bool finished = GetStatus(drp_fd);
    if (finished)
        MT_SUM_FINISH(mt_drp_time[0][drp::STATE_GAUSSIAN_BLUR][level]);

    return finished;
}

bool GaussianBlurFinish(const int drp_fd, const size_t level, cv::Mat& dst) {
    assert(dst.type() == CV_8UC1);
    assert(dst.isContinuous());
    assert(MIN_WIDTH <= dst.cols);
    assert(MIN_HEIGHT <= dst.rows);
    assert(dst.cols <= MAX_WIDTH);
    assert(dst.rows <= MAX_HEIGHT);
    assert(0 <= level && level < 8);

    const uint32_t image_bytes = sizeof(uint8_t) * dst.rows * dst.cols;

    bool finished = GaussianBlurFinish(drp_fd, level);
    if (!finished)
        return false;

    const uint32_t output_address = DRP_GAUSSIAN_BLUR_OUTPUT_ADDRESS + image_data_stride[level];

    drp_data_t output_data;
    output_data.address = output_address;
    output_data.size = image_bytes;

    MT_START(mt_drp_gaussian_blur_p2u[level]);
    MemcpyP2U(drp_fd, dst.data, &output_data);
    MT_FINISH(mt_drp_gaussian_blur_p2u[level]);

    return true;
}

bool GaussianBlur(const int drp_fd, const cv::Mat& src, const size_t level, cv::Mat& dst) {
    /*
    src.type() == CV_8UC1
    src.isContinuous() == true
    dst.type() == CV_8UC1
    dst.isContinuous() == true
    ksize == cv::Size(7, 7)
    sigma1 == 2.0
    sigma2 == 2.0
    borderType == cv::BORDER_REFLECT_101
    */

    bool setup_succeed = GaussianBlurSetup(drp_fd, level);
    if (!setup_succeed) {
        fprintf(stderr, "GaussianBlurSetup error\n");
        return false;
    }

    bool start_succeed = GaussianBlurStart(drp_fd, src, level);
    if (!start_succeed) {
        fprintf(stderr, "GaussianBlurStart error\n");
        return false;
    }

    // busy wait
    const clock_t start = clock();
    while (true) {
        bool finish_succeed = GaussianBlurFinish(drp_fd, level, dst);
        if (finish_succeed)
            break;

        usleep(500); // Sleep 0.5ms
        clock_t now = clock();
        double sec = (double)(now - start) / CLOCKS_PER_SEC;
        if (WAITING_TIME < sec) {
            fprintf(stderr, "Failed to drp::GaussianBlur\n");
#if defined(ENABLE_MEASURE_TIME)
            fprintf(stderr, "Remove last measure time of drp::GaussianBlur\n");
            MT_REMOVE_LAST(mt_drp_gaussian_blur_calc_param[level]);
            MT_REMOVE_LAST(mt_drp_gaussian_blur_u2p_param[level]);
            MT_REMOVE_LAST(mt_drp_gaussian_blur_u2p_input[level]);
            MT_REMOVE_LAST(mt_drp_gaussian_blur_activate);
            MT_REMOVE_LAST(mt_drp_gaussian_blur_start[level]);
#endif
            fprintf(stderr, "Retry drp::GaussianBlur\n");
            fprintf(stderr, "V2x does not support DRP reset function.\n");
            exit(EXIT_FAILURE);
        }
    }

    return true;
}

bool computeOrbDescriptorsSetup(const int drp_fd, const size_t level, const size_t trial) {
    const uint32_t input_image_address = DRP_ORB_DESCRIPTORS_INPUT_IMAGE_ADDRESS + image_data_stride[level];

    const uint32_t iodata_num = 3;
    iodata_info_st iodata[iodata_num];

    // Input
    iodata[0].address = input_image_address;
    iodata[0].size = MAX_WIDTH * MAX_HEIGHT;
    iodata[0].pos = 0;
    iodata[1].address = DRP_ORB_DESCRIPTORS_INPUT_KEYPOINTS_ADDRESS;
    iodata[1].size = MAX_NFEATURES * 8;
    iodata[1].pos = 4;

    // Output
    iodata[2].address = DRP_ORB_DESCRIPTORS_OUTPUT_ADDRESS;
    iodata[2].size = MAX_NFEATURES * 32;
    iodata[2].pos = 8;

    MT_START(mt_drp_orb_descriptors_activate);
    bool activate_succeed = Activate(drp_fd, iodata, iodata_num);
    MT_FINISH(mt_drp_orb_descriptors_activate);

    if (!activate_succeed) {
        fprintf(stderr, "Failed to Activate in computeOrbDescriptorsSetup.\n");
        return false;
    }
    return true;
}

bool computeOrbDescriptorsStart(const int drp_fd, const cv::Mat& src, const std::vector<cv::KeyPoint>& src_keypoints, const size_t level, const size_t trial) {
    assert(src.type() == CV_8UC1);
    assert(src.isContinuous());

    const uint32_t image_bytes = sizeof(uint8_t) * src.rows * src.cols;
    const uint32_t input_image_address = DRP_ORB_DESCRIPTORS_INPUT_IMAGE_ADDRESS + image_data_stride[level];

    drp_data_t input_data;
    input_data.address = input_image_address;
    input_data.size = image_bytes;

    MT_START(mt_drp_orb_descriptors_u2p_input_image[trial][level]);
    MemcpyU2P(drp_fd, src.data, &input_data);
    MT_FINISH(mt_drp_orb_descriptors_u2p_input_image[trial][level]);

    return computeOrbDescriptorsStart(drp_fd, src.cols, src.rows, src_keypoints, level, trial);
}

bool computeOrbDescriptorsStart(const int drp_fd, const uint16_t cols, const uint16_t rows, const std::vector<cv::KeyPoint>& src_keypoints, const size_t level, const size_t trial) {
    assert(0 < drp_fd);

    bool succeed;

    assert(MIN_WIDTH <= cols);
    assert(MIN_HEIGHT <= rows);
    assert(cols <= MAX_WIDTH);
    assert(rows <= MAX_HEIGHT);
    assert(0 <= level && level < 8);

    const uint16_t keypoints_size = src_keypoints.size();
    assert(keypoints_size <= MAX_NFEATURES);

    KeyPoint_ORB keypoints[MAX_NFEATURES];

    uint32_t parameter[6];

    const uint32_t input_image_address = DRP_ORB_DESCRIPTORS_INPUT_IMAGE_ADDRESS + image_data_stride[level];

    MT_START(mt_drp_orb_descriptors_calc_param[trial][level]);
    parameter[0] = input_image_address;                         // consider r_addr_image
    parameter[1] = DRP_ORB_DESCRIPTORS_INPUT_KEYPOINTS_ADDRESS; // consider r_addr_keypts
    parameter[2] = DRP_ORB_DESCRIPTORS_OUTPUT_ADDRESS;
    parameter[3] = cols + (rows << 16);
    parameter[4] = keypoints_size;
    parameter[5] = 0;
    MT_FINISH(mt_drp_orb_descriptors_calc_param[trial][level]);

    MT_START(mt_drp_orb_descriptors_cast_to_drp[trial][level]);
    for (uint16_t i = 0; i < keypoints_size; i++) {
        keypoints[i].pt.x = src_keypoints[i].pt.x;
        keypoints[i].pt.y = src_keypoints[i].pt.y;
        // floating point to fixed point
        uint32_t fixed_angle = src_keypoints[i].angle * FIXED_POINT;
        keypoints[i].angle[0] = fixed_angle & 0xFF;
        keypoints[i].angle[1] = (fixed_angle >> 8) & 0xFF;
        keypoints[i].angle[2] = (fixed_angle >> 16) & 0xFF;
        keypoints[i].angle[3] = (fixed_angle >> 24) & 0xFF;
    }
    MT_FINISH(mt_drp_orb_descriptors_cast_to_drp[trial][level]);

    proc[PROC_ORB_DESCRIPTORS_PARAM].address = DRP_PARAM_ADDRESS;
    proc[PROC_ORB_DESCRIPTORS_PARAM].size = 6 * 4;

    MT_START(mt_drp_orb_descriptors_u2p_param[trial][level]);
    succeed = MemcpyU2P(drp_fd, parameter, &proc[PROC_ORB_DESCRIPTORS_PARAM]);
    MT_FINISH(mt_drp_orb_descriptors_u2p_param[trial][level]);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of parameter in computeOrbDescriptorsStart.\n");
        return false;
    }

    drp_data_t input_data;
    input_data.address = DRP_ORB_DESCRIPTORS_INPUT_KEYPOINTS_ADDRESS;
    input_data.size = keypoints_size * 8;

    MT_START(mt_drp_orb_descriptors_u2p_input_keypoints[trial][level]);
    succeed = MemcpyU2P(drp_fd, keypoints, &input_data);
    MT_FINISH(mt_drp_orb_descriptors_u2p_input_keypoints[trial][level]);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of input data in computeOrbDescriptorsStart.\n");
        return false;
    }

    proc[PROC_ORB_DESCRIPTORS_CONFIG].address = drp_orb_descriptors_config_address;
    proc[PROC_ORB_DESCRIPTORS_CONFIG].size = orb_descriptors_config_size;

    MT_START(mt_drp_orb_descriptors_start[trial][level]);
    bool start_succeed = Start(drp_fd, &proc[PROC_ORB_DESCRIPTORS_CONFIG]);
    MT_FINISH(mt_drp_orb_descriptors_start[trial][level]);

    if (!start_succeed) {
        fprintf(stderr, "Failed to Start in computeOrbDescriptorsStart.\n");
        return false;
    }

    MT_SUM_START(mt_drp_time[trial][drp::STATE_ORB_DESCRIPTORS][level]);

    return true;
}

bool computeOrbDescriptorsFinish(const int drp_fd, const std::vector<cv::KeyPoint>& src_keypoints, const size_t level, const size_t trial, cv::Mat& dst) {
    assert(0 < drp_fd);

    assert(0 <= level && level < 8);

    bool finished = GetStatus(drp_fd);
    if (finished)
        MT_SUM_FINISH(mt_drp_time[trial][drp::STATE_ORB_DESCRIPTORS][level]);

    if (!finished)
        return false;

    const uint16_t keypoints_size = src_keypoints.size();
    assert(keypoints_size <= MAX_NFEATURES);

    drp_data_t output_data;
    output_data.address = DRP_ORB_DESCRIPTORS_OUTPUT_ADDRESS;
    output_data.size = keypoints_size * 32;

    MT_START(mt_drp_orb_descriptors_p2u[trial][level]);
    MemcpyP2U(drp_fd, dst.data, &output_data);
    MT_FINISH(mt_drp_orb_descriptors_p2u[trial][level]);

    return true;
}

bool computeOrbDescriptors(const int drp_fd, const cv::Mat& src, const std::vector<cv::KeyPoint>& src_keypoints, const size_t level, const size_t trial, cv::Mat& dst) {
    /*
    src.type() == CV_8UC1
    src.isContinuous() == true
    dst.type() == CV_8UC1
    dst.isContinuous() == true
    */

    bool setup_succeed = computeOrbDescriptorsSetup(drp_fd, level, trial);
    if (!setup_succeed) {
        fprintf(stderr, "computeOrbDescriptorsSetup error\n");
        return false;
    }

    bool start_succeed = computeOrbDescriptorsStart(drp_fd, src, src_keypoints, level, trial);
    if (!start_succeed) {
        fprintf(stderr, "computeOrbDescriptorsStart error\n");
        return false;
    }

    // busy wait
    const clock_t start = clock();
    while (true) {
        bool finish_succeed = computeOrbDescriptorsFinish(drp_fd, src_keypoints, level, trial, dst);
        if (finish_succeed)
            break;

        usleep(500); // Sleep 0.5ms
        clock_t now = clock();
        double sec = (double)(now - start) / CLOCKS_PER_SEC;
        if (WAITING_TIME < sec) {
            fprintf(stderr, "Failed to drp::computeOrbDescriptors\n");
#if defined(ENABLE_MEASURE_TIME)
            fprintf(stderr, "Remove last measure time of drp::computeOrbDescriptors\n");
            MT_REMOVE_LAST(mt_drp_orb_descriptors_calc_param[trial][level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_cast_to_drp[trial][level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_u2p_param[trial][level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_u2p_input_image[trial][level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_u2p_input_keypoints[trial][level]);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_activate);
            MT_REMOVE_LAST(mt_drp_orb_descriptors_start[trial][level]);
#endif
            fprintf(stderr, "Retry drp::computeOrbDescriptors\n");
            fprintf(stderr, "V2x does not support DRP reset function.\n");
            exit(EXIT_FAILURE);
        }
    }

    return true;
}

bool ResizeSetup(const int drp_fd, const size_t input_level) {
    const uint32_t iodata_num = 2;
    iodata_info_st iodata[iodata_num];

    // Input
    iodata[0].address = pyramid_address[input_level];
    iodata[0].size = MAX_WIDTH * MAX_HEIGHT;
    iodata[0].pos = 0;

    // Output
    iodata[1].address = pyramid_address[input_level + 1];
    iodata[1].size = MAX_WIDTH * MAX_HEIGHT;
    iodata[1].pos = 4;

    MT_START(mt_drp_resize_activate);
    bool activate_succeed = Activate(drp_fd, iodata, iodata_num);
    MT_FINISH(mt_drp_resize_activate);

    if (!activate_succeed) {
        fprintf(stderr, "Failed to Activate in ResizeSetup.\n");
        return false;
    }
    return true;
}

bool ResizeStart(const int drp_fd, const cv::Mat& src, const size_t input_level, const cv::Size output_image_size) {
    assert(src.type() == CV_8UC1);
    assert(src.isContinuous());

    bool succeed;

    const uint32_t image_bytes = sizeof(uint8_t) * src.rows * src.cols;

    assert(image_bytes <= max_image_data_size[input_level]);

    drp_data_t input_data;
    input_data.address = pyramid_address[input_level];
    input_data.size = image_bytes;

    MT_START(mt_drp_resize_u2p_input);
    succeed = MemcpyU2P(drp_fd, src.data, &input_data);
    MT_FINISH(mt_drp_resize_u2p_input);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of input data in ResizeStart.\n");
        return false;
    }

    return ResizeStart(drp_fd, cv::Size(src.cols, src.rows), input_level, output_image_size);
}

bool ResizeStart(const int drp_fd, const cv::Size input_image_size, const size_t input_level, const cv::Size output_image_size) {
    bool succeed;

    assert(MIN_WIDTH <= input_image_size.width);
    assert(MIN_HEIGHT <= input_image_size.height);
    assert(input_image_size.width <= MAX_WIDTH);
    assert(input_image_size.height <= MAX_HEIGHT);

    assert(0 <= input_level && input_level < 8);

    assert(MIN_WIDTH <= output_image_size.width);
    assert(MIN_HEIGHT <= output_image_size.height);
    assert(output_image_size.width <= MAX_WIDTH);
    assert(output_image_size.height <= MAX_HEIGHT);

    uint32_t parameter[14];

    const uint32_t image_bytes = sizeof(uint8_t) * input_image_size.width * input_image_size.height;
    assert(image_bytes <= max_image_data_size[input_level]);

    MT_START(mt_drp_resize_calc_param);
    parameter[0] = pyramid_address[input_level];     // input
    parameter[1] = pyramid_address[input_level + 1]; // output
    parameter[2] = input_image_size.width + (input_image_size.height << 16);
    parameter[3] = 1; // input channel
    parameter[4] = output_image_size.width + (output_image_size.height << 16);
    parameter[5] = 1; // output channel
    // OCH0_SYYNCSET_DT ～ OCH3_SYYNCSET_ID
    parameter[6] = 0;
    parameter[7] = 0;
    parameter[8] = 0;
    parameter[9] = 0;
    parameter[10] = 0; // INT_DISABLE
    parameter[11] = 0;
    parameter[12] = 1; // algorithm, data_type
    parameter[13] = 0;
    MT_FINISH(mt_drp_resize_calc_param);

    proc[PROC_RESIZE_PARAM].address = DRP_PARAM_ADDRESS;
    proc[PROC_RESIZE_PARAM].size = 14 * 4;

    MT_START(mt_drp_resize_u2p_param);
    succeed = MemcpyU2P(drp_fd, parameter, &proc[PROC_RESIZE_PARAM]);
    MT_FINISH(mt_drp_resize_u2p_param);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of parameter in ResizeStart.\n");
        return false;
    }

    proc[PROC_RESIZE_CONFIG].address = drp_resize_config_address;
    proc[PROC_RESIZE_CONFIG].size = resize_config_size;

    MT_START(mt_drp_resize_start);
    bool start_succeed = Start(drp_fd, &proc[PROC_RESIZE_CONFIG]);
    MT_FINISH(mt_drp_resize_start);

    if (!start_succeed) {
        fprintf(stderr, "Failed to Start in ResizeStart.\n");
        return false;
    }

    MT_SUM_START(mt_drp_time[0][drp::STATE_RESIZE][input_level]);

    return true;
}

bool ResizeFinish(const int drp_fd, const size_t input_level, const cv::Size output_image_size, cv::Mat& dst) {
    assert(0 < drp_fd);

    assert(0 <= input_level && input_level < 8);

    assert(MIN_WIDTH <= output_image_size.width);
    assert(MIN_HEIGHT <= output_image_size.height);
    assert(output_image_size.width <= MAX_WIDTH);
    assert(output_image_size.height <= MAX_HEIGHT);

    bool finished = GetStatus(drp_fd);
    if (finished)
        MT_SUM_FINISH(mt_drp_time[0][drp::STATE_RESIZE][input_level]);

    if (!finished)
        return false;

    const uint32_t image_bytes = sizeof(uint8_t) * output_image_size.width * output_image_size.height;
    uint8_t data[image_bytes];

    assert(image_bytes <= max_image_data_size[input_level + 1]);

    drp_data_t output_data;
    output_data.address = pyramid_address[input_level + 1];
    output_data.size = image_bytes;

    MT_START(mt_drp_resize_p2u[input_level + 1]);
    MemcpyP2U(drp_fd, data, &output_data);
    MT_FINISH(mt_drp_resize_p2u[input_level + 1]);

    MT_START(mt_drp_resize_clone_output_mat[input_level + 1]);
    cv::Mat temp(output_image_size.height, output_image_size.width, CV_8UC1, data);
    dst = temp.clone();
    MT_FINISH(mt_drp_resize_clone_output_mat[input_level + 1]);

    return true;
}

bool Resize(const int drp_fd, const cv::Mat& src, const size_t input_level, const cv::Size output_image_size, cv::Mat& dst) {
    /*
      src.type() == CV_8UC1
      src.isContinuous() == true
      dst.type() == CV_8UC1
      dst.isContinuous() == true
    */

    bool setup_succeed = ResizeSetup(drp_fd, input_level);
    if (!setup_succeed) {
        fprintf(stderr, "ResizeSetup error\n");
        return false;
    }

    bool start_succeed = ResizeStart(drp_fd, src, input_level, output_image_size);
    if (!start_succeed) {
        fprintf(stderr, "ResizeStart error\n");
        return false;
    }

    // busy wait
    const clock_t start = clock();
    MT_START(mt_drp_resize_get_status);
    while (true) {
        bool finish_succeed = ResizeFinish(drp_fd, input_level, output_image_size, dst);
        if (finish_succeed)
            break;

        clock_t now = clock();
        double sec = (double)(now - start) / CLOCKS_PER_SEC;
        if (WAITING_TIME < sec) {
            MT_FINISH(mt_drp_resize_get_status);
            fprintf(stderr, "Failed to drp::Resize\n");
#if defined(ENABLE_MEASURE_TIME)
            fprintf(stderr, "Remove last measure time of drp::Resize\n");
            MT_REMOVE_LAST(mt_drp_resize_calc_param);
            MT_REMOVE_LAST(mt_drp_resize_u2p_param);
            MT_REMOVE_LAST(mt_drp_resize_u2p_input);
            MT_REMOVE_LAST(mt_drp_resize_start);
#endif
            fprintf(stderr, "Retry drp::Resize\n");
            fprintf(stderr, "V2x does not support DRP reset function.\n");
            exit(EXIT_FAILURE);
        }
    }
    MT_FINISH(mt_drp_resize_get_status);

    return true;
}

bool CVFASTSetup(const int drp_fd) {
    const uint32_t iodata_num = 3;
    iodata_info_st iodata[iodata_num];

    // Input
    iodata[0].address = DRP_CVFAST_INPUT_ADDRESS;
    iodata[0].size = MAX_CELL_SIZE * MAX_CELL_SIZE;
    iodata[0].pos = 0;

    // Output
    iodata[1].address = DRP_CVFAST_OUTPUT_ADDRESS;
    iodata[1].size = MAX_KEYPOINTS * 8;
    iodata[1].pos = 4;
    iodata[2].address = DRP_OPTIONAL_ADDRESS;
    iodata[2].size = 2;
    iodata[2].pos = 8;

    MT_START(mt_drp_cvfast_activate);
    bool activate_succeed = Activate(drp_fd, iodata, iodata_num);
    MT_FINISH(mt_drp_cvfast_activate);

    if (!activate_succeed) {
        fprintf(stderr, "Failed to Activate in CVFASTSetup.\n");
        return false;
    }
    return true;
}

bool CVFASTStart(const int drp_fd, const cv::Mat& src, const size_t level, const int thresholds[2]) {
    bool succeed;

    assert(0 < cvfast_config_size && "CVFAST is not initialized.");

    assert(src.type() == CV_8UC1);
    assert(src.isContinuous());
    assert(0 <= level && level < 8);
    assert(thresholds[0] > thresholds[1] && "thresholds[1] must be smaller than thresholds[0]");

    const uint32_t image_bytes = sizeof(uint8_t) * src.rows * src.cols;
    uint32_t parameter[6];
    uint16_t zeros[2] = {0, 0};

    MT_SUM_START(mt_drp_cvfast_calc_param[level]);
    parameter[0] = DRP_CVFAST_INPUT_ADDRESS;
    parameter[1] = DRP_CVFAST_OUTPUT_ADDRESS; // consider w_addr_keypoints
    parameter[2] = DRP_OPTIONAL_ADDRESS;      // consider w_addr_num
    parameter[3] = src.cols + (src.rows << 16);
    parameter[4] = thresholds[0] + (thresholds[1] << 16);
    parameter[5] = 0; // 0 padding
    MT_SUM_FINISH(mt_drp_cvfast_calc_param[level]);

    proc[PROC_CVFAST_PARAM].address = DRP_PARAM_ADDRESS;
    proc[PROC_CVFAST_PARAM].size = 6 * 4;

    MT_SUM_START(mt_drp_cvfast_u2p_param[level]);
    succeed = MemcpyU2P(drp_fd, parameter, &proc[PROC_CVFAST_PARAM]);
    MT_SUM_FINISH(mt_drp_cvfast_u2p_param[level]);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of parameter in CVFASTStart.\n");
        return false;
    }

    assert(MIN_CELL_SIZE <= src.rows);
    assert(MIN_CELL_SIZE <= src.cols);
    assert(src.rows <= MAX_CELL_SIZE);
    assert(src.cols <= MAX_CELL_SIZE);
    assert(0 < image_bytes);
    assert(image_bytes <= MAX_CELL_SIZE * MAX_CELL_SIZE);

    drp_data_t input_data;
    input_data.address = DRP_CVFAST_INPUT_ADDRESS;
    input_data.size = image_bytes;

    MT_SUM_START(mt_drp_cvfast_u2p_input[level]);
    succeed = MemcpyU2P(drp_fd, src.data, &input_data);
    MT_SUM_FINISH(mt_drp_cvfast_u2p_input[level]);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of input data in CVFASTStart.\n");
        return false;
    }

    drp_data_t result_size;
    result_size.address = DRP_OPTIONAL_ADDRESS;
    result_size.size = 2;

    // clear old keypoints_size
    succeed = MemcpyU2P(drp_fd, zeros, &result_size);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of zero clear in CVFASTStart.\n");
        return false;
    }

    proc[PROC_CVFAST_CONFIG].address = drp_cvfast_config_address;
    proc[PROC_CVFAST_CONFIG].size = cvfast_config_size;

    MT_SUM_START(mt_drp_cvfast_start[level]);
    bool start_succeed = Start(drp_fd, &proc[PROC_CVFAST_CONFIG]);
    MT_SUM_FINISH(mt_drp_cvfast_start[level]);

    if (!start_succeed) {
        fprintf(stderr, "Failed to Start in CVFASTStart.\n");
        return false;
    }

    MT_SUM_START(mt_drp_time[0][drp::STATE_CVFAST][level]);

    return true;
}

bool CVFASTFinish(const int drp_fd, const size_t level, std::vector<cv::KeyPoint>& dst) {
    assert(0 < drp_fd);

    assert(0 <= level && level < 8);

    bool finished = GetStatus(drp_fd);
    if (finished)
        MT_SUM_FINISH(mt_drp_time[0][drp::STATE_CVFAST][level]);

    if (!finished)
        return false;

    KeyPoint_FAST keypoints[MAX_KEYPOINTS];
    uint16_t keypoints_size;

    drp_data_t output_size;
    output_size.address = DRP_OPTIONAL_ADDRESS;
    output_size.size = 2;

    MT_SUM_START(mt_drp_cvfast_p2u_keypoints_size[level]);
    MemcpyP2U(drp_fd, &keypoints_size, &output_size);
    MT_SUM_FINISH(mt_drp_cvfast_p2u_keypoints_size[level]);

    if (keypoints_size == 0) {
        return true;
    }

    assert(0 < keypoints_size && keypoints_size <= MAX_KEYPOINTS);

    drp_data_t output_data;
    output_data.address = DRP_CVFAST_OUTPUT_ADDRESS;
    output_data.size = keypoints_size * 8;

    MT_SUM_START(mt_drp_cvfast_p2u_keypoints[level]);
    MemcpyP2U(drp_fd, keypoints, &output_data);
    MT_SUM_FINISH(mt_drp_cvfast_p2u_keypoints[level]);

    MT_SUM_START(mt_drp_cvfast_cast_to_cv[level]);
    for (uint16_t i = 0; i < keypoints_size; i++) {
        cv::KeyPoint kp;
        kp.pt.x = keypoints[i].pt.x;
        kp.pt.y = keypoints[i].pt.y;
        kp.response = (int16_t)(keypoints[i].response & 0xFF);
        kp.size = 7.f;
        dst.emplace_back(kp);
    }
    MT_SUM_FINISH(mt_drp_cvfast_cast_to_cv[level]);

    return true;
}

bool CVFAST(const int drp_fd, const cv::Mat& src, const size_t level, const int thresholds[2], std::vector<cv::KeyPoint>& dst) {
    /*
      src.type() == CV_8UC1
      src.isContinuous() == true
      nonmax_suppression == true
    */

    bool setup_succeed = CVFASTSetup(drp_fd);
    if (!setup_succeed) {
        fprintf(stderr, "CVFASTSetup error\n");
        return false;
    }

    bool start_succeed = CVFASTStart(drp_fd, src, level, thresholds);
    if (!start_succeed) {
        fprintf(stderr, "CVFASTStart error\n");
        return false;
    }

    // busy wait
    clock_t start = clock();
    while (true) {
        bool finish_succeed = CVFASTFinish(drp_fd, level, dst);
        if (finish_succeed)
            break;

        clock_t now = clock();
        double sec = (double)(now - start) / CLOCKS_PER_SEC;
        if (WAITING_TIME < sec) {
            fprintf(stderr, "Failed to drp::CVFAST\n");
#if defined(ENABLE_MEASURE_TIME)
            fprintf(stderr, "Remove last measure time of drp::CVFAST\n");
            MT_REMOVE_LAST(mt_drp_cvfast_calc_param[level]);
            MT_REMOVE_LAST(mt_drp_cvfast_u2p_param[level]);
            MT_REMOVE_LAST(mt_drp_cvfast_u2p_input[level]);
            MT_REMOVE_LAST(mt_drp_cvfast_activate);
            MT_REMOVE_LAST(mt_drp_cvfast_start[level]);
#endif
            fprintf(stderr, "Retry drp::CVFAST\n");
            fprintf(stderr, "V2x does not support DRP reset function.\n");
            exit(EXIT_FAILURE);
        }
    }

    return true;
}

bool SLAMFASTSetup(const int drp_fd, const size_t level) {
    const uint32_t iodata_num = 3;
    iodata_info_st iodata[iodata_num];

    // Input
    iodata[0].address = pyramid_address[level];
    iodata[0].size = MAX_WIDTH * MAX_HEIGHT;
    iodata[0].pos = 0;

    // Output
    iodata[1].address = DRP_SLAMFAST_OUTPUT_ADDRESS;
    iodata[1].size = MAX_KEYPOINTS * 8;
    iodata[1].pos = 4;
    iodata[2].address = DRP_OPTIONAL_ADDRESS;
    iodata[2].size = 2;
    iodata[2].pos = 8;

    MT_START(mt_drp_slamfast_activate);
    bool activate_succeed = Activate(drp_fd, iodata, iodata_num);
    MT_FINISH(mt_drp_slamfast_activate);

    if (!activate_succeed) {
        fprintf(stderr, "Failed to Activate in SLAMFASTSetup.\n");
        return false;
    }
    return true;
}

bool SLAMFASTStart(const int drp_fd, const cv::Mat& src, const size_t level) {
    assert(src.type() == CV_8UC1);
    assert(src.isContinuous());

    bool succeed;

    const uint32_t image_bytes = sizeof(uint8_t) * src.rows * src.cols;

    assert(image_bytes <= max_image_data_size[level]);

    drp_data_t input_data;
    input_data.address = pyramid_address[level];
    input_data.size = image_bytes;

    MT_START(mt_drp_slamfast_u2p_input[level]);
    succeed = MemcpyU2P(drp_fd, src.data, &input_data);
    MT_FINISH(mt_drp_slamfast_u2p_input[level]);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of input data in SLAMFASTStart.\n");
        return false;
    }

    return SLAMFASTStart(drp_fd, src.cols, src.rows, level);
}

bool SLAMFASTStart(const int drp_fd, const uint16_t cols, const uint16_t rows, const size_t level) {
    bool succeed;

    assert(MIN_WIDTH <= cols);
    assert(MIN_HEIGHT <= rows);
    assert(cols <= MAX_WIDTH);
    assert(rows <= MAX_HEIGHT);

    assert(0 <= level && level < 8);

    uint32_t parameter[4];
    uint16_t zeros[2] = {0, 0};

    MT_START(mt_drp_slamfast_calc_param[level]);
    parameter[0] = pyramid_address[level];
    parameter[1] = DRP_SLAMFAST_OUTPUT_ADDRESS; // consider w_addr_keypoints
    parameter[2] = DRP_OPTIONAL_ADDRESS;        // consider w_addr_num
    parameter[3] = cols + (rows << 16);
    MT_FINISH(mt_drp_slamfast_calc_param[level]);

    proc[PROC_SLAMFAST_PARAM].address = DRP_PARAM_ADDRESS;
    proc[PROC_SLAMFAST_PARAM].size = 4 * 4;

    MT_SUM_START(mt_drp_slamfast_u2p_param[level]);
    succeed = MemcpyU2P(drp_fd, parameter, &proc[PROC_SLAMFAST_PARAM]);
    MT_SUM_FINISH(mt_drp_slamfast_u2p_param[level]);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of parameter in SLAMFASTStart.\n");
        return false;
    }

    drp_data_t result_size;
    result_size.address = DRP_OPTIONAL_ADDRESS;
    result_size.size = 2;

    // clear old keypoints_size
    succeed = MemcpyU2P(drp_fd, zeros, &result_size);

    if (!succeed) {
        fprintf(stderr, "Failed to MemcpyU2P of zero clear in SLAMFASTStart.\n");
        return false;
    }

    proc[PROC_SLAMFAST_CONFIG].address = drp_slamfast_config_address;
    proc[PROC_SLAMFAST_CONFIG].size = slamfast_config_size;

    MT_SUM_START(mt_drp_slamfast_start[level]);
    bool start_succeed = Start(drp_fd, &proc[PROC_SLAMFAST_CONFIG]);
    MT_SUM_FINISH(mt_drp_slamfast_start[level]);

    if (!start_succeed) {
        fprintf(stderr, "Failed to Start in SLAMFASTStart.\n");
        return false;
    }

    MT_SUM_START(mt_drp_time[0][drp::STATE_SLAMFAST][level]);

    return true;
}

bool SLAMFASTFinish(const int drp_fd, const size_t level, std::vector<cv::KeyPoint>& dst) {
    assert(0 < drp_fd);

    assert(0 <= level && level < 8);

    bool finished = GetStatus(drp_fd);
    if (finished)
        MT_SUM_FINISH(mt_drp_time[0][drp::STATE_SLAMFAST][level]);

    if (!finished)
        return false;

    KeyPoint_FAST keypoints[MAX_KEYPOINTS];
    uint16_t keypoints_size;

    drp_data_t output_size;
    output_size.address = DRP_OPTIONAL_ADDRESS;
    output_size.size = 2;

    MT_SUM_START(mt_drp_slamfast_p2u_keypoints_size[level]);
    MemcpyP2U(drp_fd, &keypoints_size, &output_size);
    MT_SUM_FINISH(mt_drp_slamfast_p2u_keypoints_size[level]);

    if (keypoints_size == 0) {
        return true;
    }

    assert(0 < keypoints_size && keypoints_size <= MAX_KEYPOINTS);

    drp_data_t output_data;
    output_data.address = DRP_SLAMFAST_OUTPUT_ADDRESS;
    output_data.size = keypoints_size * 8;

    MT_SUM_START(mt_drp_slamfast_p2u_keypoints[level]);
    MemcpyP2U(drp_fd, keypoints, &output_data);
    MT_SUM_FINISH(mt_drp_slamfast_p2u_keypoints[level]);

    //////////////////

    MT_START(mt_drp_slamfast_cast_to_cv[level]);
    for (uint16_t i = 0; i < keypoints_size; i++) {
        cv::KeyPoint kp;
        kp.pt.x = keypoints[i].pt.x;
        kp.pt.y = keypoints[i].pt.y;
        kp.response = (int16_t)(keypoints[i].response & 0xFF);
        kp.size = 7.f;
        dst.emplace_back(kp);
    }
    MT_FINISH(mt_drp_slamfast_cast_to_cv[level]);

    return true;
}

bool SLAMFAST(const int drp_fd, const cv::Mat& src, const size_t level, std::vector<cv::KeyPoint>& dst) {
    /*
    src.type() == CV_8UC1
    src.isContinuous() == true
    nonmax_suppression == true
    threshold == 20, 7
    */

    bool setup_suceed = SLAMFASTSetup(drp_fd, level);
    if (!setup_suceed) {
        fprintf(stderr, "SLAMFASTSetup error\n");
        return false;
    }

    bool start_suceed = SLAMFASTStart(drp_fd, src, level);
    if (!start_suceed) {
        fprintf(stderr, "SLAMFASTStart error\n");
        return false;
    }

    // busy wait
    const clock_t start = clock();
    MT_START(mt_drp_slamfast_get_status[level]);
    while (true) {
        bool finish_suceed = SLAMFASTFinish(drp_fd, level, dst);
        if (finish_suceed)
            break;

        clock_t now = clock();
        double sec = (double)(now - start) / CLOCKS_PER_SEC;
        if (WAITING_TIME < sec) {
            MT_FINISH(mt_drp_slamfast_get_status[level]);
            fprintf(stderr, "Failed to drp::SLAMFAST\n");
#if defined(ENABLE_MEASURE_TIME)
            fprintf(stderr, "Remove last measure time of drp::SLAMFAST\n");
            MT_REMOVE_LAST(mt_drp_slamfast_calc_param[level]);
            MT_REMOVE_LAST(mt_drp_slamfast_u2p_param[level]);
            MT_REMOVE_LAST(mt_drp_slamfast_u2p_input[level]);
            MT_REMOVE_LAST(mt_drp_slamfast_activate);
            MT_REMOVE_LAST(mt_drp_slamfast_start[level]);
#endif
            fprintf(stderr, "Retry drp::SLAMFAST\n");
            fprintf(stderr, "V2x does not support DRP reset function.\n");
            exit(EXIT_FAILURE);
        }
    }
    MT_FINISH(mt_drp_slamfast_get_status[level]);

    return true;
}

} // namespace drp
