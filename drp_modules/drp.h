#ifndef DRP_LIBRARY_H
#define DRP_LIBRARY_H

#include <vector>
#include <opencv2/core/core.hpp>

#define DRP_FIRST_ADDRESS 0x80000000
#define DRP_LAST_ADDRESS 0x9FFFFFFF

#define DRP_OPTIONAL_ADDRESS 0x8BD80000
#define DRP_PARAM_ADDRESS 0x8B980000
#define DRP_CONFIG_ADDRESS 0x8BC00000

#define DRP_SEQ_ADDRESS 0x92000000

#define DRP_MIDDLE_ADDRESS 0x8B000000
#define DRP_ALGORITHM_STRIDE 0x00400000 // 4MB
#define DRP_LEVEL_STRIDE 0x00080000     // 0.5MB

#define DRP_ALIGN_BYTE 0x200

namespace drp {

constexpr int STATE_NONE = 0;
constexpr int STATE_RESIZE = 1;
constexpr int STATE_GAUSSIAN_BLUR = 2;
constexpr int STATE_SLAMFAST = 3;
constexpr int STATE_CVFAST = 3;
constexpr int STATE_ORB_DESCRIPTORS = 4;
constexpr int STATE_FINISHED = 5;

constexpr int FILE_DESCRIPTOR_CLOSE = -1;

constexpr double WAITING_TIME = 0.2;     // sec
constexpr size_t INTERVAL_RATIO_MS = 20; // ms

constexpr uint16_t MAX_NFEATURES = 10000;
constexpr uint32_t MAX_KEYPOINTS = MAX_NFEATURES * 10;
constexpr uint32_t FIXED_POINT = (1 << 16);

constexpr uint16_t MIN_CELL_SIZE = 16;
constexpr uint16_t MAX_CELL_SIZE = 60;

// total 1.5MB
constexpr uint32_t max_image_data_size[8] = {
    480000, 333500, 231852, 161124,
    111940, 77924, 54136, 37464};

// Workaround #126
constexpr uint32_t image_data_stride[9] = {
    0, 480000 + 256, 813756 + 324, 1045932 + 84,
    1207140 + 156, 1319236 + 188, 1397348 + 412, 1451896 + 136, 1489496 + 424};

constexpr uint16_t GAUSSIAN_BLUR_PARAM_SIZE = 16;   /* size of gaussian_blur_2849_0_TB_mem_in_csim_00000000.bin */
constexpr uint16_t ORB_DESCRIPTORS_PARAM_SIZE = 24; /* size of orb_descriptors_2849_0_TB_mem_in_csim_00000000.bin */
constexpr uint16_t RESIZE_PARAM_SIZE = 56;          /* size of resize_2849_1_TB_mem_in_csim_00000000.bin */
constexpr uint16_t CVFAST_PARAM_SIZE = 24;          /* size of cvfast_2849_0_8_10_TB_mem_in_csim_00000000.bin */
constexpr uint16_t SLAMFAST_PARAM_SIZE = 16;        /* size of slamfast_2849_0_TB_mem_in_csim_00000000.bin */

#define DRP_GAUSSIAN_BLUR_OUTPUT_ADDRESS (DRP_MIDDLE_ADDRESS + 0 * DRP_ALGORITHM_STRIDE)
#define DRP_ORB_DESCRIPTORS_OUTPUT_ADDRESS 0x8BB00000
#define DRP_RESIZE_OUTPUT_ADDRESS 0x8B800000
#define DRP_CVFAST_OUTPUT_ADDRESS (DRP_MIDDLE_ADDRESS + 1 * DRP_ALGORITHM_STRIDE)
#define DRP_SLAMFAST_OUTPUT_ADDRESS DRP_CVFAST_OUTPUT_ADDRESS

#define DRP_GAUSSIAN_BLUR_INPUT_ADDRESS DRP_RESIZE_OUTPUT_ADDRESS
#define DRP_ORB_DESCRIPTORS_INPUT_IMAGE_ADDRESS DRP_GAUSSIAN_BLUR_OUTPUT_ADDRESS
#define DRP_ORB_DESCRIPTORS_INPUT_KEYPOINTS_ADDRESS 0x8BA00000
#define DRP_RESIZE_INPUT_ADDRESS DRP_RESIZE_OUTPUT_ADDRESS
#define DRP_CVFAST_INPUT_ADDRESS 0x8BA00000
#define DRP_SLAMFAST_INPUT_ADDRESS DRP_CVFAST_INPUT_ADDRESS

constexpr uintptr_t pyramid_address[8] = {
    DRP_RESIZE_INPUT_ADDRESS + image_data_stride[0],
    DRP_RESIZE_INPUT_ADDRESS + image_data_stride[1],
    DRP_RESIZE_INPUT_ADDRESS + image_data_stride[2],
    DRP_RESIZE_INPUT_ADDRESS + image_data_stride[3],
    DRP_RESIZE_INPUT_ADDRESS + image_data_stride[4],
    DRP_RESIZE_INPUT_ADDRESS + image_data_stride[5],
    DRP_RESIZE_INPUT_ADDRESS + image_data_stride[6],
    DRP_RESIZE_INPUT_ADDRESS + image_data_stride[7]};

#define GAUSSIAN_BLUR_CONFIG_PATH "configuration_code/gaussian_blur_drp_out.bin"
#define ORB_DESCRIPTORS_CONFIG_PATH "configuration_code/orb_descriptors_drp_out.bin"
#define RESIZE_CONFIG_PATH "configuration_code/resize_drp_out.bin"
#define CVFAST_CONFIG_PATH "./configuration_code/cvfast_drp_out.bin"
#define SLAMFAST_CONFIG_PATH "configuration_code/slamfast_drp_out.bin"

typedef struct {
    uint16_t x;
    uint16_t y;
} Point2us;

typedef struct {
    Point2us pt;
    int16_t response;
    uint16_t padding;
} KeyPoint_FAST;

typedef struct {
    Point2us pt;
    uint8_t angle[4]; // the upper 16bit is the integer part, the lower 16bit is the decimal part
} KeyPoint_ORB;

uint32_t GetConfigurationSize(const char* path);
bool LoadBinConfigurations(const int drp_fd, const bool use_cvfast);

bool GaussianBlurSetup(const int drp_fd, const size_t level);
bool GaussianBlurStart(const int drp_fd, const cv::Mat& src, const size_t level);
bool GaussianBlurStart(const int drp_fd, const uint16_t cols, const uint16_t rows, const size_t level);
bool GaussianBlurFinish(const int drp_fd, const size_t level);
bool GaussianBlurFinish(const int drp_fd, const size_t level, cv::Mat& dst);
bool GaussianBlur(const int drp_fd, const cv::Mat& src, const size_t level, cv::Mat& dst);

bool computeOrbDescriptorsSetup(const int drp_fd, const size_t level, const size_t trial);
bool computeOrbDescriptorsStart(const int drp_fd, const cv::Mat& src, const std::vector<cv::KeyPoint>& src_keypoints, const size_t level, const size_t trial);
bool computeOrbDescriptorsStart(const int drp_fd, const uint16_t cols, const uint16_t rows, const std::vector<cv::KeyPoint>& src_keypoints, const size_t level, const size_t trial);
bool computeOrbDescriptorsFinish(const int drp_fd, const std::vector<cv::KeyPoint>& src_keypoints, const size_t level, const size_t trial, cv::Mat& dst);
bool computeOrbDescriptors(const int drp_fd, const cv::Mat& src, const std::vector<cv::KeyPoint>& src_keypoints, const size_t level, const size_t trial, cv::Mat& dst);

bool ResizeSetup(const int drp_fd, const size_t input_level);
bool ResizeStart(const int drp_fd, const cv::Mat& src, const size_t input_level, const cv::Size output_image_size);
bool ResizeStart(const int drp_fd, const cv::Size input_image_size, const size_t input_level, const cv::Size output_image_size);
bool ResizeFinish(const int drp_fd, const size_t input_level, const cv::Size output_image_size, cv::Mat& dst);
bool Resize(const int drp_fd, const cv::Mat& src, const size_t input_level, const cv::Size output_image_size, cv::Mat& dst);

bool CVFASTSetup(const int drp_fd);
bool CVFASTStart(const int drp_fd, const cv::Mat& src, const size_t level, const int thresholds[2]);
bool CVFASTFinish(const int drp_fd, const size_t level, std::vector<cv::KeyPoint>& dst);
bool CVFAST(const int drp_fd, const cv::Mat& src, const size_t level, const int thresholds[2], std::vector<cv::KeyPoint>& dst);

bool SLAMFASTSetup(const int drp_fd, const size_t input_level);
bool SLAMFASTStart(const int drp_fd, const cv::Mat& src, const size_t level);
bool SLAMFASTStart(const int drp_fd, const uint16_t cols, const uint16_t rows, const size_t level);
bool SLAMFASTFinish(const int drp_fd, const size_t level, std::vector<cv::KeyPoint>& dst);
bool SLAMFAST(const int drp_fd, const cv::Mat& src, const size_t level, std::vector<cv::KeyPoint>& dst);

} // namespace drp

#endif // DRP_LIBRARY_H