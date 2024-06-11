#include "opencva.h"

#include <vector>
#include <cassert>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

#define OPENCVA_RESIZE_MIN_SIZE 2
#define OPENCVA_RESIZE_MAX_SIZE 65535
#define OPENCVA_RESIZE_MIN_CH 1
#define OPENCVA_RESIZE_MAX_CH 4

#define OPENCVA_GAUSSIAN_BLUR_MIN_SIZE 16
#define OPENCVA_GAUSSIAN_BLUR_MAX_WIDTH 3840
#define OPENCVA_GAUSSIAN_BLUR_MAX_HEIGHT 2160

#define OPENCVA_FAST_MIN_SIZE 16
#define OPENCVA_FAST_MAX_WIDTH 3840
#define OPENCVA_FAST_MAX_HEIGHT 2160
#define OPENCVA_FAST_MIN_THRESHOLD 0
#define OPENCVA_FAST_MAX_THRESHOLD 255

namespace opencva {

bool initialize() {
    unsigned long OCA_f[DRP_FUNC_NUM];
    for (size_t i = 0; i < DRP_FUNC_NUM; i++) {
        OCA_f[i] = OPENCVA_FUNC_DISABLE;
    }

    /* Enable OpenCV Accelerator */
    OCA_f[DRP_FUNC_RESIZE] = OPENCVA_FUNC_ENABLE;
    OCA_f[DRP_FUNC_GAUSSIAN] = OPENCVA_FUNC_ENABLE;
    OCA_f[DRP_FUNC_FAST] = OPENCVA_FUNC_ENABLE;
    int activate_succeed = OCA_Activate(&OCA_f[0]);
    if (activate_succeed != 0) {
        fprintf(stderr, "Failed to opencva::initialize.\n");
        return false;
    }

    return true;
}

bool finalize() {
    unsigned long OCA_f[DRP_FUNC_NUM];
    for (size_t i = 0; i < DRP_FUNC_NUM; i++) {
        OCA_f[i] = OPENCVA_FUNC_DISABLE;
    }

    /* Disable OpenCV Accelerator */
    int activate_succeed = OCA_Activate(&OCA_f[0]);
    if (activate_succeed != 0) {
        fprintf(stderr, "Failed to opencva::finalize.\n");
        return false;
    }

    return true;
}

bool disable() {
    return finalize();
}

bool resize(const cv::Mat& src, const size_t input_level, const cv::Size output_image_size, cv::Mat& dst) {
    assert(src.type() == CV_8UC1);

    assert(OPENCVA_RESIZE_MIN_SIZE <= src.cols);
    assert(OPENCVA_RESIZE_MIN_SIZE <= src.rows);
    assert(src.cols <= OPENCVA_RESIZE_MAX_SIZE);
    assert(src.rows <= OPENCVA_RESIZE_MAX_SIZE);

    assert(OPENCVA_RESIZE_MIN_SIZE <= output_image_size.width);
    assert(OPENCVA_RESIZE_MIN_SIZE <= output_image_size.height);
    assert(output_image_size.width <= OPENCVA_RESIZE_MAX_SIZE);
    assert(output_image_size.height <= OPENCVA_RESIZE_MAX_SIZE);

    cv::resize(src, dst, output_image_size, 0, 0, cv::INTER_LINEAR);

    return true;
}

bool gaussian_blur(const cv::Mat& src, const size_t level, cv::Mat& dst) {
    assert(src.type() == CV_8UC1);

    assert(OPENCVA_GAUSSIAN_BLUR_MIN_SIZE <= src.cols);
    assert(OPENCVA_GAUSSIAN_BLUR_MIN_SIZE <= src.rows);
    assert(src.cols <= OPENCVA_GAUSSIAN_BLUR_MAX_WIDTH);
    assert(src.rows <= OPENCVA_GAUSSIAN_BLUR_MAX_HEIGHT);

    cv::Size ksize(7, 7);
    double sigmaX = 2.0;
    double sigmaY = 2.0;
    cv::GaussianBlur(src, dst, ksize, sigmaX, sigmaY, cv::BORDER_REFLECT_101);

    return true;
}

bool cvfast(const cv::Mat& src, const size_t level, const int threshold, std::vector<cv::KeyPoint>& dst) {
    assert(src.type() == CV_8UC1);

    assert(OPENCVA_FAST_MIN_SIZE <= src.cols);
    assert(OPENCVA_FAST_MIN_SIZE <= src.rows);
    assert(src.cols <= OPENCVA_FAST_MAX_WIDTH);
    assert(src.rows <= OPENCVA_FAST_MAX_HEIGHT);

    assert(OPENCVA_FAST_MIN_THRESHOLD <= threshold);
    assert(threshold <= OPENCVA_FAST_MAX_THRESHOLD);

    bool nonmaxSuppression = true;
    cv::FAST(src, dst, threshold, nonmaxSuppression);

    return true;
}

} // namespace opencva
