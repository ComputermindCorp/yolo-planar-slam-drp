#pragma once

#include <opencv2/core.hpp>

/* OpenCVA Circuit Number */
#define DRP_FUNC_NUM (16)
#define DRP_FUNC_RESIZE (0)
#define DRP_FUNC_CVT_YUV2BGR (2)
#define DRP_FUNC_CVT_NV2BGR (2)
#define DRP_FUNC_GAUSSIAN (4)
#define DRP_FUNC_DILATE (5)
#define DRP_FUNC_ERODE (6)
#define DRP_FUNC_FILTER2D (7)
#define DRP_FUNC_SOBEL (8)
#define DRP_FUNC_A_THRESHOLD (9)
#define DRP_FUNC_TMPLEATMATCH (10)
#define DRP_FUNC_AFFINE (11)
#define DRP_FUNC_PYR_DOWN (12)
#define DRP_FUNC_PYR_UP (13)
#define DRP_FUNC_PERSPECTIVE (14)
#define DRP_FUNC_FAST (15)

/* OpenCVA Activate */
#define OPENCVA_FUNC_DISABLE (0)
#define OPENCVA_FUNC_ENABLE (1)
#define OPENCVA_FUNC_NOCHANGE (2)

namespace opencva {

bool initialize();
bool finalize();
bool disable();

bool resize(const cv::Mat& src, const size_t input_level, const cv::Size output_image_size, cv::Mat& dst);
bool gaussian_blur(const cv::Mat& src, const size_t level, cv::Mat& dst);
bool cvfast(const cv::Mat& src, const size_t level, const int threshold, std::vector<cv::KeyPoint>& dst);

} // namespace opencva