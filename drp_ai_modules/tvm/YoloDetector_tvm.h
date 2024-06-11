#pragma once

#include "YoloDetector.h"

#include <opencv2/core.hpp>

#include "MeraDrpRuntimeWrapper.h"
#include "PreRuntime.h"

/*****************************************
 * Macro
 ******************************************/
/*Input image info*/
#define INPUT_IMAGE_H (480)
#define INPUT_IMAGE_W (640)
#define INPUT_IMAGE_C (3)
/*Model input info*/
#define MODEL_IN_H (224)
#define MODEL_IN_W (224)
#define MODEL_IN_C (3)

/*BMP Header size for Windows Bitmap v3*/
#define FILEHEADERSIZE (14)
#define INFOHEADERSIZE_W_V3 (40)

namespace ORB_SLAM2 {
class YoloDetector_tvm : public YoloDetector {
public:
    YoloDetector_tvm();
    ~YoloDetector_tvm();

    bool YoloObjectDetectSetup();
    bool YoloObjectDetectStart();
    bool YoloObjectDetectFinish();

    // ResNet does not output bounding box because it is not a model of YOLO.
    bool YoloObjectDetect(const cv::Mat& img, std::vector<YoloBoundingBox>&);

private:
    /* Label list file for ImageNet*/
    const std::string labels;
    /* Model Binary */
    const std::string model_dir;
    /* Pre-processing Runtime Object */
    const std::string pre_dir;

    /* Map to store label list */
    std::map<int, std::string> label_file_map;
    /* DRP-AI TVM[*1] Runtime object */
    MeraDrpRuntimeWrapper runtime;
    /* Pre-processing Runtime object */
    PreRuntime preruntime;
    /*File descriptor for u-dma-buf*/
    uint8_t udmabuf_fd;
    /* u-dma-buf start addres */
    uint64_t udmabuf_addr_start;
    int udmabuf_size = INPUT_IMAGE_H * INPUT_IMAGE_W * INPUT_IMAGE_C;

    // const uint64_t input_image_size;
    /* Image buffer (u-dma-buf) */
    uint8_t* img_buffer;

    cv::Mat bgr_image_;
};

} // namespace ORB_SLAM2
