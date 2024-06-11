#pragma once

#include "recognize/recognize_base.h"
#include "recognize/yoloxs_dense/yoloxs_dense_model.h"

#include "YoloDetector.h"

namespace drp_ai {

constexpr double WAITING_TIME = 10.0; // sec

}

namespace ORB_SLAM2 {

class YoloDetector_drp : public YoloDetector {
public:
    YoloDetector_drp();
    ~YoloDetector_drp();

    template<int T>
    void BottomPadding(const cv::Mat& imgRGB, cv::Mat& squareRGB);

    bool YoloObjectDetectSetup();
    bool YoloObjectDetectStart();
    bool YoloObjectDetectFinish();
    bool YoloObjectDetectRetry();

    bool YoloObjectDetect(const cv::Mat img, std::vector<YoloBoundingBox>& yoloBoundingBoxList);

private:
    size_t get_file_size(const char* path);
    void YUVtoYUYV(const cv::Mat& yuv, cv::Mat& yuyv);

    std::shared_ptr<RecognizeBase> p_recog_base;
    const char* board_ip;
    const int32_t drp_max_freq;
    const int32_t drpai_freq;

    uint8_t udmabuf_fd;
    const uint64_t input_image_size;
    uint8_t* img_buffer;
};

} // namespace ORB_SLAM2
