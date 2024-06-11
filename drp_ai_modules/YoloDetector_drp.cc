#include "YoloDetector_drp.h"

#include <sys/stat.h>
#include <unistd.h>
#include <time.h>

#include "drp.h"
#include "command/object_detection.h"

#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

namespace ORB_SLAM2 {

YoloDetector_drp::YoloDetector_drp()
    : YoloDetector::YoloDetector(),
      board_ip(BOARD_IP),
      drp_max_freq(DRP_MAX_FREQ),
      drpai_freq(DRPAI_FREQ),
      input_image_size(640 * 640 * 3) {
    int32_t ret;

    p_recog_base = std::shared_ptr<RecognizeBase>(new RecognizeBase(drp_max_freq, drpai_freq));

    ret = p_recog_base->initialize(new YoloXSDenseModel());
    if (ret != 0) {
        fprintf(stderr, "Failed to RecognizeBase::initialize, Return code is %d\n", ret);
        exit(EXIT_FAILURE);
    }

    ret = p_recog_base->recognize_start();
    if (ret != 0) {
        fprintf(stderr, "Failed to RecognizeBase::recognize_start, Return code is %d\n", ret);
        exit(EXIT_FAILURE);
    }

    // Please refer to YoloExecutor.cpp provided October 13, 2023
    {
        udmabuf_fd = open("/dev/udmabuf0", O_RDWR | O_SYNC);
        if (udmabuf_fd < 0) {
            fprintf(stderr, "Failed to open /dev/udmabuf0.\n");
            exit(EXIT_FAILURE);
        }

        img_buffer = (uint8_t*)mmap(NULL, input_image_size, PROT_READ | PROT_WRITE, MAP_SHARED, udmabuf_fd, 0);
    }
}

YoloDetector_drp::~YoloDetector_drp() {
    munmap(img_buffer, input_image_size);
    close(udmabuf_fd);
    p_recog_base->recognize_end();
}

size_t YoloDetector_drp::get_file_size(const char* path) {
    struct stat statBuf;

    if (stat(path, &statBuf) != 0) {
        fprintf(stderr, "Failed to get the size of %s\n", path);
        exit(EXIT_FAILURE);
    }

    assert(0 < statBuf.st_size);

    return statBuf.st_size;
}

void YoloDetector_drp::YUVtoYUYV(const cv::Mat& yuv, cv::Mat& yuyv) {
    for (int iy = 0; iy < yuv.rows; iy++) {
        for (int ix = 0; ix < yuv.cols; ix += 2) {
            uint8_t y0 = yuv.at<cv::Vec3b>(iy, ix)[0];
            uint8_t u0 = yuv.at<cv::Vec3b>(iy, ix)[1];
            uint8_t v0 = yuv.at<cv::Vec3b>(iy, ix)[2];
            uint8_t y1 = yuv.at<cv::Vec3b>(iy, ix + 1)[0];
            uint8_t u1 = yuv.at<cv::Vec3b>(iy, ix + 1)[1];
            uint8_t v1 = yuv.at<cv::Vec3b>(iy, ix + 1)[2];

            uint8_t u = (u0 + u1 + 1) >> 1;
            uint8_t v = (v0 + v1 + 1) >> 1;

            yuyv.at<cv::Vec2b>(iy, ix)[0] = y0;
            yuyv.at<cv::Vec2b>(iy, ix)[1] = u;
            yuyv.at<cv::Vec2b>(iy, ix + 1)[0] = y1;
            yuyv.at<cv::Vec2b>(iy, ix + 1)[1] = v;
        }
    }
}

bool YoloDetector_drp::YoloObjectDetectSetup() {
    // Nothing
    return true;
}

template<>
void YoloDetector_drp::BottomPadding<CV_8UC2>(const cv::Mat& imgYuyv, cv::Mat& squareYuyv) {
    assert(imgYuyv.rows == 480);
    assert(imgYuyv.cols == 640);
    assert(imgYuyv.channels() == 2);

    assert(squareYuyv.rows == 640);
    assert(squareYuyv.cols == 640);
    assert(squareYuyv.channels() == 2);

    cv::Mat roi = squareYuyv(cv::Rect(0, 0, imgYuyv.cols, imgYuyv.rows));
    imgYuyv.copyTo(roi);

    // Please refer to R_Capture_Thread implemented in app_yolox_cam/src/main.cpp
    /** Fill buffer with the brightness 114. */
    for (int32_t y = imgYuyv.rows; y < squareYuyv.rows; y++) {
        cv::Vec2b* row_ptr = squareYuyv.ptr<cv::Vec2b>(y);
        for (int32_t x = 0; x < squareYuyv.cols; x++) {
            row_ptr[x] = cv::Vec2b(114, 128);
        }
    }
}

template<>
void YoloDetector_drp::BottomPadding<CV_8UC3>(const cv::Mat& imgRGB, cv::Mat& squareRGB) {
    assert(imgRGB.rows == 480);
    assert(imgRGB.cols == 640);
    assert(imgRGB.channels() == 3);

    assert(squareRGB.rows == 640);
    assert(squareRGB.cols == 640);
    assert(squareRGB.channels() == 3);

    cv::Mat roi = squareRGB(cv::Rect(0, 0, imgRGB.cols, imgRGB.rows));
    imgRGB.copyTo(roi);

    // Please refer to R_Capture_Thread implemented in app_yolox_cam/src/main.cpp
    /** Fill buffer with the brightness 114. */
    for (int32_t y = imgRGB.rows; y < squareRGB.rows; y++) {
        cv::Vec3b* row_ptr = squareRGB.ptr<cv::Vec3b>(y);
        for (int32_t x = 0; x < squareRGB.cols; x++) {
            row_ptr[x] = cv::Vec3b(114, 114, 114);
        }
    }
}

bool YoloDetector_drp::YoloObjectDetectStart() {
    const bool convert_to_yuyv = false;
    const bool convert_to_rgb = !convert_to_yuyv;
    const bool bottom_padding = true;

    assert(imgBGR.rows == 480);
    assert(imgBGR.cols == 640);
    assert(imgBGR.channels() == 3);

    cv::Mat img;

    MT_START(mt_drp_ai_yolo_convert_color_format);

    if (convert_to_yuyv) {
        cv::Mat imgYuyv(480, 640, CV_8UC2);
        cv::Mat imgYuv24(480, 640, CV_8UC3);

        cv::cvtColor(imgBGR, imgYuv24, cv::COLOR_BGR2YUV);
        YUVtoYUYV(imgYuv24, imgYuyv);

        assert(imgYuyv.rows == 480);
        assert(imgYuyv.cols == 640);
        assert(imgYuyv.channels() == 2);
        assert(imgYuyv.isContinuous());

        img = imgYuyv;
    }
    else if (convert_to_rgb) {
        cv::Mat imgRGB(480, 640, CV_8UC3);

        cv::cvtColor(imgBGR, imgRGB, cv::COLOR_BGR2RGB);

        assert(imgRGB.rows == 480);
        assert(imgRGB.cols == 640);
        assert(imgRGB.channels() == 3);
        assert(imgRGB.isContinuous());

        img = imgRGB;
    }
    else {
        fprintf(stderr, "Incorrect setting to convert input image to DRP-AI input image.\n");
        exit(EXIT_FAILURE);
    }

    MT_FINISH(mt_drp_ai_yolo_convert_color_format);

    MT_START(mt_drp_ai_yolo_padding);

    if (bottom_padding) {
        const int color_format = convert_to_yuyv  ? CV_8UC2
                                 : convert_to_rgb ? CV_8UC3
                                                  : -1;
        assert(color_format == CV_8UC2 || color_format == CV_8UC3);

        cv::Mat square(img.cols, img.cols, color_format);
        BottomPadding<color_format>(img, square);

        assert(square.rows == 640);
        assert(square.cols == 640);
        assert(square.channels() == CV_MAT_CN(color_format));
        assert(square.isContinuous());

        img = square;
    }

    MT_FINISH(mt_drp_ai_yolo_padding);

    assert((uint64_t)(img.cols * img.rows * img.channels()) == input_image_size);

    MT_START(mt_drp_ai_yolo_u2p_input);
    memcpy(img_buffer, img.data, input_image_size);
    MT_FINISH(mt_drp_ai_yolo_u2p_input);

    p_recog_base->set_input.store(true);

    return true;
}

bool YoloDetector_drp::YoloObjectDetectFinish() {
    bool finish_succeed = !(p_recog_base->set_input.load()) && !(p_recog_base->inference_running.load());
    if (!finish_succeed) {
        return false;
    }

    MT_START(mt_drp_ai_yolo_get_object_detection);
    auto model = std::static_pointer_cast<YoloXSDenseModel>(p_recog_base->_model);
    result_object_detection = model->get_object_detection();
    MT_FINISH(mt_drp_ai_yolo_get_object_detection);

    return true;
}

bool YoloDetector_drp::YoloObjectDetect(const cv::Mat img,
                                        std::vector<YoloBoundingBox>& yoloBoundingBoxList) {
    imgBGR = img;

    bool setup_succeed = YoloObjectDetectSetup();
    if (!setup_succeed) {
        fprintf(stderr, "Failed to YoloObjectDetectSetup in YoloDetector_drp::YoloObjectDetect\n");
        return false;
    }

    bool start_succeed = YoloObjectDetectStart();
    if (!start_succeed) {
        fprintf(stderr, "Failed to YoloObjectDetectStart in YoloDetector_drp::YoloObjectDetect\n");
        return false;
    }

    // busy wait
    const clock_t start = clock();
    MT_START(mt_drp_ai_yolo_polling);
    while (true) {
        bool finish_succeed = YoloObjectDetectFinish();
        if (finish_succeed)
            break;

        usleep(1000); // Sleep 1ms
        clock_t now = clock();
        double sec = (double)(now - start) / CLOCKS_PER_SEC;
        if (drp_ai::WAITING_TIME < sec) {
            MT_FINISH(mt_drp_ai_yolo_polling);

            return YoloObjectDetectRetry();
        }
    }
    MT_FINISH(mt_drp_ai_yolo_polling);

    yoloBoundingBoxList.clear();
    for (const bbox_t bbox : result_object_detection->predict) {
        yoloBoundingBoxList.push_back(YoloBoundingBox(bbox.X, bbox.Y, bbox.X + bbox.W, bbox.Y + bbox.H, bbox.name, bbox.pred));
    }

    return true;
}

bool YoloDetector_drp::YoloObjectDetectRetry() {
    fprintf(stderr, "Failed to YoloDetector_drp::YoloObjectDetect\n");
    fprintf(stderr, "Retry YoloDetector_drp::YoloObjectDetect\n");
    fprintf(stderr, "V2x does not support DRP-AI reset function.\n");
    exit(EXIT_FAILURE);
}

} // namespace ORB_SLAM2
