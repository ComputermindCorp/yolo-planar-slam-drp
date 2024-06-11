#include "YoloDetector.h"

#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>

#include <sys/stat.h>

namespace ORB_SLAM2 {

YoloDetector::YoloDetector() {
}

void YoloDetector::YoloObjectDetect(const cv::Mat image, std::vector<YoloBoundingBox>& yoloBoundingBoxList, int detector_size_width, int detector_size_height) {
    fprintf(stderr, "YoloDetector::YoloObjectDetect is not available.\n");
    exit(EXIT_FAILURE);
}

YoloBoundingBox::YoloBoundingBox(cv::Rect2f input_rect, std::string input_label, float input_confidence) {
    this->rect = input_rect;
    this->label = input_label;
    this->confidence = input_confidence;
    this->id = 0;
}

YoloBoundingBox::YoloBoundingBox(float x1, float y1, float x2, float y2, std::string input_label, float input_confidence) {
    cv::Point2i p1 = cv::Point2i(x1, y1);
    cv::Point2i p2 = cv::Point2i(x2, y2);
    this->rect = cv::Rect2i(p1, p2);
    this->label = input_label;
    this->confidence = input_confidence;
    this->id = 0;
}

} // namespace ORB_SLAM2