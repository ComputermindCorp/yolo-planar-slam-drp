#ifndef YOLODETECTOR_H
#define YOLODETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

#if defined(ENABLE_DRP_AI)
#include "command/object_detection.h"
#endif

namespace ORB_SLAM2 {

class YoloBoundingBox {
private:
    cv::Rect2f rect;
    std::string label;
    float confidence;

    // This id was used for multi object tracking, but in experiment I found that MOT cost too much time, so I delete the MOT module
    // Right now, this id only used for draw colors of bbox
    int id;

public:
    YoloBoundingBox(cv::Rect2f input_rect, std::string input_label, float input_confidence);
    YoloBoundingBox(float x1, float y1, float x2, float y2, std::string input_label, float input_confidence);
    std::string GetLabel() { return this->label; }
    cv::Rect2f GetRect() { return this->rect; }
    int GetId() { return this->id; }
    float GetConfidence() { return this->confidence; }
    void SetId(int inputId) { this->id = inputId; }
};

class YoloDetector {
public:
    YoloDetector();
    void YoloObjectDetect(const cv::Mat image, std::vector<YoloBoundingBox>& yoloBoundingBoxList, int detector_size_width, int detector_size_height);

    // Dummy function
    virtual bool YoloObjectDetectSetup() { return false; };
    virtual bool YoloObjectDetectStart() { return false; };
    virtual bool YoloObjectDetectFinish() { return false; };
    virtual bool YoloObjectDetectRetry() { return false; };

    virtual void initialize_net() { return; };
    virtual void postprocess(std::vector<YoloBoundingBox>& yoloBoundingBoxList) { return; };

    cv::Mat imgBGR;

#if defined(ENABLE_DRP_AI)
    std::shared_ptr<ObjectDetection> result_object_detection;
#endif
};

} // namespace ORB_SLAM2

#endif