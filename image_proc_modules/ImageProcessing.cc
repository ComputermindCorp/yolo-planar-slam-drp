#include "ImageProcessing.h"

#include <cassert>

#include "Frame_drp.h"

#if !defined(ENABLE_SLAMFAST)
#include "Frame_opencva.h"
#endif

#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

namespace ORB_SLAM2 {

ImageProcessing::ImageProcessing(const int drp_fd,
                                 const bool bRGB,
                                 const eSensor sensor,
#if defined(ENABLE_DRP)
                                 ORBextractor_drp* extractor,
#else
                                 ORBextractor* extractor,
#endif
                                 ORBVocabulary* vocabulary,
                                 YoloDetector* yolo_detector,
                                 const cv::Mat camera_k,
                                 const cv::Mat dist_coef,
                                 const float bf,
                                 const float th_depth,
                                 int max_features,
                                 const bool use_drp,
                                 const bool use_drp_ai,
                                 const bool use_opencva)
    : tracking_ptr_(NULL),
      drp_fd_(drp_fd),
      bRGB_(bRGB),
      sensor_(sensor),
      use_drp_(use_drp),
      use_drp_ai_(use_drp_ai),
      use_opencva_(use_opencva),
      frame_id_(0),
      timestamp_(0.0),
      max_features_(max_features),
      extractor_(extractor),
      vocabulary_(vocabulary),
      yolo_detector_(yolo_detector),
      camera_k_(camera_k),
      dist_coef_(dist_coef),
      bf_(bf),
      th_depth_(th_depth),
      mbFinishRequested(false),
      mbFinished(true) {
    wait_for_next_frame.store(true);
    wait_for_next_max_features.store(false); // Given in the argument.
    changed_max_features.store(false);       // Given in the argument.
    finish_processing.store(false);
}

void ImageProcessing::Run() {
    mbFinished = false;

    while (true) {
        if (tracking_ptr_
            && !wait_for_next_frame.load()
            && !finish_processing.load()) {
            MT_START(mt_image_processing);

            const int nlevels = extractor_->GetLevels();
            vToDistributeKeys_.reserve(nlevels);

            MT_START(mt_image_processing_first);
            FirstProcess();
            MT_FINISH(mt_image_processing_first);

            MT_START(mt_image_processing_second[0]);
            SecondProcess(0);
            MT_FINISH(mt_image_processing_second[0]);

            MT_START(mt_wait_for_next_max_features_to_false);
            while (wait_for_next_max_features.load()) {
                usleep(1000); // Sleep 1ms
            }
            MT_FINISH(mt_wait_for_next_max_features_to_false);

            assert(0 <= GetMaxFeatures());

            if (!wait_for_next_max_features.load() && changed_max_features.load()) {
                changed_max_features.store(false);

                if (use_drp_) {
                    frame_ptr_->reset_to_orb_descriptors();
                }

                MT_START(mt_image_processing_second[1]);
                SecondProcess(1);
                MT_FINISH(mt_image_processing_second[1]);
            }
            else {
                for (int level = 0; level < nlevels; level++) {
                    MT_PUSH_BACK_ZERO(mt_cell_postprocess[1][level]);
                    MT_PUSH_BACK_ZERO(mt_compute_orb[1][level]);
                    MT_PUSH_BACK_ZERO(mt_compute_orientation[1][level]);
                    MT_PUSH_BACK_ZERO(mt_distribute_keypoints_via_tree[1][level]);
                    MT_PUSH_BACK_ZERO(mt_drp_orb_descriptors_calc_param[1][level]);
                    MT_PUSH_BACK_ZERO(mt_drp_orb_descriptors_cast_to_drp[1][level]);
                    MT_PUSH_BACK_ZERO(mt_drp_orb_descriptors_p2u[1][level]);
                    MT_PUSH_BACK_ZERO(mt_drp_orb_descriptors_postprocess[1][level]);
                    MT_PUSH_BACK_ZERO(mt_drp_orb_descriptors_start[1][level]);
                    MT_PUSH_BACK_ZERO(mt_drp_orb_descriptors_u2p_input_image[1][level]);
                    MT_PUSH_BACK_ZERO(mt_drp_orb_descriptors_u2p_input_keypoints[1][level]);
                    MT_PUSH_BACK_ZERO(mt_drp_orb_descriptors_u2p_param[1][level]);
                    MT_PUSH_BACK_ZERO(mt_drp_time[1][drp::STATE_ORB_DESCRIPTORS][level]);
                }
                MT_PUSH_BACK_ZERO(mt_image_processing_second[1]);
                MT_PUSH_BACK_ZERO(mt_extract_orb_desc[1]);
                MT_PUSH_BACK_ZERO(mt_last_processing_of_constructor[1]);
            }

            finish_processing.store(true);

            MT_FINISH(mt_image_processing);

#if defined(ENABLE_DUMP) || defined(ENABLE_DEBUG_OUTPUT)
            ORBextractor::frame_idx++;
#endif
        }

        if (tracking_ptr_
            && tracking_ptr_->wait_for_next_frame.load()
            && finish_processing.load()) {
            MT_START(mt_image_processing_push_result);
            PushResult();
            MT_FINISH(mt_image_processing_push_result);

            frame_id_++;
        }

        if (CheckFinish())
            break;

        usleep(1000); // Sleep 1ms
    }

    SetFinish();
}

void ImageProcessing::FirstProcess() {
    // Please refer to https://github.com/BZDOLiVE/YoloPlanarSLAM/blob/45510b59d41eb6bcd28e2657c39cb6ab664b5c1f/src/Tracking.cc#L276-L296

    cv::Mat input_image;

    if (sensor_ == eSensor::MONOCULAR) {
        input_image = *input_image_ptr_;
    }
    else if (sensor_ == eSensor::STEREO) {
        fprintf(stderr, "ImageProcessing not support Stereo mode.\n");
        exit(EXIT_FAILURE);
    }
    else if (sensor_ == eSensor::RGBD) {
        input_image = *input_image_ptr_;
        depth_image_ = *depth_image_ptr_;
    }
    else {
        fprintf(stderr, "Unknown mode detected in ImageProcessing::Run.\n");
        exit(EXIT_FAILURE);
    }

    MT_START(mt_input_to_gray);

    if (input_image.channels() == 1) {
        gray_image_ = input_image;
    }
    else if (input_image.channels() == 3) {
        if (bRGB_)
            cv::cvtColor(input_image, gray_image_, cv::COLOR_RGB2GRAY);
        else
            cv::cvtColor(input_image, gray_image_, cv::COLOR_BGR2GRAY);
    }
    else if (input_image.channels() == 4) {
        if (bRGB_)
            cv::cvtColor(input_image, gray_image_, cv::COLOR_RGBA2GRAY);
        else
            cv::cvtColor(input_image, gray_image_, cv::COLOR_BGRA2GRAY);
    }

    MT_FINISH(mt_input_to_gray);

    MT_START(mt_convert_depth);

    if (sensor_ == eSensor::RGBD) {
        if ((fabs(tracking_ptr_->mDepthMapFactor - 1.0f) > 1e-5) || depth_image_.type() != CV_32F)
            depth_image_.convertTo(depth_image_, CV_32F, tracking_ptr_->mDepthMapFactor);
    }

    MT_FINISH(mt_convert_depth);

    MT_START(mt_copy_to_tracking);
    tracking_ptr_->mImGray = gray_image_.clone();
    tracking_ptr_->mImDepth = depth_image_.clone();
    tracking_ptr_->timestamp = timestamp_;
    MT_FINISH(mt_copy_to_tracking);

    MT_START(mt_create_frame);

    if (sensor_ == eSensor::MONOCULAR) {
#if defined(ENABLE_DRP)
#if !defined(ENABLE_SLAMFAST)
        if (use_opencva_) {
            frame_ptr_.reset(new Frame_opencva(drp_fd_,
                                               tracking_ptr_->mImGray,
                                               timestamp_,
                                               extractor_,
                                               vocabulary_,
                                               this,
#if defined(ENABLE_DRP_AI)
                                               yolo_detector_,
#endif
                                               camera_k_,
                                               dist_coef_,
                                               bf_,
                                               th_depth_,
                                               sensor_));
        }
        else
#endif
            if (use_drp_) {
            frame_ptr_.reset(new Frame_drp(drp_fd_,
                                           tracking_ptr_->mImGray,
                                           timestamp_,
                                           extractor_,
                                           vocabulary_,
                                           this,
#if defined(ENABLE_DRP_AI)
                                           yolo_detector_,
#endif
                                           camera_k_,
                                           dist_coef_,
                                           bf_,
                                           th_depth_,
                                           sensor_));
        }
        else
#endif
        {
            frame_ptr_.reset(new Frame(tracking_ptr_->mImGray,
                                       timestamp_,
                                       extractor_,
                                       vocabulary_,
                                       this,
                                       camera_k_,
                                       dist_coef_,
                                       bf_,
                                       th_depth_,
                                       use_drp_));
        }
    }
    else if (sensor_ == eSensor::RGBD) {
#if defined(ENABLE_DRP)
#if !defined(ENABLE_SLAMFAST)
        if (use_opencva_) {
            frame_ptr_.reset(new Frame_opencva(drp_fd_,
                                               tracking_ptr_->mImGray,
                                               tracking_ptr_->mImDepth,
                                               timestamp_,
                                               extractor_,
                                               vocabulary_,
                                               this,
#if defined(ENABLE_DRP_AI)
                                               yolo_detector_,
#endif
                                               camera_k_,
                                               dist_coef_,
                                               bf_,
                                               th_depth_,
                                               sensor_));
        }
        else
#endif
            if (use_drp_) {
            frame_ptr_.reset(new Frame_drp(drp_fd_,
                                           tracking_ptr_->mImGray,
                                           tracking_ptr_->mImDepth,
                                           timestamp_,
                                           extractor_,
                                           vocabulary_,
                                           this,
#if defined(ENABLE_DRP_AI)
                                           yolo_detector_,
#endif
                                           camera_k_,
                                           dist_coef_,
                                           bf_,
                                           th_depth_,
                                           sensor_));
        }
        else
#endif
        {
            frame_ptr_.reset(new Frame(tracking_ptr_->mImGray,
                                       tracking_ptr_->mImDepth,
                                       timestamp_,
                                       extractor_,
                                       vocabulary_,
                                       this,
                                       camera_k_,
                                       dist_coef_,
                                       bf_,
                                       th_depth_,
                                       use_drp_));
        }
    }

    MT_FINISH(mt_create_frame);

    MT_START(mt_input_to_bgr);

    cv::Mat bgr_image;
    if (input_image.channels() == 1) {
        if (bRGB_)
            cvtColor(input_image, bgr_image, cv::COLOR_GRAY2BGR);
    }
    else if (input_image.channels() == 3) {
        if (bRGB_)
            cvtColor(input_image, bgr_image, cv::COLOR_RGB2BGR);
        else
            bgr_image = input_image;
    }
    else if (input_image.channels() == 4) {
        if (bRGB_)
            cvtColor(input_image, bgr_image, cv::COLOR_RGBA2BGR);
        else
            cvtColor(input_image, bgr_image, cv::COLOR_BGRA2BGR);
    }

    MT_FINISH(mt_input_to_bgr);

    MT_START(mt_yolo_object_detect);
#if defined(ENABLE_DRP_AI)
    if (use_drp_ai_) {
        yolo_detector_->imgBGR = bgr_image;

        bool setup_succeed = yolo_detector_->YoloObjectDetectSetup();
        if (!setup_succeed) {
            std::cerr << "YoloObjectDetectSetup error" << std::endl;
            exit(EXIT_FAILURE);
        }

        bool start_succeed = yolo_detector_->YoloObjectDetectStart();
        if (!start_succeed) {
            std::cerr << "YoloObjectDetectStart error" << std::endl;
            exit(EXIT_FAILURE);
        }

        frame_ptr_->SetYoloDetector(yolo_detector_);
    }
    else
#endif
    {
        std::vector<YoloBoundingBox> yoloBoundingBoxList;
        yolo_detector_->YoloObjectDetect(bgr_image, yoloBoundingBoxList, 416, 416);
        frame_ptr_->SetYoloBoundingBoxList(yoloBoundingBoxList);
    }
    MT_FINISH(mt_yolo_object_detect);

    if (!use_drp_) {
        frame_ptr_->mpORBextractorLeft->OperatorImagePyramid(
            gray_image_);

        const int nlevels = frame_ptr_->mpORBextractorLeft->GetLevels();
        blurred_images_.resize(nlevels);
        for (int level = 0; level < nlevels; ++level) {
            // Preprocess the resized image
            blurred_images_[level] = frame_ptr_->mpORBextractorLeft->mvImagePyramid[level].clone();
            frame_ptr_->mpORBextractorLeft->OperatorGaussianBlur(blurred_images_[level], level);
        }

        for (int level = 0; level < nlevels; ++level) {
            vToDistributeKeys_[level].clear();
            vToDistributeKeys_[level].reserve(frame_ptr_->mpORBextractorLeft->mvImagePyramid[level].cols
                                              * frame_ptr_->mpORBextractorLeft->mvImagePyramid[level].rows
                                              * MAX_RATIO_OF_KEYPOINTS_TO_PIXELS);

            frame_ptr_->mpORBextractorLeft->OperatorSLAMFAST(vToDistributeKeys_[level], level);
        }
    }

#if defined(ENABLE_DRP)
    if (use_drp_) {
        // busy wait
        while (!frame_ptr_->done_fast()) {
            frame_ptr_->drp_preprocess();
            bool execute_succeed = frame_ptr_->drp_execute(0);
            if (!execute_succeed) {
                fprintf(stderr, "Failed to Frame_drp::drp_execute.\n");
                exit(EXIT_FAILURE);
            }
            frame_ptr_->drp_postprocess();
        }
    }
#endif
}

void ImageProcessing::SecondProcess(const size_t trial) {
    if (!use_drp_) {
        const int nlevels = frame_ptr_->mpORBextractorLeft->GetLevels();
        frame_ptr_->mvKeyPoints.resize(nlevels);
        for (int level = 0; level < nlevels; ++level) {
            frame_ptr_->mpORBextractorLeft->OperatorPrune(vToDistributeKeys_[level], frame_ptr_->mvKeyPoints, GetMaxFeatures(), level, trial);
            // compute orientations
            frame_ptr_->mpORBextractorLeft->OperatorOrientation(frame_ptr_->mvKeyPoints, level, trial);

            if (!wait_for_next_max_features.load() && changed_max_features.load()) {
                // Interrupt to redo ImageProcessing::SecondProcess
                return;
            }
        }
    }

#if defined(ENABLE_DRP)
    if (use_drp_) {
        assert(frame_ptr_->get_status_algorithm() == drp::STATE_ORB_DESCRIPTORS);
        assert(frame_ptr_->get_status_level() == 0);
        frame_ptr_->postprocess_fast(GetMaxFeatures(), trial);
    }
#endif

    if (!wait_for_next_max_features.load() && changed_max_features.load()) {
        // Interrupt to redo ImageProcessing::SecondProcess
        return;
    }

    frame_ptr_->LastProcessingOfConstructor(gray_image_, camera_k_);

    if (!use_drp_) {
        // calculate descriptors
        MT_START(mt_extract_orb_desc[trial]);
        frame_ptr_->ExtractOrbDescriptors(blurred_images_, trial);
        MT_FINISH(mt_extract_orb_desc[trial]);
    }

#if defined(ENABLE_DRP)
    if (use_drp_) {
        // busy wait
        while (!frame_ptr_->drp_is_finished()) {
            frame_ptr_->drp_preprocess();
            bool execute_succeed = frame_ptr_->drp_execute(trial);
            if (!execute_succeed) {
                fprintf(stderr, "Failed to Frame_drp::drp_execute.\n");
                exit(EXIT_FAILURE);
            }
            frame_ptr_->drp_postprocess();

            if (!wait_for_next_max_features.load() && changed_max_features.load()) {
                // Interrupt to redo ImageProcessing::SecondProcess
                return;
            }
        }

        MT_START(mt_last_processing_of_constructor[trial]);
        frame_ptr_->LastProcessingOfConstructor();
        MT_FINISH(mt_last_processing_of_constructor[trial]);
    }
#endif
}

void ImageProcessing::PushResult() {
    tracking_ptr_->mCurrentFrame = std::move(frame_ptr_);

    wait_for_next_frame.store(true);
    wait_for_next_max_features.store(true);
    changed_max_features.store(false);
    finish_processing.store(false);
    tracking_ptr_->wait_for_next_frame.store(false);
}

void ImageProcessing::SetMaxFeatures(const int max_features) {
    std::unique_lock<std::mutex> lock(mutex_max_features);

    if (max_features_ != max_features) {
        max_features_ = max_features;
        changed_max_features.store(true);
    }
    wait_for_next_max_features.store(false);
}

int ImageProcessing::GetMaxFeatures() {
    std::unique_lock<std::mutex> lock(mutex_max_features);

    return max_features_;
}

void ImageProcessing::SetTracker(Tracking* tracking_ptr) {
    tracking_ptr_ = tracking_ptr;
}

void ImageProcessing::RequestFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool ImageProcessing::isFinished() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

bool ImageProcessing::CheckFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ImageProcessing::SetFinish() {
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

} // namespace ORB_SLAM2