#include "DatasetImageLoading.h"

#include <cassert>

#include <opencv2/imgcodecs.hpp>

#include "measure_time.h"

#if defined(NDEBUG)

#undef NDEBUG
#include <cassert>
#define NDEBUG

#endif

namespace ORB_SLAM2 {

DatasetImageLoading::DatasetImageLoading(const eSensor sensor,
                                         const std::string image_dir,
                                         const std::vector<std::string> image_filenames,
                                         const std::vector<std::string> depth_filenames,
                                         const std::vector<double> timestamps)
    : ImageLoading(sensor),
      image_dir_(image_dir),
      image_filenames_(image_filenames),
      depth_filenames_(depth_filenames),
      timestamps_(timestamps),
      frane_n_(image_filenames_.size()) {
    assert(depth_filenames_.empty() || image_filenames_.size() == depth_filenames_.size());

    loaded_images.store(false);
}

void DatasetImageLoading::Run() {
    mbFinished = false;

    while (true) {
        if (frame_id < frane_n_ && !loaded_images.load()) {
            MT_START(mt_image_loading);
            if (mSensor == eSensor::MONOCULAR) {
                MT_START(mt_imread);
                cv::Mat img = cv::imread(image_dir_ + image_filenames_[frame_id], cv::IMREAD_UNCHANGED);

                if(img.empty()) {
                    for(int nRead_image = 0 ; nRead_image < 10 ; nRead_image++ ) {
                        if (nRead_image < 8) {
                            img = cv::imread(image_dir_ + image_filenames_[frame_id], cv::IMREAD_UNCHANGED);          
                            fprintf(stderr, "Retry to read the image data of Frame No. %ld, File name: %s \n", frame_id,  image_filenames_[frame_id].c_str());
                        } else {
                            if (frame_id == 0) {
                                img = cv::imread(image_dir_ + image_filenames_[frame_id+1], cv::IMREAD_UNCHANGED);          
                                fprintf(stderr, "Retry to read the image data of Frame No. %ld instead of No. %ld \n", frame_id+1,  frame_id);
                            } else {
                                img = cv::imread(image_dir_ + image_filenames_[frame_id-1], cv::IMREAD_UNCHANGED);          
                                fprintf(stderr, "Retry to read the image data of Frame No. %ld instead of No. %ld \n", frame_id-1,  frame_id);
                            }
                        }

                        if(!img.empty()) break;
                        usleep(1000); // Sleep 1ms
                    }
                }
                MT_FINISH(mt_imread);

                assert(!img.empty());

                {
                    std::unique_lock<std::mutex> lock(mutex_frame);
                    img_ptr_.reset(new cv::Mat(img));
                    timestamp_ = timestamps_[frame_id];
                }

                assert(!img_ptr_->empty());
            }
            else if (mSensor == eSensor::STEREO) {
                fprintf(stderr, "DatasetImageLoading not support Stereo mode.\n");
                exit(EXIT_FAILURE);
            }
            else if (mSensor == eSensor::RGBD) {
                MT_START(mt_imread);
                cv::Mat img = cv::imread(image_dir_ + image_filenames_[frame_id], cv::IMREAD_UNCHANGED);
                cv::Mat depth_img = cv::imread(image_dir_ + depth_filenames_[frame_id], cv::IMREAD_UNCHANGED);
                MT_FINISH(mt_imread);

                if(img.empty() || depth_img.empty()) {
                    for(int nRead_image = 0 ; nRead_image < 10 ; nRead_image++ ) {
                        if (nRead_image < 8) {
                            if (img.empty()) {
                                img = cv::imread(image_dir_ + image_filenames_[frame_id], cv::IMREAD_UNCHANGED);
                                fprintf(stderr, "Retry to read the image data of Frame No. %ld, File name: %s \n", frame_id,  image_filenames_[frame_id].c_str());
                            }
                            if(depth_img.empty()) {
                                depth_img = cv::imread(image_dir_ + depth_filenames_[frame_id], cv::IMREAD_UNCHANGED); 
                                fprintf(stderr, "Retry to read the depth data of Frame No. %ld, File name: %s \n", frame_id,  depth_filenames_[frame_id].c_str());
                            }
                        } else {
                            if (frame_id == 0) {
                                img       = cv::imread(image_dir_ + image_filenames_[frame_id+1], cv::IMREAD_UNCHANGED);          
                                depth_img = cv::imread(image_dir_ + depth_filenames_[frame_id+1], cv::IMREAD_UNCHANGED); 
                                fprintf(stderr, "Retry to read the image and depth data of Frame No. %ld instead of No. %ld \n", frame_id+1,  frame_id);
                            } else {
                                img       = cv::imread(image_dir_ + image_filenames_[frame_id-1], cv::IMREAD_UNCHANGED);          
                                depth_img = cv::imread(image_dir_ + depth_filenames_[frame_id-1], cv::IMREAD_UNCHANGED); 
                                fprintf(stderr, "Retry to read the image and depth data of Frame No. %ld instead of No. %ld \n", frame_id-1,  frame_id);
                            }
                        }

                        if((!img.empty()) && (!depth_img.empty())) break;
                        usleep(1000); // Sleep 1ms
                    }
                }

                assert(!img.empty());
                assert(!depth_img.empty());

                {
                    std::unique_lock<std::mutex> lock(mutex_frame);
                    img_ptr_.reset(new cv::Mat(img));
                    depth_img_ptr_.reset(new cv::Mat(depth_img));
                    timestamp_ = timestamps_[frame_id];
                }
            }
            else {
                fprintf(stderr, "Unknown mode detected in DatasetImageLoading::Run.\n");
                exit(EXIT_FAILURE);
            }

            assert(!img_ptr_->empty());

            assert(mSensor != eSensor::RGBD || !depth_img_ptr_->empty());

            MT_FINISH(mt_image_loading);

            loaded_images.store(true);
        }

        if (mpImageProcessor && mpImageProcessor->wait_for_next_frame.load() && loaded_images.load()) {
            MT_START(mt_image_loading_push_result);
            PushResult();
            MT_FINISH(mt_image_loading_push_result);

            frame_id++;
        }

        if (CheckFinish())
            break;

        usleep(1000); // Sleep 1ms
    }

    SetFinish();
}

} // namespace ORB_SLAM2
