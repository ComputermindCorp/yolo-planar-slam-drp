#pragma once

#include "ImageLoading.h"
#include "Enum.h"

namespace ORB_SLAM2 {

class ImageLoading;

class DatasetImageLoading : public ImageLoading {
public:
    DatasetImageLoading(const eSensor sensor,
                        const std::string image_dir,
                        const std::vector<std::string> image_filenames,
                        const std::vector<std::string> depth_filenames,
                        const std::vector<double> timestamps);

    void Run();

protected:
    const std::string image_dir_;
    const std::vector<std::string> image_filenames_;
    const std::vector<std::string> depth_filenames_;
    const std::vector<double> timestamps_;
    const size_t frane_n_;
};

} // namespace ORB_SLAM2
