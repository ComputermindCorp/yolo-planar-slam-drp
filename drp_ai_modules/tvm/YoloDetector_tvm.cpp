/*
 * Original Code (C) Copyright Edgecortix, Inc. 2022
 * Modified Code (C) Copyright Renesas Electronics Corporation 2023
 *　
 *  *1 DRP-AI TVM is powered by EdgeCortix MERA(TM) Compiler Framework.
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 */

/***********************************************************************************************************************
 * File Name    : tutorial_app.cpp
 * Version      : 1.1.0
 * Description  : DRP-AI TVM[*1] Application Example
 ***********************************************************************************************************************/

/*****************************************
 * includes
 ******************************************/
#include "YoloDetector_tvm.h"

#include <linux/drpai.h>
#include <builtin_fp16.h>
#include <fstream>
#include <sys/time.h>
#include <climits>

/* Edgecortex Functions */
std::ostream& operator<<(std::ostream& os, InOutDataType type) {
    switch (type) {
        case InOutDataType::FLOAT32:
            os << "FLOAT32";
            break;
        case InOutDataType::FLOAT16:
            os << "FLOAT16";
            break;
        case InOutDataType::OTHER:
            os << "OTHER";
            break;
        default:;
    }
    return os;
}

/*****************************************
 * Function Name     : float16_to_float32
 * Description       : Function by Edgecortex. Cast uint16_t a into float value.
 * Arguments         : a = uint16_t number
 * Return value      : float = float32 number
 ******************************************/
float float16_to_float32(uint16_t a) {
    return __extendXfYf2__<uint16_t, uint16_t, 10, float, uint32_t, 23>(a);
}

/*****************************************
 * Function Name     : LoadBinary
 * Description       : Function by Edgecortex. Load bin file into std::vector.
 * Arguments         : bin_file = *.bin filename to be read
 * Return value      : std::vector<T> = file content
 ******************************************/
template<typename T>
static std::vector<T> LoadBinary(const std::string& bin_file) {
    std::ifstream file(bin_file.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        LOG(FATAL) << "unable to open file " + bin_file;
    }

    file.seekg(0, file.end);
    const uint32_t file_size = static_cast<uint32_t>(file.tellg());
    file.seekg(0, file.beg);

    const auto file_buffer = std::unique_ptr<char>(new char[file_size]);
    file.read(file_buffer.get(), file_size);

    if (file.bad() || file.fail()) {
        LOG(FATAL) << "error occured while reading the file";
    }

    file.close();

    auto ptr = reinterpret_cast<T*>(file_buffer.get());
    const auto num_elements = file_size / sizeof(T);
    return std::vector<T>(ptr, ptr + num_elements);
}

/*****************************************
 * Function Name     : load_label_file
 * Description       : Load label list text file and return the label list that contains the label.
 * Arguments         : label_file_name = filename of label list. must be in txt format
 * Return value      : std::map<int, std::string> list = list text file which contains labels
 *                     empty if error occured
 ******************************************/
std::map<int, std::string> load_label_file(std::string label_file_name) {
    int n = 0;
    std::map<int, std::string> list;
    std::ifstream infile(label_file_name);

    if (!infile.is_open()) {
        return list;
    }

    std::string line;
    while (getline(infile, line)) {
        list[n++] = line;
        if (infile.fail()) {
            std::map<int, std::string> empty;
            return empty;
        }
    }

    return list;
}

/*****************************************
 * Function Name : softmax
 * Description   : Function for Post Processing
 * Arguments     : val[] = array to be computed Softmax
 *                 size = size of array
 * Return value  : -
 ******************************************/
void softmax(float* val, int32_t size) {
    float max_num = -INT_MAX;
    float sum = 0;
    int32_t i;
    for (i = 0; i < size; i++) {
        max_num = std::max(max_num, val[i]);
    }

    for (i = 0; i < size; i++) {
        val[i] = (float)exp(val[i] - max_num);
        sum += val[i];
    }

    for (i = 0; i < size; i++) {
        val[i] = val[i] / sum;
    }
    return;
}

/*****************************************
 * Function Name : get_drpai_start_addr
 * Description   : Function to get the start address of DRPAImem.
 * Arguments     : -
 * Return value  : uint32_t = DRPAImem start address in 32-bit.
 ******************************************/
uint64_t get_drpai_start_addr() {
    int fd = 0;
    int ret = 0;
    drpai_data_t drpai_data;

    errno = 0;

    fd = open("/dev/drpai0", O_RDWR);
    if (0 > fd) {
        LOG(FATAL) << "[ERROR] Failed to open DRP-AI Driver : errno=" << errno;
        return (uint64_t)NULL;
    }

    /* Get DRP-AI Memory Area Address via DRP-AI Driver */
    ret = ioctl(fd, DRPAI_GET_DRPAI_AREA, &drpai_data);
    if (-1 == ret) {
        LOG(FATAL) << "[ERROR] Failed to get DRP-AI Memory Area : errno=" << errno;
        return (uint64_t)NULL;
    }
    close(fd);

    return drpai_data.address;
}

/*****************************************
 * Function Name : get_udmabuf_addr
 * Description   : Function to obtain the u-dma-buf start address.
 * Arguments     : -
 * Return value  : uint32_t = u-dma-buf start address in 32-bit.
 ******************************************/
namespace {
uint64_t get_udmabuf_addr() {
    int fd = 0;
    char addr[1024];
    int32_t read_ret = 0;
    uint64_t udmabuf_addr_start = 0;
    errno = 0;

    fd = open("/sys/class/u-dma-buf/udmabuf0/phys_addr", O_RDONLY);
    if (0 > fd) {
        std::cerr << "[ERROR] Failed to open udmabuf phys_addr " << std::endl;
        return 0;
    }
    read_ret = read(fd, addr, 1024);
    if (0 > read_ret) {
        std::cerr << "[ERROR] Failed to read udmabuf phys_addr " << std::endl;
        close(fd);
        return 0;
    }
    sscanf(addr, "%lx", &udmabuf_addr_start);
    close(fd);

    return udmabuf_addr_start;
}
} // namespace

namespace ORB_SLAM2 {

YoloDetector_tvm::YoloDetector_tvm()
    : labels("synset_words_imagenet.txt"),
      model_dir("resnet50_v1_onnx"),
      pre_dir("resnet50_v1_onnx/preprocess") {
    /* About u-dma-buf
        Pre-processing Runtime requires the input buffer to be allocated in continuous memory area.
        This application uses imagebuf (u-dma-buf) memory area.
        Refer to RZ/V2MA DRP-AI Support Package for imagebuf details. */
    /* Load Label list */
    label_file_map = load_label_file(labels);
    if (label_file_map.empty()) {
        std::cerr << "[ERROR] Label file : failed to load " << labels << std::endl;
        exit(EXIT_FAILURE);
    }

    /*Load model_dir structure and its weight to runtime object */
    /*Load pre_dir object to DRP-AI */
    uint8_t ret = preruntime.Load(pre_dir, (uint32_t)0x04000000);
    if (0 < ret) {
        std::cerr << "[ERROR] Failed to run Pre-processing Runtime Load()." << std::endl;
        exit(EXIT_FAILURE);
    }

    uint64_t drpaimem_addr_start = get_drpai_start_addr();
    if (drpaimem_addr_start == (uint64_t)NULL)
        exit(EXIT_FAILURE);

    /* Currently, the start address can only use the head of the area managed by the DRP-AI. */
    runtime.LoadModel(model_dir, drpaimem_addr_start);

    /*Obtain u-dma-buf memory area starting address*/
    udmabuf_addr_start = get_udmabuf_addr();
    if (0 == udmabuf_addr_start) {
        std::cerr << "[ERROR] Failed to get u-dma-buf." << std::endl;
        exit(EXIT_FAILURE);
    }

    /* The udmabuf has already registered in driver. */

    /* Allocate image buffer in u-dma-buf memory area */
    udmabuf_fd = open("/dev/udmabuf0", O_RDWR | O_SYNC);
    if (0 > udmabuf_fd) {
        std::cerr << "[ERROR] Failed to open udmabuf " << std::endl;
        exit(EXIT_FAILURE);
    }

    img_buffer = (unsigned char*)mmap(NULL, udmabuf_size, PROT_READ | PROT_WRITE, MAP_SHARED, udmabuf_fd, 0);
    if (MAP_FAILED == img_buffer) {
        std::cerr << "[ERROR] Failed to run mmap: udmabuf " << std::endl;
        close(udmabuf_fd);
        exit(EXIT_FAILURE);
    }

    /* Write once to allocate physical memory to u-dma-buf virtual space.
     * Note: Do not use memset() for this.
     *       Because it does not work as expected. */
    for (int i = 0; i < udmabuf_size; i++) {
        img_buffer[i] = 0;
    }
}

YoloDetector_tvm::~YoloDetector_tvm() {
    munmap(img_buffer, udmabuf_size);
    close(udmabuf_fd);
}

bool YoloDetector_tvm::YoloObjectDetectSetup() {
    // Nothing
    return true;
}

bool YoloDetector_tvm::YoloObjectDetectStart() {
    /*Get input data */
    auto input_data_type = runtime.GetInputDataType(0);

    /*Load input data */
    /*Input data type can be either FLOAT32 or FLOAT16, which depends on the model */
    if (InOutDataType::FLOAT32 == input_data_type) {
        /* Pre-processing */
        assert(bgr_image_.rows == INPUT_IMAGE_H);
        assert(bgr_image_.cols == INPUT_IMAGE_W);
        assert(bgr_image_.channels() == INPUT_IMAGE_C);
        assert(bgr_image_.isContinuous());

        memcpy(img_buffer, bgr_image_.data, sizeof(uint8_t) * bgr_image_.cols * bgr_image_.rows * bgr_image_.channels());

        /*Define parameter to be changed in Pre-processing Runtime*/
        s_preproc_param_t in_param;
        in_param.pre_in_addr = (uint32_t)udmabuf_addr_start;
        in_param.pre_in_shape_w = INPUT_IMAGE_W;
        in_param.pre_in_shape_h = INPUT_IMAGE_H;
        in_param.pre_in_format = FORMAT_BGR;
        in_param.pre_out_format = FORMAT_RGB;
        /*Output variables for Pre-processing Runtime */
        void* output_ptr;
        uint32_t out_size;

        /*Run pre-processing*/
        uint8_t ret = preruntime.Pre(&in_param, &output_ptr, &out_size);
        if (0 < ret) {
            std::cerr << "[ERROR] Failed to run Pre-processing Runtime Pre()." << std::endl;
            return false;
        }

        /*Set Pre-processing output to be inference input. */
        runtime.SetInput(0, (float*)output_ptr);
    }
    else if (InOutDataType::FLOAT16 == input_data_type) {
        std::cerr << "[ERROR] Input data type : FP16." << std::endl;
        /*If your model input data type is FP16, use std::vector<uint16_t> for reading input data. */
        return false;
    }
    else {
        std::cerr << "[ERROR] Input data type : neither FP32 nor FP16." << std::endl;
        return false;
    }

    std::cout << "Running tvm runtime" << std::endl;
    runtime.Run();

    return true;
}

bool YoloDetector_tvm::YoloObjectDetectFinish() {
    /* Map list to store the classification result. */
    std::map<float, int> result;
    int result_cnt = 0;

    /* Get the number of output of the target model. For ResNet, 1 output. */
    auto output_num = runtime.GetNumOutput();
    if (output_num == 3) {
        std::cout << "[INFO] Output layer =3::maybe yolov3. End." << std::endl;
        return false;
    }
    else if (output_num != 1) {
        std::cerr << "[ERROR] Output size : not 1." << std::endl;
        return false;
    }

    /* Comparing output with reference.*/
    /* output_buffer below is tuple, which is { data type, address of output data, number of elements } */
    auto output_buffer = runtime.GetOutput(0);
    int64_t out_size = std::get<2>(output_buffer);
    /* Array to store the FP32 output data from inference. */
    float floatarr[out_size];

    /* Clear the classification result. */
    result.clear();

    if (InOutDataType::FLOAT16 == std::get<0>(output_buffer)) {
        std::cout << "Output data type : FP16." << std::endl;
        /* Extract data in FP16 <uint16_t>. */
        uint16_t* data_ptr = reinterpret_cast<uint16_t*>(std::get<1>(output_buffer));

        /* Post-processing for FP16 */
        /* Cast FP16 output data to FP32. */
        for (int n = 0; n < out_size; n++) {
            floatarr[n] = float16_to_float32(data_ptr[n]);
        }
    }
    else if (InOutDataType::FLOAT32 == std::get<0>(output_buffer)) {
        std::cout << "Output data type : FP32." << std::endl;
        /* Extract data in FP32 <float>. */
        float* data_ptr = reinterpret_cast<float*>(std::get<1>(output_buffer));
        /*Copy output data to buffer for post-processing. */
        for (int n = 0; n < out_size; n++) {
            floatarr[n] = data_ptr[n];
        }
    }
    else {
        std::cerr << "[ERROR] Output data type : not floating point type." << std::endl;
        return false;
    }

    /*Post-processing: common for FP16/FP32*/
    /* Softmax 1000 class scores. */
    softmax(&floatarr[0], out_size);
    /* Sort in decending order. */
    for (int n = 0; n < out_size; n++) {
        result[floatarr[n]] = n;
    }

    result_cnt = 0;
    /* Print Top-5 results. */
    std::cout << "Result ----------------------- " << std::endl;
    for (auto it = result.rbegin(); it != result.rend(); it++) {
        result_cnt++;
        if (result_cnt > 5)
            break;
        std::cout << "  Top " << result_cnt << " ["
                  << std::right << std::setw(5) << std::fixed << std::setprecision(1) << (float)(*it).first * 100
                  << "%] : [" << label_file_map[(*it).second] << "]" << std::endl;
    }

    return true;
}

// ResNet does not output bounding box because it is not a model of YOLO.
bool YoloDetector_tvm::YoloObjectDetect(const cv::Mat& img, std::vector<YoloBoundingBox>&) {
    bool setup_succeed = YoloObjectDetectSetup();
    if (!setup_succeed) {
        std::cerr << "[ERROR] Failed to YoloDetector_tvm::YoloObjectDetectSetup" << std::endl;
        return false;
    }

    bgr_image_ = img.clone();

    bool start_succeed = YoloObjectDetectStart();
    if (!start_succeed) {
        std::cerr << "[ERROR] Failed to YoloDetector_tvm::YoloObjectDetectStart" << std::endl;
        return false;
    }

    bool finish_succeed = YoloObjectDetectFinish();
    if (!finish_succeed) {
        std::cerr << "[ERROR] Failed to YoloDetector_tvm::YoloObjectDetectFinish" << std::endl;
        return false;
    }

    return true;
}

} // namespace ORB_SLAM2
