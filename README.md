# YOLO-Planar-SLAM-DRP

This is a YOLO-Planar-SLAM-DRP system developed based on [YoloPlanarSLAM](https://github.com/BZDOLiVE/YoloPlanarSLAM/).<br> 
This SLAM can improve tracking accuracy by removing moving object (human) with object detection AI model (YOLO).<br> 
Renesas' high-performance AI accelerator (DRP-AI) enable to operate YOLO in real-time.

This system has been tested on ubuntu 20.04.<br>
YOLO-Planar-SLAM-DRP is released under a GPLv3 license. 

**RZ-V2H EVK:** 
This evaluation board kit is ideal for evaluation of the RZ/V2H, a high-end MPU with Renesas' proprietary Dynamic Reconfigurable Processor (DRP).<br>
https://www.renesas.com/jp/ja/products/microcontrollers-microprocessors/rz-mpus/rzv2h-evk-rzv2h-quad-core-vision-ai-mpu-evaluation-kit


## Test video on TUM dataset (Grayscale)

https://github.com/ComputermindCorp/yolo-planar-slam-drp/assets/5689250/d4742b5c-7fca-461d-af36-8582b0c80f35

This video is the result of a run on 859 images with a resolution of 640 x 480 of the grayscale TUM data set.

## Test video on our dataset (include person / exclude person)

https://github.com/ComputermindCorp/yolo-planar-slam-drp/assets/5689250/1a39b602-788d-4e9e-9f0c-fc2a1e72a2db

This video is the result of the data captured by the USB camera "ELP-USBGS720P02-L36".<br>
By excluding moving objects (persons) from the feature points, tracking accuracy can be improved and loop closings are made smoother.

## Setup

To see the install guide, please refer to the [Install guide](installguide.md).

## contact us

oss_slam_contact@compmind.co.jp
