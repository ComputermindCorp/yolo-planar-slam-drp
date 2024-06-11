#!/bin/bash -u

export OPENCV_VERSION=4.1.0
export OpenCV_DIR=$HOME/lib/${OPENCV_VERSION}/share/OpenCV

export PATH=$HOME/yolo-planar-slam/script:$PATH
export PATH=$HOME/yolo-planar-slam/script/install:$PATH
export PATH=$HOME/yolo-planar-slam/script/patch:$PATH

export LD_LIBRARY_PATH=$HOME/lib/${OPENCV_VERSION}/lib:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=$HOME/lib/Pangolin/lib:${LD_LIBRARY_PATH}
# export LD_LIBRARY_PATH=$HOME/lib/socket.io-client-cpp/lib:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/Thirdparty/DBoW2/lib:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/Thirdparty/g2o/lib:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/lib:${LD_LIBRARY_PATH}

cpu_num=`nproc --all`
if [ $cpu_num -le 2 ]; then
  export NUM_BUILD_THREADS=1
elif  [ $cpu_num -le 4 ]; then
  export NUM_BUILD_THREADS=2
else
  export NUM_BUILD_THREADS=$cpu_num
fi
echo NUM_BUILD_THREADS=$NUM_BUILD_THREADS

export YOLO_PLANAR_SLAM_VERSION=2.9.0
export YOLO_PLANAR_SLAM_DOCKER_IMAGE_VERSION=$YOLO_PLANAR_SLAM_VERSION

echo YOLO_PLANAR_SLAM_DOCKER_IMAGE_VERSION=$YOLO_PLANAR_SLAM_DOCKER_IMAGE_VERSION
