#!/bin/bash

export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=~/local/lib:$LD_LIBRARY_PATH

/lib64/ld-linux-aarch64.so.1 --list ~/stella_vslam_examples/build/run_tum_rgbd_slam | grep -E 'vslam|g2o|socket_publisher|tvm|yaml-cpp|sioclient|fbow'
/lib64/ld-linux-aarch64.so.1 --list ~/local/lib/libstella_vslam.so | grep -E 'vslam|g2o|socket_publisher|tvm|yaml-cpp|sioclient|fbow'
