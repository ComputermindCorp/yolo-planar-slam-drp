#!/bin/bash

export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/Thirdparty/g2o/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/Thirdparty/DBoW2/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/build/drp_ai_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/build/drp_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/build/image_load_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/build/image_proc_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/build/opencva_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/yolo-planar-slam/build/socket_modules:$LD_LIBRARY_PATH

/lib64/ld-linux-aarch64.so.1 --list $HOME/yolo-planar-slam/Examples/Monocular/mono_tum | grep -E 'modules|SLAM|g2o|DBoW2'
