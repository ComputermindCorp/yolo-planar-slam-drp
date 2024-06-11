#!/bin/bash

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

unset LD_LIBRARY_PATH
source ${YOCTO_DIR:-/yocto_rzv2x_alpha2_workdir}/bsp_sdk/environment-setup-aarch64-poky-linux

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_MEASURE_TIME=${MEASURE_TIME:-OFF} \
    -DENABLE_DUMP=${DUMP:-OFF} \
    -DENABLE_DRP=ON \
    -DENABLE_DRP_AI=ON \
    -DENABLE_REALSENSE2=${REALSENSE2:-OFF} \
    -DENABLE_GOOGLE_PERF=${GOOGLE_PERF:-OFF} \
    -DENABLE_YOCTO=ON \
    -DENABLE_SLAMFAST=${SLAMFAST:-OFF} \
    -DENABLE_TVM=${TVM:-OFF} \
    -DOpenMP_C_FLAGS='-fopenmp' \
    -DOpenMP_CXX_FLAGS='-fopenmp' \
    -DOpenMP_C_LIB_NAMES='gomp;pthread' \
    -DOpenMP_CXX_LIB_NAMES='gomp;pthread' \
    -DOpenMP_gomp_LIBRARY=${SDKTARGETSYSROOT:?}/usr/lib64/libgomp.so \
    -DOpenMP_pthread_LIBRARY=${SDKTARGETSYSROOT:?}/usr/lib64/libpthread.so \
    -DProtobuf_PROTOC_EXECUTABLE=${YOCTO_DIR:-/yocto_rzv2x_alpha2_workdir}/build/tmp/sysroots-components/x86_64/protobuf-native/usr/bin/protoc \
    -DYOLO_PLANAR_SLAM_VERSION=${YOLO_PLANAR_SLAM_VERSION:?} \
    ..
make -j${NUM_BUILD_THREADS:-}
