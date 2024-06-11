#!/bin/bash

cmake -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_MEASURE_TIME=ON \
  -DENABLE_DUMP=OFF \
  -DENABLE_DRP=ON \
  -DENABLE_DRP_AI=ON \
  -DENABLE_REALSENSE2=OFF \
  -DENABLE_GOOGLE_PERF=OFF \
  -DENABLE_YOCTO=ON \
  -DENABLE_SLAMFAST=ON \
  -DENABLE_DRPAI_EXPAND_MEMORY_SPACE=OFF \
  -DENABLE_CALL_OPENCVA_DIRECTLY=ON \
  -DOpenMP_C_FLAGS='-fopenmp' \
  -DOpenMP_CXX_FLAGS='-fopenmp' \
  -DOpenMP_C_LIB_NAMES='gomp;pthread' \
  -DOpenMP_CXX_LIB_NAMES='gomp;pthread' \
  -DOpenMP_gomp_LIBRARY=${SDKTARGETSYSROOT}/usr/lib64/libgomp.so \
  -DOpenMP_pthread_LIBRARY=${SDKTARGETSYSROOT}/usr/lib64/libpthread.so \
  -DYOLO_PLANAR_SLAM_VERSION=${YOLO_PLANAR_SLAM_VERSION} \
  ..
make -j