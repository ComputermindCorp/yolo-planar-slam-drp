#!/bin/bash

cmake -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX:?} \
  -DENABLE_MEASURE_TIME=ON \
  -DENABLE_DEBUG_OUTPUT=OFF \
  -DENABLE_DRP_DRIVER_NATIVE=ON \
  -DENABLE_DRP_AI_TVM=ON \
  -DDISABLE_YOCTO=OFF \
  -DUSE_OPENMP=ON \
  -Dg2o_DIR=${CMAKE_INSTALL_PREFIX:?}/lib/cmake/g2o \
  -DOpenMP_C_FLAGS="-fopenmp" \
  -DOpenMP_CXX_FLAGS="-fopenmp" \
  -DOpenMP_C_LIB_NAMES="gomp;pthread" \
  -DOpenMP_CXX_LIB_NAMES="gomp;pthread" \
  -DOpenMP_gomp_LIBRARY=${SDKTARGETSYSROOT:?}/usr/lib64/libgomp.so \
  -DOpenMP_pthread_LIBRARY=${SDKTARGETSYSROOT:?}/usr/lib64/libpthread.so \
  ..
make -j
