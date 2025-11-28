#!/bin/bash

cmake -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX:?} \
  -Dstella_vslam_DIR=${CMAKE_INSTALL_PREFIX:?}/lib/cmake/stella_vslam \
  -Dg2o_DIR=${CMAKE_INSTALL_PREFIX:?}/lib/cmake/g2o \
  -Dfbow_DIR=${CMAKE_INSTALL_PREFIX:?}/share/cmake/fbow \
  -DOpenMP_C_FLAGS="-fopenmp -I${SDKTARGETSYSROOT}/usr/lib64/aarch64-poky-linux/8.3.0/include" \
  -DOpenMP_CXX_FLAGS="-fopenmp -I${SDKTARGETSYSROOT}/usr/lib64/aarch64-poky-linux/8.3.0/include" \
  -DOpenMP_C_LIB_NAMES="gomp;pthread" \
  -DOpenMP_CXX_LIB_NAMES="gomp;pthread" \
  -DOpenMP_gomp_LIBRARY=${SDKTARGETSYSROOT:?}/usr/lib64/libgomp.so \
  -DOpenMP_pthread_LIBRARY=${SDKTARGETSYSROOT:?}/usr/lib64/libpthread.so \
  ..
make -j10
