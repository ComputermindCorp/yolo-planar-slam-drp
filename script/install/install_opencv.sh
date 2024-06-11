#!/bin/bash -u

# Tested with version OpenCV 3.2.0

(
  set -ex

  mkdir -p $HOME/lib/$OPENCV_VERSION
  unzip -q ${OPENCV_VERSION}.zip
  cd opencv-${OPENCV_VERSION}

  mkdir -p build
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/lib/$OPENCV_VERSION \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_OPENEXR=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_dnn=OFF \
    -DBUILD_opencv_ml=ON \
    -DBUILD_opencv_objdetect=ON \
    -DBUILD_opencv_python_bindings_generator=OFF \
    -DENABLE_FAST_MATH=OFF \
    -DWITH_EIGEN=ON \
    -DWITH_FFMPEG=OFF \
    -DWITH_OPENMP=ON \
    -DWITH_CUDA=OFF \
    -DWITH_GTK_2_X=ON \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -DBUILD_opencv_python2=OFF \
    -DBUILD_opencv_python3=ON \
    -DPYTHON_DEFAULT_EXECUTABLE=python3 \
    ..
  make -j${NUM_BUILD_THREADS:-''}
  make install
)