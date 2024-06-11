#!/bin/bash -u

(
  set -ex

  mkdir -p $HOME/lib/ncnn

  cd ncnn
  git checkout $NCNN_VERSION
  git submodule update --init
  
  mkdir -p build
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/lib/ncnn \
    -DNCNN_VULKAN=OFF \
    -DNCNN_BUILD_EXAMPLES=OFF \
    -DNCNN_BUILD_BENCHMARK=OFF \
    ..
  make -j${NUM_BUILD_THREADS:-''}
  make install
)