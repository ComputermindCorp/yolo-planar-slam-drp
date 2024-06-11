#!/bin/bash -u

(
  set -ex

  if  [ $NUM_BUILD_THREADS -ge 6 ]; then
    NUM_BUILD_THREADS=6
  fi

  mkdir -p $HOME/lib/pcl
  tar xzvf pcl-${PCL_VERSION}.tar.gz
  cd pcl-pcl-${PCL_VERSION}

  mkdir -p build
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/lib/pcl \
    -DWITH_CUDA=OFF \
    ..
  make -j${NUM_BUILD_THREADS:-''}
  make install
)
