#!/bin/bash -u

(
  set -ex

  cd $HOME/Pangolin
  git checkout $PANGOLIN_VERSION
  mkdir -p $HOME/Pangolin/build
  mkdir -p $HOME/lib/Pangolin
  cd build
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/lib/Pangolin \
    ..
  make -j${NUM_BUILD_THREADS:-''}
  make install
)