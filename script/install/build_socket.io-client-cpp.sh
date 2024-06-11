#!/bin/bash -u

(
  set -ex

  cd $HOME/socket.io-client-cpp
  git checkout $SIO_CLIENT_VERSION
  git submodule init
  git submodule update
  mkdir -p $HOME/socket.io-client-cpp/build
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_UNIT_TESTS=OFF \
    ..
  make -j${NUM_BUILD_THREADS:-''}
)