#!/bin/bash -u

trap 'exit 1' SIGINT

docker build \
  --build-arg USER_NAME=rns \
  --build-arg USER_ID=$(id -u) \
  --build-arg GROUP_ID=$(id -g) \
  --build-arg NUM_BUILD_THREADS=${NUM_BUILD_THREADS:-''} \
  --build-arg YOLO_PLANAR_SLAM_DOCKER_IMAGE_VERSION=${YOLO_PLANAR_SLAM_DOCKER_IMAGE_VERSION} \
  -t rns/yolo-planar-slam:${YOLO_PLANAR_SLAM_DOCKER_IMAGE_VERSION} \
  .
