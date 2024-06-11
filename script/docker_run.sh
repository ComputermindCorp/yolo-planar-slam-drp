#!/bin/bash -u

trap 'exit 1' SIGINT

(
  set -ex

  XSOCK=/tmp/.X11-unix
  XAUTH=/tmp/.docker.xauth
  touch $XAUTH
  xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

  docker run -it --rm \
    --env "XAUTHORITY=${XAUTH}" \
    --env "DISPLAY=${DISPLAY}" \
    --env TERM=xterm-256color \
    --env QT_X11_NO_MITSHM=1 \
    --volume $XSOCK:$XSOCK:rw \
    --volume $XAUTH:$XAUTH:rw \
    --volume /opt/dataset:/opt/dataset:ro \
    --volume $HOME/yolo-planar-slam:/home/rns/yolo-planar-slam \
    --net host \
    --name yolo-planar-slam \
    rns/yolo-planar-slam:${YOLO_PLANAR_SLAM_DOCKER_IMAGE_VERSION} \
    bash
)


