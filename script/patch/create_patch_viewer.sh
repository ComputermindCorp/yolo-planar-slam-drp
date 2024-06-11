#!/bin/bash -u

trap 'exit 1' SIGINT

COMMIT_ID=6cb5cacc0f79f6833121d5ebb54e32ba635745d9

DAY=`date +"%Y-%m-%d"`
TIME=`date +"%H-%M-%S"`
echo DAY=$DAY
echo TIME=$TIME

(
  set -ex

  if [ ! -d stella_vslam ]; then
    git clone https://github.com/stella-cv/stella_vslam.git
  fi

  cd stella_vslam
  rm -r *
  rm -rf .dockerignore
  git checkout ${COMMIT_ID}
  git reset --hard HEAD

  mkdir -p viewer
  cp -r $HOME/yolo-planar-slam/viewer/* viewer/

  if [ `git branch | grep -E " +update-viewer$" | wc -l` -ge 1 ]; then
    git branch -D update-viewer
  fi

  git checkout -b update-viewer
  git add viewer
  git commit -m 'update viewer'

  git diff --binary ${COMMIT_ID} HEAD > patch-${DAY}-${TIME}.txt
  cp patch-${DAY}-${TIME}.txt /tmp/viewer.patch
)
