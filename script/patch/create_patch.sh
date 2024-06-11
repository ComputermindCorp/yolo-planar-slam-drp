#!/bin/bash -u

trap 'exit 1' SIGINT

DAY=`date +"%Y-%m-%d"`
TIME=`date +"%H-%M-%S"`
echo DAY=$DAY
echo TIME=$TIME

if [ `git diff | wc -l` -gt 0 ]; then
    echo "Please resolve the difference" >&2
    exit 1
fi

(
  set -ex

  if [ `git branch | grep -E " +shrink$" | wc -l` -ge 1 ]; then
    git branch -D shrink
  fi

  git checkout -b shrink

  git submodule update --init

  git rm configuration_code/cvfast_drp_out.bin
  git rm configuration_code/gaussian_blur_drp_out.bin
  git rm configuration_code/orb_descriptors_drp_out.bin
  git rm configuration_code/resize_drp_out.bin
  git rm configuration_code/slamfast_drp_out.bin
  git rm configuration_code/cvfast_drp_out.20230123.STP4C.bin
  git rm configuration_code/gaussian_blur_drp_out.20230123.STP4C.bin
  git rm configuration_code/orb_descriptors_drp_out.20230123.STP4C.bin
  git rm configuration_code/resize_drp_out.20230123.STP4C.bin
  git rm configuration_code/slamfast_drp_out.20230123.STP4C.bin

  git rm -r drp
  git rm v2x
  cp -r drp_cv_lib/drp/util/v2x .
  git add v2x/linux/*.h

  git rm .clang-format
  git rm .gitlab-ci.yml
  git rm -f .gitmodules
  git rm Dockerfile

  mkdir -p script.backup
  cp -r script/* script.backup/
  git rm -r script

  mkdir -p script
  cp -r script.backup/install script/
  cp -r script.backup/patch script/
  cp script.backup/setup.sh script/
  git add script

  git submodule deinit --force drp_cv_lib
  git rm -r --force drp_cv_lib

  git commit -m 'shrink'

  git diff --binary original HEAD > patch-${DAY}-${TIME}.txt
  cp patch-${DAY}-${TIME}.txt /tmp/yolo-planar-slam.patch
)

failed=$?

(
  set -ex

  git checkout -
  git submodule update --init
  git reset --hard HEAD
)

if [ $failed -ne 0 ]; then
    echo "Failed to create patch file" >&2
    exit 1
fi

echo "All commands have been executed successfully."
