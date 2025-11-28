#!/bin/bash

echo "---------------------------------------------------------------------"
echo "run_stella_vslam [1] [2] [3] [4]"
echo ""
echo "[1] MODE     0: Monocular : rgbd_dataset_freiburg3_walking_halfsphere"
echo "             1: Monocular : ELP USB Camera"
echo "             2: RGB-D     : rgbd_dataset_freiburg3_walking_halfsphere"
echo "             3: Stereo    : V1_01_easy"
echo "[2] SLAM     0: CPU"
echo "             1: OpenCVA(DRP)"
echo "             2: Custom(DRP)"	
echo "[3] YOLOX-S  0: -----"
echo "             1: DRPAI"	
echo "[4] Viewer   0: OFF"
echo "             1: ON"	
echo "---------------------------------------------------------------------"

case $1 in
    "0")
        echo "[1] MODE      : Monocular";
        echo "    Dataset   : rgbd_dataset_freiburg3_walking_halfsphere";
	COM="./build/run_tum_rgbd_slam";
        YAML="../stella_vslam/example/tum_rgbd/TUM_RGBD_mono_3.yaml";
	DATASET="/opt/dataset/tum/rgbd_dataset_freiburg3_walking_halfsphere";
	OPTION="--no-sleep --auto-term";
	ASS="";
	echo "    YAML      : ${YAML}";;
    "1")
        echo "[1] MODE      : Monocular";
        echo "    Dataset   : USB ELP camera";
	COM="./build/run_camera_slam";
        YAML="../stella_vslam/example/cam/ELP_rel.yaml";
	DATASET="./";
	OPTION="--number 0";
	ASS="";
	echo "    YAML      : ${YAML}";;
    "2")
        echo "[1] MODE      : RGB-D";
        echo "    Dataset   : rgbd_dataset_freiburg3_walking_halfsphere";
	COM="./build/run_tum_rgbd_slam";
        YAML="../stella_vslam/example/tum_rgbd/TUM_RGBD_rgbd_3.yaml";
	DATASET="/opt/dataset/tum/rgbd_dataset_freiburg3_walking_halfsphere";
	OPTION="--no-sleep --auto-term";
	ASS="";
	echo "    YAML      : ${YAML}";;
    "3")
        echo "[1] MODE      : Stereo";
        echo "    Dataset   : V1_01_easy";
	COM="./build/run_euroc_slam";
        YAML="../stella_vslam/example/euroc/EuRoC_stereo.yaml";
	DATASET="/opt/dataset/V1_01_easy/mav0/ ";
	OPTION="--no-sleep --auto-term";
	ASS="";
	echo "    YAML      : ${YAML}";;
    *)
        echo "[1] MODE      : !!! Error !!!";
	exit 1;;
esac

VOC="./orb_vocab.fbow"
EVAL="./eval"

case $2 in
    "0")
        echo "[2] SLAM      : CPU";
        cat ${YAML}     | tr '\n' '~' | sed 's/OpenCVA:~  resize: true~  gaussian_blur: true~  cvfast: true/OpenCVA:~  resize: false~  gaussian_blur: false~  cvfast: false/g' | tr '~' '\n' > ${YAML}.tmp;
        cat ${YAML}.tmp | tr '\n' '~' | sed 's/DRP-Driver-Native:.*orb_descriptors: true/DRP-Driver-Native:~  resize: false~  gaussian_blur: false~  cvfast: false~  slamfast: false~  orb_descriptors: false/g' | tr '~' '\n' > ${YAML};;
    "1")
        echo "[2] SLAM      : OpenCVA(DRP)";
        cat ${YAML}     | tr '\n' '~' | sed 's/OpenCVA:~  resize: false~  gaussian_blur: false~  cvfast: false/OpenCVA:~  resize: true~  gaussian_blur: true~  cvfast: true/g' | tr '~' '\n' > ${YAML}.tmp;
        cat ${YAML}.tmp | tr '\n' '~' | sed 's/DRP-Driver-Native:.*orb_descriptors: true/DRP-Driver-Native:~  resize: false~  gaussian_blur: false~  cvfast: false~  slamfast: false~  orb_descriptors: false/g' | tr '~' '\n' > ${YAML};;
    "2")
        echo "[2] SLAM      : Custom(DRP)";
        cat ${YAML}     | tr '\n' '~' | sed 's/OpenCVA:~  resize: true~  gaussian_blur: true~  cvfast: true/OpenCVA:~  resize: false~  gaussian_blur: false~  cvfast: false/g' | tr '~' '\n' > ${YAML}.tmp;
        cat ${YAML}.tmp | tr '\n' '~' | sed 's/DRP-Driver-Native:.*orb_descriptors: false/DRP-Driver-Native:~  resize: true~  gaussian_blur: true~  cvfast: false~  slamfast: true~  orb_descriptors: true/g' | tr '~' '\n' > ${YAML};;
    *)
        echo "[2] SLAM      : !!! Error !!!";
	exit 1;;
esac

case $3 in
    "0")
        echo "[3] YOLOX-S   : CPU";
	sed -i 's/use_yolo: true/use_yolo: false/g' ${YAML};;
    "1")
        echo "[3] YOLOX-S   : DRPAI";
	sed -i 's/use_yolo: false/use_yolo: true/g' ${YAML};;
    *)
        echo "[3] YOLOX-S   : !!! Error !!!";
	exit 1;;
esac

case $4 in
    "0")
        echo "[4] Viewer    : OFF";
	VIEWER="none";;
    "1")
        echo "[4] Viewer    : ON";
        VIEWER="socket_publisher";;
    *)
        echo "[4] Viewer    : !!! Error !!!";
	exit 1;;
esac

echo "---------------------------------------------------------------------"
egrep -A 17 "OpenCVA:" ${YAML}
echo "---------------------------------------------------------------------"
echo ${COM} --vocab ${VOC} --data-dir ${DATASET} --config ${YAML} ${OPTION} --viewer ${VIEWER} --eval-log-dir ${EVAL}
case $1 in
    "1")
        echo ${COM} --vocab ${VOC} --config ${YAML} ${OPTION} --viewer ${VIEWER} --eval-log-dir ${EVAL};
        ${COM} --vocab ${VOC} --config ${YAML} ${OPTION} --viewer ${VIEWER} --eval-log-dir ${EVAL};;
    *)
        echo ${COM} --vocab ${VOC} --data-dir ${DATASET} --config ${YAML} ${OPTION} --viewer ${VIEWER} --eval-log-dir ${EVAL};
        ${COM} --vocab ${VOC} --data-dir ${DATASET} --config ${YAML} ${OPTION} --viewer ${VIEWER} --eval-log-dir ${EVAL};;
esac
echo "---------------------------------------------------------------------"
