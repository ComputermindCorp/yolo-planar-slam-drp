#!/bin/bash

echo "---------------------------------------------------"
echo "run_planar_vslam [1] [2] [3]"
echo ""
echo "[1] MODE     0: Monocular : rgbd_dataset_freiburg3_walking_xyz"
echo "             1: Monocular : GRAY_rgbd_dataset_freiburg3_walking_xyz"
echo "             2: Monocular : USB ELP camera"
echo "             3: RGB-D     : rgbd_dataset_freiburg3_walking_xyz"
echo "[2] SLAM     0: CPU"
echo "             1: OpenCVA(DRP)"
echo "             2: Custom(DRP)"	
echo "[3] Viewer   0: OFF"
echo "             1: ON"	
echo "---------------------------------------------------"

case $1 in
    "0")
        echo "[1] MODE      : Monocular";
        echo "    Dataset   : rgbd_dataset_freiburg3_walking_xyz";
	COM="./Examples/Monocular/mono_tum";
        YAML="./Examples/Monocular/TUM3.yaml";
	DATASET="/opt/dataset/tum/rgbd_dataset_freiburg3_walking_xyz";
	ASS="";
	echo "    YAML      : ${YAML}";;
    "1")
        echo "[1] MODE      : Monocular";
        echo "    Dataset   : GRAY_rgbd_dataset_freiburg3_walking_xyz";
	COM="./Examples/Monocular/mono_tum";
        YAML="./Examples/Monocular/TUM3.yaml";
	DATASET="/opt/dataset/tum/GRAY_rgbd_dataset_freiburg3_walking_xyz";
	ASS="";
	echo "    YAML      : ${YAML}";;
    "2")
        echo "[1] MODE      : Monocular";
        echo "    Dataset   : USB ELP camera";
	COM="./Examples/Monocular/mono_usbcam";
        YAML="./Examples/Monocular/ELP_rns-2022-0901.yaml";
	DATASET="";
	ASS="";
	echo "    YAML      : ${YAML}";;
    "3")
        echo "[1] MODE      : RGB-D";
        echo "    Dataset   : rgbd_dataset_freiburg3_walking_xyz";
	COM="./Examples/RGB-D/rgbd_tum";
        YAML="./Examples/RGB-D/TUM3.yaml";
	DATASET="/opt/dataset/tum/rgbd_dataset_freiburg3_walking_xyz";
	ASS="${DATASET}/associate.txt";
	echo "    YAML      : ${YAML}";;
    *)
        echo "[1] Error";
	exit 1;;
esac

VOC="./Vocabulary/ORBvoc.txt"


case $2 in
    "0")
        echo "[2] SLAM      : CPU";
        sed -i -e 's/UseDrp: true/UseDrp: false/g'                  ${YAML};
        sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g'          ${YAML};;
    "1")
        echo "[2] SLAM      : OpenCVA(DRP)";
        sed -i -e 's/UseDrp: false/UseDrp: true/g'                  ${YAML};
        sed -i -e 's/UseOpenCVA: false/UseOpenCVA: true/g'          ${YAML};;
    "2")
        echo "[2] SLAM      : Custom(DRP)";
	sed -i -e 's/UseDrp: false/UseDrp: true/g'                  ${YAML};
        sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g'          ${YAML};;
    *)
        echo "[2] Error";
	exit 1;;
esac

case $3 in
    "0")
        echo "[3] Viewer    : OFF";
	sed -i -e 's/Viewer.Type: SocketViewer/Viewer.Type: None/g' ${YAML};;
    "1")
        echo "[3] Viewer    : ON";
        sed -i -e 's/Viewer.Type: None/Viewer.Type: SocketViewer/g' ${YAML};;
    *)
        echo "[3] Error";
	exit 1;;
esac

echo "---------------------------------------------------"
echo ${COM} ${VOC} ${YAML} ${DATASET} ${ASS}  
${COM} ${VOC} ${YAML} ${DATASET} ${ASS}  
echo "---------------------------------------------------"
