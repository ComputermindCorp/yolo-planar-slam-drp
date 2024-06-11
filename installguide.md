# Install guide

The target of this install guide is to run the AI-VSLAM application on the RZ / V2H EVK board and understand the application behavior and processing performance.

Before running this application, it is necessary to calibrate the camera according to "SoCDD-08571-01_RZV2H_USBcamera_Calibration".
If you do not calibrate your camera, the accuracy may be reduced.

## 1. Build Linux Environment
### Deliverables

Refer the link below
[Download Link](https://renesas-rz.github.io/rzv_ai_sdk/3.00/howto_build_aisdk_v2h.html)

Provided items from Renesas are listed below

#### RZ/V2H AI SDK
| Category | File name | Download site / Description |
| ---- | ---- | ---- |
| RZ/V2H AI SDK Source Code | RTK0EF0180F03000SJ_linux-src.zip | https://www.renesas.com/us/en/document/sws/rzv2h-ai-sdk-v300-source-code |
|   |  - README.txt | README file. | 
|   |  - rzv2h_ai-sdk_yocto_recipe_v3.00.tar.gz | Yocto recipe.<br> Yocto recipes are text files that contains necessary information to build Yocto Linux.|

### EVALUATION ENVIRONMENT

Equipment and Software Necessary for Developing Environments as follows.

| Equipment | Description |
| ---- | ---- |
| Target Board | RZ/V2H EVK |
| Linux host PC | Build embedded Linux, Create microSD Card.<br><span style="color: red;">Ubuntu version 20.04 LTS</span> (64 bit OS must be used.)<br>100GB free space on HDD is necessary. |
| Windows host PC | Communicate Target Board with terminal software.<br>Windows 10 is recommended. |
| Terminal software | Used for controlling serial console of the target board<br>Tera Term (latest version) is recommended<br>Available at https://ttssh2.osdn.jp/index.html.en |
| VCP Driver | Virtual COM Port driver which enables to communicate<br>Windows Host PC and the target board via USB<br>which is virtually used as serial port. <br>Available at: http://www.ftdichip.com/Drivers/VCP.htm<br>Please install VCP Driver corresponding to the target board. |
| Broadband router | DHCP server |
| microSD Card | <span style="color: red;">8GB or more</span>. Store Kernel image, device tree, rootfs |
| USB camera | ELP-USBGS720P02-L36<br>(Camera is not required when using Datasets.) |
| USB power supply | Power supply |
| USB Cable micro-B | Connect Win PC and Target board |
| USB Cable Type-C<br><span style="color: red;">PD 100W / 5A</span> | Connect AC adapter and Target board<br><span style="color: red;">The board will not power on if the cable is less than 100W.</span> |
| FFC Cable | Connect CPU board and EXP board |

<img src="./img/img1.png" width="60%">
 
### ALL FLOW of building AI-SLAM

Build an AI-SLAM application in the following order.<br>
If calibrate the USB camera, please refer to the [Calibration guide](calibrationguide.md). <br>


<img src="./img/img2.png" width="60%">

<br>

- STEP1 Build Linux Environment<br>
  ![](./img/img3.png)

  - (1) Download RZ/V2H AI SDK Source Code
  - (2) Deploy the files of RZ/V2H AI SDK and vslam
  - (3) Install the tools for building
  - (4) Extract Yocto recipe package
  - (5) Initialize a build using the 'oe-init-build-env' script in Poky
  - (6) Add layer Graphics Library, drpai, opencva, etc.
  - (7) Change the size of the microSD card image in WIC format
  - (8) Bitbake the Image
  - (9) Add SLAM recipe
  - (10) Bitbake the Image
  - (11) BItbake the SDK
  - (12) Install the SDK
  - (13) Make application

<br>

- STEP2 Prepare a micro SD card to boot Linux<br>
  ![](./img/img4.png)
  
  - (14) Prepare microSD
  - (15) Write image to microSD
  - (16) Write SLAM Script/Dataset/Application 

<br>

- STEP3 Execute DRP-AI sample application<br>
  ![](./img/img5.png)

  - (17) Start EVK board and VSLAM application
  - (18) Re-build AI-VSLAM application

### Build Instructions

- (1) Download RZ/V2H AI SDK Source Code.

  https://www.renesas.com/us/en/document/sws/rzv2h-ai-sdk-v300-source-code
  <span style="color: black;">RTK0EF0180F03000SJ_linux-src.zip</span>

- (2) Deploy the files of RZ/V2H AI SDK and vslam.

  Create a working directory(${WORK}/src_setup) at Linux Host PC and deploy these files in your work directory.
  - Linux BSP			: <span style="color: black;"> RTK0EF0180F03000SJ_linux-src.zip </span>
  - AI SLAM application and recipe		: <span style="color: black;"> yolo-planar-slam-drp </span>  
  - AI SLAM dataset		: <span style="color: black;"> If using a dataset, see [Dataset preparation](datasetusage.md).

  ```
  $ export WORK="<working directory>"
  $ mkdir -p ${WORK}/src_setup
  $ cd ${WORK}/src_setup
  $ unzip RTK0EF0180F03000SJ_linux-src.zip -d ${WORK}/src_setup
  $ ls -1 ${WORK}/src_setup
    README.txt
    rzv2h_ai-sdk_yocto_recipe_v3.00.tar.gz
  ```

  ```
  $ cd ${WORK}/src_setup
  $ git clone https://github.com/ComputermindCorp/yolo-planar-slam-drp.git
  ```

- (3) Install the tools for building.

  Before starting the build, run the command below on the Linux Host PC to install packages used for building the BSP. <br>After that, Set git config (if it is NOT set).

  ```
  $ sudo apt-get update 
  $ sudo apt-get install gawk wget git-core diffstat unzip texinfo gcc-multilib build-essential chrpath socat cpio python python3 python3-pip python3-pexpect xz-utils debianutils iputils-ping libsdl1.2-dev xterm p7zip-full libyaml-dev libssl-dev 
  $ git config --global user.email “you@example.com”
  $ git config --global user.name “Your Name”
  ```

- (4) Extract Yocto recipe package.

  ```
  $ export YOCTO_WORK=${WORK}/src_setup/yocto
  $ mkdir -p ${YOCTO_WORK}
  $ cd ${YOCTO_WORK}
  $ tar zxvf ${WORK}/src_setup/rzv2h_ai-sdk_yocto_recipe_v3.00.tar.gz
  ```

  **Apply a patch file to fix lind error:**

  Obtain the patch file below from [this Link](https://github.com/renesas-rz/rzv_ai_sdk/releases/download/v3.00/0001-recipes-debian-buster-glibc-Update-version-from-2.28.patch).

  | File name | Description |
  | ---- | ---- |
  | 0001-recipes-debian-buster-glibc-Update-version-from-2.28.patch | patch file for fixing glibc link error |

  Copy and apply the patch file.

  ```
  $ cp <Path to the file>/0001-recipes-debian-buster-glibc-Update-version-from-2.28.patch ${YOCTO_WORK}
  $ cd ${YOCTO_WORK}/meta-renesas
  $ patch -p1 < ../0001-recipes-debian-buster-glibc-Update-version-from-2.28.patch
  ```

  Check the working directory to confirm Yocto recipes content.

  ```
  $ ls -1 ${YOCTO_WORK}
    0001-recipes-debian-buster-glibc-Update-version-from-2.28.patch
    0001-tesseract.patch
    meta-gplv2
    meta-openembedded
    meta-renesas
    meta-rz-features
    meta-virtualization
    poky
  ```

- (5) Initialize a build using the 'oe-init-build-env' script in Poky.
  ```
  $ cd ${YOCTO_WORK}
  $ TEMPLATECONF=${PWD}/meta-renesas/meta-rzv2h/docs/template/conf/ source poky/oe-init-build-env
  ```

- (6) Add layers of graphics Library, drpai, opencv accelerator, codecs and patch.

  ```
  $ bitbake-layers add-layer ../meta-rz-features/meta-rz-graphics
  $ bitbake-layers add-layer ../meta-rz-features/meta-rz-drpai
  $ bitbake-layers add-layer ../meta-rz-features/meta-rz-opencva
  $ bitbake-layers add-layer ../meta-rz-features/meta-rz-codecs
  $ bitbake-layers add-layer ../meta-openembedded/meta-filesystems
  $ bitbake-layers add-layer ../meta-openembedded/meta-networking
  $ bitbake-layers add-layer ../meta-virtualization

  $ patch -p1 < ../0001-tesseract.patch
  ```

- (7) Change the size of the microSD card image in WIC format

  The case of using 8GB microSD
  ```
  $ sed -i 's/1048576/3145728/g' conf/local.conf
  $ egrep 3145728 conf/local.conf
    IMAGE_ROOTFS_EXTRA_SPACE = "3145728"
  ```
  
  The case of using 16GB microSD
  ```
  $ sed -i 's/1048576/8388608/g' conf/local.conf
  $ egrep 8388608 conf/local.conf
    IMAGE_ROOTFS_EXTRA_SPACE = "8388608"
  ```

  The case of using 32GB microSD
  ```
  $ sed -i 's/1048576/16777216/g' conf/local.conf
  $ egrep 16777216 conf/local.conf
    IMAGE_ROOTFS_EXTRA_SPACE = "16777216"
  ```

- (8) Build the target file system image using bitbake.

  Run the commands below to start a build. Building an image can take up to a few hours depending on the user’s host system performance.
  ```
  $ cd ${YOCTO_WORK}/build

  $ MACHINE=rzv2h-evk-ver1 bitbake core-image-weston
  ```

- (9) Add SLAM recipe
  
  Unzip "meta-yolo-planar-slam-2.9.0.tar.gz" and link "meta-yolo-planar-slam-2.9.0"  to "meta-yolo-planar-slam".
  ```
  $ cd ${YOCTO_WORK}
  $ tar zxvf ${WORK}/src_setup/yolo-planar-slam-drp/setup_files/meta-yolo-planar-slam-2.9.0.tar.gz
  $ ln -s ./meta-yolo-planar-slam-2.9.0 ./meta-yolo-planar-slam
  ```

  Make sure the following linked files.
  ```
  $ ls -al meta-yolo-planar-slam
    meta-yolo-planar-slam -> ./meta-yolo-planar-slam-2.9.0
  ```

  Insert the following text into "bblayers.conf" and "local.conf".
  ```
  $ cd ${YOCTO_WORK}
  $ sed -i '$i ${TOPDIR}/../meta-yolo-planar-slam \\' ./build/conf/bblayers.conf
  $ sed -i '$a SDKIMAGE_FEATURES_append = " libopencv-dev " ' ./build/conf/local.conf
  ```

  Make sure you have inserted the following text in the "bblayers.conf".
  ```
  $ tail -2 ./build/conf/bblayers.conf
    ${TOPDIR}/../meta-yolo-planar-slam \
    "
  ```

  Make sure you have inserted the following text in the "local.conf".
  ```
  $ tail -1 ./build/conf/local.conf
    SDKIMAGE_FEATURES_append = " libopencv-dev "
  ```

- (10) Build the target file system image using bitbake.
  ```
  $ cd ${YOCTO_WORK}
  $ TEMPLATECONF=${PWD}/meta-renesas/meta-rzv2/docs/template/conf/ source poky/oe-init-build-env
  $ MACHINE=rzv2h-evk-ver1 bitbake core-image-weston
  ```
  After completing the images for the target machine will be available in the output directory<br>
  <span style="color: blue;"> ‘${YOCTO_WORK}/build/tmp/deploy/images/rzv2h-evk-ver1’.</span>

  - <span style="color: red;">core-image-weston-rzv2h-evk-ver1.wic.bmap</span>
  - <span style="color: red;">core-image-weston-rzv2h-evk-ver1.wic.gz</span>

  
  *The bitbake may occur errors due to lack of memory.*<br>
  *In that case, reduce the number of cores by adding the following command to local.conf.*<br> 
  *This is an example with 2 cores．*<br>
  ![](./img/img7.png)

- (11) Build the target SDK
  ```
  $ cd ${YOCTO_WORK}
  $ TEMPLATECONF=${PWD}/meta-renesas/meta-rzv2/docs/template/conf/ source poky/oe-init-build-env
  $ MACHINE=rzv2h-evk-ver1 bitbake core-image-weston -c populate_sdk
  ```
  The resulting SDK installer will be located in <span style="color: blue;">'${YOCTO_WORK}/build/tmp/deploy/sdk'</span>.

  <span style="color: red;">poky-glibc-x86_64-core-image-weston-aarch64-rzv2h-evk-ver1-toolchain-3.1.26.sh</span> : Cross compiler installer

- (12) Install the SDK

  To run the installer, you would execute the following command:<br>
  The target directory name and place "/opt/poky/3.1.26" can be changed as necessary.<br>
  In case of default name of install directory for SDK
  ```
  $ cd ${YOCTO_WORK}/build/tmp/deploy/sdk
  $ sudo sh poky-glibc-x86_64-core-image-weston-aarch64-rzv2h-evk-ver1-toolchain-3.1.26.sh 

  Poky (Yocto Project Reference Distro) SDK installer version 3.1.26
  ==================================================================
  Enter target directory for SDK (default: /opt/poky/3.1.26): <Press Enter>
  You are about to install the SDK to "/opt/poky/3.1.26". Proceed [Y/n]?Y
  ```

- (13) Make AI-VSLAM application

  <span style="color: red;">Open new terminal.</span> link "yolo-planar-slam-drp"  to "yolo-planar-slam".
  ```
  $ export WORK="<working directory>"
  $ export YOCTO_WORK=${WORK}/src_setup/yocto
  $ cd ${YOCTO_WORK}
  $ ln -s ${WORK}/src_setup/yolo-planar-slam-drp ./yolo-planar-slam
  ```
  Make sure the following linked files.
  ```
  $ ls -al yolo-planar-slam
    yolo-planar-slam -> "<working directory>"/src_setup/yolo-planar-slam-drp
  ```
  Link drp binaries.
  ```
  $ cd ${YOCTO_WORK}/yolo-planar-slam/configuration_code
  $ ln -fs gaussian_blur_drp_out.20230810_05.STP4C.bin gaussian_blur_drp_out.bin
  $ ln -fs resize_drp_out.20230810_01.STP4C.bin resize_drp_out.bin
  $ ln -fs orb_descriptors_drp_out.20230810_17.STP4C.bin orb_descriptors_drp_out.bin
  $ ln -fs slamfast_drp_out.20230810_16.STP4C.bin slamfast_drp_out.bin
  ```
  Set the SDK environment and make the application.
  ```
  $ cd ${YOCTO_WORK}
  $ source /opt/poky/3.1.26/environment-setup-aarch64-poky-linux
  $ source ./yolo-planar-slam/script/setup.sh

  $ cd ${YOCTO_WORK}/yolo-planar-slam
  $ mkdir -p build
  $ cd ./build
  $ cp ${WORK}/src_setup/yolo-planar-slam-drp/setup_files/make_yolo-planar-slam.sh .
  $ . ./make_yolo-planar-slam.sh
  ```
  <img src="./img/img8.png" width="70%">

## 2. Prepare SD card to boot Linux
### Create a micro SD Card
- (14) Prepare a microSD Card
    
  To boot from microSD Card, over <span style="color: red;">8GB</span> capacity of blank microSD card is needed,<br>
  Please use Linux Host PC to write image data using USB card reader or other equipment.<br>
  Please write image data to your microSD Card according to the following steps.

- (15)  Write image to microSD card

  Here, we use "<span style="color: red;">/dev/sdc</span>" as microSD card device name.<br>
  <span style="color: red;">"/dev/sdc" needs to be changed according to your environment of Ubuntu PC.</span>

  Copy the image file to microSD
  ```
  $ cd ${YOCTO_WORK}/build/tmp/deploy/images/rzv2h-evk-ver1/
  $ sudo bmaptool copy --bmap core-image-weston-rzv2h-evk-ver1.wic.bmap core-image-weston-rzv2h-evk-ver1.wic.gz /dev/sdc
  ``` 
  Mount microSD Card. The name and the place of the mount directory can be changed as necessary. 
  ``` 
  $ export SD_VFAT="/media/<user-name>/3A98-31EA"
  $ export SD_EXT4="/media/<user-name>/rootfs"
  $ sudo mount -t vfat /dev/sdc1 $SD_VFAT
  $ sudo mount -t ext4 /dev/sdc2 $SD_EXT4
  ``` 

- (16) - 1. Write files to the card

  TUM dataset
  ``` 
  $ sudo mkdir -p $SD_EXT4/opt/dataset/tum

  $ sudo cp -rp ${WORK}/dataset/GRAY_rgbd_dataset_freiburg3_walking_xyz $SD_EXT4/opt/dataset/tum/
  $ sync

  $ sudo cp -rp ${WORK}/dataset/rgbd_dataset_freiburg3_walking_xyz $SD_EXT4/opt/dataset/tum/
  $ sync
  ``` 


- (16) - 2. Write files to the card
  Slam execution file and viewer and so on
  ```
  $ cd ${WORK}/src_setup/yolo-planar-slam-drp/setup_files
  $ tar zxvf ./YOLOX_S_dense_640x640_RGB_10271351.tar.gz -C  ${YOCTO_WORK}/yolo-planar-slam
  $ cp ./run_setenv_slam.sh ${YOCTO_WORK}/yolo-planar-slam/script
  ``` 
  
  ```
  $ cd ${YOCTO_WORK}/yolo-planar-slam
  $ sudo mkdir -p $SD_EXT4/home/root/yolo-planar-slam/Vocabulary
  $ sudo tar zxvf ./Vocabulary/ORBvoc.txt.tar.gz -C $SD_EXT4/home/root/yolo-planar-slam/Vocabulary
  $ sudo cp -r YOLOX_S_dense_640x640_RGB_10271351 configuration_code viewer script $SD_EXT4/home/root/yolo-planar-slam
  $ sync
  $ sudo cp -r Examples build lib Thirdparty $SD_EXT4/home/root/yolo-planar-slam
  $ sync
  ```

  Unmount microSD Card 
  ```
  $ sudo umount $SD_VFAT
  $ sudo umount $SD_EXT4
  ```

## 3. Execute AI-VSLAM Sample Application
### Start board 

<img src="./img/img9.png" width="60%">

- (17) - 1. Power OFF(SW2:OFF, SW3:OFF)
- (17) - 2. Connect equipment (see right the figure)
- (17) - 3. Change DSw1 and DSW2 setting
              as shown in the figure.
- (17) - 3. Attach microSD Card
- (17) - 4. Turn the SW3 ON.
- (17) - 5  Turn the SW2 ON.
- (17) - 6. After the Boot-up, open terminal app.

Terminal app setting
  - speed	: 115200bps
  - data 	: 8bit
  - Parity	: None
  - Stop bit        	: 1bit
  - Flow control 	: None

<img src="./img/img10.png" width="40%">

### Linux boot and Login

- (17) - 7. Login as root <br>
   ![](./img/img11.png)

### Terminal with SSH
- (17) - 8. Check the IP address of eth0(EVK) with ifconfig.
  ```
  # ifconfig
    eth0      Link encap:Ethernet  HWaddr xx:xx:xx:xx:xx:xx
              inet addr:192.xxx.xxx.xxx  Bcast:xxx.xxx.xxx.xxx  Mask:xxx.xxx.xxx.xxx
  ```
- (17) - 9. Open two Terminals of TeraTerm with SSH. (Term1,Term2)
  
  User name: <span style="color: red;">root</span><br>
  Passphrase: <span style="color: red;">(blank)</span>
<img src="./img/img12.png" width="80%">

### Run viewer
- (17) - 10. Run viewer server with <span style="color: red;">Term1</span>. (Run the docker container of rns/vslam-socket-server.)
  ```
  # cd $HOME/yolo-planar-slam/viewer
  # node app.js
  ```

- (17) - 11. The following message is displayed. (Term1)
  ```
    WebSocket: listening on *:3000
    HTTP server: listening on *:3001
  ```

- (17) - 12. Launch the browser on the PC side and input [IP address of smarc board(eth0 192.xxx.xxx.xxx)]: 3001 <br>
![](./img/img13.png)

### AI-VSLAM with using Monocular of TUM Dataset
- (17) - 13. Initialize and modify yaml file with <span style="color: red;">Term2</span>.
  ```
  # cd $HOME/yolo-planar-slam
  # source ./script/run_setenv_slam.sh
  ```

- (17) - 14. Modify yaml file. (Term2)   Refer to ./script/run_mono_tum_*.sh 

  Viewer on
  ```
  # sed -i -e 's/Viewer.Type: None/Viewer.Type: SocketViewer/g' Examples/Monocular/TUM3.yaml
  ```

  Viewer off
  ```
  # sed -i -e 's/Viewer.Type: SocketViewer/Viewer.Type: None/g' Examples/Monocular/TUM3.yaml
  ```

  In case of  build option "-DENABLE_SLAMFAST=OFF" of make_yolo-planar-slam.sh (default)

  w/o DRP
  ```
  # sed -i -e 's/UseDrp: true/UseDrp: false/g'         Examples/Monocular/TUM3.yaml
  # sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g' Examples/Monocular/TUM3.yaml
  ```
  
  w/ DRP
  ```
  # sed -i -e 's/UseDrp: false/UseDrp: true/g'         Examples/Monocular/TUM3.yaml
  # sed -i -e 's/UseOpenCVA: false/UseOpenCVA: true/g' Examples/Monocular/TUM3.yaml
  ```

  In case of  build option "-DENABLE_SLAMFAST=ON" of make_yolo-planar-slam-on.sh 
  w/ DRP
  ```
  # sed -i -e 's/UseDrp: false/UseDrp: true/g'         Examples/Monocular/TUM3.yaml
  # sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g' Examples/Monocular/TUM3.yaml
  ```

- (17) - 15. Run the following program. (Term2)
  ```
  # ./Examples/Monocular/mono_tum \
      $HOME/yolo-planar-slam/Vocabulary/ORBvoc.txt \
      $HOME/yolo-planar-slam/Examples/Monocular/TUM3.yaml \
      /opt/dataset/tum/GRAY_rgbd_dataset_freiburg3_walking_xyz
  ```
  The SLAM result is output on the browser of the PC. (Right figure)

  Parameter settings and operating modes
  | No. | ENABLE_SLAMFAST | UseDrpAI | UseDrp | UseOpenCVA | Yolo | SLAM |
  | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
  | 1 | OFF | true | false | false | DRP-AI | <span style="color: red;">CPU</span> |
  | 2 | OFF | true | true | true | DRP-AI | <span style="color: red;">DRP lib : OpenCVA</span> |
  | 3 | ON | true | true | false | DRP-AI | <span style="color: red;">DRP lib : Custom</span> |

### AI-VSLAM with using RGB-D of TUM Dataset
- (17) - 16. Initialize and modify yaml file with <span style="color: red;">Term2</span>.
  ```
  # cd $HOME/yolo-planar-slam
  # source ./script/run_setenv_slam.sh
  ```

- (17) - 17. Modify yaml file. (Term2)   Refer to ./script/run_rgbd_tum_*.sh 

  Viewer on
  ```
  # sed -i -e 's/Viewer.Type: None/Viewer.Type: SocketViewer/g' Examples/RGB-D/TUM3.yaml
  ```

  Viewer off
  ```
  # sed -i -e 's/Viewer.Type: SocketViewer/Viewer.Type: None/g' Examples/RGB-D/TUM3.yaml
  ```

  In case of  build option "-DENABLE_SLAMFAST=OFF" of make_yolo-planar-slam.sh (default)

  w/o DRP
  ```
  # sed -i -e 's/UseDrp: true/UseDrp: false/g'         Examples/RGB-D/TUM3.yaml
  # sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g' Examples/RGB-D/TUM3.yaml
  ```
  
  w/ DRP
  ```
  # sed -i -e 's/UseDrp: false/UseDrp: true/g'         Examples/RGB-D/TUM3.yaml
  # sed -i -e 's/UseOpenCVA: false/UseOpenCVA: true/g' Examples/RGB-D/TUM3.yaml
  ```

  In case of  build option "-DENABLE_SLAMFAST=ON" of make_yolo-planar-slam-on.sh 
  w/ DRP
  ```
  # sed -i -e 's/UseDrp: false/UseDrp: true/g'         Examples/RGB-D/TUM3.yaml
  # sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g' Examples/RGB-D/TUM3.yaml
  ```

- (17) - 18. Run the following program. (Term2)
  ```
  # ./Examples/RGB-D/rgbd_tum \
      $HOME/yolo-planar-slam/Vocabulary/ORBvoc.txt \
      $HOME/yolo-planar-slam/Examples/RGB-D/TUM3.yaml \
      /opt/dataset/tum/rgbd_dataset_freiburg3_walking_xyz \
      /opt/dataset/tum/rgbd_dataset_freiburg3_walking_xyz/associate.txt
  ```
  The SLAM result is output on the browser of the PC. (Right figure)

  Parameter settings and operating modes
  | No. | ENABLE_SLAMFAST | UseDrpAI | UseDrp | UseOpenCVA | Yolo | SLAM |
  | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
  | 1 | OFF | true | false | false | DRP-AI | <span style="color: red;">CPU</span> |
  | 2 | OFF | true | true | true | DRP-AI | <span style="color: red;">DRP lib : OpenCVA</span> |
  | 3 | ON | true | true | false | DRP-AI | <span style="color: red;">DRP lib : Custom</span> |

### AI-VSLAM with using USB camera

It is necessary to use parameters that have been calibrated for the camera.<br>
Change "ELP_rns-2022-0901.yaml" to the calibrated parameters. 

- (17) - 19. Initialize and modify yaml file with  <span style="color: red;">Term2</span>.
  ```
  # cd $HOME/yolo-planar-slam
  # source ./script/run_setenv_slam.sh
  ```

- (17) - 20. Modify yaml file. (Term2) Refer to ./script/run_mono_usbcam_*.sh 

  Viewer on
  ```
  # sed -i -e 's/Viewer.Type: None/Viewer.Type: SocketViewer/g' Examples/Monocular/ELP_rns-2022-0901.yaml
  ```

  Viewer off
  ```
  # sed -i -e 's/Viewer.Type: SocketViewer/Viewer.Type: None/g' Examples/Monocular/ELP_rns-2022-0901.yaml
  ```

  In case of  build option "-DENABLE_SLAMFAST=OFF" of make_yolo-planar-slam.sh (default)

  w/o DRP
  ```
  # sed -i -e 's/UseDrp: true/UseDrp: false/g'         Examples/Monocular/ELP_rns-2022-0901.yaml
  # sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g' Examples/Monocular/ELP_rns-2022-0901.yaml
  ```
  
  w/ DRP
  ```
  # sed -i -e 's/UseDrp: false/UseDrp: true/g'         Examples/Monocular/ELP_rns-2022-0901.yaml
  # sed -i -e 's/UseOpenCVA: false/UseOpenCVA: true/g' Examples/Monocular/ELP_rns-2022-0901.yaml

  ```

  In case of  build option "-DENABLE_SLAMFAST=ON" of make_yolo-planar-slam-on.sh 
  w/ DRP
  ```
  # sed -i -e 's/UseDrp: false/UseDrp: true/g'         Examples/Monocular/ELP_rns-2022-0901.yaml
  # sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g' Examples/Monocular/ELP_rns-2022-0901.yaml
  ```

- (17) - 21. Run the following program. (Term2)
  ```
  # ./Examples/Monocular/mono_usbcam \
      $HOME/yolo-planar-slam/Vocabulary/ORBvoc.txt \
      $HOME/yolo-planar-slam/Examples/Monocular/ELP_rns-2022-0901.yaml
  ```

  Parameter settings and operating modes
  | No. | ENABLE_SLAMFAST | UseDrpAI | UseDrp | UseOpenCVA | Yolo | SLAM |
  | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
  | 1 | OFF | true | false | false | DRP-AI | <span style="color: red;">CPU</span> |
  | 2 | OFF | true | true | true | DRP-AI | <span style="color: red;">DRP lib : OpenCVA</span> |
  | 3 | ON | true | true | false | DRP-AI | <span style="color: red;">DRP lib : Custom</span> |

### Browser
- (17) - 22. Browser 
<img src="./img/img14.png" width="80%">

### Re-build and Transfer the application
- (18) Re-build AI-VSLAM application
  
  <span style="color: red;">Open new terminal.</span><br>
  After editing software and build option, execute following command.<br>
  *Example for Modify  "-DENABLE_SLAMFAST=OFF → ON" in make_yolo-planar-slam-on.sh*
  ```
  $ export WORK="<working directory>"
  $ export YOCTO_WORK=${WORK}/src_setup/yocto
  $ cd ${YOCTO_WORK}/yolo-planar-slam

  $ source /opt/poky/3.1.26/environment-setup-aarch64-poky-linux
  $ source ./script/setup.sh

  $ rm -r build
  $ mkdir -p build
  $ cd ./build
  $ cp ${WORK}/src_setup/yolo-planar-slam-drp/setup_files/make_yolo-planar-slam-on.sh .
  
  $ ./make_yolo-planar-slam-on.sh
  ```

  Check the IP address of eth0 (EVK)  and remove the old data (EVK).

  ```
  # ifconfig
    eth0Link encap:Ethernet  HWaddr xx:xx:xx:xx:xx:xx
                   inet addr:192.xxx.xxx.xxx  Bcast:xxx.xxx.xxx.xxx  Mask:xxx.xxx.xxx.xxx

  # cd ${YOCTO_WORK}/yolo-planar-slam
  # rm -r Examples build lib Thirdparty
  ```

  Copy the following directory to EVK.
  ```
  # cd $WORK/yolo-planar-slam
  # scp -r Examples build lib Thirdparty root@192.xxx.xxx.xxx:/home/root/yolo-planar-slam
  ```

  Sync on EVK.
  ```
  # sync
  ```
