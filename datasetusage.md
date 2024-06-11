# Dataset preparation

To use the dataset, please follow the steps below to download and preprocess the data.

## Download and preprocess TUM Dataset
### RGB-D Dataset

- (1) Setup to run associate.py.

  Download associate.py.<br>
  Requires python2 and numpy to run.

  ```
  $ wget https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/associate.py
  $ python2 -m pip install --user numpy==1.16.6
  ```

- (2) Download TUM RGBD dataset.

  ```
  $ export WORK="<working directory>"
  $ mkdir -p ${WORK}/dataset
  $ cd ${WORK}/dataset
  $ wget -q https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_walking_xyz.tgz
  $ tar xzf rgbd_dataset_freiburg3_walking_xyz.tgz
  ```

- (3) Preprocessing TUM RGBD dataset.

  ```
  $ python2 associate.py rgbd_dataset_freiburg3_walking_xyz/rgb.txt rgbd_dataset_freiburg3_walking_xyz/depth.txt >  associate.txt
  $ sudo mv associate.txt rgbd_dataset_freiburg3_walking_xyz/associate.txt
  ```

### Monocular Dataset

- (4) Install the imagemagick.

  ```
  $sudo apt install imagemagick
  ```

- (5) Copy TUM RGBD dataset and gray conversion.

  Copy the dataset from gbd_dataset_freiburg3_walking_xyz and create a gray conversion.

  ```
  $ cp -r rgbd_dataset_freiburg3_walking_xyz GRAY_rgbd_dataset_freiburg3_walking_xyz
  $ cd GRAY_rgbd_dataset_freiburg3_walking_xyz
  $ sed -i "s/rgb/gray/g" rgb.txt
  $ mkdir gray
  $ cd rgb/
  $ find *.png | xargs -I {} convert {} -colorspace Gray ../gray/{}
  ```
