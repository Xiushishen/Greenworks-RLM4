# Greenworks-RLM4
This repository illustrates the steps needed to set up the environment on NVIDIA AGX Orin for robotic mower development.
# 1. Prerequisites  
The essential software environment is based on Ubuntu 20.04. We have tested all the repositories on Ubuntu 20.04 with the OpenCV verison of 4.6.0. It is strongly recommended to put all the **Dependencies** and **SDKs** into one specific folder to mak the environment neat.

## Basic libraries

``` 
sudo apt-get install gcc g++ vim cmake git
sudo apt-get install libglew-dev
sudo apt-get install libpython2.7-dev
sudo apt-get install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
sudo apt-get install libdc1394-22-dev libraw1394-dev
sudo apt-get install libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev
sudo apt-get install liblapack-dev libsuitesparse-dev libgflags-dev libgoogle-glog-dev libgtest-dev libcxsparse3 -y
sudo apt-get install libboost-all-dev
sudo apt-get install libxkbcommon-devsudo apt-get install libeigen3-dev qtdeclarative5-dev qt5-qmake
sudo apt-get install libqglviewer-dev-qt5 wayland-protocols
sudo apt-get install libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libusb-1.0-0-dev libssl-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```
## ROS 1
(http://wiki.ros.org/noetic/Installation/Ubuntu)

To make life easier, please consider add the following line into bash file.
```
gedit ~/.bashrc
source /opt/ros/noetic/setup.bash
```
## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface.

```
# Build and Install:
git clone https://github.com/stevenlovegrove/Pangolin.git (version 0.9/0.6)
cd Pangolin
mkdir build
cd build
cmake ..
make –j8
sudo make install
```
## PCL
```
sudo apt update
sudo apt-get install libpcl-dev pcl-tools
```
## ceres-solver

Download the source code:

(https://github.com/ceres-solver/ceres-solver/releases/tag/1.14.0)

```
mkdir build
cd build
cmake ..
make -j8
sudo make install
```
## G2O

```
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
make
sudo make install
```
## Sophus

Download the source code from the link below:

(https://github.com/strasdat/Sophus/releases/tag/1.22.10)

Most of the time, we need to compile fmt library to get it work. Below is about how to do this:

```
git clone https://github.com/fmtlib/fmt.git
cd fmt
mkdir build 
cd build
cmake ..
make
sudo make install
```

Let's compile Sophus:
```
cd Sophus
mkdir build 
cd build
cmake ..
make
sudo make install
```
## GTSAM

Download the source code:

https://github.com/borglab/gtsam/releases/tag/4.1.0

```
mkdir build
cd build
cmake ..
make -j8
sudo make install
```
## Eigen3 3.3.7

Install via command:

```
sudo apt-get install libeigen3-dev (not recommended because it would be better if we install the specific version.)
```

Install via source:

Get source code from website:

(https://gitlab.com/libeigen/eigen/-/releases/3.3.7)

```
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

Header files can be found in "/usr/include/eigen3"

## OpenCV 4.6.0 with CUDA 

Please follow the steps below carefully.

Delete the auto-installed OpenCV on Nvidia Orin:

```
dpkg -l | grep libopencv
sudo apt-get remove libopencv*
sudo apt-get autoremove
sudo apt-get autoclean
```
If we don't delete other version OpenCV, we probably will get conflict and some reposities will fail to run.

Download source code of **OpenCV 4.6.0** and **OpenCV_Contrib 4.6.0**. Also, put the OpenCV and OpenCV_Contrib source code into the specific folder mentioned before. 

**Opencv 4.6.0**:

(https://github.com/opencv/opencv/releases/tag/4.6.0)

**OpenCV_contrib**:

(https://github.com/opencv/opencv_contrib/releases/tag/4.6.0)

Computing power for NVIDIA Orin:

<img src="https://github.com/Xiushishen/Greenworks-RLM4/blob/main/support_files/cp.png" width = 70% height = 50% div align=center />


```
cd ~/path_to_folder/opencv-4.6.0/
mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local/ \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.6.0/modules \
        -D WITH_CUDA=ON \
        -D CUDA_ARCH_BIN=8.7 \
        -D CUDA_ARCH_PTX="" \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_LIBV4L=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_GSTREAMER_0_10=OFF \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
        -D WITH_TBB=ON \
        ..

# CUDA_ARCCH_BIN=8.7 (the amount of computing power for NVIDIA Orin)
# CMAKE_INSTALL_PREFIX=/usr/local/ (path to installation)
# OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.6.0/modules (path to additional module)

sudo make install -j8
```
Check if the CUDA-accelerated OpenCV is successfully installed:
```
# Install jtop:
sudo apt-get install python3-pip -y
sudo -H pip3 install -U jetson-stats
sudo systemctl restart jtop.service

# reboot if needed
reboot
```

Open terminal:
```
jtop
7 (go to INFO)

# if there is one line saying OpenCV: 4.6.0 with CUDA: YES, the OpenCV is successfully installed.
```
## cv_bridge

```
git clone https://github.com/ros-perception/vision_opencv.git -b noetic
# we only need the cv_bridge folder
cd cv_bridge
gedit ./CMakeLists.txt
# include OpenCV cmake file
include("/home/nvidia/ThirdParty/opencv-4.6.0/build/OpenCVConfig.cmake")
mkdir build && cd build
cmake ..
make -j8
sudo make install
```
How to use the cv_bridge:
```
# set this line in your project's CMakeLists.txt file just after the project(xxxx) syntax
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
```


## Livox-SDK

```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install
```
## Livox-SDK2

```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j8
sudo make install
```

## realsense-t265 SDK

Please follow the link to install:

(https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

Download the source code:

(https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0)

```
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release .. 
make -j8
sudo make install
```
How to visualize and obtain parameters of T265:
```
realsense-viewer

rs-enumerate-devices -c
```

# 2. Repositories

## ORB-SLAM3

To better use ORBSLAM3 for robot mower software development, we are going to use a revised version for better intergration.

```
cd ~/orb_slam3_ws/src
git clone https://github.com/thien94/orb_slam3_ros.git
# add one line into the CMakeLists.txt
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
cd ../
catkin_make
```
Set config files (T265 Stereo-Inertial camera):
```
# According to T256 camera
Intrinsic Params:

PPX  -->  Camera.cx
PPY  -->  Camera.cy
Fx   -->  Camera.fx
Fy   -->  Camera.fy
Coeffs[0]  -->  Camera.k1
Coeffs[1]  -->  Camera.k2
Coeffs[2]  -->  Camera.k3
Coeffs[3]  -->  Camera.k4

Extrinsic Params:

Rotation_Matrix (Extrinsic from "Gyro" To "Fisheye 1")  --> Tbc.data.R
Rotation_Matrix[0][0]  -->  Tbc.data[0][0]
Rotation_Matrix[0][1]  -->  Tbc.data[0][1]
Rotation_Matrix[0][2]  -->  Tbc.data[0][2]
Rotation_Matrix[1][0]  -->  Tbc.data[1][0]
Rotation_Matrix[1][1]  -->  Tbc.data[1][1]
Rotation_Matrix[1][2]  -->  Tbc.data[1][2]
Rotation_Matrix[2][0]  -->  Tbc.data[2][0]
Rotation_Matrix[2][1]  -->  Tbc.data[2][1]
Rotation_Matrix[2][2]  -->  Tbc.data[2][2]

Translation Vector (Extrinsic from "Gyro" To "Fisheye 1")  --> Tbc.data.t
Translation_Vector[0]  -->  Tbc.data[0][3]
Translation_Vector[1]  -->  Tbc.data[1][3]
Translation_Vector[2]  -->  Tbc.data[2][3]

IMU intrinsic params:
# The four parameters are needed to be calibrated using imu_utils.
Gyr.avg-axis.gyr_n  -->  IMU.NoiseGyro
Gyr.avg-axis.gyr_w  -->  IMU.GyroWalk
Acc.avg-axis.acc_n  -->  IMU.NoiseAcc
Acc.avg-axis.acc_w  -->  IMU.AccWalk

```
Live stereo-inertial mode with Realsense T265

Modify the original rs_t265.launch to enable fisheye images and imu data (change unite_imu_method to linear_interpolation). Run rs-enumerate-devices -c to get the calibration parameters and modify config/Stereo-Inertial/RealSense_T265.yaml accordingly.

```
roslaunch realsense2_camera rs_t265.launch
roslaunch orb_slam3_ros rs_t265_stereo_inertial.launch
```

Please also check this repo for more information.
(https://github.com/shanpenghui/ORB_SLAM3_Fixed#73-set-camera-intrinsic--extrinsic-parameters)

## VINS-FUSION-GPU

```
mkdir -p ~/vins_gpu_ws/src/vins-fusion-gpu/src/
cd ~/vins_gpu_ws/src/vins-fusion-gpu/src/
git clone https://github.com/pjrambo/VINS-Fusion-gpu.git

# go to vins_estimator/CMakeLists.txt
comment:Next, install PyTorch with the following steps: 

#include(/home/dji/opencv/build/OpenCVConfig.cmake)

add:
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
include(/home/nvidia/ThirdParty/opencv-4.6.0/build/OpenCVConfig.cmake)

# go to loop_fusion/CMakeLists.txt
comment:
#include(/home/dji/opencv/build/OpenCVConfig.cmake)

add:
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
include(/home/nvidia/ThirdParty/opencv-4.6.0/build/OpenCVConfig.cmake)

catkin_make
```
If your other application do not require much GPU resources, I recommanded you to set

```
use_gpu: 1
use_gpu_acc_flow: 1
```
According to my test, on Nvidia Orin if you set this two parameters to 1 at the same time, the GPU usage is about 20%.

How to run vins:
```
roslaunch vins vins_rviz.launch
rosrun vins vins_node src/VINS-Fusion-gpu/config/realsense_t265/stereo_imu.yaml
(optional) rosrun loop_fusion loop_fusion_node src/VINS-Fusion-gpu/config/realsense_t265/stereo_imu.yaml
(optional only if you want to fuse gps) rosrun global_fusion global_fusion_node 
rosbag play YOUR_DATASET_FOLDER/data.bag
```

Dataset for testing:

**EuRoC:**
(https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

**KITTI:**
(https://www.cvlibs.net/datasets/kitti/eval_odometry.php)

## livox_ros_driver

Most of the time, this repo will only be used for compiling other packages if we use Livox HAP lidar.

## livox_ros_driver2

Install and build:

```
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

How to run HAP Lidar:
```
ros2 launch livox_ros_driver2 rviz_HAP.launch
ros2 launch livox_ros_driver2 msg_HAP.launch
```
See official documentation for reference:

(https://github.com/Livox-SDK/livox_ros_driver2)

## FAST_LIO2

Install and build:

```
cd ~/$A_ROS_DIR$/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
catkin_make
source devel/setup.bash
```
How to run:

```
cd ~/$FAST_LIO_ROS_DIR$
source devel/setup.bash
roslaunch fast_lio mapping_HAP.launch
```
Below is the config file for HAP Lidar:
```
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
    time_sync_en: false         # ONLY turn on when external time synchronization is really not possible
    time_offset_lidar_to_imu: 0.0 # Time offset between lidar and IMU calibrated by other algorithms, e.g. LI-Init (can be found in README).
                                  # This param will take effect no matter what time_sync_en is. So if the time offset is not known exactly, please set as 0.0

preprocess:
    lidar_type: 1                # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR, 
    scan_line: 128
    blind: 0.1

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree:    120
    det_range:     150.0
    extrinsic_est_en:  true      # true: enable the online estimation of IMU-LiDAR extrinsic
    extrinsic_T: [ 0, 0, 0 ]
    extrinsic_R: [ 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1]

publish:
    path_en:  false
    scan_publish_en:  true       # false: close all the point cloud output
    dense_publish_en: true       # false: low down the points number in a global-frame point clouds scan.
    scan_bodyframe_pub_en: true  # true: output the point cloud scans in IMU-body-frame

pcd_save:
    pcd_save_en: true
    interval: -1                 # how many LiDAR frames saved in each pcd file; 
                                 # -1 : all frames will be saved in ONE pcd file, may lead to memory crash when having too much frames.
```
## R3LIVE

R3LIVE is not supported on NVIDIA AGX Orin.

## robot_pose_ekf

EKF based robotic mower localization via IMU, GPS, odom and VIO (VINS-FUSION).

If this is your first time installing this package, you probably counter this error especially on Ubuntu 20.04.
```
No package 'orocos-bfl' found
```
Here is how to solve it:
```
sudo apt-get install liborocos-bfl-dev
```
Detailed information:

(https://blog.csdn.net/shoufei403/article/details/102655696)

## OpenVINS

The [document](https://docs.openvins.com/) made by Dr. Huang's lab is really comprehensive and precise. It provides all the knowledge about how to compile and understand the codes.

However, we would like to illustrate more details based on the NVIDIA AGX Orin.

Build and compile on ROS 1 Noetic:
```
# Prerequisite:
sudo apt-get install libboost-all-dev
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
export ROS1_DISTRO=noetic # kinetic=16.04, melodic=18.04, noetic=20.04
sudo apt-get install ros-$ROS1_DISTRO-desktop-full
sudo apt-get install python3-catkin-tools python3-osrf-pycommon # ubuntu 20.04
```
```
# Clone and compile:
mkdir -p ~/workspace/open_vins_ws/src/
cd ~/workspace/open_vins_ws/src/
git clone https://github.com/rpng/open_vins/
cd ..
catkin_catkin # ROS1
```
```
# Because we have made the **cv_bridge** from source code, we have to add the dependencies into CMakeLists.txt

# In ov_core, add the line below to its CMakeLists.txt.
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

# In ov_msckf, add the line below to its CMakeLists.txt.
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
```
```
# Additional Evaluation Requirements
sudo apt-get install python3-dev python3-matplotlib python3-numpy python3-psutil python3-tk # for python3 systems
```


## realsense-ros

Build and compile:
```
mkdir t265_ws && cd t265_ws
mkdir src && cd src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense2_camera
gedit CMakeLists.txt

# add one line before **find_package**.
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

# go back to the worksapce and compile the package
catkin_make
```

How to run:
```
source devel/setup.bash
roslaunch realsense2_camera rs_t265.launch 
```

Here is how to revise the rs_t265.launch based on RealSense T265 camera:
```
<!--
Important Notice: For wheeled robots, odometer input is a requirement for robust
and accurate tracking. The relevant APIs will be added to librealsense and
ROS/realsense in upcoming releases. Currently, the API is available in the
https://github.com/IntelRealSense/librealsense/blob/master/third-party/libtm/libtm/include/TrackingDevice.h#L508-L515.
-->
<launch>
  <arg name="serial_no"           default=""/>
  <arg name="usb_port_id"         default=""/>
  <arg name="device_type"         default="t265"/>
  <arg name="json_file_path"      default=""/>
  <arg name="camera"              default="camera"/>
  <arg name="tf_prefix"           default="$(arg camera)"/>

  <arg name="fisheye_width"       default="848"/> 
  <arg name="fisheye_height"      default="800"/>
  <arg name="enable_fisheye1"     default="true"/>
  <arg name="enable_fisheye2"     default="true"/>

  <arg name="fisheye_fps"         default="30"/>

  <arg name="gyro_fps"            default="200"/>
  <arg name="accel_fps"           default="62"/>
  <arg name="enable_gyro"         default="true"/>
  <arg name="enable_accel"        default="true"/>
  <arg name="enable_pose"         default="true"/>

  <arg name="enable_sync"           default="true"/>

  <arg name="linear_accel_cov"      default="0.01"/>
  <arg name="initial_reset"         default="false"/>
  <arg name="reconnect_timeout"     default="6.0"/>
  <arg name="unite_imu_method"      default="linear_interpolation"/>

  <arg name="publish_odom_tf"     default="true"/>
  
  <group ns="$(arg camera)">
    <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">
      <arg name="tf_prefix"                value="$(arg tf_prefix)"/>
      <arg name="serial_no"                value="$(arg serial_no)"/>
      <arg name="usb_port_id"              value="$(arg usb_port_id)"/>
      <arg name="device_type"              value="$(arg device_type)"/>
      <arg name="json_file_path"           value="$(arg json_file_path)"/>

      <arg name="enable_sync"              value="$(arg enable_sync)"/>

      <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
      <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
      <arg name="enable_fisheye1"          value="$(arg enable_fisheye1)"/>
      <arg name="enable_fisheye2"          value="$(arg enable_fisheye2)"/>

      <arg name="fisheye_fps"              value="$(arg fisheye_fps)"/>
      <arg name="gyro_fps"                 value="$(arg gyro_fps)"/>
      <arg name="accel_fps"                value="$(arg accel_fps)"/>
      <arg name="enable_gyro"              value="$(arg enable_gyro)"/>
      <arg name="enable_accel"             value="$(arg enable_accel)"/>
      <arg name="enable_pose"              value="$(arg enable_pose)"/>

      <arg name="linear_accel_cov"         value="$(arg linear_accel_cov)"/>
      <arg name="initial_reset"            value="$(arg initial_reset)"/>
      <arg name="reconnect_timeout"        value="$(arg reconnect_timeout)"/>
      <arg name="unite_imu_method"         value="$(arg unite_imu_method)"/>

      <arg name="publish_odom_tf"          value="$(arg publish_odom_tf)"/>
    </include>
  </group>
</launch>
```
# 3. Calibration
## livox_camera_calib

Install and build:

```
cd ~/lidar_cam_calib_ws/src
git clone https://github.com/hku-mars/livox_camera_calib.git
cd ../
catkin_make
source ~/lidar_cam_calib_ws/devel/setup.bash
```

How to run it:

```
roslaunch livox_camera_calib calib.launch
```
Below is the calib.yaml:
```
# Data path. adjust them!
common:
    image_file: "/home/nvidia/robotics/lidar_cam_calib_ws/src/livox_camera_calib/data/7.jpg"
    pcd_file: "/home/nvidia/robotics/lidar_cam_calib_ws/src/livox_camera_calib/data/7.pcd"
    result_file: "/home/nvidia/robotics/lidar_cam_balib_ws/src/livox_camera_calib/result/extrinsic.txt"

# Camera Parameters. Adjust them!
camera:
    camera_matrix: [701.417236328125, 0.0, 639.5635986328125,
                0.0,     701.0556030273438,  362.57061767578125,
                0.0,     0.0,      1.0     ] 
    dist_coeffs: [-0.047434888780117035, -0.012185392901301384, 0.000674429873470217, -0.0011172627564519644, 0.000000] 

# Calibration Parameters.!
calib:
    calib_config_file: "/home/nvidia/robotics/lidar_cam_calib_ws/src/livox_camera_calib/config/config_outdoor.yaml"
    use_rough_calib: true # set true if your initial_extrinsic is bad
```
Below is the config_outdoor.yaml:
```
%YAML:1.0

# Topic name in rosbag
PointCloudTopic: "/livox/lidar"
ImageTopic: "/camera/color/image_raw"

# Lidar Data type(custom msg or pointcloud2)
Data.custom_msg: 0
# Initial extrinsic (usually provided by hand measurement or cad design)

ExtrinsicMat: !!opencv-matrix # Lidar -> camera
  rows: 4
  cols: 4
  dt: d
  data: [0.0,   -1.0,   0.0,    0.0,
         0.0,  0.0,  -1.0,    0.0,
         1.0,   0.0,    0.0,    0.0,
         0.0,   0.0,    0.0,    1.0]
# Params for Canny Edge Extraction

Canny.gray_threshold: 20
Canny.len_threshold: 200

# Params for Voxel Cutting & Plane Fitting & Edge Extraction
Voxel.size: 1.0
Voxel.down_sample_size: 0.02
Plane.min_points_size: 60
Plane.normal_theta_min: 30
Plane.normal_theta_max: 150
Plane.max_size: 5
Ransac.dis_threshold: 0.015
Ransac.iter_num: 200
Edge.min_dis_threshold: 0.03
Edge.max_dis_threshold: 0.06

# Params for color point clouds
Color.dense: 1
Color.intensity_threshold: 10
```
The ExtrinsicMat is the extrinsic matrix between camera and Lidar, which is needed to modify depending on your sensor layout.
## imu_utils

## kalibr

# 4. Percpetion

## YOLOv7

Below is about how to install and compile YOLOv7 on NVIDIA AGX Orin. Before we get started, please make sure **conda** is successfully installed.

```
# Create conda environment for YOLOv7
conda create -n yolov7 python=3.8
conda activate yolov7

# Install system packages required by PyTorch
sudo apt-get -y update
sudo apt-get -y install autoconf bc build-essential g++-8 gcc-8 clang-8 lld-8 gettext-base gfortran-8 iputils-ping libbz2-dev libc++-dev libcgal-dev libffi-dev libfreetype6-dev libhdf5-dev libjpeg-dev liblzma-dev libncurses5-dev libncursesw5-dev libpng-dev libreadline-dev libssl-dev libsqlite3-dev libxml2-dev libxslt-dev locales moreutils openssl python-openssl rsync scons python3-pip libopenblas-dev;
```
Before installing Pytorch, please check the version of your Jetpack.
```
sudo apt-cache show nvidia-jetpack
```
Below is the detailed information about your Jetpack. You need to find the corresponding Pytorch version based on Jetpack version.
```
nvidia@nvidia-desktop:~$ sudo apt-cache show nvidia-jetpack
[sudo] password for nvidia: 
Package: nvidia-jetpack
Version: 5.1.2-b104
Architecture: arm64
Maintainer: NVIDIA Corporation
Installed-Size: 194
Depends: nvidia-jetpack-runtime (= 5.1.2-b104), nvidia-jetpack-dev (= 5.1.2-b104)
Homepage: http://developer.nvidia.com/jetson
Priority: standard
Section: metapackages
Filename: pool/main/n/nvidia-jetpack/nvidia-jetpack_5.1.2-b104_arm64.deb
Size: 29304
SHA256: fda2eed24747319ccd9fee9a8548c0e5dd52812363877ebe90e223b5a6e7e827
SHA1: 78c7d9e02490f96f8fbd5a091c8bef280b03ae84
MD5sum: 6be522b5542ab2af5dcf62837b34a5f0
Description: NVIDIA Jetpack Meta Package
Description-md5: ad1462289bdbc54909ae109d1d32c0a8
```
The link below you can check the version correspondence. For me, I have the Jetpack version of **Version: 5.1.2-b104**, I should be using JetPack 5 PyTorch v2.1.0.

https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

<img src="https://github.com/Xiushishen/Greenworks-RLM4/blob/main/support_files/jetpackversion.png" width = 70% height = 50% div align=center />

Now, you can install PyTorch with the following steps:
```
export TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v$JP_VERSION/pytorch/$PYT_VERSION
Where:
**JP_VERSION**
    The major and minor version of JetPack you are using, such as 461 for JetPack 4.6.1 or 50 for JetPack 5.0. 
**PYT_VERSION**
    The released version of the PyTorch wheel.

For example:

export TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
python3 -m pip install --upgrade pip; python3 -m pip install numpy==’1.26.1’ python3 -m pip install --no-cache $TORCH_INSTALL
```

Attention: Python 3.8 does not support numpy version that is over 1.24, so the step above might fail. Please consider doing below:
```
pip install numpy==1.19.5
```
How to verify if Pytorch is successfully installed:

```
# From the terminal, run:
python
# Import PyTorch:
>>> import torch
```
If PyTorch was installed correctly, this command should execute without error. 

For more reference, please check the link below.

https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html

Now you can install torchvision:

Check your corresponding torchvision for the Pytorch installed. For me, the corresponding torchvision is **0.16.0**.

https://blog.csdn.net/shiwanghualuo/article/details/122860521

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev 
git clone --branch v0.16.0 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.16.0
# the step below might take longer time because we build from source.
python3 setup.py install --user
```
If there are no errors, torchvision is successfully installed. Repeat the previous verification step to check if everything is good.
```
python
>>> import torch
>>> import torchvision
```
If we miss any libraries, we can just pip install it until there are no errors.

Finally, we can install and compile YOLOv7:
```
git clone https://github.com/WongKinYiu/yolov7
```
```
vim requirements.txt
# comment out the two lines.
torch>=1.7.0,!=1.12.0
torchvision>=0.8.1,!=0.13.0
```
**If you do not comment out the two lines above, the torch will be CPU version instead of GPU version.**
```
pip install -r requirements.txt
```
Download the weight from github, and you can test:
```
python detect.py --weights weights/yolov7.pt --source inference/images 
```

References:

https://blog.csdn.net/lanyan90/article/details/131439255

https://blog.csdn.net/lanyan90/article/details/131411549?spm=1001.2014.3001.5502

-----------------------------------------------------
# 5. CANbus and ROS 1 Connection

https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html




## 2.1 Change the opencv path in the CMakeLists
In /vins_estimator/CMakeLists.txt, change Line 20 to your path.  
In /loop_fusion/CmakeLists.txt, change Line 19 to your path.
### 2.2 Change the acceleration parameters as you need.
In the config file, there are two parameters for gpu acceleration.  
use_gpu: 0 for off, 1 for on  
use_gpu_acc_flow:  0 for off, 1 for on  
If your GPU resources is limitted or you want to use GPU for other computaion. You can set  
use_gpu: 1  
use_gpu_acc_flow: 0  
If your other application do not require much GPU resources, I recommanded you to set  
use_gpu: 1  
use_gpu_acc_flow: 1  
According to my test, on TX2 if you set this two parameters to 1 at the same time, the GPU usage is about 20%.

# VINS-Fusion
## An optimization-based multi-sensor state estimator

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/vins_logo.png" width = 55% height = 55% div align=left />
<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.png" width = 34% height = 34% div align=center />

VINS-Fusion is an optimization-based multi-sensor state estimator, which achieves accurate self-localization for autonomous applications (drones, cars, and AR/VR). VINS-Fusion is an extension of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono), which supports multiple visual-inertial sensor types (mono camera + IMU, stereo cameras + IMU, even stereo cameras only). We also show a toy example of fusing VINS with GPS. 
**Features:**
- multiple sensors support (stereo cameras / mono camera+IMU / stereo cameras+IMU)
- online spatial calibration (transformation between camera and IMU)
- online temporal calibration (time offset between camera and IMU)
- visual loop closure

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti_rank.png" width = 80% height = 80% />

We are the **top** open-sourced stereo algorithm on [KITTI Odometry Benchmark](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) (12.Jan.2019).

**Authors:** [Tong Qin](http://www.qintonguav.com), Shaozu Cao, Jie Pan, [Peiliang Li](https://peiliangli.github.io/), and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [Aerial Robotics Group](http://uav.ust.hk/), [HKUST](https://www.ust.hk/)

**Videos:**

<a href="https://www.youtube.com/embed/1qye82aW7nI" target="_blank"><img src="http://img.youtube.com/vi/1qye82aW7nI/0.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>


**Related Papers:** (papers are not exactly same with code)
* **A General Optimization-based Framework for Local Odometry Estimation with Multiple Sensors**, Tong Qin, Jie Pan, Shaozu Cao, Shaojie Shen, aiXiv [pdf](https://arxiv.org/abs/1901.03638) 

* **A General Optimization-based Framework for Global Pose Estimation with Multiple Sensors**, Tong Qin, Shaozu Cao, Jie Pan, Shaojie Shen, aiXiv [pdf](https://arxiv.org/abs/1901.03642) 

* **Online Temporal Calibration for Monocular Visual-Inertial Systems**, Tong Qin, Shaojie Shen, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2018), **best student paper award** [pdf](https://ieeexplore.ieee.org/abstract/document/8593603)

* **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Shaojie Shen, IEEE Transactions on Robotics [pdf](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert) 


*If you use VINS-Fusion for your academic research, please cite our related papers. [bib](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/paper_bib.txt)*

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## 2. Build VINS-Fusion
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. EuRoC Example
Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) to YOUR_DATASET_FOLDER. Take MH_01 for example, you can run VINS-Fusion with three sensor types (monocular camera + IMU, stereo cameras + IMU and stereo cameras). 
Open four terminals, run vins odometry, visual loop closure(optional), rviz and play the bag file respectively. 
Green path is VIO odometry; red path is odometry under visual loop closure.

### 3.1 Monocualr camera + IMU

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

### 3.2 Stereo cameras + IMU

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

### 3.3 Stereo cameras

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/euroc.gif" width = 430 height = 240 />


## 4. KITTI Example
### 4.1 KITTI Odometry (Stereo)
Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER. Take sequences 00 for example,
Open two terminals, run vins and rviz respectively. 
(We evaluated odometry on KITTI benchmark without loop closure funtion)
```
    roslaunch vins vins_rviz.launch
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml
    rosrun vins kitti_odom_test ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/ 
```
### 4.2 KITTI GPS Fusion (Stereo + GPS)
Download [KITTI raw dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) to YOUR_DATASET_FOLDER. Take [2011_10_03_drive_0027_synced](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip) for example.
Open three terminals, run vins, global fusion and rviz respectively. 
Green path is VIO odometry; blue path is odometry under GPS global fusion.
```
    roslaunch vins vins_rviz.launch
    rosrun vins kitti_gps_test ~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/ 
    rosrun global_fusion global_fusion_node
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.gif" width = 430 height = 240 />

## 5. VINS-Fusion on car demonstration
Download [car bag](https://drive.google.com/open?id=10t9H1u8pMGDOI6Q2w2uezEq5Ib-Z8tLz) to YOUR_DATASET_FOLDER.
Open four terminals, run vins odometry, visual loop closure(optional), rviz and play the bag file respectively. 
Green path is VIO odometry; red path is odometry under visual loop closure.
```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/vi_car/vi_car.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/vi_car/vi_car.yaml 
    rosbag play YOUR_DATASET_FOLDER/car.bag
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/car_gif.gif" width = 430 height = 240  />


## 6. Run with your devices 
VIO is not only a software algorithm, it heavily relies on hardware quality. For beginners, we recommend you to run VIO with professional equipment, which contains global shutter cameras and hardware synchronization.

### 6.1 Configuration file
Write a config file for your device. You can take config files of EuRoC and KITTI as the example. 

### 6.2 Camera calibration
VINS-Fusion support several camera models (pinhole, mei, equidistant). You can use [camera model](https://github.com/hengli/camodocal) to calibrate your cameras. We put some example data under /camera_models/calibrationdata to tell you how to calibrate.
```
cd ~/catkin_ws/src/VINS-Fusion/camera_models/camera_calib_example/
rosrun camera_models Calibrations -w 12 -h 8 -s 80 -i calibrationdata --camera-model pinhole
```


## 7. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, a generic [camera model](https://github.com/hengli/camodocal) and [GeographicLib](https://geographiclib.sourceforge.io/).

## 8. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong Qin <qintonguavATgmail.com>.

For commercial inquiries, please contact Shaojie Shen <eeshaojieATust.hk>.
