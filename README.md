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
sudo apt-get install libglfw3-dev libgl1-mesa-dev at
```
## ROS 1 Noetic

Follow the [link](http://wiki.ros.org/noetic/Installation/Ubuntu) to install and compile ROS 1 Noetic on you Ubuntu 20.04.

To make life easier, please consider add the following line into bash file.
```
gedit ~/.bashrc
source /opt/ros/noetic/setup.bash
```
## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface.

```
# Install and compile:
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

Download the [source code](https://github.com/ceres-solver/ceres-solver/releases/tag/1.14.0) from offical website.

```
# Build and install:
mkdir build
cd build
cmake ..
make -j8
sudo make install
```
## g2o

```
# Build and install:
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ..
make
sudo make install
```
## Sophus

Download the source code from the [link](https://github.com/strasdat/Sophus/releases/tag/1.22.10).

Most of the time, we need to compile fmt library to get it work. Below is about how to do this:

```
# Build and install:
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
# Build and install:
cd Sophus
mkdir build 
cd build
cmake ..
make
sudo make install
```
## GTSAM

Download the [source code](https://github.com/borglab/gtsam/releases/tag/4.1.0) from the offical webiste.

```
# Build and install:
mkdir build
cd build
cmake ..
make -j8
sudo make install
```
## Eigen 3.3.7

Install from command:

```
sudo apt-get install libeigen3-dev (not recommended because it would be better if we install the specific version.)
```

Install from source:

Get [source code](https://gitlab.com/libeigen/eigen/-/releases/3.3.7) from website.

```
# Build and install:
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

https://github.com/opencv/opencv/releases/tag/4.6.0

**OpenCV_contrib**:

https://github.com/opencv/opencv_contrib/releases/tag/4.6.0

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
## CV_Bridge

```
# Build and install:
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
# Build and install:
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install
```
## Livox-SDK2

```
# Build and install:
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j8
sudo make install
```

## RealSense T265 SDK

Please follow the [link](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) to install.

Download the [source code](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0) from offical website.

```
# Build and install:
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
If you are going to use OpenCV 4.x, you might encounter problems related deprecated outdated APIs. Please check [this](https://blog.csdn.net/m0_52457734/article/details/125343557) to solve them.

```
# Build and compile：

mkdir -p ~/vins_gpu_ws/src/vins-fusion-gpu/src/
cd ~/vins_gpu_ws/src/vins-fusion-gpu/src/
git clone https://github.com/pjrambo/VINS-Fusion-gpu.git

# Go to vins_estimator/CMakeLists.txt

Comment:
#include(/home/dji/opencv/build/OpenCVConfig.cmake)

Add:
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
include(/home/nvidia/ThirdParty/opencv-4.6.0/build/OpenCVConfig.cmake)

# Go to loop_fusion/CMakeLists.txt

Aomment:
#include(/home/dji/opencv/build/OpenCVConfig.cmake)

Add:
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

How to run vins_fusion:
```
roslaunch vins vins_rviz.launch
rosrun vins vins_node src/VINS-Fusion-gpu/config/realsense_t265/stereo_imu.yaml
(optional) rosrun loop_fusion loop_fusion_node src/VINS-Fusion-gpu/config/realsense_t265/stereo_imu.yaml
(optional only if you want to fuse gps) rosrun global_fusion global_fusion_node 
rosbag play YOUR_DATASET_FOLDER/data.bag
```

Dataset for testing:

**EuRoC:**
[EuRoc Datasets](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

**KITTI:**
[KITTI Datasets](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)

## 超核IMU ROS Driver

```
# Build and run:
catkin_make
source devel/setup.bash
roslaunch imu_launch imu_msg.launch
```

Insufficient serial port permissions can be solved by:
```
sudo usermod -a -G dialout $USER
reboot
```

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
See official [documentation](https://github.com/Livox-SDK/livox_ros_driver2) for reference.

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
## Field2Cover 

Please follow the offical [document](https://fields2cover.github.io/index.html) for more details about the coverage planning package.

```
# Build and install:
git clone https://github.com/Fields2Cover/Fields2Cover src/fields2cover
git clone https://github.com/Fields2Cover/fields2cover_ros src/fields2cover_ros
rosdep install -r --ignore-src --from-paths .

If using ROS 1:
catkin_make_isolated

If using ROS 2:
colcon build
```
We have encountered some errors or problems during roslaunch. Fortunately, we have provided several solutions to these errors.

if **rviz_plugins/AerialMapDisplay’ fail to load**:
```
sudo apt-get install ros-noetic-rviz-satellite
```
```
# If you have such errors, it is probably related to the version of PROJ library.
  
ERROR 1: PROJ: proj_create_from_database: SQLite error on SELECT name, type, coordinate_system_auth_name, coordinate_system_code, datum_auth_name, datum_code, area_of_use_auth_name, area_of_use_code, text_definition, deprecated FROM geodetic_crs WHERE auth_name = ? AND code = ?: no such column: area_of_use_auth_name
ERROR 1: PROJ: proj_create_from_database: SQLite error on SELECT name, type, coordinate_system_auth_name, coordinate_system_code, datum_auth_name, datum_code, area_of_use_auth_name, area_of_use_code, text_definition, deprecated FROM geodetic_crs WHERE auth_name = ? AND code = ?: no such column: area_of_use_auth_name
ERROR 1: PROJ: proj_create_from_database: SQLite error on SELECT name, coordinate_system_auth_name, coordinate_system_code, geodetic_crs_auth_name, geodetic_crs_code, conversion_auth_name, conversion_code, area_of_use_auth_name, area_of_use_code, text_definition, deprecated FROM projected_crs WHERE auth_name = ? AND code = ?: no such column: area_of_use_auth_name
ERROR 1: PROJ: proj_create: unrecognized format / unknown name
ERROR 6: Cannot find coordinate operations from `' to `'
```
How to solve it:
```
# Please check the current version or PROJ. Packages named like libproj-dev, proj-bin, proj-data are related to PROJ.
dpkg -l | grep proj

ii  libapache-pom-java                         18-1                                  all          Maven metadata for all Apache Software projects
ii  libcommons-parent-java                     43-1                                  all          Maven metadata for Apache Commons project
ii  libfprint-2-2:arm64                        1:1.90.2+tod1-0ubuntu1~20.04.10       arm64        async fingerprint library of fprint project, shared libraries
ii  libproj-dev:arm64                          6.3.1-1                               arm64        Cartographic projection library (development files)
ii  libproj15:arm64                            6.3.1-1                               arm64        Cartographic projection library
ii  libwebrtc-audio-processing1:arm64          0.3.1-0ubuntu3                        arm64        AudioProcessing module from the WebRTC project.
ii  proj-bin                                   6.3.1-1                               arm64        Cartographic projection library (tools)
ii  proj-data                                  6.3.1-1                               all          Cartographic projection filter and library (datum package)
ii  python3-incremental                        16.10.1-3.2                           all          Library for versioning Python projects.

The compilable version should be 6.3.1-1. If you have the version proj-data which is not 6.3.1-1, please use the lines below to delete it and install the right one.
sudo apt-get remove --purge proj-data

# check the available version if you want.
apt-cache policy proj-data
sudo apt-get install proj-data=6.3.1-1
```
Now, you should be good to go. 
```
roslaunch fields2cover_ros view_field.launch
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
python3 -m pip install --upgrade pip;
python3 -m pip install numpy==1.26.1
python3 -m pip install --no-cache $TORCH_INSTALL
```

Attention: Python 3.8 does not support numpy version that is over 1.24, so the step above might fail. Please consider doing below:
```
pip install numpy==1.19.5
```
How to verify if PyTorch is successfully installed and CUDA is associated with it:

```
# From the terminal, run:
python
# Import PyTorch:
>>> import torch
>>> torch.cuda.is_available()
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
### Train your own model:

1. To train your model with robust dataset, you can try using the [software](https://www.makesense.ai/) to mark the images.

2. Divide the dataset into training set, testing set, and validation set with a ratio of 0.96 : 0.02 : 0.02.

3. Put the individual subdataset into one folder with one images and labels associated with it.
<img src="https://github.com/Xiushishen/Greenworks-RLM4/blob/main/support_files/dataset.png" width = 30% height = 40% div align=center/>

4. Create a yaml file and put it into the data folder in this project.
```
# For my platform, the paths are shown below.
train: /home/nvidia/perception/datasets/car_people_dataset/train/images # path to train dataset
val: /home/nvidia/perception/datasets/car_people_dataset/valid/images # path to val dataset
test: /home/nvidia/perception/datasets/car_people_dataset/test/images # path to test dataset

nc: 2 # Number of classes
names: ['1', '2'] # class names
```

References:

https://blog.csdn.net/lanyan90/article/details/131439255

https://blog.csdn.net/lanyan90/article/details/131411549?spm=1001.2014.3001.5502

-----------------------------------------------------
# 5. CANbus and ROS Connection
The software for CANbus and ROS communication can be found in this repo. It subscribes the /cmd_vel message and output the RPMs for both rear wheels. Meanwhile, it receives the CANbus message outputed from both rear wheels and converts them to ROS message of odometry.

## Jetson AGX Orin CAN Communication

Please follow the [offical document](https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html#enabling-can) for detailed information. Based on my testing experiences, the following steps should be executed and should work well.

Before getting started, we should be familer with the [hardware layout](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html) and parameter settings of the Nvidia Orin. The image below illustrates the 40-pin connecter which let us know which pins we have to connect to.

<img src="https://github.com/Xiushishen/Greenworks-RLM4/blob/main/support_files/can.png" width = 70% height = 50% div align=center />
We will be using the pin positions as shown below:

CAN0: RX——Pin29 ；TX——Pin31

CAN1: RX——Pin37 ；TX——Pin33

GND: Pin 39 / 30

VCC: Pin 17

NVIDIA recommends the [WaveShare SN65HVD230 CAN board](https://www.amazon.com/SN65HVD230-CAN-Board-Communication-Development/dp/B00KM6XMXO/ref=asc_df_B00KM6XMXO/?tag=hyprod-20&linkCode=df0&hvadid=319955522114&hvpos=&hvnetw=g&hvrand=5262066500605912549&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1026076&hvtargid=pla-644638418146&psc=1&mcid=50ab433395023d048fc639fac28b9abf&gclid=CjwKCAjwh4-wBhB3EiwAeJsppPYc8R-u8CA_QxcSyxF63JcCLxxZUwuhfxV0l7Kx5SpDmqPauSvuvxoCduwQAvD_BwE) for development systems. Your choice of transceiver for production devices depends on your application’s requirements.

Make the following connections from the transceiver to the Jetson carrier board:

Transceiver Rx to Jetson CAN_RX

Transceiver Tx to Jetson CAN_TX

Transceiver VCC to Jetson 3.3V pin

Transceiver GND to Jetson GND pin

Now, you should have the proper hardware connection. You should start registering the settings.

### Pinmux

Make sure that the pinmux register settings are applied.
```
sudo apt-get install busybox
sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400
sudo busybox devmem 0x0c303008 w 0xc458
sudo busybox devmem 0x0c303000 w 0xc400
```

### Kernel Drivers

Load the CAN kernel drivers.
```
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```

### Managing the Network

To set the interface properties。
```
sudo ip link set down can0
ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on
sudo ip link set up can0
```
If we have finished the setting, we are now able to test CANbus communication.
```
sudo apt-get install can-utils
cansend can0 123#abcdabcd
candump can0
```
### Loopback test

```
ip link set can0 type can bitrate 500000 loopback on
ip link set can0 up
candump can0 &
cansend can0 123#abcdabcd
```
If the loopback test is successful, the last command displays this:
```
can0 123 [4] AB CD AB CD
can0 123 [4] AB CD AB CD
```



## ROS 1 Message to CAN
https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/developer_kit_layout.html
https://forums.developer.nvidia.com/t/can-communication-issue/263493/31?page=2

https://forums.developer.nvidia.com/t/jetson-orin-nx-can-candump-can0-messages-cannot-be-received-but-cansend-can-be-sent/261012/7

https://docs.nvidia.com/jetson/archives/r35.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html#enabling-can

https://forums.developer.nvidia.com/t/can-communication-issue/263493/31?page=2
