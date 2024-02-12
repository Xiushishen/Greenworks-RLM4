# Greenworks-RLM4
This repository illustrates the steps needed to set up the environment on NVIDIA Orin for robotic mower development.
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
sudo apt-get install libqglviewer-dev-qt5 
sudp apt-get install wayland-protocols
sudo apt-get install libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libglfw3-dev
sudo apt-get install libssl-dev
```
## ROS 1
(http://wiki.ros.org/noetic/Installation/Ubuntu)

To make life easiesr, please consider add the following line into bash file.
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

sudo apt-get install libglew-dev
sudo apt-get install cmake
sudo apt-get install libpython2.7-dev
sudo apt-get install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
sudo apt-get install libdc1394-22-dev libraw1394-dev
sudo apt-get install libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev
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

Download the source code:

(https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0)

```
mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true
sudo make uninstall
make clean && make -j8
sudo make install
```
# 2. Repositories

## ORB-SLAM3
## VINS-FUSION-GPU
## livox_ros_driver
## livox_ros_driver2
## FAST_LIO2
## livox_camera_calib
## R3LIVE
## robot_pose_ekf
## realsense-ros
## imu_utils
## kalibr
## open-vins
## 



















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
