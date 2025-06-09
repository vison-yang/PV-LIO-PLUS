# PV-LIO-PLUS

This code is forked from [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git). The [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git) algorithm is optimized based on the original source code of the [VoxelMap](https://github.com/hku-mars/VoxelMap.git) algorithm. Inspired by [Fast-LIO2](https://github.com/hku-mars/FAST_LIO.git), it utilizes iKFoM as its solver and incorporates tightly-coupled IMU integration. Compared to the original code, PV-LIO features a clearer structure, more stable performance, and higher pose accuracy.

Building upon [VoxelMap](https://github.com/hku-mars/VoxelMap.git), [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git) optimizes the original local map manager and enhances the residual calculation method. This results in improved computational efficiency and reduced memory consumption.

However, during actual testing, the original [VoxelMap](https://github.com/hku-mars/VoxelMap.git), [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git), and [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git) often crashed, despite demonstrating excellent performance on some datasets, particularly when fusing IMU data.

We contribute the following improvements:

- Fixed an issue in the error propagation formula used for calculating residual weights in PV-LIO, enabling stable operation.

- Integrated the VoxelMap++ algorithm into PV-LIO framework by referencing the source code and papers of both VoxelMap and VoxelMap++, allowing algorithm selection via configuration files.

- Addressed stability issues of the VoxelMap++ algorithm in PV-LIO framework, ensuring robust program execution.


## Update
- 2023.07.18: Fix eigen failed error for Ubuntu 20.04. 


## 1. Prerequisites

### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 16.04**

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for PV-LIO to work normally.

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### 1.3. **livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

*Remarks:*
- The **livox_ros_driver** must be installed and **sourced** before run any PV-LIO launch file.
- How to source? The easiest way is add the line ``` source $Livox_ros_driver_dir$/devel/setup.bash ``` to the end of file ``` ~/.bashrc ```, where ``` $Livox_ros_driver_dir$ ``` is the directory of the livox ros driver workspace (should be the ``` ws_livox ``` directory if you completely followed the livox official document).


## 2. Build
Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/vison-yang/PV-LIO-PLUS.git
    cd PV_LIO_PLUS
    cd ../..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 1.3 **livox_ros_driver**)
  
## 3. Directly run
Noted:

A. Please make sure the IMU and LiDAR are **Synchronized**, that's important.

B. The warning message "Failed to find match for field 'time'." means the timestamps of each LiDAR points are missed in the rosbag file. That is important for the forward propagation and backwark propagation.

## Related Works
1. [VoxelMap](https://github.com/hku-mars/VoxelMap): An efficient and probabilistic adaptive voxel mapping method for LiDAR odometry.
2. [FAST-LIO](https://github.com/hku-mars/FAST_LIO): A computationally efficient and robust LiDAR-inertial odometry (LIO) package.
3. [IKFoM](https://github.com/hku-mars/IKFoM): A computationally efficient and convenient toolkit of iterated Kalman filter.
4. [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git).
5. [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git).


## Acknowledgments
Thanks a lot for the authors of [VoxelMap](https://github.com/hku-mars/VoxelMap), [IKFoM](https://github.com/hku-mars/IKFoM), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git), and [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git).
