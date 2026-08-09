# PV-LIO-PLUS

This code is forked from [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git). The [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git) algorithm is optimized based on the original source code of the [VoxelMap](https://github.com/hku-mars/VoxelMap.git) algorithm. Inspired by [Fast-LIO2](https://github.com/hku-mars/FAST_LIO.git), it utilizes iKFoM as its solver and incorporates tightly-coupled IMU integration. Compared to the original code, PV-LIO features a clearer structure, more stable performance, and higher pose accuracy.

Building upon [VoxelMap](https://github.com/hku-mars/VoxelMap.git), [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git) optimizes the original local map manager and enhances the residual calculation method. This results in improved computational efficiency and reduced memory consumption.

However, during actual testing, the original [VoxelMap](https://github.com/hku-mars/VoxelMap.git), [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git), and [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git) often crashed, despite demonstrating excellent performance on some datasets, particularly when fusing IMU data.

We contribute the following improvements:

- Fixed an issue in the error propagation formula used for calculating residual weights in PV-LIO, enabling stable operation.

- Integrated the VoxelMap++ algorithm into PV-LIO framework by referencing the source code and papers of both VoxelMap and VoxelMap++, allowing algorithm selection via configuration files.

- Addressed stability issues of the VoxelMap++ algorithm in PV-LIO framework, ensuring robust program execution.

- Added a local `MapManager` facade.  It keeps the native PV VoxelMap and
  VoxelMap++ implementations available and adds selectable adapters for
  FAST-LIO2's ikd-tree, Faster-LIO's iVox, and C3P-VoxelMap.  All local map
  sources are grouped as sibling backend directories under
  `include/map_manager/native/`; PV-LIO-PLUS does not link against the other
  ROS packages or their source trees.

- Added map-independent lifecycle operations (initialize, update, search,
  snapshot, local-window movement, and erase) and a common point-to-plane
  match contract.  Backend-specific differences remain in the manager: ikd-tree
  uses native point deletion, while iVox and C3P rebuild from retained points
  when a local window moves because their native APIs do not expose safe erase
  or iteration operations.

- The node now selects the observation model and map lifecycle from
  `mapping/map_type`, and records a trajectory and a finite world-frame point
  cloud when the corresponding output options are enabled.


## Update
- 2023.07.18: Fix eigen failed error for Ubuntu 20.04. 
- 2026.08.09: Add the unified local-map manager, selectable ikd-tree/iVox/C3P
  backends, local-window handling, and explicit trajectory/PCD output semantics.


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
Clone the repository and build the workspace with `catkin_make`:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/vison-yang/PV-LIO-PLUS.git
    cd PV_LIO_PLUS
    cd ../..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 1.3 **livox_ros_driver**)

The current implementation uses C++17.  A normal workspace build is the
acceptance build; `catkin_make --pkg pv_lio_plus` is useful for a quick
iteration but does not replace the full workspace build.

PV-LIO-PLUS does not define its own ROS messages, so its CMake configuration
does not invoke `generate_messages()`.  Legacy warnings from the locally
vendored IKD-Tree/C3P/iVox implementations are scoped to those backend files;
they do not suppress diagnostics in the PV-LIO-PLUS application code.  A full
workspace configure may still print warnings from the installed PCL/VTK
packages or other packages in the workspace; those indicate external package
metadata or optional components and are independent of this node's build.

## 3. Local-map backends

Set one of the following values in the launch YAML under `mapping/map_type`:

| `map_type` | implementation and search contract |
| --- | --- |
| `voxelmap` | PV's original probabilistic adaptive voxel map |
| `voxelmap_plus` | PV's VoxelMap++ map and residual logic |
| `ikdtree` | FAST-LIO2 native k-nearest search, plane fit, and incremental downsample/update |
| `ivox` | Faster-LIO native iVox grid search and incremental downsample/update |
| `c3p_voxelmap` | C3P-VoxelMap native octree residual and optional plane-merging logic |

The legacy `mapping/b_use_voxelmap_plus` switch is still accepted when
`mapping/map_type` is absent.  When both are present, `map_type` has priority.
The point-map parameters (`nearest_point_count`, `nearest_max_range`,
`plane_fit_threshold`, `point_map_downsample_size`, `ivox_*`, `ikd_*`) and the
C3P merge parameters are optional and fall back to native-compatible defaults.

The local backend sources are kept together without changing their native
algorithms:

```
include/map_manager/native/
  voxelmap/       voxel_map_util.hpp
  voxelmap_plus/  voxelmapplus_util.hpp
  ikdtree/        ikd_tree.h, ikd_tree.cpp
  ivox/           ivox3d.h, ivox3d_node.hpp, eigen_types.h, hilbert.hpp
  c3p_voxelmap/   c3p_voxel_map_util.hpp
```

`mapping/local_window_en` is opt-in.  Its half extent is `mapping/det_range`.
The window is centered on the current estimated LiDAR position after each map
update.  VoxelMap/++ retain their native voxel deletion behavior; ikd-tree
deletes points outside the window; iVox and C3P rebuild from the manager's
retained point list.

## 4. Running and outputs

For the included Mid-360 launch file:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch pv_lio_plus mapping_mid360.launch rviz:=false
```

The node subscribes to `/livox/lidar` and `/livox/imu`, publishes odometry on
`/Odometry`, the path on `/path`, and the current manager snapshot on
`/Laser_map`.  Plane markers on `/planes` are published for the voxel-plane
backends; point-map backends intentionally do not synthesize plane markers.

When `pcd_save/pcd_save_en` is true, the node records the undistorted world
point cloud.  On shutdown it writes `output/<backend>_pos.txt` and
`output/<backend>.pcd`, where the backend stems are `pv_lio`, `pv_lio_plus`,
`pv_lio_ikdtree`, `pv_lio_ivox`, and `pv_lio_c3p_voxelmap`.  A positive
`pcd_save/interval` additionally writes intermediate chunks under
`output/PCD/`; `-1` keeps one final PCD.  The saved PCD is the accumulated
world-frame scan cloud, while `/Laser_map` is the selected local-map snapshot.
Trajectory recording is independent of whether `/path` visualization is
enabled.

For backend comparison, run each backend with the same bag and move only the
two final files after shutdown into
`output/full_map_tests_<date>/<backend>/`; switching can be done with runtime
`rosparam` values for `mapping/map_type` and does not require source or YAML
changes.  The completed full-bag verification for this workspace is under
`../../output/full_map_tests_20260809/` with one directory per backend.

For a short local replay, start `roscore` first and then use:

```bash
rosbag play --clock /home/yxy/data/outdoor_Mainbuilding_10hz_2020-12-24-16-38-00.bag
```

The PCL warning about a very small VoxelGrid leaf can occur with this dataset;
it is non-fatal.  The expected acceptance checks are finite trajectory values,
finite PCD fields, and a clean shutdown after Ctrl-C.

## 5. Directly run
Noted:

A. Please make sure the IMU and LiDAR are **Synchronized**, that's important.

B. The warning message "Failed to find match for field 'time'." means the timestamps of each LiDAR points are missed in the rosbag file. That is important for the forward propagation and backwark propagation.

## 6. Related Works
1. [VoxelMap](https://github.com/hku-mars/VoxelMap): An efficient and probabilistic adaptive voxel mapping method for LiDAR odometry.
2. [FAST-LIO](https://github.com/hku-mars/FAST_LIO): A computationally efficient and robust LiDAR-inertial odometry (LIO) package.
3. [IKFoM](https://github.com/hku-mars/IKFoM): A computationally efficient and convenient toolkit of iterated Kalman filter.
4. [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git).
5. [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git).
6. [Faster-LIO](https://github.com/gaoxiang12/faster-lio): source of the local
   iVox implementation copied into `include/map_manager/native/ivox/`.
7. [C3P-VoxelMap](https://github.com/deptrum/c3p-voxelmap): source of the local
   C3P voxel-map implementation copied into
   `include/map_manager/native/c3p_voxelmap/c3p_voxel_map_util.hpp`.

The local copies preserve their upstream headers and implementation logic;
please retain the corresponding upstream license/notice requirements when
redistributing this package.


## 7. Acknowledgments
Thanks a lot for the authors of [VoxelMap](https://github.com/hku-mars/VoxelMap), [IKFoM](https://github.com/hku-mars/IKFoM), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git), and [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git).
