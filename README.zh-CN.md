# PV-LIO-PLUS

[English version](README.md)

PV-LIO-PLUS 是一个 ROS1 激光-惯性里程计（LIO）框架，使用相同的点面估计前端对比不同的局部地图模型。本项目主要继承自 [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git)，继承了 [VoxelMap](https://github.com/hku-mars/VoxelMap.git) 的概率体素地图思想，以及 FAST-LIO 的紧耦合迭代滤波结构。

本项目保留了原 PV-LIO、VoxelMap 和 VoxelMap++ 中已有的问题修复与优化，并新增 `MapManager`，允许通过同一份配置文件选择和对比五种局部地图。所有已合入的地图源码都放在本包内部；PV-LIO-PLUS 不链接其它完整 LIO 工程的源码。

## 主要工作

- 保留 PV-LIO 的紧耦合激光-惯性估计和 IKFoM 迭代滤波。
- 保留原 PV-LIO 观测流程和残差权重相关修复。
- 保留 VoxelMap 的概率自适应体素平面，以及 VoxelMap++ 的局部地图和残差优化。
- 增加 `MapManager`，提供统一的搜索、插入、删除、局部窗口和发布接口，同时保留各地图的原有逻辑。
- 合入 VoxelMap、VoxelMap++、FAST-LIO2 ikd-tree、Faster-LIO iVox 和 C3P-VoxelMap，并支持配置切换。
- 增加明确的轨迹和世界坐标系地图点云输出，便于对比实验。
- 升级为 C++17，并将已合入的局部地图实现放在本包内部。

## 算法清单

清单区分“局部地图后端”和“完整 LIO 软件包”。PV-LIO-PLUS 将局部地图层接入自身的状态估计循环，不会把其它软件包的完整状态估计节点嵌入进来。

### 已合入的局部地图后端

| 算法 / 地图 | 工作区源码或参考位置 | 配置项 | 说明 |
| --- | --- | --- | --- |
| VoxelMap | `src/VoxelMap`；本包副本位于 `include/map_manager/native/voxelmap` | `voxelmap` | 概率自适应体素平面，是 PV-LIO 技术路线中的首个核心局部地图。 |
| VoxelMap++ | 本包副本位于 `include/map_manager/native/voxelmap_plus` | `voxelmap_plus` | 基于 VoxelMap 的地图和残差优化；独立副本已不再保留。 |
| FAST-LIO2 / ikd-tree | `src/FAST_LIO`；本包副本位于 `include/map_manager/native/ikdtree` | `ikdtree` | FAST-LIO2 的增量 KD-tree 地图已合入；完整 `fast_lio` 节点仍作为独立软件包保留。 |
| Faster-LIO / iVox | `src/faster_lio`；本包副本位于 `include/map_manager/native/ivox` | `ivox` | Faster-LIO 的稀疏增量体素地图已合入；完整 `faster_lio` 节点仍作为独立软件包保留。 |
| C3P-VoxelMap | `src/c3p_voxelmap`；本包副本位于 `include/map_manager/native/c3p_voxelmap` | `c3p_voxelmap` | 紧凑概率体素地图，使用原生平面匹配和地图合并逻辑。 |

因此，FAST-LIO2 和 Faster-LIO 已经分别通过 ikd-tree 和 iVox 体现在已合入的局部地图后端中。它们的完整独立节点仍可用于端到端对比，但不再作为另一项 `MapManager` 待合入后端。

### 其它待合入算法

以下算法存在于工作区，但其地图和/或残差逻辑尚未合入 `MapManager`。

| 算法 | 位置 | 当前状态 / 特点 |
| --- | --- | --- |
| Hybrid-VoxelMap | `src/Hybrid-VoxelMap` | 待合入；混合体素/平面表示。 |
| R-VoxelMap | `src/R-VoxelMap` | 待合入；鲁棒递归体素平面估计。 |
| Super-LIO / OctVox | `src/Super-LIO` | 待合入；OctVox 是 Super-LIO 内部的地图结构，当前不是独立软件包。 |
| BIEVR-LIO | `research/BIEVR-LIO` | 待合入；体素化地图、地图引导采样和几何统计。 |
| Surfel-LIO / hVox | `research/Surfel-LIO` | 待合入；层次化体素哈希和预计算 surfel。 |
| LIO-GVM | `research/lio_gvm` | 待合入；高斯体素地图和基于高斯的点云匹配。 |

完整的 `fast_lio` 和 `faster_lio` 节点仍可在 `src/FAST_LIO` 和
`src/faster_lio` 中独立编译，用于端到端对比。由于它们的局部地图层已在上表合入，因此不再列为 `MapManager` 的待合入后端。

`src/livox_ros_driver` 是传感器驱动，不计入算法清单；`research/build` 是编译产物，也不计入算法清单。CT-VoxelMap 和 RC-Vox 在当前工作区没有确认的源码目录，已记录到工作区的 [TODO.md](../../TODO.md)。

## 结构

```text
激光雷达 / IMU
      |
      v
PV-LIO 状态估计与点面残差
      |
      v
MapManager
  |-- VoxelMap
  |-- VoxelMap++
  |-- ikd-tree
  |-- iVox
  `-- C3P-VoxelMap
```

VoxelMap 和 VoxelMap++ 保留原生平面地图匹配；ikd-tree 和 iVox 保留点地图搜索，并使用管理器中的平面拟合适配层；C3P-VoxelMap 使用原生平面地图实现。这样可以在不改变 LIO 状态估计循环的情况下进行局部地图对比。

## 环境与编译

- Ubuntu 20.04
- ROS Noetic
- PCL、Eigen、Sophus、YAML-CPP 及 ROS 工作区依赖

```bash
cd ~/src/lio_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 选择局部地图

在 PV-LIO-PLUS 的 YAML 配置文件中设置地图类型：

```yaml
mapping:
  map_type: ikdtree   # voxelmap, voxelmap_plus, ikdtree, ivox, c3p_voxelmap
```

局部窗口模式使用现有的 `mapping` 窗口参数配置。各后端的专用参数仍放在同一配置文件中，选择其它后端时会被忽略。

## 运行

```bash
source /opt/ros/noetic/setup.bash
source ~/src/lio_ws/devel/setup.bash
roscore
roslaunch pv_lio_plus mapping_avia.launch
rosbag play --clock /path/to/your.bag
```

请根据数据集选择匹配的 launch 文件和传感器参数。

## ROS 接口与结果文件

节点通过 launch/YAML 文件配置的 topic 发布里程计、轨迹/路径、注册点云和局部地图。最终结果写入配置的输出目录：

| 后端 | 轨迹文件 | 最终地图 |
| --- | --- | --- |
| `voxelmap` | `pv_lio_pos.txt` | `pv_lio.pcd` |
| `voxelmap_plus` | `pv_lio_plus_pos.txt` | `pv_lio_plus.pcd` |
| `ikdtree` | `pv_lio_ikdtree_pos.txt` | `pv_lio_ikdtree.pcd` |
| `ivox` | `pv_lio_ivox_pos.txt` | `pv_lio_ivox.pcd` |
| `c3p_voxelmap` | `pv_lio_c3p_voxelmap_pos.txt` | `pv_lio_c3p_voxelmap.pcd` |

轨迹文件保存带时间戳的位姿记录。根据点云保存配置，还可能在 `output/PCD/` 下生成分段点云。进行地图对比时，应对所有后端使用相同的数据包片段、传感器参数、窗口设置和输出目录组织方式。

## 参考项目

- [PV-LIO](https://github.com/HViktorTsoi/PV-LIO)
- [VoxelMap](https://github.com/hku-mars/VoxelMap)
- [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public)
- [FAST-LIO / ikd-tree](https://github.com/hku-mars/FAST_LIO)
- [Faster-LIO / iVox](https://github.com/gaoxiang12/faster-lio)
- [C3P-VoxelMap](https://github.com/deptrum/c3p-voxelmap)
- [Hybrid-VoxelMap](https://github.com/haiyang2022/Hybrid-VoxelMap)
- [R-VoxelMap](https://github.com/NKU-MobFly-Robotics/R-VoxelMap)
- [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1)
- [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO)
- [Surfel-LIO](https://github.com/93won/lidar_inertial_odometry)
- [LIO-GVM](https://github.com/Ji1Xingyu/lio_gvm)
