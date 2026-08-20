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

清单区分“局部地图后端”和“完整 LIO 软件包”。PV-LIO-PLUS 将局部地图层接入自身的状态估计循环，不会把其它软件包的完整状态估计节点嵌入进来。本仓库只包含 `include/map_manager/native/` 下的局部地图层。

### 已合入的局部地图后端

| 算法 / 地图 | 本仓库包含的源码 | 配置项 | 说明 |
| --- | --- | --- | --- |
| VoxelMap | `include/map_manager/native/voxelmap` | `voxelmap` | 概率自适应体素平面，是 PV-LIO 技术路线中的首个核心局部地图。 |
| VoxelMap++ | `include/map_manager/native/voxelmap_plus` | `voxelmap_plus` | 基于 VoxelMap 的地图和残差优化。 |
| FAST-LIO2 / ikd-tree | `include/map_manager/native/ikdtree` | `ikdtree` | FAST-LIO2 的增量 KD-tree 地图。 |
| Faster-LIO / iVox | `include/map_manager/native/ivox` | `ivox` | Faster-LIO 的稀疏增量体素地图。 |
| C3P-VoxelMap | `include/map_manager/native/c3p_voxelmap` | `c3p_voxelmap` | 紧凑概率体素地图，使用原生平面匹配和地图合并逻辑。 |

因此，FAST-LIO2 和 Faster-LIO 已经分别通过 ikd-tree 和 iVox 体现在已合入的局部地图后端中。本仓库不包含它们的完整节点；如需端到端基线对比，请另行获取并编译上游工程。

### 候选局部地图后端

以下公开项目是未来 `MapManager` 后端的候选项，不随 PV-LIO-PLUS 发布，也不是本包的构建依赖。

| 算法 | 上游项目 | 当前状态 / 特点 |
| --- | --- | --- |
| Hybrid-VoxelMap | [Hybrid-VoxelMap](https://github.com/haiyang2022/Hybrid-VoxelMap) | 候选；混合体素/平面表示。 |
| R-VoxelMap | [R-VoxelMap](https://github.com/NKU-MobFly-Robotics/R-VoxelMap) | 候选；鲁棒递归体素平面估计。 |
| Super-LIO / OctVox | [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1) | 候选；OctVox 是 Super-LIO 内部的地图结构。 |
| BIEVR-LIO | [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO) | 候选；体素化地图、地图引导采样和几何统计。 |
| Surfel-LIO / hVox | [Surfel-LIO](https://github.com/93won/lidar_inertial_odometry) | 候选；层次化体素哈希和预计算 surfel。 |
| LIO-GVM | [LIO-GVM](https://github.com/Ji1Xingyu/lio_gvm) | 候选；高斯体素地图和基于高斯的点云匹配。 |
| [FR-LIO](https://arxiv.org/abs/2302.04031) | — | 论文已有；当前未发现公开源码，等待开源或自行实现。 |
| [CT-VoxelMap](https://arxiv.org/abs/2604.03747) | — | 论文已有；当前未发现公开源码，等待开源或自行实现。 |
| [GenZ-LIO](https://arxiv.org/abs/2603.16273) | [GenZ-ICP](https://github.com/cocel-postech/genz-icp)（相关项目） | 论文已有；相关仓库目前未释放 GenZ-LIO 源码，等待开源或自行实现。 |
| [SA-LIVO](https://arxiv.org/abs/2606.25699) | — | 论文已有；作者声明将开源，但当前尚未释放源码，等待开源或自行实现。 |
| [SV-LIO](https://doi.org/10.3390/electronics15081744) | — | 论文已有；当前未发现公开源码，等待开源或自行实现。 |
| [Environment-Adaptive Solid-State LiDAR-Inertial Odometry](https://arxiv.org/abs/2604.15864) | — | 论文已有；当前未发现公开源码，等待开源或自行实现。 |

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

- Ubuntu 20.04 和 ROS Noetic
- PCL 1.8 或更高版本、Eigen3、Boost（Timer）和 Python 开发头文件
- 与 PV-LIO-PLUS 位于同一 catkin 工作空间的 `livox_ros_driver`（提供
  `livox_ros_driver/CustomMsg.h`）

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/vison-yang/PV-LIO-PLUS.git
# 仅当工作空间中尚未有 livox_ros_driver 时执行。
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
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
source ~/catkin_ws/devel/setup.bash
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

## 许可证与第三方声明

PV-LIO-PLUS 以 GNU 通用公共许可证第二版或更高版本发布。完整许可证文本见
[LICENSE](LICENSE)，本包内局部地图实现和其他第三方代码的来源、许可证及当前状态见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。其中 VoxelMap++ 和
C3P-VoxelMap 的上游许可证声明仍需进一步确认，在确认前不应公开再分发这两份本地副本。

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
