# PV-LIO-PLUS

[English version](README.md)

PV-LIO-PLUS 是一个 ROS1 激光-惯性里程计（LIO）框架，使用相同的点面估计前端对比不同的局部地图模型。本项目主要继承自 [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git)，继承了 [VoxelMap](https://github.com/hku-mars/VoxelMap.git) 的概率体素地图思想，以及 FAST-LIO 的紧耦合迭代滤波结构。

本项目保留了原 PV-LIO、VoxelMap 和 VoxelMap++ 中已有的问题修复与优化，并新增 `MapManager`，允许通过同一份配置文件选择和对比六种局部地图。所有已合入的地图源码都放在本包内部；PV-LIO-PLUS 不链接其它完整 LIO 工程的源码。

## 主要工作

- 保留 PV-LIO 的紧耦合激光-惯性估计和 IKFoM 迭代滤波。
- 保留原 PV-LIO 观测流程和残差权重相关修复。
- 保留 VoxelMap 的概率自适应体素平面，以及 VoxelMap++ 的局部地图和残差优化。
- 增加 `MapManager`，提供统一的搜索、插入、删除、局部窗口和发布接口，同时保留各地图的原有逻辑。
- 合入 VoxelMap、VoxelMap++、FAST-LIO2 ikd-tree、Faster-LIO iVox、C3P-VoxelMap 和 Super-LIO OctVox，并支持配置切换。
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
| Super-LIO / OctVox | `include/map_manager/native/octvox` | `octvox` | 保留有序 TopK、4/5 点拟平面和固定观测信息量的 LRU 八分体点地图。 |

因此，FAST-LIO2 和 Faster-LIO 已经分别通过 ikd-tree 和 iVox 体现在已合入的局部地图后端中。本仓库不包含它们的完整节点；如需端到端基线对比，请另行获取并编译上游工程。

### 候选局部地图后端

候选项必须能拆出独立的几何地图或索引，只把该部分接入 `MapManager`；PV-LIO 的 IMU 传播、去畸变、状态量、迭代时序、扫描采样及点面观测约定保持一致。有完整作者源码并不表示整个方法符合地图替换实验。

以下结果于 **2026-11-14** 对照本地源码版本、作者论文和仓库核验。具体代码符号、移除理由和抽取边界见[范围审查](paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)。下表均为**研究候选，尚未新增可选后端**。

| 地图组件 | 作者源码 | 仅移植地图的边界与待办 |
| --- | --- | --- |
| R-VoxelMap 递归平面 | [R-VoxelMap](https://github.com/NKU-MobFly-Robotics/R-VoxelMap) | 只提取鲁棒体素平面构建与查询；固定估计器、扫描采样和观测权重。 |
| Surfel-LIO / hVox | [Surfel-LIO](https://github.com/93won/lidar_inertial_odometry) | 固定本地 `73348c1`，抽取预计算 L1 surfel 及 `GetSurfelAtPoint` 查询；作者较新 HEAD 已改用 KD-tree/PCA 路径，不能静默更新替代此快照。 |
| FR-LIO / RC-Vox | [论文](https://arxiv.org/abs/2302.04031v3) | 仅作地图组件储备，官方源码仍未确认；排除自适应子帧和迭代平滑器。 |
| GenZ-LIO 体素地图及查询 | [genz-lio](https://github.com/cocel-postech/genz-lio) | 只提取纯平面地图与查询分支，并固定尺度；排除自适应扫描降采样及点点观测回退。 |
| **i-Octree** | [i-octree](https://github.com/zhujun3753/i-octree) | 新增：支持插入、kNN/半径搜索和盒删除的动态八叉树，复用 PV-LIO 点面适配层。 |
| **KISS-ICP / VoxelHashMap** | [kiss-icp](https://github.com/PRBonn/kiss-icp) | 新增：来自纯激光里程计的地图组件，使用体素哈希、体素点数上限和距离淘汰；原生最近一点查询需扩展为 kNN，排除点点 ICP、运动补偿和阈值流程。 |
| **iG-LIO 增量体素地图** | [ig_lio](https://github.com/zijiechenrobotics/ig_lio) | 新增：只提取 `VoxelMap::AddCloud` / `KNNByCondition` 及点存储；排除 GICP、扫描协方差估计与上游优化器。 |

Hybrid-VoxelMap、BIEVR-LIO、LIO-GVM、CT-VoxelMap、SA-LIVO、SV-LIO 和 Environment-Adaptive Solid-State LIO 已从候选列表移除：其论文区别于现有方法的主要机制依赖语义输入、新观测或估计器/预处理修改。论文、笔记仍作为相关研究保留在[论文清单](paper/README.md)；若以后能证明其中有独立且不同于现有后端的几何地图贡献，再另行评估。

### 对照实验约束

现有六个后端共享估计循环，但仍保留各自的平面门限、噪声/信息权重及淘汰策略。例如 OctVox 使用固定观测信息量，因此目前**还不能声称是严格只改变地图表示的消融实验**。每次结果都需要记录这些设置。严格地图对照还应固定残差雅可比、观测方差、对应门限、保留扫描点和地图预算；对其中任何一项的改变都应单独作为消融。统一观测约定的要求见范围审查。

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
  |-- C3P-VoxelMap
  `-- OctVox
```

VoxelMap 和 VoxelMap++ 保留原生平面地图匹配；ikd-tree 和 iVox 保留点地图搜索，并使用管理器中的平面拟合适配层；C3P-VoxelMap 使用原生平面地图实现；OctVox 保留其原生有序 TopK、平面门限和固定信息观测，不复用 iVox 的协方差加权路径。状态估计循环保持共用，但原生权重和门限仍不同，实验需遵守上文的对照约束。

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
  map_type: octvox    # voxelmap, voxelmap_plus, ikdtree, ivox, c3p_voxelmap, octvox
```

局部窗口模式使用现有的 `mapping` 窗口参数配置。各后端的专用参数仍放在同一配置文件中，选择其它后端时会被忽略。`octvox` 保留 Super-LIO 的容量/LRU 淘汰语义，因此会拒绝几何局部窗口模式。

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
| `octvox` | `pv_lio_octvox_pos.txt` | `pv_lio_octvox.pcd` |

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
- [Super-LIO / OctVox](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1)
- [Hybrid-VoxelMap](https://github.com/haiyang2022/Hybrid-VoxelMap)
- [R-VoxelMap](https://github.com/NKU-MobFly-Robotics/R-VoxelMap)
- [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO)
- [Surfel-LIO](https://github.com/93won/lidar_inertial_odometry)
- [LIO-GVM](https://github.com/Ji1Xingyu/lio_gvm)
- [GenZ-LIO](https://github.com/cocel-postech/genz-lio)

## 工作空间源码与编译记录

本工作空间由多个独立 Git 仓库组成，固定来源见[源码清单](doc/workspace/SOURCE_MANIFEST.json)。可共同编译的 ROS1 包位于工作空间 `src/`；独立 CMake、ROS2/混合构建、包名冲突或源码不完整的项目放在 `upstream/`，逐个独立编译。已从地图候选列表移除的相关研究仍保留源码和编译结果。依赖前缀放在工作空间内。
