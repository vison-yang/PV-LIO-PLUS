# PV-LIO-PLUS

[English](README.md) | 中文

PV-LIO-PLUS 是一个 ROS1 激光雷达-惯性里程计（LIO）框架，用于在统一的
实验与点面匹配框架下评估不同的局部地图模型。

本项目主要继承自 [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git)，保留
其紧耦合 LiDAR-IMU 状态估计结构和基于 IKFoM 的迭代滤波器。PV-LIO 本身建立
在 [VoxelMap](https://github.com/hku-mars/VoxelMap.git) 的概率体素地图思想
之上，并受到 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO.git) 的启发。

VoxelMap 是本项目继承关系中的首个、也是核心局部地图算法。它贡献了高效的
概率自适应体素地图：使用平面表示局部表面，并在扫描到来时更新平面参数和
不确定性。在此基础上，PV-LIO-PLUS 将另外四种局部地图后端汇总到同一套实验
框架中：VoxelMap++、ikd-tree、iVox 和 C3P-VoxelMap。

研究者可以通过配置文件选择后端，在不修改 LIO 前端的情况下，对比相同实验
条件下的地图搜索、插入、删除/局部窗口维护以及点面匹配行为。所有内置局部
地图源码均位于 PV-LIO-PLUS 内部，节点不链接其他 LIO 包的源码树。

## 项目继承关系与地图定位

| 角色 | 组件 | 本项目保留的贡献或特性 |
| --- | --- | --- |
| LIO 基础 | [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git) | 紧耦合 LiDAR-IMU 里程计、基于 IKFoM 的迭代滤波以及原始 PV 观测流程。 |
| 首个/核心局部地图 | [VoxelMap](https://github.com/hku-mars/VoxelMap.git) | 以平面及平面不确定性为表示的概率自适应粗到细体素地图。 |
| 额外后端 | [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git) | 基于 VoxelMap 的局部地图管理器和残差计算优化，使用原生体素平面匹配。 |
| 额外后端 | [FAST-LIO2 ikd-tree](https://github.com/hku-mars/FAST_LIO.git) | 动态增量 KD-tree 点地图、高效近邻搜索以及原生点删除。 |
| 额外后端 | [Faster-LIO iVox](https://github.com/gaoxiang12/faster-lio) | 面向高吞吐近邻搜索的稀疏增量式体素点地图。 |
| 额外后端 | [C3P-VoxelMap](https://github.com/deptrum/c3p-voxelmap) | 紧凑、累积、可合并的概率体素地图，并支持按需平面合并。 |
| 集成层 | `MapManager` | 提供统一的初始化、搜索、插入、快照、删除/窗口维护和发布接口，同时保留后端差异。 |

## 主要工作

项目保留原 README 中关于 PV-LIO、VoxelMap 和 VoxelMap++ 的修复与优化记录，
并明确各项工作的边界。以下内容属于工程集成和数值健壮性处理，不能据此
宣称提出了新的协方差模型，或普遍提升了稳定性、效率和精度：

- **PV-LIO 残差权重：** 保留原项目记录的误差传播路径调整，用于形成残差
  权重。
- **VoxelMap 集成：** 保留原生概率平面地图逻辑，并在统一管理器中显式传递
  点及其协方差，明确 LiDAR 坐标系到世界坐标系的协方差约定。
- **VoxelMap++ 集成：** 保留其上游局部地图管理器和残差计算优化，支持配置
  选择；将正确坐标系下的点协方差传入残差计算，并对已观察到的无效或接近
  零的残差方差情况增加边界防护。
- **统一的局部地图评测：** 增加 `MapManager`，并接入上述四种额外后端。原生
  平面后端保留各自的匹配逻辑，点地图后端搜索近邻点并拟合平面，以适配公共
  点面匹配契约。
- **可复现实验输出：** 明确轨迹和世界坐标系扫描点云的输出语义，并在下文
  给出随地图后端变化的文件名。
- **编译与组织：** 将包提升到 C++17，并把局部地图实现放在 PV-LIO-PLUS
  内部，后端对比无需链接外部 LIO 源码树。

## 1. 系统架构

PV-LIO-PLUS 将 LIO 状态估计流程与局部地图存储、搜索解耦。观测模型根据
`mapping/map_type` 选择相应的原生匹配实现，`MapManager` 则提供统一的
地图生命周期和匹配接口。

```text
激光雷达 / IMU
      │
      ▼
PV-LIO 状态估计与点面残差计算
      │
      ▼
MapManager
      ├── VoxelMap       ─ 概率自适应体素平面地图
      ├── VoxelMap++     ─ 增强的体素平面与残差逻辑
      ├── ikd-tree       ─ 增量式点地图与平面拟合
      ├── iVox           ─ 体素化点地图与平面拟合
      └── C3P-VoxelMap   ─ 紧凑累积概率体素地图
```

管理器在各后端 API 允许的范围内最大程度保持其原有算法逻辑。不同后端的
差异被隔离在管理器适配层中：

- VoxelMap 和 VoxelMap++ 使用各自原生的体素平面搜索与更新逻辑，VoxelMap++
  还保留其原生残差计算方式。
- ikd-tree 使用增量式 KD-tree 点地图及其原生删除路径。
- iVox 使用稀疏体素点地图搜索近邻点，再由适配层拟合局部平面，生成公共的
  点面匹配结果。
- C3P-VoxelMap 使用紧凑概率平面表示，并支持按需平面合并。
- 启用局部窗口后，ikd-tree 使用原生点删除接口；iVox 和 C3P-VoxelMap
  根据管理器保留的点重新构建地图，因为其原生 API 没有提供安全的删除和
  遍历接口。

### 源码布局

```text
PV_LIO_PLUS/
├── include/map_manager/map_manager.h
├── src/map_manager/map_manager.cpp
└── include/map_manager/native/
    ├── voxelmap/       # PV-LIO VoxelMap 实现
    ├── voxelmap_plus/  # VoxelMap++ 实现
    ├── ikdtree/        # FAST-LIO2 ikd-tree 实现
    ├── ivox/           # Faster-LIO iVox 实现
    └── c3p_voxelmap/   # C3P-VoxelMap 实现
```

## 2. 依赖环境

当前工作空间在 Ubuntu 20.04、ROS Noetic、C++17、PCL、Eigen 和 Livox ROS
消息环境下进行测试。只要编译器和依赖支持 C++17，其他 ROS1 发行版也可能
能够运行。

- ROS1 Melodic 或更高版本，推荐 ROS Noetic。
- PCL >= 1.8。
- Eigen >= 3.3.4。
- 已安装并 source 的 `livox_ros_driver`，需要提供
  `livox_ros_driver/CustomMsg.h`。
- 支持 C++17 的编译器。

PV-LIO-PLUS 及其内置局部地图后端不依赖 Ceres。局部地图代码作为本包的一
部分编译，不链接其他 LIO 包的源码树。

## 3. 编译

在 catkin 工作空间根目录执行：

```bash
cd ~/src/lio_ws
source /opt/ros/noetic/setup.bash
# 如果 livox_ros_driver 位于独立工作空间，请先 source 它。
source <livox_driver_ws>/devel/setup.bash
catkin_make
source devel/setup.bash
```

推荐使用完整工作空间的 `catkin_make` 作为编译检查。`catkin_make --pkg
pv_lio_plus` 适合快速迭代，但不能替代完整工作空间编译。

## 4. 选择局部地图后端

在所用传感器对应的 YAML 中设置 `mapping/map_type`，例如
[`config/mid360_indoor.yaml`](config/mid360_indoor.yaml) 或
[`config/avia.yaml`](config/avia.yaml)：

| `map_type` | 局部地图模型 | 匹配方式 |
| --- | --- | --- |
| `voxelmap` | PV-LIO 原始 VoxelMap | 原生自适应体素平面匹配 |
| `voxelmap_plus` | VoxelMap++ | 原生体素平面与残差逻辑 |
| `ikdtree` | FAST-LIO2 ikd-tree | 近邻点搜索后进行平面拟合 |
| `ivox` | Faster-LIO iVox | 体素近邻点搜索后进行平面拟合 |
| `c3p_voxelmap` | C3P-VoxelMap | 紧凑概率平面匹配，可选平面合并 |

配置示例：

```yaml
mapping:
    map_type: ikdtree
    local_window_en: false
    det_range: 100.0
```

`mapping/map_type` 是唯一的局部地图后端选择入口。如果没有设置
`map_type`，运行时默认使用原始 VoxelMap。

五种后端的全部参数都集中在对应传感器 YAML 的 `mapping` 段中。launch 文件
只负责加载 YAML 和启动节点，不再覆盖地图类型或其他算法参数。

通用点地图参数包括 `nearest_point_count`、`nearest_max_range`、
`plane_fit_threshold` 和 `point_map_downsample_size`。后端专用参数包括
`ikd_*`、`ivox_*` 和 `c3p_*`；省略时使用与原生实现兼容的默认值。

### 局部窗口模式

将 `mapping/local_window_en` 设置为 `true` 可以启用移动局部地图。窗口以
当前激光雷达位置为中心，`mapping/det_range` 表示窗口半边长。管理器根据
后端能力采用相应的删除或重建策略，具体见系统架构部分。

## 5. 运行

启动 ROS，并运行与传感器对应的配置：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
roslaunch pv_lio_plus mapping_avia.launch
```

在另一个终端播放带模拟时间的 ROS1 数据包：

```bash
source /opt/ros/noetic/setup.bash
source ~/src/lio_ws/devel/setup.bash
rosbag play --clock /home/yxy/data/avia/outdoor_Mainbuilding_100Hz_2020-12-24-16-46-29.bag
```

启动文件会加载 YAML 配置，并订阅 `common/lid_topic` 和
`common/imu_topic` 指定的话题。请确保激光雷达与 IMU 时间同步。

如果出现 `Failed to find match for field 'time'.`，说明 ROS bag 中缺少逐点
激光时间戳，这会影响运动补偿以及前向/后向传播结果。

## 6. ROS 接口与保存结果

| 接口 | 含义 |
| --- | --- |
| `/Odometry` | 估计的 LiDAR/IMU 里程计 |
| `/path` | 用于可视化的 `nav_msgs/Path` 轨迹 |
| `/Laser_map` | 当前选择的局部地图快照 |
| `/planes` | 体素平面后端的平面标记 |
| `/cloud_registered` | 世界坐标系下的配准点云 |
| `/cloud_registered_body` | 机体坐标系下的配准点云 |
| `/cloud_registered_lidar` | LiDAR 坐标系下的配准点云 |

节点退出时会将结果写入工作空间的 `output/` 目录：

| 后端 | 轨迹文件 | 启用保存时的后端命名 PCD |
| --- | --- | --- |
| `voxelmap` | `pv_lio_pos.txt` | `pv_lio.pcd` |
| `voxelmap_plus` | `pv_lio_plus_pos.txt` | `pv_lio_plus.pcd` |
| `ikdtree` | `pv_lio_ikdtree_pos.txt` | `pv_lio_ikdtree.pcd` |
| `ivox` | `pv_lio_ivox_pos.txt` | `pv_lio_ivox.pcd` |
| `c3p_voxelmap` | `pv_lio_c3p_voxelmap_pos.txt` | `pv_lio_c3p_voxelmap.pcd` |

轨迹文件每行包含：

```text
timestamp  tx  ty  tz  qw  qx  qy  qz
```

保存的 PCD 是累积的、去畸变后的世界坐标系扫描点云，与 `/Laser_map` 的当前
局部地图快照不同，后者不是统一格式的保存地图文件。当
`pcd_save/interval: -1` 时，退出时保存一个完整的最终 PCD；设置为正数时，
已完成的分块会保存到 `output/PCD/`，退出时写入的后端命名 PCD（如果存在）
只包含尚未刷新的尾段，也可能不存在。

结果文件名前缀由 `mapping/map_type` 决定，因此不同地图类型会故意使用不同的
最终文件名，不能假定每次都是 `pv_lio_pos.txt` 和 `pv_lio.pcd`。中间分块统一
使用 `output/PCD/scans_N.pcd`，对比不同后端时应在每次运行后移动或重命名该
目录。

## 7. 多局部地图对比建议

为了获得有意义的对比结果，建议遵循以下流程：

1. 所有后端使用相同的数据包、话题、初始状态、噪声参数和预处理参数。
2. 每次只修改 `mapping/map_type`，不要修改 LIO 残差和状态估计代码。
3. 使用 `Ctrl-C` 正常结束每次运行，确保轨迹和 PCD 刷新到磁盘。
4. 将不同运行产生的文件移动到独立目录，例如：

   ```bash
   mkdir -p output/map_comparison/ikdtree
   mv output/pv_lio_ikdtree_pos.txt output/map_comparison/ikdtree/
   mv output/pv_lio_ikdtree.pcd output/map_comparison/ikdtree/
   ```

5. 在比较精度或场景相关失败案例之前，先检查轨迹长度、运行时间、地图
   大小以及输出中的数值是否有限。

该流程无需修改源代码，即可为 `voxelmap`、`voxelmap_plus`、`ikdtree`、
`ivox` 和 `c3p_voxelmap` 保留相互独立的实验结果。

## 8. 相关工作与上游实现

- [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git)
- [VoxelMap](https://github.com/hku-mars/VoxelMap.git)
- [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git)
- [FAST-LIO2 / ikd-tree](https://github.com/hku-mars/FAST_LIO.git)
- [Faster-LIO / iVox](https://github.com/gaoxiang12/faster-lio)
- [C3P-VoxelMap](https://github.com/deptrum/c3p-voxelmap)
- [IKFoM](https://github.com/hku-mars/IKFoM)

上游实现均已本地集成到 `include/map_manager/native/`，从而将局部地图选择
统一纳入 PV-LIO-PLUS。重新发布项目时，请保留对应上游项目的许可证和版权
声明，详见 [`LICENSE`](LICENSE)。

## 9. 致谢

感谢 PV-LIO、VoxelMap、VoxelMap++、FAST-LIO2、Faster-LIO、C3P-VoxelMap
和 IKFoM 的作者及贡献者。
