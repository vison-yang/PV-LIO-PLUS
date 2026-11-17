# PV-LIO-PLUS

[English version](README.md)

PV-LIO-PLUS 是一个 ROS1 激光-惯性里程计（LIO）框架，集成多种地图结构及其对应的残差模型，旨在相同框架下评估不同地图的效果。各后端共享激光/IMU 处理、状态传播、迭代滤波主循环和结果输出，同时保留各自的地图表示、匹配规则与观测模型。

本项目最早源自 [PV-LIO](https://github.com/HViktorTsoi/PV-LIO)。PV-LIO 在 **FAST-LIO2 的源码框架**中，以概率体素地图 [VoxelMap](https://github.com/hku-mars/VoxelMap) 替换 ikd-tree。PV-LIO-PLUS 在此基础上先集成 [VoxelMapPlus（VoxelMap++）](https://github.com/uestc-icsp/VoxelMapPlus_Public)，随后大幅重构并优化代码框架，再进一步集成更多地图结构及其对应的残差模型。

当前统一配置包含十一个地图后端。i-Octree feature `5c27a79` 已以 `6876a8e` 合入；隔离构建、数值复核、测试和30/141秒回放通过，主线节点/链接、CTest 7/7（49 个普通 GTest 条目，主线 ASan 关闭）及30秒完整输入回放也已通过。iG-LIO 的 `ig_lio_plane` 源码已接入；round 2 节点构建与 `ldd -r` 通过，指定九个构建目标通过，CTest 9/9（65 个 GTest 条目，含 11 个 iG 用例、ASan 关闭），但尚未执行回放。详见[iG-LIO 结果记录](doc/workspace/evidence/IG_LIO_RESULTS.json)。工作空间另行收集官方上游工程，用于源码追溯与独立的完整系统基线；这些工程的完整估计节点分别构建。

## 主要工作

- 共用 PV-LIO 的紧耦合激光-惯性估计和 IKFoM 迭代滤波。
- 集成 VoxelMap、VoxelMap++ 地图及其概率平面残差。
- 重构配置、地图管理、观测分发和结果输出，形成统一的 C++17 框架。
- 进一步集成 ikd-tree、iVox、C3P-VoxelMap、OctVox、R-VoxelMap、hVox、i-Octree、iG-LIO 增量体素地图与 BIEVR 定向高度图及其匹配、观测路径，共十一个可选后端。
- 提供轨迹、世界坐标系地图点云和逐帧耗时输出，支持可复现对比。

[集成计划](PLAN.md)说明源码收集状态、移植范围、实施顺序及验收条件；[地图与残差范围审查](paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)记录现有观测差异和后续适配边界。

## 算法清单

已集成后端由地图结构及其对应的匹配/残差路径组成，并接入本项目的状态估计循环。上游地图实现位于 `include/map_manager/native/`，观测分发位于 `src/mapping.cpp`。源码收集、上游编译通过和接入本项目是三个独立阶段。

当前算法清单要求：**提出新的地图数据结构或实质改进地图数据结构，是该方法的创新点之一**。仅使用已有地图，或只改变残差、采样、查询策略及估计流程，不满足收录条件。[资格审查](paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)记录逐项论文页码、结构证据和判定。实际移植须保留支撑资格的结构创新；删去这些特征后留下的通用容器，不能继续代表原地图。

本项目另设第二类观测与权重组件集成，独立于 `mapping/map_type` 控制，默认保留原后端行为。GM 固定尺度点面权重已接入，GenZ-ICP 混合观测仍待实现；均不导入上游求解器或额外前后处理。地图结构优先，组件实施状态及拟议选项见[观测与权重方案](doc/OBSERVATION_INTEGRATION_PLAN.md)。

### 已集成的局部地图后端

以下十一类地图均满足结构创新条件。FAST-LIO2 在此对应 ikd-tree 的结构贡献，不与 FAST-LIO1 混称。

| 算法 / 地图 | 本包内地图源码 | 配置项 | 地图与残差模型 |
| --- | --- | --- | --- |
| VoxelMap | `include/map_manager/native/voxelmap` | `voxelmap` | 自适应概率体素平面；结合平面与点不确定性的点面残差。 |
| VoxelMap++ | `include/map_manager/native/voxelmap_plus` | `voxelmap_plus` | VoxelMapPlus 平面与合并；原生平面参数化及不确定性传播。 |
| FAST-LIO2 / ikd-tree | `include/map_manager/native/ikdtree` | `ikdtree` | 增量 KD-tree 点地图；邻域拟平面与点不确定性适配层。 |
| Faster-LIO / iVox | `include/map_manager/native/ivox` | `ivox` | 稀疏增量体素点地图；与 ikd-tree 共用点面适配路径。 |
| C3P-VoxelMap | `include/map_manager/native/c3p_voxelmap` | `c3p_voxelmap` | 紧凑概率平面与合并；原生匹配及不确定性加权残差。 |
| Super-LIO / OctVox | `include/map_manager/native/octvox` | `octvox` | LRU 八分体点地图；有序 TopK、4/5 点拟平面和固定观测信息量。 |
| R-VoxelMap | `include/map_manager/native/r_voxelmap` | `r_voxelmap` | 递归内部/子层平面、离群点复用、原生 root LRU 及协方差概率平面观测。 |
| Surfel-LIO / hVox | `include/map_manager/native/hvox` | `hvox` | L0 质心 / L1 预计算 surfel 双层地图、原生移动盒维护及适配共用 IKFoM 的 PKO 加权有符号平面观测。 |
| i-Octree | `include/map_manager/native/i_octree` | `i_octree` | 动态八叉树及连续 RAII 叶块，使用数据 AABB 裁剪全树 kNN，并接入共用点面观测适配器。 |
| iG-LIO / 增量 VoxelMap | `include/map_manager/native/ig_lio` | `ig_lio_plane` | 增量体素充分统计与原生保留点维护；局部搜索结果送入共用五点 float QR 标量点面观测。本变体不包含作者 GICP 观测。 |
| BIEVR 高度图变体 | `include/map_manager/native/bievr` | `bievr` | 定向像素高度、融合/重投影和 LRU；共享滤波 bilinear 高度观测，非作者 LM/Ceres 系统。 |

FAST-LIO2 和 Faster-LIO 分别通过 ikd-tree、iVox 及本框架的观测适配层参与对比。完整官方工程已收集在工作空间 `src/FAST_LIO`、`src/faster_lio`，用于独立系统基线。

### 已收集源码与后续集成

原始源码快照于 **2026-11-15** 核对；路径相对于工作空间根目录，固定作者提交和兼容性补丁见[源码清单](doc/workspace/SOURCE_MANIFEST.json)。当前工作区已没有完整 `src/ig_lio` checkout；native 地图核心是基于作者提交 `2d79259e3aaa5bbb10d6b13234398760bbacf8f5` 源文件的安全重实现。2026-11-14 的共享构建记录仅描述较早的完整 checkout。新的 `ig_lio_plane` 选择器已接入，round 2 指定构建和合成测试通过；本轮尚未回放。完整 iG-LIO GICP 是单独的观测设计任务。BIEVR 已接入 `bievr`；以下两项地图候选仍无选择器。

| 地图 / 方法 | 已收集的官方源码 | 已记录的编译结果 | 后续集成工作 |
| --- | --- | --- | --- |
| Hybrid-VoxelMap | `src/Hybrid-VoxelMap` | 历史共享构建；当前快照待核 | 条件候选：定义语义标签输入及 Plane/NDT 匹配类型，再适配两类残差；纯平面或无标签变体须明确命名。 |
| LIO-GVM | `upstream/lio_gvm` | 作者发布源码不完整，构建阻塞 | 每体素维护质心/协方差的增量高斯地图；先补齐作者定制 Fast-VGICP、IKFoM 及地图源码，再适配地图与残差。 |

FR-LIO/RC-Vox、SV-LIO 的论文具备地图结构创新依据，但本地收集记录尚未确认官方实现；两者保留为论文储备，待源码核验后再进入实现排期。

### 对照实验约束

主要实验比较共同框架中的**地图结构及其对应残差模型**。统一传感器标定、输入片段、IMU 处理、状态定义、滤波时序及评测方法；记录各后端的残差类型、协方差/信息量、鲁棒核、匹配门限、扫描采样和地图预算/淘汰策略。语义输入或地图引导采样须有明确的实验档案与消融。

现有十条路径均采用平面观测，但参数化、不确定性模型和门限不同：OctVox 使用固定信息量，R-VoxelMap 保留递归概率平面匹配，hVox 使用分组内固定 L1 surfel 与适配后的 PKO 权重，i-Octree 使用全树 kNN 和共用点面适配器；`ig_lio_plane` 保留增量体素地图并使用共用五点 float QR 标量点面拟合，不是作者 GICP。VoxelMap++ 现已遵循 `extrinsic_est_en`：启用时，解析雅可比包含右扰动外参旋转和平移列；关闭时，这些列为零。`h = -float(dist)`、原生权重和面片门限未变。细节与测试边界见[集成状态](doc/INTEGRATION_STATUS.md)和[范围审查](paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)。

ikd-tree 和 iVox 的 `nearest_max_range` 现统一以米表示。ikd-tree 将第五近邻平方距离与 `r²` 比较并包含边界；iVox 将 `r` 传给原生查询，采用严格的 `< r`，且查询受配置的附近体素范围限制。因此数值表示相同物理单位，但边界行为和搜索域仍不同。旧 ikd-tree 曾把配置值 `v` 当平方距离，若要复现旧阈值，新半径应设为 `sqrt(v)` 米；现有 iVox 数值不变。

既有示例数据清单包含此前十个 selector；新增 `bievr` 可用 runner 的 `--algorithms bievr` 指定。原有六后端契约测试、修复计时后的长短回归、R-VoxelMap 与 hVox 主线验证均有记录。i-Octree feature 工作树构建和链接检查、独立数值核验通过；隔离 CTest 9/9（60 次执行，含 sanitizer 重复，不代表 60 个独立用例），30/141 秒完整输入回放通过。主线 `6876a8e` 节点/链接、CTest 7/7（49 个普通 GTest 条目，ASan 关闭）和30秒完整输入回放通过。feature 正向 ASan/UBSan 检查通过，LeakSanitizer 未启用。iG-LIO round 2 的指定构建、CTest 9/9（65 个 GTest 条目）及配置合成矩阵已通过；该轮没有启用 ASan，也没有回放、GUI/设备验收、ATE 或性能 A/B。历史测试数仍只描述各自旧阶段。详细边界见[集成状态](doc/INTEGRATION_STATUS.md)、[iG-LIO 结果](doc/workspace/evidence/IG_LIO_RESULTS.json)与[i-Octree 结果](doc/workspace/evidence/I_OCTREE_RESULTS.json)。

严格的索引/地图表示消融是可选实验，需额外对齐残差、雅可比、权重、门限、保留点集和地图预算。完整上游系统则作为独立的端到端基线。结果应按实际改变的因素解释。

## 结构

```text
激光雷达 / IMU
      |
      v
共用预处理、去畸变和 IMU 状态传播
      |
      v
MapManager：地图存储、更新与匹配
  |-- VoxelMap       --> 概率平面观测
  |-- VoxelMap++     --> 原生平面观测
  |-- ikd-tree       --> 共用点面适配器
  |-- iVox           --> 共用点面适配器
  |-- C3P-VoxelMap   --> 原生概率平面观测
  |-- OctVox         --> 固定信息量的平面观测
  |-- R-VoxelMap     --> 递归概率平面观测
  |-- hVox           --> PKO 加权的预计算 surfel 观测
  |-- i-Octree       --> 共用点面观测适配器
  `-- iG-LIO         --> 共用五点 float QR 标量点面观测
      |
      v
后端观测分发 --> 共用 IKFoM 更新
      |
      v
地图更新、轨迹 / 地图 / 耗时输出
```

`MapManager` 管理地图存储与查询，对应观测路径构造残差、雅可比和信息量。通用带类型 surfel、高斯与高度图残差接口仍是未来设计；hVox 当前通过专用适配接入共用 `PlaneMatch` 与滤波循环。

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

通过 launch 的 `backend` 参数选择后端，其配置文件声明对应地图类型：

```yaml
mapping:
  map_type: octvox    # voxelmap, voxelmap_plus, ikdtree, ivox, c3p_voxelmap, octvox, r_voxelmap, hvox, i_octree, ig_lio_plane, bievr
```

11 份后端 YAML 各自保存该后端的地图参数。三份主传感器 YAML 的 `mapping` 下仅保留 `down_sample_size`、`max_iteration`、`extrinsic_est_en`、`extrinsic_T`、`extrinsic_R` 五项。加载顺序为“传感器 → 所选后端 → 可选观测”，之后由 launch 根据 `backend` 强制设置 `mapping/map_type`；同名参数以后加载的后端值为准。没有 `common.yaml` 或 `sensors/` 层。Solid 基线值的变化见[配置说明](config/README.md)。局部窗口模式通过现有 `mapping` 窗口参数配置。

```bash
roslaunch pv_lio_plus mapping_avia.launch backend:=ig_lio_plane rviz:=false
roslaunch pv_lio_plus mapping_solid.launch backend:=ivox observation:=geman_mcclure rviz:=false
```

批量 runner 与 launch 使用相同的“传感器 → 后端 → 可选观测”顺序，记录模式为 `sensor_then_backend`。后端始终加载，旧参数 `--backend-presets` 现在是兼容用的 no-op。后端、传感器和可选观测配置均归档副本及 SHA-256，生成的 launch 只读取归档副本。

`octvox` 保留 Super-LIO 的容量/LRU 淘汰语义，因此会拒绝几何局部窗口模式。`hvox` 使用原生移动盒，不支持公共几何局部窗口模式。`i_octree` 通过树上裁剪保留闭合公共窗口内的点，不重建整棵树；`ig_lio_plane` 拒绝几何局部窗口并使用原生容量/LRU 行为。

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
| `r_voxelmap` | `pv_lio_r_voxelmap_pos.txt` | `pv_lio_r_voxelmap.pcd` |
| `hvox` | `pv_lio_hvox_pos.txt` | `pv_lio_hvox.pcd` |
| `i_octree` | `pv_lio_i_octree_pos.txt` | `pv_lio_i_octree.pcd` |
| `ig_lio_plane` | `pv_lio_ig_lio_plane_pos.txt` | `pv_lio_ig_lio_plane.pcd` |

轨迹文件保存带时间戳的位姿记录。根据点云保存配置，还可能在 `output/PCD/` 下生成分段点云。进行地图对比时，应对所有后端使用相同的数据包片段、传感器参数、窗口设置和输出目录组织方式。

### 逐帧耗时 CSV

设置 `logging/timing_en: true` 后，节点会输出每帧校正位姿，以及当前帧和累计平均的搜索、IKF、地图更新与总耗时，单位为毫秒，并保存 CSV。`logging/timing_path` 可指定文件；留空时写入 `output/<backend>_timing.csv`，例如 `pv_lio_ivox_timing.csv`。

每个完成 EKF 更新和地图更新的帧对应一行，记录 `frame`、传感器时间 `stamp_sec`、位姿字段 `pos_*_m` / `quat_*` 及各阶段的 `<stage>_ms` / `<stage>_avg_ms`。`search_ms` 汇总所有迭代观测调用中的地图搜索耗时，已包含在 `ikf_update_ms` 内，不能再次相加。`total_ms` 覆盖本帧处理直到 ROS 发布/地图快照，不含耗时日志自身写入。文件使用缓冲写入，在节点退出时关闭，不逐帧刷新。

## 许可证与第三方声明

包元数据声明 GNU 通用公共许可证第二版或更高版本。根目录许可证文本见
[LICENSE](LICENSE)，本包内局部地图实现和其他第三方代码的来源、许可证及当前状态见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。iG-LIO 导入来源附带 GNU GPL 第二版文本；本次未推断更高版本授权或许可证兼容性，包元数据对该组件的适用范围仍待审查。VoxelMap++ 和 C3P-VoxelMap 的上游许可证声明也需进一步确认，在确认前不应公开再分发这两份本地副本。

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
- [i-Octree](https://github.com/zhujun3753/i-octree)
- [iG-LIO](https://github.com/zijiechenrobotics/ig_lio)

## 工作空间源码与编译记录

本工作空间由多个独立 Git 仓库组成，固定来源见[源码清单](doc/workspace/SOURCE_MANIFEST.json)，结果见[编译报告](doc/workspace/BUILD_REPORT.md)。可共同编译的 ROS1 包位于工作空间 `src/`；独立 CMake、ROS2/混合构建、包名冲突或源码不完整的项目放在 `upstream/`，逐个独立编译。已收集源码同时服务于后端开发与独立完整系统基线。依赖前缀放在工作空间内。

现有多项目工作空间应使用本地 Eigen/Ceres 前缀与明确的头文件顺序：

```bash
bash src/PV_LIO_PLUS/scripts/prepare_workspace_deps.sh
bash src/PV_LIO_PLUS/scripts/build_workspace.sh \
  --build-dir build/maintenance-20261114-unified \
  --devel-dir devel/maintenance-20261114-unified --jobs 2
source devel/maintenance-20261114-unified/setup.bash
python3 src/PV_LIO_PLUS/scripts/build_upstream.py --jobs 2
```

独立构建脚本逐项保存并继续处理失败项；LIO-GVM 仍需作者补齐源码。编译结果不等同地图精度结论。

本轮 BIEVR 与 GM 的源码、构建及合成验收见[结果记录](doc/workspace/evidence/TWO_TRACK_RESULTS.json)。启用示例：

```bash
roslaunch pv_lio_plus mapping.launch backend:=bievr
roslaunch pv_lio_plus mapping.launch backend:=i_octree observation:=geman_mcclure
```
