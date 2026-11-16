# 地图结构与框架接入边界

状态核对日期：**2026-11-16**，当前代码基线为 `3a21000`。地图结构依据固定版本论文和来源记录；当前十个 selector 的接入、构建、合成测试及各自回放边界见[集成状态](../../doc/INTEGRATION_STATUS.md)。`ig_lio_plane` 已接入并通过本轮构建及合成测试，尚无本轮回放。**BIEVR-LIO 尚未集成，仅有移植设计草稿**；上游独立工程构建通过不代表已接入本框架。历史验证记录分别保留其日期、源码和二进制范围。

## 项目定位与硬准入条件

本项目源自 PV-LIO：PV-LIO 在 FAST-LIO2 源码框架中用 VoxelMap 替换 ikd-tree，并利用点与平面的不确定性构造滤波观测，见 [PV-LIO 说明](../../../../src/PV-LIO/README.md#L2)。PV-LIO-PLUS 随后集成 VoxelMap++，大幅重构框架，再继续集成更多地图结构及其对应残差模型。主目标是在相同框架中评估不同地图与配套观测的效果。

**进入当前地图集成/候选清单的必要条件：原论文将提出新的地图数据结构或改进地图数据结构列为创新之一，并能指出存储单元、空间组织、层级、融合统计或地图维护结构的实际变化。** 使用体素/哈希容器、仅改查询算法、残差、语义权重、扫描采样、估计器缓存或更新门控均不自动满足。结构创新资格、源码可用性、编译和集成是独立状态；源码已收集或已构建不能补足论文资格。

实际移植必须保留支撑资格的结构创新。若抽取时去掉该创新，只剩通用点容器，就不能继续以原方法命名或宣称评估了该地图。尤其 iG-LIO 必须保留增量体素充分统计及相应维护；只借 `AddCloud` 的点集作为普通哈希索引不构成 iG 地图评估。点面适配可以作为明确标记的观测变体，不能抹去地图结构。同一原则适用于合并、递归、层级 surfel 和高度图等其它合格方法。

| 实验范围 | 可变化的内容 | 必须明确的边界 |
| --- | --- | --- |
| **共同框架中的地图与对应残差，主路线** | 合格地图的表示、增删/淘汰、查询与匹配，以及相应残差、Jacobian、不确定性、鲁棒权重和门限。 | 统一消息同步、IMU 传播、去畸变、状态定义、滤波主循环、输出和评测；记录各地图与观测配置，保留支撑资格的结构机制。 |
| **严格地图表示 / 索引消融，可选实验** | 实验事先声明的地图表示或索引因素。 | 另行统一残差、Jacobian、权重/噪声策略、门限、采样点集和地图预算；纯索引对照还需相同保留点集与拟平面适配器。 |

完整外部 LIO 系统作为独立端到端基线运行。不得在 `MapManager` 后隐藏第二套传播、状态、优化器、平滑器或连续时间估计器。额外语义输入、地图引导采样等须单独声明条件。**不同残差可以随满足资格的地图集成，残差不同本身不是排除理由。** 不得把散点几何协方差当作平面参数不确定性，也不得把不兼容的观测硬塞进 `PlaneMatch`。

## 地图结构依据与当前接入状态

页码采用对应固定 PDF 的 1 起始页码，版本与来源见 [catalog.json](../catalog.json)。本地资料包括 17 篇论文；FAST-LIO2 与 ikd-Tree 在表中合并说明。结构依据、源码可用性和框架接入状态分别记录。

| 方法 / 固定论文 | 当前状态 | 贡献陈述与实际结构证据 |
| --- | --- | --- |
| [VoxelMap](../VoxelMap.pdf) | **已接入** | p1 贡献 1–2，pp2–3 §III-A/B：自适应粗到细 hash-octree、概率平面及参数不确定性；改变单元尺度、层级构造和存储统计。 |
| [VoxelMap++](../VoxelMap++.pdf) | **已接入** | p1 贡献 1–2，pp3–4 地图构建/合并：3-DoF 平面参数与协方差，union-find 将共面体素归并为父平面，改变单元表示与跨格组织。 |
| [FAST-LIO2](../FAST-LIO2.pdf) / [ikd-Tree](../ikd-Tree.pdf) | **已接入 ikd-tree 后端** | FAST-LIO2 p2 贡献 1；ikd-Tree p1 摘要/引言及 §III：增量 KD-tree、惰性删除、盒操作、树内降采样和局部重平衡。创新归于 ikd-Tree，不重复计两个地图。 |
| [Faster-LIO / iVox](../Faster-LIO.pdf) | **已接入** | p2 贡献 1–2：稀疏增量体素及 linear/iVox-PHC 两种底层结构，改变点组织与格内布局；不是仅提出并行查询。 |
| [C3P-VoxelMap](../C3P-VoxelMap.pdf) | **已接入** | p2 贡献 1–3，pp3–5 §IV/V：点无关的固定规模充分统计、累积更新、平面合并与 locality-sensitive hash 合并桶。 |
| [Super-LIO / OctVox](../Super-LIO.pdf) | **已接入** | p1 摘要、p3 §III-C：每格八个子体素代表点、有限密度聚合/增量降噪及 LRU 组织；资格来自 OctVox，不能只保留 HKNN 查询。 |
| [R-VoxelMap](../R-VoxelMap.pdf) | **已接入** | p2 贡献 1，p3 §III-A/B：内部与叶节点均可保存平面/点，链表划分点集、离群点递归下推复用，改变传统仅叶平面组织与构建。 |
| [Surfel-LIO / hVox](../Surfel-LIO.pdf) | **已接入** | p2 贡献 1–2，p4 §III-B：两层 L0 质心 / L1 预计算 surfel 及 Z-order 键组织；必须保留更新期计算与层级，而不只是加速查找。 |
| [i-Octree](../i-Octree.pdf) | **已接入** | p1 摘要，p2 §II-A：叶端连续内存点块、仅叶保存点、局部树上更新/降采样和盒删除；明确提出动态八叉树数据结构。 |
| [iG-LIO](../iG-LIO.pdf) | **已接入 `ig_lio_plane`** | p2 第三项贡献，pp3–4 §II-C、式 (7)–(9)/算法 1：每格 `(N,Σp,Σppᵀ,μ,Σ)`，活跃格增量分布更新及 LRU 地图维护。资格不靠 VSCE 扫描协方差或 GICP 残差。 |
| [Hybrid-VoxelMap](../Hybrid-VoxelMap.pdf) | **未集成** | p1 第一项贡献，p3 §III-B：语义决定每格 Plane/Gaussian 表示并动态转换；改变实际几何单元类型与维护，区别于只按语义调残差权重。 |
| [BIEVR-LIO](../BIEVR-LIO.pdf) | **未集成，仅有设计草稿** | p2 贡献 1，p3 §III-A：每体素存定向 bump image、高度/权重像素与局部坐标变换，迭代融合细节；资格来自新地图单元，不依赖采样创新。 |
| [LIO-GVM](../LIO-GVM.pdf) | **未集成，固定源码不完整** | p2 第二项贡献，p5 §IV-D/式 (18)：将 VGICP 式分布提升为可插入/更新/删除的全局哈希高斯地图，维护均值/协方差/计数并以 `M=max(M,N)` 防止历史统计冻结。不是仅更改 Gaussian 残差。 |
| [FR-LIO / RC-Vox](../FR-LIO.pdf) | **保留论文地图储备，源码未核验** | p2 第二项贡献，p4 §V-A/B：机器人中心、固定大小双层 3D 数组及滚动重用索引，明确提出 RC-Vox 结构；完整子帧与 smoother 不随地图移植。 |
| [SV-LIO](../SV-LIO.pdf) | **保留论文地图储备，源码/语义输入未核验** | p3 贡献 2，pp9–11 §4.3/式 (10)–(14)：语义引导聚类，离群点下推八个子体素，剩余内点重估平面并维护叶平面的类别概率。改的是递归构建与单元统计，不只是 §4.4/4.5 的语义关联/加权。 |
| [FAST-LIO](../FAST-LIO.pdf) | **框架来源，不单列地图候选** | p1 贡献为紧耦合滤波、去畸变与 Kalman gain 计算；不能把 FAST-LIO2/ikd-Tree 的结构创新回填给 FAST-LIO 原论文。 |

PV-LIO、FAST-LIO 属于框架来源。当前十个地图 selector 的实现与验证范围以[集成状态](../../doc/INTEGRATION_STATUS.md)为准；结构依据不代替代码接入或运行验收。

## 十个已接入后端的观测边界

[公共循环](../../src/mapping.cpp)共用滤波初始化、`imu_process_->Process` 与 `update_iterated_dyn_share_diagonal`；[ObservationModel 分派](../../src/mapping.cpp)保留不同观测实现。下表 `w` 为 `ekfom_data.R` 中的逆方差/信息量。

| 后端 | 地图与观测证据 | 当前适配行为与限制 |
| --- | --- | --- |
| VoxelMap | 自适应体素八叉树保存概率平面；[build_single_residual](../../include/map_manager/native/voxelmap/voxel_map_util.hpp#L848)使用平面范围、概率门限与 6 维平面协方差。[ObservationModelVoxelMap](../../src/mapping.cpp)：`w=1/(JΣplaneJᵀ+nᵀΣpoint n)`。 | 信息分母没有点图路径的 `1e-9` 下限。 |
| VoxelMap++ | UnionFind 平面地图及合并，按主方向使用 3 参数平面；[BuildSingleResidual](../../include/map_manager/native/voxelmap_plus/voxelmapplus_util.hpp#L719)与 [ObservationModelVoxelMapPlus](../../src/mapping.cpp)传播 `J_abd` 协方差，`h=-float(dist)`。 | 外参 Jacobian 现由 [`VoxelMapPlusPointPlaneJacobian`](../../include/map_manager/point_plane_jacobian.h)遵循 `extrinsic_est_en_`：启用时包含右扰动外参旋转/平移列，关闭时后 6 列为零。原生权重和面片门限未改；数值测试范围见[集成状态](../../doc/INTEGRATION_STATUS.md)。 |
| ikd-tree | 增量 KD-tree 查询点邻域；[fit_point_plane](../../src/map_manager/map_manager.cpp)拟平面，[ObservationModelPointBackend](../../src/mapping.cpp)构造观测。 | `plane_cov=0`，仅保留点协方差，总方差下限为 `1e-9`；是本项目观测适配器，不等同完整 FAST-LIO2 观测。半径 `r` 以米输入，比较第五近邻距离平方 `<=r²`。 |
| iVox | 稀疏增量体素与配置邻域；[`search_ivox`](../../src/map_manager/map_manager.cpp)复用上述拟平面及点协方差权重。 | `nearest_max_range=r` 以米传入，原生查询使用严格 `<r`，且只搜索选定的附近格域；不等同全图半径 kNN。 |
| C3P-VoxelMap | 紧凑概率体素及可选合并；[search_c3p](../../src/map_manager/map_manager.cpp)调用原生残差生成，[ObservationModelC3P](../../src/mapping.cpp)传播 6 维平面与点协方差。 | 使用独立 C3P 观测，没有点图路径的方差下限。 |
| OctVox | 八分体代表点聚合、容量/LRU、有序 TopK；[search_octvox](../../src/map_manager/map_manager.cpp)执行 4/5 点拟平面、0.1 m 邻点平面与距离门限；[ObservationModelOctVox](../../src/mapping.cpp)使用 `w=1000`。 | 固定信息量；LRU 不等于几何窗口，启用公共局部窗口会被配置检查拒绝。 |
| R-VoxelMap | 递归层级地图在内部/子节点存平面，链表组织离群点下推及重建并保留 root LRU；[`search_r_voxelmap`](../../src/map_manager/map_manager.cpp)查当前 root，原生条件满足时再查单个相邻 root。[`ObservationModelRVoxelMap`](../../src/mapping.cpp)使用平面协方差与旋转到世界系的 LiDAR 测量协方差。 | 原生匹配用完整世界点协方差与 3-sigma 门限；滤波观测不重复加入状态协方差。非全域 kNN。`size()` 是 root voxel 数；snapshot 为保留 native 点列表；批次后冻结的 root 点预算不是严格上限。几何窗口/显式 erase 不支持。 |
| hVox | 原生 L0/L1 层级与更新期预计算 surfel；[`search_hvox`](../../src/map_manager/map_manager.cpp)按包含点的 L1 键调用 `GetSurfelAtPoint`，`hvox_pko_ikfom` 通过[`HVoxObservationGroup`](../../src/map_manager/hvox_observation.cpp#L38)保留有符号残差和 PKO 分组尺度。 | 固定匹配组内重算当前状态残差、Jacobian 和 PKO/Huber 信息；不重建作者双循环。`size()` 是 L0 voxel 数，snapshot 为 L0 质心，point count 使用原生点数；原生 moving box，拒绝公共几何窗口。 |
| i-Octree | 动态八叉树、RAII 连续叶块与树上增删/降采样；数值适配采用实点 AABB 和 binary64 查询/距离。[`search_i_octree`](../../src/map_manager/map_manager.cpp)走全树 kNN，再拟合公共点面平面。 | 第五近邻按 `d² <= R²` 通过；native radius 查询为严格 `<r²`。`size()`/`point_count()` 为存活点数，snapshot 为 XYZ；闭合局部窗口调用树上 retain，不重建全树。 |
| iG-LIO 点面变体 | [增量统计与 LRU 地图](../../include/map_manager/native/ig_lio/voxel_map.cpp)、[适配器](../../src/map_manager/backends/ig_lio_backend.cpp)保留统计冻结、整批存点、实际邻域及严格半径；用公共五点 float QR 拟平面。 | `plane_cov=0`，点协方差信息沿公共点面路径；完整 GICP 未接入。`size()` 为格数，存点数与统计样本数不同；拒绝公共几何滑窗。[来源与差异](../../include/map_manager/native/ig_lio/SOURCE.md)。 |

[calc_lidar_cov_for_backend](../../src/mapping.cpp)还为 VoxelMap++ 选择单独的传感器协方差实现。这些差异属于当前地图与观测组合，应如实记录；现有结果不能直接称为严格地图表示消融。

**距离单位与迁移：** [`search_ikd_tree`](../../src/map_manager/map_manager.cpp)比较第 5 近邻平方距离与 `r²`，包含边界；iVox 的[原生查询](../../include/map_manager/native/ivox/ivox3d_node.hpp)采用严格 `<r`，并受 `ivox_nearby_type` 限制。`nearest_max_range` 现对两者统一以米配置。旧 ikd-tree 配置值 `v` 曾直接作为平方距离阈值，要复现旧门限应设置新半径 `r=sqrt(v)` 米；已有 iVox 数值不变。半径必须为正有限值，平方后也须有限且非零；MapManager 在清除既有地图前执行校验。ikd-tree 和 iVox 的点面残差门限都等价于严格条件 `range > 81 t²`（`t=plane_fit_threshold`），但这与查询半径无关，查询域也不相同。

**未匹配返回：** `MapManager::search` 每次清空 `matches` 与 `non_match`。ikd-tree、iVox、OctVox、R-VoxelMap、hVox、i-Octree 和 `ig_lio_plane` 会把其后端搜索失败的查询点写入 `non_match`；VoxelMap、VoxelMap++ 和 C3P 的原生 OMP residual builder 不保证逐点填充该列表。共同 API 只可依赖“无有效匹配时 `matches` 为空”，不可假定 `non_match` 与查询点逐项对应。

**原有六后端合成测试：** 两个 gtest 目标已实际构建，CTest 2/2 通过，共 11 个 GTest 条目。六后端各有空图/平面生命周期测试；半径测试覆盖 4.99/5/5.01 m、两种 iVox 实现、有限附近格域、非法值及 OctVox 窗口拒绝。VoxelMap++ Jacobian helper 对 12 列、三组非单位姿态做中心差分并检查外参开关。生产 `h=-float(dist)`、原生权重及门限仅作静态核对，未作自动集成测试；亚米级半径、负世界坐标和仅 1–4 个近邻的查询未专项覆盖。修复单调时钟后的六后端30秒和141秒回归均通过；长回归各有1402行、逐帧非零约束及有限非负计时。早期141秒运行的WallTime回拨仍作为历史失败记录，不作为计时证据。

**R-VoxelMap 独立验证：** 隔离 CTest 4/4、23 个 GTest 条目通过；正向 ASan fixture 连续30次通过。一次全量 ASan 调用曾挂起并中止，后续单项/重复/全量重试通过但原因未知；ptrace 不兼容导致泄漏检测关闭。主线节点编译退出0、`ldd -r`通过。30秒与141秒输入完整性回放均通过；长回放 1414 LiDAR/28595 IMU，1402行，每帧473至4850个有效约束（中位2418），root体素最多122755，耗时单调且有限非负，姿态单位范数误差小于 `5e-6`，PCD非空。未做真值精度或受控性能结论。详见[集成状态](../../doc/INTEGRATION_STATUS.md)及[R结果](../../doc/workspace/evidence/R_VOXELMAP_RESULTS.json)。

**hVox 独立验证：** feature `997c00a` 已以 `c81f12f` cherry-pick 到主线。功能工作树节点构建和 `ldd -r` 通过，CTest 6/6（34 个 GTest 条目）及独立数学/源码核验通过。30秒短回放 301 LiDAR/6088 IMU 输入对应 288 输出行；141秒回放 1414 LiDAR/28595 IMU 输入对应 1402 行，每帧都有非零约束，耗时列有限且非负，四元数单位范数误差小于 `7.0e-7`。细节见[状态记录](../../doc/INTEGRATION_STATUS.md)及[hVox结果](../../doc/workspace/evidence/HVOX_RESULTS.json)。这是输入完整性回归，不是精度或受控性能结论；主线构建、链接、CTest 5/5（33 个 GTest 条目）及30秒完整回归通过。

**i-Octree 独立验证：** feature `5c27a79` 已以 `6876a8e` cherry-pick 到主线。适配保留动态树、连续叶块和树上维护，但对查询增加实际点 AABB、binary64 距离及 RN/FTZ/DAZ 模式约束，因此不是未修改上游实现的性能复现。feature 节点构建/链接、独立数值审查和隔离 CTest 9/9 通过；记录为49个常规 GTest 用例和11次 sanitizer 重复（共60次执行，不是60个不同用例），正向 ASan/UBSan 检查通过，LeakSanitizer 未启用。30秒回放 301 LiDAR/6088 IMU→288 行；141秒 1414/28595→1402 行，每帧有效约束513–5620（中位2792），存活点数15,763–3,918,943，单位四元数误差小于 `7.07e-7`，8个计时列有限且非负，PCD有30,411,349点。主线节点/链接、CTest 7/7（49个常规 GTest，ASan关闭）及30秒完整输入回放通过。记录见[状态记录](../../doc/INTEGRATION_STATUS.md)和[i-Octree结果](../../doc/workspace/evidence/I_OCTREE_RESULTS.json)；无真值精度或受控性能结论。

**iG-LIO 点面变体当前验证：** `3a21000` 包含的实现已完成节点及八个 C++ 测试目标构建、节点动态链接检查、CTest 9/9（65 个 GTest 与 6 项配置测试）。本轮没有回放、GUI/设备验证或精度/性能结论；详见[iG-LIO 结果](../../doc/workspace/evidence/IG_LIO_RESULTS.json)。

## 历史源码核验记录

下表保留 2026-11-15 核验时的工作区记录，PV-LIO-PLUS 当时基线为 `985fd799fa61`。目录、HEAD 和原完整 checkout 的构建范围属于历史记录，当前状态见源码清单的 `current_snapshot`。

| 路径 | 本地 HEAD | 作者基线 / 核验范围 |
| --- | --- | --- |
| `src/R-VoxelMap` | `5166480ebec0` | 作者基线 `50ffbba`；地图头与该基线相同。 |
| `upstream/Surfel-LIO` | `2ed04104a3e1` | 作者基线 `73348c1`；`VoxelMap.cpp`、`Estimator.cpp` 与基线相同。 |
| `src/Hybrid-VoxelMap` | `917ff8020be4` | 作者基线 `89ed71c`；地图头相同，节点仅有消息类型/节点名兼容修改。 |
| `upstream/BIEVR-LIO` | `38f98b900dad` | 地图、高度图残差和采样/注册核心与作者基线相同。 |
| `src/ig_lio`（历史完整 checkout） | `a75b362f0852` | 作者基线 `2d79259`；当时核心源与基线相同。当前只导入本项目 native 地图核心，来源见[本地清单](../../include/map_manager/native/ig_lio/UPSTREAM_MANIFEST.json)。 |
| `upstream/lio_gvm` | `c80c968ed7ff` | README、`laserMapping.cpp` 与文件清单，定制地图头仍缺失。 |
| `upstream/i-Octree` | `25903a33dd7a` | 作者基线 `5b640e05`；固定作者快照及本地兼容性 patch 见 manifest；主线适配 feature `5c27a79`。 |

## 三项待接入地图的实施边界

当前已有十个地图 selector。Hybrid-VoxelMap、BIEVR-LIO、LIO-GVM 尚未接入；其中 BIEVR 只有[移植设计草稿](../../doc/BIEVR_IKFOM_PORT.md)，下表均为后续实施边界。iG-LIO 的增量地图点面变体已接入，完整 GICP 是另行设计的观测任务。

| 地图 | 必须保留的结构与对应观测 / 实施前置条件 |
| --- | --- |
| Hybrid-VoxelMap | [UnionFindNode](../../../../src/Hybrid-VoxelMap/include/voxelmapplus_util.hpp#L123)的 Plane/Gaussian 类型及语义驱动切换；[NDT 残差](../../../../src/Hybrid-VoxelMap/include/voxelmapplus_util.hpp#L707)和[消费端](../../../../src/Hybrid-VoxelMap/src/voxelMapping.cpp#L915)可抽取。须补标签来源、类别转换、缺标签和退化协方差语义；只留普通平面会丢失支撑资格的混合结构。 |
| BIEVR-LIO | [BIEVRMap](../../../../upstream/BIEVR-LIO/BIEVR/include/bievr_lio/bievr_map.h#L14)的定向高度图/像素融合及 [linearize](../../../../upstream/BIEVR-LIO/BIEVR/src/ls_optimizer.cpp#L55)的 `r=p_o.z-I(x,y)`、图像梯度和 Huber。设计拟适配共同状态，不引入 `computeTransformation/stepLm` 或惯性后端；[地图引导采样](../../../../upstream/BIEVR-LIO/BIEVR/src/pipeline.cpp#L239)单列实验。 |
| LIO-GVM | 保留全局高斯地图增量融合统计；[h_share_model](../../../../upstream/LIO-GVM/src/laserMapping.cpp#L671)调用 `kalmanCalculate`。但 [README](../../../../upstream/LIO-GVM/README.md#L9)仍注明 `.hpp` 待更新，[包含处](../../../../upstream/LIO-GVM/src/laserMapping.cpp#L62)依赖缺失的 `fast_vgicp/fast_vgicp.hpp`；补齐完整作者实现后才能核对地图/信息模型。 |
| FR-LIO / RC-Vox；SV-LIO | 论文证明结构资格，尚未核验作者实现；分别跟踪双层数组地图、语义引导递归地图的官方源码与可分离性，不进入当前源码移植排期。 |

**历史版本观察：** 11-14 审核记录 Surfel 远程 `2cbfad9` 的 [GetClosestSurfel](https://github.com/93won/lidar_inertial_odometry/blob/2cbfad923583b44d04d3ab078888bd9fe55450db/src/core/VoxelMap.cpp#L533)使用 KD-tree/PCA，以及 Hybrid 远程 `fb60275` 的[无语义 Gaussian-only 说明](https://github.com/haiyang2022/Hybrid-VoxelMap/blob/fb60275e78e010ef85f5c7574fa330774f714373/README.md)。它们不是本地基线。当前本地 Surfel 查询入口仍是 `UpdateWithLidar → FindCorrespondences → GetSurfelAtPoint`，`GetL1Surfels` 供 viewer 遍历；不得跨版本替换这些事实。

## 未来接口与验收

当前 `PlaneMatch` 和十个 selector 已有实现。hVox 通过专用适配将原生 surfel 对应关系转换为 `PlaneMatch`；通用带类型的 `Surfel`、`Gaussian`、`HeightImage` 匹配，以及通用标量/向量残差、状态 Jacobian 和信息接口，仍是**未来设计，本轮未实现**。现有滤波使用对角信息，向量观测需处理相关协方差，可评估白化为标量行，不能直接丢掉非对角项或照搬上游状态列顺序。

- **资格验收：** 导入文件、实际数据布局和维护调用链须保留论文支撑资格的结构创新；裁剪/改写须说明保留机制及影响，不能以原项目名包装通用索引。
- **地图 oracle：** 点图在声明的搜索域内对照暴力排序；概率平面、surfel 和高度图按各自的构造/查询规则建立可穷举参照。检查充分统计、层级/合并、像素融合、淘汰和快照，覆盖空图、少于 k 点、等距、负坐标与边界及插入删除。有限邻域 kNN 不得称为全图精确 kNN。
- **每后端观测 oracle：** 固定该后端地图与匹配，按其数学定义/作者消费者核对残差、Jacobian、信息和门限；以符合本项目流形扰动的有限差分验证。覆盖零距离、奇异协方差、缺像素、无匹配和外参开关。主路线不要求跨后端 `h/H/w` 相同，仅统一观测消融检查其等价。
- **噪声与数值：** 明确坐标、单位、平面参数不确定性与散点分布协方差的区别；检查方差/信息有限性、下界和白化二次型一致性。iG 鲁棒二阶项须单独证明等价条件或记录近似。
- **实验真实性：** 先合成、再短回放和长序列，记录数据版本、输入/语义条件、采样、门限、地图预算、观测、有效匹配、时间/内存及 ATE/RPE。源码收集、上游构建、共同框架集成和性能复现分别陈述，不把完整系统论文数字归因于抽取组件。

可机械执行的后续是同步 README/PLAN、论文笔记与源码清单中的资格和状态、补各合格地图的版本/生产者/消费者证据。带类型观测、结构保真移植、状态/噪声映射、Hybrid 双结构、BIEVR 高度图、iG GICP 及其关键验证属于后续设计与实现。源码不完整或边界未核验者继续保持阻断/论文储备状态。
