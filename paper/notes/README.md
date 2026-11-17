# PV-LIO-PLUS 论文笔记索引

本索引对应 17 篇本地 canonical 论文及其独立笔记。PDF 来源和固定版本见[论文资料清单](../README.md)。论文与本地 PDF 的资料核验日期为 **2026-11-14**；本项目的地图/观测集成范围更新日期为 **2026-11-17**。源代码本地路径和构建状态见[集成计划](../../PLAN.md)及[工作空间构建报告](../../doc/workspace/BUILD_REPORT.md)。

当前候选须把新的或实质改进的地图数据结构作为创新点之一；复用已有地图而只改残差、采样、查询算法或估计流程不满足结构准入。获准移植还须保留支撑资格的结构，不能只抽取通用点容器后仍以原方法名义评估。主路线在共享 PV-LIO-PLUS 框架内比较**合格地图结构及其对应观测**。同步、去畸变、IMU 传播、状态、滤波循环和输出由同一框架管理；后端可保留匹配、残差、Jacobian、不确定性和鲁棒权重。严格统一点面观测/权重是单独可选的地图表示或索引消融。完整上游系统是独立端到端基线。资格证据见[地图与观测范围审查](MAP_BACKEND_SCOPE_REVIEW.md)。

## 当前已接入的十一个地图与观测分支

| 方法 | 独立笔记 / 本地 PDF | 当前框架路径 |
| --- | --- | --- |
| VoxelMap | [笔记](VoxelMap.md) / [PDF](../VoxelMap.pdf) | 概率体素平面及其点面不确定性观测。 |
| VoxelMap++ | [笔记](VoxelMap++.md) / [PDF](../VoxelMap++.pdf) | 概率平面合并及原生观测协方差传播；外参 Jacobian 现按估计开关计算。 |
| FAST-LIO2 / ikd-Tree | [FAST-LIO2](FAST-LIO2.md)、[ikd-Tree](ikd-Tree.md) / [PDF](../FAST-LIO2.pdf)、[PDF](../ikd-Tree.pdf) | 增量点图与本地拟平面观测适配。 |
| Faster-LIO / iVox | [笔记](Faster-LIO.md) / [PDF](../Faster-LIO.pdf) | 稀疏增量点图与本地拟平面观测适配。 |
| C3P-VoxelMap | [笔记](C3P-VoxelMap.md) / [PDF](../C3P-VoxelMap.pdf) | 概率体素平面及其原生不确定性观测。 |
| Super-LIO / OctVox | [笔记](Super-LIO.md) / [PDF](../Super-LIO.pdf) | 八分体代表点图及 TopK 平面观测，使用固定信息量。 |
| R-VoxelMap | [笔记](R-VoxelMap.md) / [PDF](../R-VoxelMap.pdf) | 递归内部/子层平面地图及原生概率平面观测；集成与运行验证边界见[状态记录](../../doc/INTEGRATION_STATUS.md)。 |
| Surfel-LIO / hVox | [笔记](Surfel-LIO.md) / [PDF](../Surfel-LIO.pdf) | L0/L1 双层地图、更新期预计算 surfel 与适配共享 IKFoM 的 PKO 加权有符号平面观测。 |
| i-Octree | [笔记](i-Octree.md) / [PDF](../i-Octree.pdf) | 动态八叉树、连续 RAII 叶块、数值约束下的全树 kNN 与共享点面适配；实现差异和验证边界见[状态记录](../../doc/INTEGRATION_STATUS.md)。 |
| iG-LIO / incremental VoxelMap | [笔记](iG-LIO.md) / [PDF](../iG-LIO.pdf) | 保留增量充分统计与原生点维护的体素地图；`ig_lio_plane` 适配共用五点标量点面观测，不包含作者 GICP。 |
| BIEVR-LIO 高度图变体 | [笔记](BIEVR-LIO.md) / [PDF](../BIEVR-LIO.pdf) | `bievr`；当前作者 `2306022e` 的定向像素地图、公共 IKFoM bilinear 高度观测；源码已接入；11-16 [构建/合成](../../doc/workspace/evidence/TWO_TRACK_RESULTS.json)与 11-17 [短回归](../../doc/workspace/evidence/INTEGRATION_PROGRESS_20261117.json)分别记录。 |

十一条分支共用状态估计循环，但其观测模型并不相同；本表不表示严格地图消融或已完成数据集精度复现。

2026-11-15 的原有六后端契约回归包括两个合成 gtest 目标 CTest 2/2（11 个 GTest 条目），其中六个生命周期用例针对原有六后端。R-VoxelMap 另有隔离 CTest 4/4、23 个 GTest 用例，正向 ASan fixture 连续 30 次通过（ptrace 限制下未启用泄漏检测）；主节点构建并通过链接检查，主线常规 CTest 3/3（22 个 GTest 条目）通过。R 的 30 秒和 141 秒完整输入回放通过，其中 141 秒记录每帧都有有效约束、时长有限且非负。hVox 功能工作树构建、CTest 6/6（34 个 GTest 条目）、数学核验和 30/141 秒完整输入回放通过；hVox 已 cherry-pick 至主线 `c81f12f`，主线构建、链接、CTest 5/5（33 个 GTest 条目）及30秒完整回归通过。i-Octree feature 构建/链接、独立数值审查和 30/141 秒完整输入回放通过；隔离 CTest 9/9（49 个常规用例与 11 个 sanitizer 重复执行，共 60 次执行），主线 `6876a8e` 构建/链接、CTest 7/7（49 个常规 GTest 条目，ASan 关闭）及 30 秒回归通过。i-Octree 的 LeakSanitizer 未启用。上述回归均不构成精度或受控性能结论；细节和缺口见[集成状态](../../doc/INTEGRATION_STATUS.md)、[R 结果记录](../../doc/workspace/evidence/R_VOXELMAP_RESULTS.json)、[hVox 结果记录](../../doc/workspace/evidence/HVOX_RESULTS.json)和[i-Octree 结果记录](../../doc/workspace/evidence/I_OCTREE_RESULTS.json)。

2026-11-16 的 `ig_lio_plane` 源码已接入；round 2 node build/link、九个指定构建目标、CTest 9/9（65 个 GTest 条目，ASan 关闭）和配置合成测试通过，该 11-16 Round 2 当时未回放；11-17 同一份 SOLID 逐点时标数据（回归记录 01）上 iG-LIO 与 iG-LIO+GM 的 30 秒回归另已通过，见[运行记录](../../doc/workspace/evidence/INTEGRATION_PROGRESS_20261117.json)。此前 2026-11-14 的 ROS1 共享构建记录指向当时的完整作者 checkout，当前完整 `src/ig_lio` checkout 缺失，不能据此声称当前完整 iG-LIO 包仍存在或可构建。该 selector 的地图核心基于固定作者源文件本地重实现，并使用共享五点 float QR 点面观测。详情及失败轮次说明见[iG-LIO 结果记录](../../doc/workspace/evidence/IG_LIO_RESULTS.json)、[iG-LIO 笔记](iG-LIO.md)、[集成状态](../../doc/INTEGRATION_STATUS.md)和[源码清单](../../doc/workspace/SOURCE_MANIFEST.json)。

## 两项仍待集成的合格地图源码

这些工作仍需逐项完成地图/观测适配，不是当前 selector。表中注明的结构创新必须在移植后保留。源码固定版本及本地上游构建范围见[集成计划](../../PLAN.md)；构建通过只代表指定目标通过。iG-LIO 的点面地图 selector 已列于上节；完整 GICP 是独立后续观测设计，不代表还缺一个 iG 地图 selector。

| 方法 | 独立笔记 / 本地 PDF | 后续地图与观测设计 |
| --- | --- | --- |
| Hybrid-VoxelMap | [笔记](Hybrid-VoxelMap.md) / [PDF](../Hybrid-VoxelMap.pdf) | 保留语义地图层级并评估 Plane/NDT 观测；标签、缺失输入和两种观测分别记录。 |
| LIO-GVM | [笔记](LIO-GVM.md) / [PDF](../LIO-GVM.pdf) | 保留 Gaussian 地图结构与对应分布观测设计；作者源码缺失阻断，不以点面退化替代。 |

## 结构合格、但本地来源记录尚未确认作者实现的两项论文储备

以下论文中的结构贡献符合储备条件，但当前本地收集记录未确认作者官方源码；不据此断言远程不存在公开实现。来源状态按本地固定快照记录，未拉取远程更新。

| 方法 | 独立笔记 / 本地 PDF | 后续处理 |
| --- | --- | --- |
| FR-LIO / RC-Vox | [笔记](FR-LIO.md) / [PDF](../FR-LIO.pdf) | 论文储备：跟踪两层地图、机器人中心滑动和邻域索引；完整 ESKS/子帧属独立估计器范围。 |
| SV-LIO | [笔记](SV-LIO.md) / [PDF](../SV-LIO.pdf) | 论文储备：跟踪语义引导子八叉体和叶平面概率结构；源码、标签输入和对应观测尚待核验。 |

## 其它方法资料

| 论文 | 独立笔记 / 本地 PDF | 阅读边界 |
| --- | --- | --- |
| FAST-LIO | [笔记](FAST-LIO.md) / [PDF](../FAST-LIO.pdf) | 紧耦合滤波和点级去畸变的来源背景；完整系统单独作为端到端方法理解。 |

## 论文版本与结果说明

- 其余笔记按每条列出的 PDF 版本整理。论文数值是各自作者在其数据、传感器、参数和平台下的报告，不是本工作区统一复现实验。
- 不同地图的 `h/H/R` 无须在主路线中相同；若做统一点面/权重消融，须另外声明采样、搜索门限、点集与地图预算。地图协方差和散点分布协方差不能互换，有限邻域 kNN 也不能称为全图精确查询。

独立观测与权重组件见[第二类方案](../../doc/OBSERVATION_INTEGRATION_PLAN.md)：GM 首包已接入，GenZ 混合观测仍待实现；不单独增加或冒充地图结构。

本轮当前节点 i-Octree adapter 专项 30 秒回归通过（287 帧）；BIEVR 与 iG-LIO 默认权重各通过 197.190 秒 IMU 支持窗回归（1959 帧），缺 IMU 支持的末尾两帧裁出。见[最新验收](../../doc/workspace/evidence/MAP_ACCEPTANCE_20261117.json)。这些记录不替代噪声标定、真值精度或受控性能验收。
