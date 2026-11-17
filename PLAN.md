# PV-LIO-PLUS 地图与观测模型集成计划

> 集成范围更新日期：2026-11-17。论文版本与来源核验仍以各自记录的 2026-11-14 为准。
> 本文记录本地代码状态、上游构建证据和未来集成门槛；不会把构建成功写成数据集精度复现。
> 历史名称清理已改写提交 ID；旧构建/测试记录保留实际运行时的原始 ID，对应新历史见[提交映射](doc/workspace/evidence/HISTORY_COMMIT_MAP_20261117.json)。

## 1. 项目演进与目标

PV-LIO 最早在 FAST-LIO2 源码框架中以概率体素地图 VoxelMap 替换 ikd-tree，并使用点与平面的不确定性形成滤波观测；工作区保留其作者基线于 `src/PV-LIO`（`494c27f6`，完整 SHA 见源码清单）。PV-LIO-PLUS 随后集成 VoxelMapPlus（VoxelMap++），再大幅重构、优化代码框架，继续加入更多地图及其对应匹配与残差路径。

集成分为两类，**优先第一类地图数据结构，再推进第二类观测与权重组件**。消息同步、IMU 传播、点级去畸变、状态定义、滤波主循环、输出和评测保持在 PV-LIO-PLUS。

| 类别 | 准入范围 | 选择与比较方式 |
| --- | --- | --- |
| 第一类：地图结构 | 新的或有实质改进的地图数据结构，以及依赖该结构的对应观测。 | 继续由 `mapping/map_type` 选择；保留结构创新及其维护规则，比较地图与观测的组合效果。 |
| 第二类：观测与权重 | 可独立抽取的残差、Jacobian、信息/协方差、鲁棒权重或轻量匹配门限；主要创新集中在这些模型。 | 新增独立 `observation/*` 选项，默认使用原后端观测与权重；按兼容表显式开启，不随地图选择器隐式启用。 |

第二类不引入另一套状态/滤波/求解器，也不依赖大幅改写采样、去畸变、语义前处理、后处理或后端优化。必要的局部几何统计与观测关联可纳入，但必须定义输入与代价。只移植组件时按组件或适配变体命名，不声称复现完整上游系统。第二类的候选、开关和验收见[观测与权重集成方案](doc/OBSERVATION_INTEGRATION_PLAN.md)。

进入第一类地图候选/集成排期的硬条件是：论文或作者实现明确提出了新的、或有实质改进的地图数据结构，并且该创新是贡献之一。只复用既有地图、仅改变残差/采样/查询算法或估计流程，不满足这一结构资格条件。资格判断及论文证据见[地图与观测范围审查](paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)。候选移植必须保留支撑资格的结构创新；不能抽掉该结构，只留下通用点容器，再以原算法名义评估。例如 iG-LIO 的点面观测变体仍须保留增量充分统计和原生地图维护，GICP 可作为后续观测适配；Hybrid 的语义地图层级、BIEVR 的定向高度图、hVox 的预计算 surfel 也须保留。

严格统一点面残差、Jacobian、权重、门限、采样点集和地图预算，是可另行声明的地图表示/索引消融，不是所有候选进入主线的前置条件。完整外部 LIO 工程可作为独立端到端基线；不得在 `MapManager` 后隐藏第二套传播、状态、滤波、优化器或平滑器，并把它称作地图后端。

语义输入、地图引导采样和自适应扫描处理须单独声明输入、配置及实验档案，不能由地图 selector 静默改变公共流程。

## 2. 状态用语与当前证据

以下状态必须分别记录：

| 状态 | 含义 | 不能据此声称 |
| --- | --- | --- |
| **源码已收集** | 本地保存可追溯的作者源码快照；作者基线、当前本地提交与兼容性差异见 `doc/workspace/SOURCE_MANIFEST.json`。 | 上游完整、可构建或已接入本框架。 |
| **上游构建通过 / 阻塞** | 指定的完整包、组件或示例有实际构建退出码；范围与遗漏见 `doc/workspace/BUILD_REPORT.md`。 | 地图/残差已由 `MapManager` 使用，或数据集效果有效。 |
| **框架已集成** | 地图实现及对应观测分支进入 PV-LIO-PLUS selector 和估计循环；当前共有十一个 selector。BIEVR 与 iG-LIO 适配分支已有本轮同一份 SOLID 逐点时标数据（回归记录 01）上的三次完整输入回放；结果仅证明所列运行完整性，不证明精度或性能优越。 | 所有后端已有相同权重，或结果构成严格地图消融。 |
| **精度/运行复现** | 有逐次运行清单、输入和配置标识、完整输出及明确的数值检查。 | 不能由编译日志、符号检查、旧地图文件或论文作者结果替代。 |

### 当前验证基线

- 2026-11-14 的[构建报告](doc/workspace/BUILD_REPORT.md)记录 ROS1 共享工作区 14 个包编译退出 0、当时的上游包 CTest 21/21 通过、23 个 ELF 链接检查通过，且 PV-LIO-PLUS 单节点含六组地图实现符号。R-VoxelMap、iG-LIO 和 Hybrid-VoxelMap 的上游包包含在该共享构建中；完整构建范围见原报告。
- 同一份报告记录 BIEVR-LIO、Surfel-LIO、i-Octree 独立上游目标构建通过；LIO-GVM 因作者发布缺少定制头文件而以退出码 1 阻塞。
- 这份 2026-11-14 构建报告**没有运行 rosbag 回放**，没有评估轨迹或地图精度。回放证据另记于 2026-11-15 的结果文件；两类记录不能混为一谈。
- `9fa3111` 的 141 秒旧计时运行输入、轨迹覆盖完整，但 WallTime 回拨导致负时长；不能将其用作性能证据。六后端修复单调时钟后的 30 秒和 141 秒回归均已通过，详见阶段1结果记录。
- 2026-11-15 已修复 VoxelMap++ 外参 Jacobian 开关和点图查询半径契约，并完成两个合成 gtest 目标的实际构建与运行：CTest 2/2、共 11 个 GTest 条目。测试覆盖原有六后端生命周期、4.99/5/5.01 m 边界、两种 iVox 查询实现/有限格邻域、非法半径在清图前拒绝、OctVox 窗口拒绝，以及 12 列右扰动 Jacobian 在三组非单位姿态下的中心差分。精确覆盖与遗漏见[集成状态](doc/INTEGRATION_STATUS.md)。
- 阶段1代码 `f86ebd6` 修复单调计时后，六后端均完成同一 30 秒、0.5 米配置和 141 秒长序列回归；长回归逐后端记录 1414 LiDAR / 28595 IMU 输入、1402 轨迹/计时行、每帧非零约束和八列有限非负耗时。早先 `9fa3111` 的 141 秒序列虽有完整输入/轨迹，WallTime 回拨导致负耗时，不作计时证据。早先名为 coarse 的目录实际使用 0.1 米配置，事实保留在[阶段1结果](doc/workspace/evidence/STAGE1_RESULTS.json)，不与 0.5 米档案混淆。
- R-VoxelMap feature `af44fc0` 已以 `b820b08` 合入主线。隔离套件 CTest 4/4、23 个 GTest 条目通过；正向 ASan fixture 连续 30 次通过。一次全量 ASan 调用曾挂起并中止，后续重试通过、原因未知；ptrace 不兼容令泄漏检查关闭。主线 R 节点构建退出 0 并通过 `ldd -r`。30 秒和 141 秒完整输入回放通过；141 秒有 1414 个 LiDAR、28595 个 IMU、1402 帧输出且逐帧有非零有效约束，时长有限且非负。详见[R 结果](doc/workspace/evidence/R_VOXELMAP_RESULTS.json)；无真值精度或受控性能结论。
- hVox feature `997c00a` 已 cherry-pick 至主线 `c81f12f`。功能工作树构建、`ldd -r`、CTest 6/6（34 个 GTest 条目）、独立数学/源码核验及 30/141 秒完整输入回放通过。长回放含 1414 个 LiDAR、28595 个 IMU、1402 行；每帧有非零有效约束（243–2193），地图大小为 3341–112385，四元数单位范数误差小于 `7.0e-7`，时长列有限且非负。记录见[hVox 结果](doc/workspace/evidence/HVOX_RESULTS.json)。这是回归检查，不构成真值精度或受控性能结论；主线构建、链接、CTest 5/5（33 个 GTest 条目）及30秒完整回归通过。
- 2026-11-16 在 `master` 基线 `e666799` 接入 `ig_lio_plane`：当前代码保留 iG-LIO 增量体素充分统计与地图维护，并使用共享五点 float QR 标量点面观测；这是适配变体，不是作者 GICP。固定作者提交为 `2d79259e3aaa5bbb10d6b13234398760bbacf8f5`。原完整 checkout `src/ig_lio` 当前缺失；本包基于固定源文件安全重实现 native 地图，2026-11-14 对原完整 checkout 的 ROS1 共享构建记录仅作历史证据。Round 2 指定九个构建目标退出码 0，node `ldd -r` 通过，CTest 9/9（65 个 GTest 条目，ASan 未启用）及配置合成测试通过；**该 11-16 轮当时未回放**，所需记录输入在其清单中缺失。11-17 新的 SOLID 回归记录 01 运行验收见下方独立记录，不替代旧构建历史。完整记录见[iG-LIO 结果](doc/workspace/evidence/IG_LIO_RESULTS.json)及[源码清单](doc/workspace/SOURCE_MANIFEST.json)。

- 2026-11-16 第一类 `bievr` 与第二类 GM 首包：12 个指定构建目标、节点链接和 CTest 12/12（89 个 GTest、9 项配置测试）通过；130 文件哈希一致，**该首包当时未回放**。源版本、失败修复与验收边界仍见[原结果](doc/workspace/evidence/TWO_TRACK_RESULTS.json)；不要把其测试数替换成 11-17 新回归数字。
- 2026-11-17 当前 `master` 基线 `3d4d0e3` 加本轮未提交源码的 13 构建目标、`ldd -r`、CTest 13/13（105 GTest，ASan 关闭）通过；另有 12 项 Python 配置测试与 185 次 mapping 比较。新 SOLID 回归记录 01 的 BIEVR、iG-LIO、iG-LIO+GM 三次 fresh master 回放均完成输入监测、所选窗口自然耗尽和 287 行输出检查。具体输入身份、逐帧范围及限制见[本轮运行记录](doc/workspace/evidence/INTEGRATION_PROGRESS_20261117.json)。

## 3. 十一个已接入后端

下表中的“作者版本”来自本地源清单中的固定作者提交；如有本地兼容性提交，完整 SHA 和补丁均以 manifest 为准。集成地图位于 `include/map_manager/native/`。表中上游路径/提交属于固定来源记录，当前可用性以源码清单的 `current_snapshot` 和各本地 SOURCE 为准，不能假定完整作者工程仍存在或保持原快照。

| 后端 | 集成地图与观测 | 作者工程路径 / 作者版本 | 当前证据 |
| --- | --- | --- | --- |
| VoxelMap | `voxelmap`；概率体素平面及其点面不确定性观测 | `src/VoxelMap`，`c184a6c2` | selector/观测已集成；六组符号包含于 11-14 编译记录。 |
| VoxelMap++ | `voxelmap_plus`；平面合并及其原生平面参数观测 | `src/VoxelMapPlus_Public`，`fe73d5c2` | 同上。本地兼容性提交为 `d0cf184d`；`extrinsic_est_en_` 启用时现计算右扰动外参旋转/平移 Jacobian，关闭时外参列为零。`h=-float(dist)`、原生权重和面片门限未改。 |
| FAST-LIO2 / ikd-tree | `ikdtree`；增量点树及 PV 点面拟合观测 | `src/FAST_LIO`，`d30656a5`；ikd-Tree 子模块 `e2e3f4e9` | 同上。 |
| Faster-LIO / iVox | `ivox`；稀疏体素点图及 PV 点面拟合观测 | `src/faster_lio`，`ea0e0910` | 同上；本地兼容性提交 `1273553f`。 |
| C3P-VoxelMap | `c3p_voxelmap`；概率体素平面及原生不确定性观测 | `src/c3p_voxelmap`，`ab4a8020` | 同上；本地兼容性提交 `69f84b88`。 |
| Super-LIO / OctVox | `octvox`；八分体代表点、TopK 与固定信息量平面观测 | `src/Super-LIO`，`60b57aaa`（`ros1`） | 同上；本地兼容性提交 `9bd033b3`。 |
| R-VoxelMap | `r_voxelmap`；递归内部/子层概率平面及原生协方差观测 | `src/R-VoxelMap`，`50ffbbab` | selector 与观测在 `b820b08` 合入；隔离测试/回放通过，主线节点构建/动态链接通过，主线常规 CTest 3/3（22 GTest）通过。 |
| Surfel-LIO / hVox | `hvox`；L0/L1 双层地图、预计算 surfel 与 PKO 加权有符号平面观测 | `upstream/Surfel-LIO`，`73348c1e` | feature `997c00a` 已以 `c81f12f` 合入主线；功能工作树构建、测试和 30/141 秒回归通过；主线构建、链接、CTest 5/5（33 个 GTest 条目）及30秒完整回归通过。 |
| i-Octree | `i_octree`；结构保真的动态八叉树、RAII 连续叶块、数据 AABB 全树 kNN 及点面适配 | `upstream/i-Octree`，`5b640e05` | feature `5c27a79` 已以 `6876a8e` 合入主线；feature 隔离测试/数值复核和 30/141 秒回归通过；主线构建/链接、CTest 7/7（49 个 GTest，ASan 关闭）及 30 秒回归通过。 |
| iG-LIO / incremental VoxelMap | `ig_lio_plane`；保留每格增量充分统计和原生点保留规则，以共享五点 float QR 拟平面生成标量点面观测；不含作者 GICP | 当前 checkout 缺少完整 `src/ig_lio`；基于固定源文件重实现，见 `include/map_manager/native/ig_lio/UPSTREAM_MANIFEST.json` 与 `SOURCE.md` | 11-17 本轮 SOLID 回归记录 01 上 iG-LIO 与 iG-LIO+GM 两次回放通过输入/输出完整性检查；旧 Round 2 build/test 证据见其独立结果。此前共享构建仅指 2026-11-14 的完整上游 checkout。 |
| BIEVR 高度图变体 | `bievr`；定向像素图及 bilinear 高度/Huber 信息 | 当前作者 `2306022e`，本地 `native/bievr` | 11-17 本轮 SOLID 回归记录 01 回放通过输入/输出完整性检查；11-16 build/synthetic 结果仍见[历史结果](doc/workspace/evidence/TWO_TRACK_RESULTS.json)。 |

共用滤波主循环不等于共用观测。VoxelMap++、点图适配层、C3P、OctVox 和 R-VoxelMap 的残差、Jacobian、信息量及门限按实际路径分别记录。`nearest_max_range` 对 ikd-tree/iVox 统一为米：ikd-tree 比较第五近邻平方距离与 `r²` 并接受边界，iVox 使用严格 `<r` 且仍受配置附近格域限制。旧 ikd-tree 设置 `v` 原来代表平方距离；要复现旧阈值，新半径应设 `sqrt(v)` 米。现有 iVox 数值不变。细节见[地图与观测范围审查](paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)。

## 4. 两项仍待集成的合格地图候选

下列两个项目具有历史收集记录，当前可用源码及差异需分别复核，且其地图数据结构创新满足当前硬条件；它们尚未成为新的 PV-LIO-PLUS selector。主路线评估结构创新及其对应观测；严格统一点面的版本可作为单独消融。完整上游系统仍可在独立工作区作为端到端基线。不得剥离资格结构后，以原算法名义只测通用点容器。iG-LIO 增量体素地图已有点面 selector；其完整 GICP 观测仍是另一项未完成的设计与实现工作，见阶段 5。

| 方法 | 本地工程 / 固定作者版本 | 已有上游构建记录 | 计划范围与前置条件 |
| --- | --- | --- | --- |
| Hybrid-VoxelMap | `src/Hybrid-VoxelMap`，静态核验 HEAD `b57ce03e`；历史 `89ed71c2` | 历史共享构建；当前 checkout 有未提交修改，旧对象不可读取，未重新验证；不转用旧构建证据 | 当前地图为 UnionFind voxel/Gaussian 与合并 Plane 组合，不是八叉树。逐点语义源、同步、过滤/降采样投票及类别 3 object 处置尚待确认；先满足[输入契约](doc/HYBRID_INPUT_CONTRACT.md)，再评估 typed Plane/NDT 对应观测。无标签路径不得称完整 Hybrid。 |
| LIO-GVM | `upstream/lio_gvm` / `c80c968e` | 阻塞：构建退出码 1 | 保留 Gaussian 地图/观测设计；需作者补齐定制 Fast-VGICP、IKFoM 和地图头文件后，再验证完整信息模型与状态映射。不可用通用依赖冒充作者接口。 |

## 5. 结构资格通过、但本地作者源码尚未确认的论文储备

以下两项论文描述了新的地图结构，可保留为结构研究储备；本地收集记录尚未确认作者官方源码，因此不是当前可实施源码候选。完整论文内容与版本保持在 [paper/README.md](paper/README.md) 和 catalog。

| 方法 | 当前安排 | 继续条件 |
| --- | --- | --- |
| FR-LIO / RC-Vox | 结构合格的论文储备；本地未确认作者源码 | 跟踪两层地图、机器人中心滑动与邻域索引；若取得固定作者快照，再审查地图/观测可分离性。完整子帧/ESKS 属于估计器实验。 |
| SV-LIO | 结构合格的论文储备；本地未确认作者源码 | 跟踪语义引导子八叉体及叶平面概率结构；若取得可核验实现，再核验标签输入和对应观测。语义数据单列实验。 |

## 6. 集成顺序与每阶段完成条件

2026-11-17 BIEVR/iG-LIO 首包运行验收已完成：相同独立 SOLID 逐点时标数据（回归记录 01）下，BIEVR、iG-LIO、iG-LIO+GM 三次 fresh master 回放均通过运行完整性门。后续优先完成长序列与高度噪声标定；GM 目前仅 iG-LIO `c=0.1 m` 这一组合做固定输入对照。随后再进入 GenZ 第二类观测实现；Hybrid 只有逐点语义输入契约及未决类别策略确认后才排实现。LIO-GVM 仍待作者源码完整。完整数字见[本轮运行记录](doc/workspace/evidence/INTEGRATION_PROGRESS_20261117.json)。GenZ 设计见[GenZ 混合观测设计](doc/GENZ_HYBRID_DESIGN.md)，Hybrid 前置条件见[Hybrid 输入契约](doc/HYBRID_INPUT_CONTRACT.md)。

### 阶段 1：原有六个后端的契约与复现基线

**状态：完成（契约与输入完整性回归）。** VoxelMap++ 外参 Jacobian 与点图物理半径契约已修复；两个合成 gtest 目标已构建并通过 CTest 2/2（11 个 GTest 条目），其中六个生命周期用例覆盖原有六后端；该阶段 2026-11-15 的 `scripts/datasets.example.json` 只列出当时的原有九个 selector，是历史测试范围，不表示当前清单或当前回归覆盖。六后端均通过修复单调计时后的 30 秒、0.5 米回归（各 288 帧）及 141 秒长回归；长回归逐后端收到 1414 LiDAR / 28595 IMU，输出 1402 轨迹/计时行，每帧有非零约束，全部八个耗时列有限且非负。早先 `9fa3111` 的记录因 WallTime 回拨仍保留为非计时证据；早先 coarse 目录实际使用 0.1 米配置，勿与 0.5 米回归混淆。此阶段完成表示流程回归通过，不构成精度或受控性能结论。具体边界见[集成状态](doc/INTEGRATION_STATUS.md)和[阶段1结果](doc/workspace/evidence/STAGE1_RESULTS.json)。

- 逐后端记录坐标系、长度/距离/平方距离单位、协方差语义、地图 `size` 口径、当前帧自匹配、插入时机、搜索域、门限、窗口/淘汰和观测模型。
- 点图参数以米输入，并按 ikd-tree `r²`/含边界与 iVox 严格 `<r`/附近格域的原生约定查询；拒绝非正、非有限或平方后非有限/零的半径，配置检查必须先于已初始化地图的清理。
- 已有六后端合成测试包含生命周期、半径边界和外参 Jacobian 数值核验；修复后的六个 30 秒短 bag 完整性回归已通过。`scripts/run_backend_tests.py` 可按清单构造运行与归档，但每个结果须引用实际 run manifest、输入/配置身份和产物检查。
- 长序列完成条件：六个 selector 均有修复后单调时钟的逐次构建、运行/退出、轨迹、地图质量和非负耗时检查记录。此前 141 秒记录的输入/轨迹覆盖可留档，但负 WallTime 不能支持时延比较。

### 阶段 2：R-VoxelMap 与 hVox 已集成

- R-VoxelMap：selector 与对应原生概率观测已在 `b820b08` 集成，并保留内部/子层递归平面、剩余点链表下推/重建及 root LRU。隔离 CTest 4/4、23 个 GTest 用例、正向 ASan fixture 30 次及 30/141 秒完整输入检查通过；主线节点构建退出 0、`ldd -r` 通过，主线三个常规 CTest 目标 3/3（22 GTest）通过，主配置未启用 ASan。141 秒每帧有非零约束且单调时长有限非负。一次 ASan 全量运行曾挂起后中止，后续重试通过、原因未知；ptrace 限制下关闭泄漏检测。目前不宣称精度或受控性能结论。详见[R 结果](doc/workspace/evidence/R_VOXELMAP_RESULTS.json)。
- hVox：feature `997c00a` 已 cherry-pick 至主线 `c81f12f`，保留 `73348c1e` 的 L0/L1 双层地图、更新期预计算 surfel、原生移动盒与精确 L1 键查询。功能工作树构建与 `ldd -r`、CTest 6/6（34 个 GTest 条目）、数学核验和 30/141 秒完整输入回放通过。141 秒 1402 帧全部有有效约束、耗时有限非负；主线构建、链接、CTest 5/5（33 个 GTest 条目）及30秒完整回归通过。无独立真值或受控性能结论。详细数据见[hVox 结果](doc/workspace/evidence/HVOX_RESULTS.json)。

### 阶段 3：i-Octree 地图结构接入

**状态：完成（selector、隔离数值/测试及主线构建、常规测试和短回归均通过）。** Feature `5c27a79` 已以 `6876a8e` 合入主线。该适配保留动态八叉树、RAII 连续叶块与树上维护，同时使用实点 AABB、binary64 查询/距离和受控 RN/FTZ/DAZ 路径；这不是未修改上游的性能复现。隔离测试 CTest 9/9，包含 49 个独立常规用例及 11 个 sanitizer 重复执行，总计 60 次执行；正向 ASan/UBSan 检查通过，LeakSanitizer 未启用。独立数值审查通过，C1/C2 条件已关闭。feature 构建及 `ldd -r` 通过，30 秒和 141 秒完整输入回归通过。主线 `6876a8e` 节点构建与 `ldd -r`、常规 CTest 7/7（49 个 GTest 条目，ASan 关闭）及 30 秒完整输入回归通过。完整边界见[i-Octree 结果](doc/workspace/evidence/I_OCTREE_RESULTS.json)。

- 查询契约：全树 kNN 按声明的 binary64 度量搜索，第五近邻以 `<=R²` 接受；native radius API 使用严格 `<r²`。局部窗口通过树上保留闭盒内点，不重建整棵树；`size()`/`point_count()` 是存活点数，快照为 XYZ。其余结构、数值模式和异常保证见[集成状态](doc/INTEGRATION_STATUS.md)及[本地适配说明](include/map_manager/native/i_octree/SOURCE.md)。
- 回放完整性：feature 30 秒 301 LiDAR/6088 IMU 输入对应 288 行；141 秒 1414 LiDAR/28595 IMU 输入对应 1402 行，每帧均有非零约束（513–5620），所有八个计时列有限且非负。主线也通过 30 秒 301/6088 输入、288 行完整性检查。记录不是精度或受控性能结论。
- 完成条件：已取得主线构建/链接、常规 CTest 和短回放的可追溯记录；feature 长序列与隔离测试独立注明。不得把改造版描述成原封不动上游实现。

### 阶段 4：地图管理职责评估与 BIEVR 共同滤波设计

- i-Octree 主线验收后，先评估 `src/map_manager/map_manager.cpp` 中 `MapManager` 类及各后端职责的拆分，明确统一门面、后端状态、原生实现所有权和 ODR 边界；头文件整理只是辅助。[类职责评估及分阶段方案](doc/MAP_MANAGER_REFACTOR_PLAN.md)已完成；2026-11-16 在当前 `lio_map_ws` 实施首个 i-Octree 非拥有 adapter 试点，抽出公共类型/配置及点面拟合，保留门面的状态所有权和更新顺序，并补五项黑盒契约用例。当前 checkout 节点及七个测试目标增量构建、`ldd -r`、CTest 7/7（54 用例，ASan 关闭）通过，详见[本轮结果](doc/workspace/evidence/MAP_MANAGER_ADAPTER_RESULTS.json)；尚未执行回放、逐位数值或性能 A/B。运行验收前不继续迁移其他后端，也不宣称已解决其余原生头的 ODR/静态状态问题。
- BIEVR 的初始设计形成于 2026-11-15；首版已按当前作者快照 `2306022e` 实施定向高度图和公共 IKFoM bilinear 观测。历史草稿、当前差异和限制见[设计与实施记录](doc/BIEVR_IKFOM_PORT.md)；不把该适配称为作者 LM/Ceres 系统复现。
- 当前已完成职责拆分评估、BIEVR 源契约/数学审查、首包接入及合成 oracle；11-17 SOLID 回归记录 01 短回归通过。高度噪声标定、退化效果、长序列和独立采样扩展仍待验，不把初始设计稿的旧前置列表当作当前实施状态。i-Octree adapter 的专门回归尚未执行，其待验状态不由本轮 BIEVR/iG-LIO 结果替代。

### 阶段 5：iG-LIO 点面地图已接入；GICP 观测仍待设计

- `ig_lio_plane` 保留每格 `N, S=Σp, Q=Σppᵀ` 充分统计和原生点保留/维护规则；当前使用共享五点 float QR 平面拟合和标量点面观测。11-16 Round 2 构建、`ldd -r`、CTest 与配置合成测试通过，但该轮所需输入文件缺失，**当时未回放**；11-17 新 SOLID 回归记录 01 单独完成 iG-LIO 与 iG-LIO+GM 回归，不能以旧完整上游构建替代本地验证。新增配置字段与 `unique_ptr` 改变类布局，验收需全量重编所有消费者。详见[本轮运行记录](doc/workspace/evidence/INTEGRATION_PROGRESS_20261117.json)与[iG-LIO 历史构建记录](doc/workspace/evidence/IG_LIO_RESULTS.json)。Round 1 的构建失败及修复见[iG-LIO 结果](doc/workspace/evidence/IG_LIO_RESULTS.json)。
- 完整 GICP 是后续独立观测设计：逐项研究 3 维残差、两侧协方差、完整信息矩阵与鲁棒权重。Cauchy 模型含 `rho[2]` 二阶项；不得把相关 3×3 信息或鲁棒二阶项直接压成一个标量对角 `R` 并声称与原生更新等价。若设计白化为标量观测行，必须证明/测试二次型和信息保持；否则注明近似及影响。
- 完成条件：点面地图更新、容量淘汰和查询通过 oracle；另行完成 GICP 数学设计与状态映射审查，合成退化/奇异协方差/Jacobian 测试通过，再做短序列；原生适配与白化近似分开记录。

### 阶段 6：BIEVR 短回归通过；Hybrid 等待语义输入

- 11-17 SOLID 回归记录 01 短回归已完成：数据 SHA-256 为 `0e01bca27ae119dc9f04bf736b55dc2f5fbb500389fc944102f07f6d7816e694`，record 时间范围 `[12614334000000,12644334000000] ns`，输入 299 LiDAR / 6056 IMU。配置 `config/solid_point_time.yaml` 保留 SOLID 逐点绝对 timestamp；BIEVR、iG-LIO、iG-LIO+GM 三个 fresh master 运行均自然读至 EOF 并排空末帧，各输出 287 行。完整逐帧和哈希核对见[本轮运行记录](doc/workspace/evidence/INTEGRATION_PROGRESS_20261117.json)。
- Hybrid：当前静态审查显示作者地图是原 voxel Gaussian 与 UnionFind 合并 Plane 的组合；逐点标签源和处理策略未定，且语义 bridge topic 当前未被 mapper 消费。先按[Hybrid 输入契约](doc/HYBRID_INPUT_CONTRACT.md)确认标签、帧/点对应、unknown/object 策略及过滤/降采样投票，再做带类型的 Plane/NDT 匹配和各自观测；缺标签的 Gaussian-only 路径独立命名，不计为完整 Hybrid。语义推理开销与地图/残差实验分别报告。
- BIEVR：`bievr` 源码已接入，保留定向图像、融合/重投影与 LRU，使用公共完整去畸变云写图、公共降采样点注册；独立高度残差和 Huber 信息接共享滤波。11-17 SOLID 回归记录 01 回放通过，其他验收项仍待：长序列、噪声标定、GUI/设备、ATE、受控性能与地图引导采样。11-16 构建/合成历史记录见[TWO_TRACK_RESULTS](doc/workspace/evidence/TWO_TRACK_RESULTS.json)，其当时未回放。
- 完成条件：所需输入可追溯、结构创新保留、残差和失效语义有合成测试、公共估计循环中没有隐藏求解器、每种采样/语义设置有清楚的 run manifest 与消融标签。

### 阶段 7：LIO-GVM 源码恢复后再设计 Gaussian 观测

- 先要求作者发布树包含编译所需的 Fast-VGICP、IKFoM 和地图定义；核对新提交与缺失接口的完整性，固定作者版本。
- 之后才设计 Gaussian 匹配、残差、Jacobian、协方差/信息和退化处理；不得先将其降成点面残差以绕过源码缺口。
- 完成条件：完整作者树可独立构建，观测模型有独立类型/接口与合成 oracle，再进入共享框架短回放。

### 阶段 8：FR-LIO 与 SV-LIO 论文储备跟踪

- 当前仅作为合格结构论文储备，不进入可实施候选排期。若后续获得固定作者源码，再核验地图结构实现、输入条件和地图/观测边界。
- 完成条件：取得作者快照并通过结构资格与可分离性审查后，由计划变更另行准入；论文和现存历史材料本身不触发排期。

### 阶段 9：第二类观测与权重组件

- 第一包已接入：默认关闭的 `observation/weight_model=geman_mcclure` 与米制固定尺度；仅四个已核准的共享点面分支可用，默认保留原后端路径。
- 第二包：按[GenZ 混合观测设计](doc/GENZ_HYBRID_DESIGN.md)增加独立 typed 匹配/原始邻域接口、点面/点点观测行与公共 12 列右扰动映射；替换本地变体的 27 格搜索为 ikdtree 原生 KNN，并显式记录其不是完整 GenZ 复现。不导入上游 ICP 求解器或自适应扫描体素化。
- 未来的自适应匹配门限独立开关、独立统计状态和重置条件，不由权重选项隐式改变搜索。
- 完成条件：兼容组合校验、真实 `boxplus` 逐列 Jacobian oracle、二次型/梯度/信息块 oracle、退化与边界用例、失败回滚和 map-update 抑制、真实启动配置与归档通过，再单列固定输入回归。GM 首包源码已接入；GenZ 混合观测与自适应门限仍待实现，`observation/model=genz_hybrid` 当前会明确拒绝；新模型启用时 `observation/weight_model` 必须为 `backend_default`，避免双重 GM。

## 7. 共同契约与验证要求

每个后端的当前存储/查询/观测、`size`、窗口和未匹配返回口径见[集成状态](doc/INTEGRATION_STATUS.md)。逐帧契约仍须明确：协方差描述哪个变量、当前帧如何避免自匹配、地图何时更新、门限是什么以及失败时如何处理；native `non_match` 并非统一的逐点失败列表。

| 文件 | 职责 |
| --- | --- |
| `include/map_manager/map_manager.h`、`src/map_manager/map_manager.cpp` | 地图生命周期、初始化、搜索、更新、快照、局部窗口及大小语义。 |
| `src/mapping.cpp` | selector 对应的匹配消费与后端观测分派。 |
| `include/map_manager/map_types.h` 中的 `MapType`、`src/config.cpp` 与 `config/*.yaml` | 已有地图选择及参数解析。 |
| `src/utils.cpp` | 轨迹和地图输出命名。 |
| `scripts/run_backend_tests.py` | 输入清单驱动的构建、运行与结果归档；运行结论仍须指向实际记录。 |

当前十一个地图分支包含十个点面/概率平面分支和一个 BIEVR 高度分支。BIEVR 使用按值返回的 `HeightSample`，不伪装为 `PlaneMatch`；通用 Surfel/Gaussian/Point 类型及相关向量信息接口仍待设计。

观测可为点面、点点、GICP、Gaussian 或高度图。接口必须保留其残差维度、Jacobian 与完整信息含义；不能把不兼容类型硬塞入 `PlaneMatch`，也不能丢掉相关协方差后称数学等价。主路线中，各后端无需得到相同 `h/H/R`；仅严格统一观测消融检查跨地图等价。

验证分四层：

1. **地图 oracle**：按各自声明搜索域，与小规模暴力结果核对查询、插入、删除、淘汰、快照和空输入。
2. **观测 oracle**：固定匹配与状态，用正确流形扰动有限差分核对残差和 Jacobian；检查信息矩阵有限、对称、正定或按设计处理退化。
3. **运行证据**：记录源码/本地提交、配置、数据身份、输入预处理、采样、语义、门限、地图预算、命令、退出码、逐帧统计、轨迹和地图质量门。
4. **解释范围**：共同框架组合、严格消融、完整 native-stack 基线分开归档；报告中注明成功定义、缺帧、实际有效支持、时间/内存与指标限制。编译通过或一次回放都不单独证明精度提高。

## 8. 发布与收尾

- 新 selector 只有在来源与本地差异记录、地图/观测契约、合成测试、短回放和长序列基线均有可追溯证据后，才从“待集成”改为“已验证接入”；“已集成”与“精度复现”仍分开标记。
- 更新中英文 README、`SOURCE_MANIFEST.json` 的 source scope、论文索引和各笔记；只写本地实际检查过的来源/提交。`catalog.json` 的论文版本、页数和 SHA-256 与集成范围说明分开维护。
- 保留 `THIRD_PARTY_NOTICES.md` 和各上游许可。来源、许可或作者版本有不确定项时，先注明边界再分发。
- 完整系统基线放入隔离运行/构建记录；不得把运行完整上游节点的结果误记为该系统地图已适配到 PV-LIO-PLUS。
