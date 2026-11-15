# 地图数据结构资格与对应残差集成范围审核

核验日期：**2026-11-15**。资格判定依据固定版本的本地论文及其分页全文、作者贡献陈述和本地作者源码；关键论文另经官方一手来源对照。未更新远程仓库或源码基线，本次没有重新编译或回放。下文的远程版本差异观察单独保留为 2026-11-14 历史证据。

## 项目定位与硬准入条件

本项目源自 PV-LIO：PV-LIO 在 FAST-LIO2 源码框架中用 VoxelMap 替换 ikd-tree，并利用点与平面的不确定性构造滤波观测，见 [PV-LIO 说明](../../../PV-LIO/README.md#L2)。PV-LIO-PLUS 随后集成 VoxelMap++，大幅重构框架，再继续集成更多地图结构及其对应残差模型。主目标是在相同框架中评估不同地图与配套观测的效果。

**进入当前地图集成/候选清单的必要条件：原论文将提出新的地图数据结构或改进地图数据结构列为创新之一，并能指出存储单元、空间组织、层级、融合统计或地图维护结构的实际变化。** 使用体素/哈希容器、仅改查询算法、残差、语义权重、扫描采样、估计器缓存或更新门控均不自动满足。结构创新资格、源码可用性、编译和集成是独立状态；源码已收集或已构建不能补足论文资格。

实际移植必须保留支撑资格的结构创新。若抽取时去掉该创新，只剩通用点容器，就不能继续以原方法命名或宣称评估了该地图。尤其 iG-LIO 必须保留增量体素充分统计及相应维护；只借 `AddCloud` 的点集作为普通哈希索引不构成 iG 地图评估。点面适配可以作为明确标记的观测变体，不能抹去地图结构。同一原则适用于合并、递归、层级 surfel 和高度图等其它合格方法。

| 实验范围 | 可变化的内容 | 必须明确的边界 |
| --- | --- | --- |
| **共同框架中的地图与对应残差，主路线** | 合格地图的表示、增删/淘汰、查询与匹配，以及相应残差、Jacobian、不确定性、鲁棒权重和门限。 | 统一消息同步、IMU 传播、去畸变、状态定义、滤波主循环、输出和评测；记录各地图与观测配置，保留支撑资格的结构机制。 |
| **严格地图表示 / 索引消融，可选实验** | 实验事先声明的地图表示或索引因素。 | 另行统一残差、Jacobian、权重/噪声策略、门限、采样点集和地图预算；纯索引对照还需相同保留点集与拟平面适配器。 |

完整外部 LIO 系统作为独立端到端基线运行。不得在 `MapManager` 后隐藏第二套传播、状态、优化器、平滑器或连续时间估计器。额外语义输入、地图引导采样等须单独声明条件。**不同残差可以随满足资格的地图集成，残差不同本身不是排除理由。** 不得把散点几何协方差当作平面参数不确定性，也不得把不兼容的观测硬塞进 `PlaneMatch`。

## 全部 22 篇固定论文的结构创新资格

页码均为本地 PDF 的 1 起始页码；版本见 [catalog.json](../catalog.json)。FAST-LIO2 与 ikd-Tree 两篇合并一行，表内共覆盖 22 篇 canonical 论文，不含两份旧版归档。**“保留”仅表示通过结构创新筛选，不表示已经集成或可立即实现。** “证据不足”项退出当前地图候选/排期，论文和来源仍保留。

| 方法 / 固定论文 | 判定 | 贡献陈述与实际结构证据 |
| --- | --- | --- |
| [VoxelMap](../VoxelMap.pdf) | **保留，已集成** | p1 贡献 1–2，pp2–3 §III-A/B：自适应粗到细 hash-octree、概率平面及参数不确定性；改变单元尺度、层级构造和存储统计。 |
| [VoxelMap++](../VoxelMap++.pdf) | **保留，已集成** | p1 贡献 1–2，pp3–4 地图构建/合并：3-DoF 平面参数与协方差，union-find 将共面体素归并为父平面，改变单元表示与跨格组织。 |
| [FAST-LIO2](../FAST-LIO2.pdf) / [ikd-Tree](../ikd-Tree.pdf) | **保留一个 ikd-tree 后端，已集成** | FAST-LIO2 p2 贡献 1；ikd-Tree p1 摘要/引言及 §III：增量 KD-tree、惰性删除、盒操作、树内降采样和局部重平衡。创新归于 ikd-Tree，不重复计两个地图。 |
| [Faster-LIO / iVox](../Faster-LIO.pdf) | **保留，已集成** | p2 贡献 1–2：稀疏增量体素及 linear/iVox-PHC 两种底层结构，改变点组织与格内布局；不是仅提出并行查询。 |
| [C3P-VoxelMap](../C3P-VoxelMap.pdf) | **保留，已集成** | p2 贡献 1–3，pp3–5 §IV/V：点无关的固定规模充分统计、累积更新、平面合并与 locality-sensitive hash 合并桶。 |
| [Super-LIO / OctVox](../Super-LIO.pdf) | **保留，已集成** | p1 摘要、p3 §III-C：每格八个子体素代表点、有限密度聚合/增量降噪及 LRU 组织；资格来自 OctVox，不能只保留 HKNN 查询。 |
| [R-VoxelMap](../R-VoxelMap.pdf) | **保留，待集成** | p2 贡献 1，p3 §III-A/B：内部与叶节点均可保存平面/点，链表划分点集、离群点递归下推复用，改变传统仅叶平面组织与构建。 |
| [Surfel-LIO / hVox](../Surfel-LIO.pdf) | **保留，待集成** | p2 贡献 1–2，p4 §III-B：两层 L0 质心 / L1 预计算 surfel 及 Z-order 键组织；必须保留更新期计算与层级，而不只是加速查找。 |
| [i-Octree](../i-Octree.pdf) | **保留，待集成** | p1 摘要，p2 §II-A：叶端连续内存点块、仅叶保存点、局部树上更新/降采样和盒删除；明确提出动态八叉树数据结构。 |
| [iG-LIO](../iG-LIO.pdf) | **保留，待集成** | p2 第三项贡献，pp3–4 §II-C、式 (7)–(9)/算法 1：每格 `(N,Σp,Σppᵀ,μ,Σ)`，活跃格增量分布更新及 LRU 地图维护。资格不靠 VSCE 扫描协方差或 GICP 残差。 |
| [KISS-ICP](../KISS-ICP.pdf) | **移除地图候选** | p2 贡献是简洁系统、模块评估与自适应阈值；p4 §III-C 利用已有 voxel grid/hash 并引用既有实现思路，未将新的地图结构列为贡献。有界点存储和距离淘汰的实现存在，不足以证明该论文提出结构创新。 |
| [GenZ-LIO v3](../GenZ-LIO.pdf) | **移除地图候选** | pp2–3 贡献/五项主张是扫描尺度控制、混合状态观测和剪枝查询；p9 §V-B 明确查询 VoxelMap [11]，所谓 27 区域是查询分类，非新存储层级。[原地图源码](../../../GenZ-LIO/cpp/genz_lio/core/VoxelMap.hpp#L3)也注明继承 VoxelMap/PV-LIO；未证明独立地图结构创新。 |
| [Hybrid-VoxelMap](../Hybrid-VoxelMap.pdf) | **保留，条件化待集成** | p1 第一项贡献，p3 §III-B：语义决定每格 Plane/Gaussian 表示并动态转换；改变实际几何单元类型与维护，区别于只按语义调残差权重。 |
| [BIEVR-LIO](../BIEVR-LIO.pdf) | **保留，条件化待集成** | p2 贡献 1，p3 §III-A：每体素存定向 bump image、高度/权重像素与局部坐标变换，迭代融合细节；资格来自新地图单元，不依赖采样创新。 |
| [LIO-GVM](../LIO-GVM.pdf) | **保留资格，源码阻断** | p2 第二项贡献，p5 §IV-D/式 (18)：将 VGICP 式分布提升为可插入/更新/删除的全局哈希高斯地图，维护均值/协方差/计数并以 `M=max(M,N)` 防止历史统计冻结。不是仅更改 Gaussian 残差。 |
| [FR-LIO / RC-Vox](../FR-LIO.pdf) | **保留论文地图储备，源码未核验** | p2 第二项贡献，p4 §V-A/B：机器人中心、固定大小双层 3D 数组及滚动重用索引，明确提出 RC-Vox 结构；完整子帧与 smoother 不随地图移植。 |
| [CT-VoxelMap](../CT-VoxelMap.pdf) | **结构创新证据不足，移出当前候选** | p2 贡献重点为 spline/融合/重估；p7 图 5 与正文明确采用类似 VoxelMap [38] 的粗到细构造，将非平面叶称为 voxel feature，pp7–8 主要新增特征不确定性和观测。未核实独立存储、组织或维护结构变化，不能因名称/混合残差保留。 |
| [SA-LIVO v3](../SA-LIVO.pdf) | **移除地图候选，仅相关视觉/估计研究** | p1 贡献为 SAIF、估计器效率和系统验证；pp3–4 §V-B/C 继承 VoxelMap 八叉树，冻结平面与邻域 PCA 统计缓存服务估计/查询，未证明作者将新的环境地图结构作为创新。[原 VoxelMap](../../../VoxelMap/include/voxel_map_util.hpp#L409)已有饱和后停止更新机制，不能据“冻结”直接认定新结构。 |
| [SV-LIO](../SV-LIO.pdf) | **保留论文地图储备，源码/语义输入未核验** | p3 贡献 2，pp9–11 §4.3/式 (10)–(14)：语义引导聚类，离群点下推八个子体素，剩余内点重估平面并维护叶平面的类别概率。改的是递归构建与单元统计，不只是 §4.4/4.5 的语义关联/加权。 |
| [Environment-Adaptive Solid-State LIO](../Environment-Adaptive-Solid-State-LIO.pdf) | **结构创新证据不足，移出当前候选** | p2 贡献 2 是退化感知更新策略；p4 §III-C2/式 (16)–(17)给出帧级门控及置信度 `q_v` 的 EMA，但未清楚规定它如何改变空间组织、层级融合或几何统计结构。持久置信度/更新门控本身不充分，作者源码也未核验。 |
| [FAST-LIO](../FAST-LIO.pdf) | **框架来源，不单列地图候选** | p1 贡献为紧耦合滤波、去畸变与 Kalman gain 计算；不能把 FAST-LIO2/ikd-Tree 的结构创新回填给 FAST-LIO 原论文。 |

PV-LIO 未收录独立论文，其项目说明证明它在 FAST-LIO2 框架采用 VoxelMap，不能据此另认定一项新地图结构。PV-LIO/FAST-LIO 可以作为框架来源或独立系统比较基线，须与合格地图清单分开。当前六个 selector 均通过资格筛选，**本轮不删除或新增 selector 代码**；KISS/GenZ 等退出候选也不等于删除已收集源码、论文或历史构建结果。

## 现有六后端：公共循环相同，观测并非完全相同

[公共循环](../../src/mapping.cpp#L896)共用滤波初始化、`imu_process_->Process` 与 `update_iterated_dyn_share_diagonal`；[ObservationModel 分派](../../src/mapping.cpp#L711)保留不同观测实现。下表 `w` 为 `ekfom_data.R` 中的逆方差/信息量。

| 后端 | 地图与观测证据 | 当前适配行为与限制 |
| --- | --- | --- |
| VoxelMap | 自适应体素八叉树保存概率平面；[build_single_residual](../../include/map_manager/native/voxelmap/voxel_map_util.hpp#L848)使用平面范围、概率门限与 6 维平面协方差。[ObservationModelVoxelMap](../../src/mapping.cpp#L246)：`w=1/(JΣplaneJᵀ+nᵀΣpoint n)`。 | 信息分母没有点图路径的 `1e-9` 下限。 |
| VoxelMap++ | UnionFind 平面地图及合并，按主方向使用 3 参数平面；[BuildSingleResidual](../../include/map_manager/native/voxelmap_plus/voxelmapplus_util.hpp#L719)与 [ObservationModelVoxelMapPlus](../../src/mapping.cpp#L346)传播 `J_abd` 协方差，残差为 `-float(dist)`。 | **当前外参 Jacobian 后 6 列仍恒零**，见 [mapping.cpp](../../src/mapping.cpp#L394)；其余观测分支遵循外参估计开关。 |
| ikd-tree | 增量 KD-tree 查询点邻域；[fit_point_plane](../../src/map_manager/map_manager.cpp#L388)拟平面，[ObservationModelPointBackend](../../src/mapping.cpp#L449)构造观测。 | `plane_cov=0`，仅保留点协方差，总方差下限为 `1e-9`；是本项目观测适配器，不等同完整 FAST-LIO2 观测。 |
| iVox | 稀疏增量体素与配置邻域；[search_ivox](../../src/map_manager/map_manager.cpp#L1218)复用上述拟平面及点协方差权重。 | ikd-tree score 与 iVox 的残差门限在实数运算下等价为 `range>81*r²`，搜索域、保留点与容量策略仍不同。 |
| C3P-VoxelMap | 紧凑概率体素及可选合并；[search_c3p](../../src/map_manager/map_manager.cpp#L1249)调用原生残差生成，[ObservationModelC3P](../../src/mapping.cpp#L619)传播 6 维平面与点协方差。 | 使用独立 C3P 观测，没有点图路径的方差下限。 |
| OctVox | 八分体代表点聚合、容量/LRU、有序 TopK；[search_octvox](../../src/map_manager/map_manager.cpp#L1281)执行 4/5 点拟平面、0.1 m 邻点平面与距离门限；[ObservationModelOctVox](../../src/mapping.cpp#L537)使用 `w=1000`。 | 固定信息量；LRU 不等于几何窗口，启用公共局部窗口会被配置检查拒绝。 |

[calc_lidar_cov_for_backend](../../src/mapping.cpp#L227)还为 VoxelMap++ 选择单独的传感器协方差实现。这些差异属于当前地图与观测组合，应如实记录；现有结果不能直接称为严格地图表示消融。

**距离单位：** [search_ikd_tree](../../src/map_manager/map_manager.cpp#L1192)将第 5 近邻平方距离与 `nearest_max_range` 比较；iVox 的 [KNNPointByCondition](../../include/map_manager/native/ivox/ivox3d_node.hpp#L158)与 `max_range²` 比较。同一 YAML 数值不意味着相同物理查询半径；残差门限等价不能推广为查询范围相同。

## 本次所审源码版本（包含已移出候选者）

PV-LIO-PLUS 核验基线为 `985fd799fa61`。作者基线和本地构建提交分别记录；表内出现不表示通过资格。

| 路径 | 本地 HEAD | 作者基线 / 核验范围 |
| --- | --- | --- |
| `src/R-VoxelMap` | `5166480ebec0` | 作者基线 `50ffbba`；地图头与该基线相同。 |
| `upstream/Surfel-LIO` | `2ed04104a3e1` | 作者基线 `73348c1`；`VoxelMap.cpp`、`Estimator.cpp` 与基线相同。 |
| `src/GenZ-LIO` | `24dce4f497df` | 作者基线 `bf444f7`；实读 `VoxelMap.hpp`、混合观测与剪枝查询，不将实现优化等同论文结构创新。 |
| `src/Hybrid-VoxelMap` | `917ff8020be4` | 作者基线 `89ed71c`；地图头相同，节点仅有消息类型/节点名兼容修改。 |
| `upstream/BIEVR-LIO` | `38f98b900dad` | 地图、高度图残差和采样/注册核心与作者基线相同。 |
| `src/ig_lio` | `a75b362f0852` | 作者基线 `2d79259`；`lio.cpp`、地图实现/地图头与基线相同。 |
| `upstream/lio_gvm` | `c80c968ed7ff` | README、`laserMapping.cpp` 与文件清单，定制地图头仍缺失。 |

## 合格候选的实施边界

下表均未实现新 selector。只有通过结构资格的项目进入下列设计工作；FR/SV 仍为论文储备。

| 地图 | 必须保留的结构与对应观测 / 实施前置条件 |
| --- | --- |
| R-VoxelMap | [内部节点平面与递归残差](../../../R-VoxelMap/include/voxel_map_util.hpp#L640)、链表点组织及离群点下推复用；导入递归地图与概率观测，不能退化为普通 PCA 叶平面。 |
| Surfel-LIO / hVox | [UpdateVoxelMap](../../../../upstream/Surfel-LIO/src/core/VoxelMap.cpp#L281)保存 L1 surfel，[FindCorrespondences](../../../../upstream/Surfel-LIO/src/core/Estimator.cpp#L822)调用 [GetSurfelAtPoint](../../../../upstream/Surfel-LIO/src/core/VoxelMap.cpp#L472)，保留双层统计和更新期预计算；可评估原生自适应 Huber 权重，不移植原 Estimator 循环。 |
| i-Octree | [Octree.h](../../../../upstream/i-Octree/octree2/Octree.h#L535)中的动态树、叶端连续点块、增删/树上降采样；明确空树输出、盒边界、删除后点 ID 与返回计数，点面适配不改变结构资格。 |
| iG-LIO | [Grid](../../../ig_lio/include/ig_lio/voxel_map.h#L27)与 [AddCloud](../../../ig_lio/src/voxel_map.cpp#L87)保存充分统计并增量更新；论文与实现的统计饱和、点数限制和 LRU 须记录。可设计点面变体或 [GICP 观测](../../../ig_lio/src/lio.cpp#L198)，但不能只抽点存储。GICP 需表面协方差、3 维残差和相关信息；Cauchy 的 `rho[2]` 二阶项不能等同简单对角加权。 |
| Hybrid-VoxelMap | [UnionFindNode](../../../Hybrid-VoxelMap/include/voxelmapplus_util.hpp#L123)的 Plane/Gaussian 类型及语义驱动切换；[NDT 残差](../../../Hybrid-VoxelMap/include/voxelmapplus_util.hpp#L707)和[消费端](../../../Hybrid-VoxelMap/src/voxelMapping.cpp#L915)可抽取。须补标签来源、类别转换、缺标签和退化协方差语义；只留普通平面会丢失支撑资格的混合结构。 |
| BIEVR-LIO | [BIEVRMap](../../../../upstream/BIEVR-LIO/BIEVR/include/bievr_lio/bievr_map.h#L14)的定向高度图/像素融合及 [linearize](../../../../upstream/BIEVR-LIO/BIEVR/src/ls_optimizer.cpp#L55)的 `r=p_o.z-I(x,y)`、图像梯度和 Huber。适配共同状态，不调用 `computeTransformation/stepLm` 或惯性后端；[地图引导采样](../../../../upstream/BIEVR-LIO/BIEVR/src/pipeline.cpp#L239)单列实验。 |
| LIO-GVM | 保留全局高斯地图增量融合统计；[h_share_model](../../../../upstream/lio_gvm/src/laserMapping.cpp#L671)调用 `kalmanCalculate`。但 [README](../../../../upstream/lio_gvm/README.md#L9)仍注明 `.hpp` 待更新，[包含处](../../../../upstream/lio_gvm/src/laserMapping.cpp#L62)依赖缺失的 `fast_vgicp/fast_vgicp.hpp`；补齐完整作者实现后才能核对地图/信息模型。 |
| FR-LIO / RC-Vox；SV-LIO | 论文证明结构资格，尚未核验作者实现；分别跟踪双层数组地图、语义引导递归地图的官方源码与可分离性，不进入当前源码移植排期。 |

KISS/GenZ 仅保留历史源码与论文、完整系统基线资料，不再排期它们的 kNN 适配、混合观测或尺度采样。CT、SA 和环境自适应方法保留相关研究；只有取得新的、可核验的结构创新证据后才重新评估地图资格。“作者源码未核验”不等于作者没有发布源码。

**历史版本观察：** 11-14 审核记录 Surfel 远程 `2cbfad9` 的 [GetClosestSurfel](https://github.com/93won/lidar_inertial_odometry/blob/2cbfad923583b44d04d3ab078888bd9fe55450db/src/core/VoxelMap.cpp#L533)使用 KD-tree/PCA，以及 Hybrid 远程 `fb60275` 的[无语义 Gaussian-only 说明](https://github.com/haiyang2022/Hybrid-VoxelMap/blob/fb60275e78e010ef85f5c7574fa330774f714373/README.md)。它们不是本地基线。当前本地 Surfel 查询入口仍是 `UpdateWithLidar → FindCorrespondences → GetSurfelAtPoint`，`GetL1Surfels` 供 viewer 遍历；不得跨版本替换这些事实。

## 未来接口与验收

当前 `PlaneMatch` 与六个 selector 不变。供合格候选使用的 `Plane`、`Surfel`、`Gaussian`、`HeightImage` 等带类型匹配，以及标量/向量残差、状态 Jacobian 和信息接口，均是**未来设计，本轮未实现**。现有滤波使用对角信息，向量观测需处理相关协方差，可评估白化为标量行，不能直接丢掉非对角项或照搬上游状态列顺序。

- **资格验收：** 导入文件、实际数据布局和维护调用链须保留论文支撑资格的结构创新；裁剪/改写须说明保留机制及影响，不能以原项目名包装通用索引。
- **地图 oracle：** 点图在声明的搜索域内对照暴力排序；概率平面、surfel 和高度图按各自的构造/查询规则建立可穷举参照。检查充分统计、层级/合并、像素融合、淘汰和快照，覆盖空图、少于 k 点、等距、负坐标与边界及插入删除。有限邻域 kNN 不得称为全图精确 kNN。
- **每后端观测 oracle：** 固定该后端地图与匹配，按其数学定义/作者消费者核对残差、Jacobian、信息和门限；以符合本项目流形扰动的有限差分验证。覆盖零距离、奇异协方差、缺像素、无匹配和外参开关。主路线不要求跨后端 `h/H/w` 相同，仅统一观测消融检查其等价。
- **噪声与数值：** 明确坐标、单位、平面参数不确定性与散点分布协方差的区别；检查方差/信息有限性、下界和白化二次型一致性。iG 鲁棒二阶项须单独证明等价条件或记录近似。
- **实验真实性：** 先合成、再短回放和长序列，记录数据版本、输入/语义条件、采样、门限、地图预算、观测、有效匹配、时间/内存及 ATE/RPE。源码收集、上游构建、共同框架集成和性能复现分别陈述，不把完整系统论文数字归因于抽取组件。

可机械执行的后续是同步 README/PLAN、论文笔记与源码清单中的资格和状态、补各合格地图的版本/生产者/消费者证据。带类型观测、结构保真移植、状态/噪声映射、i-Octree 生命周期、Hybrid 双结构、BIEVR 高度图、iG GICP 及其关键验证属于高阶设计与实现。源码不完整或边界未核验者继续保持阻断/论文储备状态。
