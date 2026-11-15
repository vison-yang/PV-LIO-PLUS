# i-Octree 接入设计与阻断审计（只读，2026-11-15）

## 范围、状态和证据

- 比较对象：`upstream/i-Octree` 的作者版本 `5b640e05`，核心为 `octree2/Octree.h`。本地 HEAD `25903a33` 与该版本仅有顶层 CMake 比较依赖差异，核心头文件相同；不要引入示例里的 ikd-Tree、Python/Torch 绑定或独立 LIO。
- 主线审计基准 `src/PV_LIO_PLUS` 为 `f25cfcf`，后续代码应以主 agent 整合 hVox 之后的版本为准。本次没有编译、运行或改动任何源码/feature 分支；以下为静态证据及待验证设计，不是回放或正确性结论。
- 结构资格：节点为 `Octant`，叶的坐标按 `dim=4` 的连续 `float[]` 段存储，通过指针向量索引（`Octree.h:366–428,1321–1368`）；`update` 按需扩根、按八分路由，局部裂分/拷贝（`601–707,1370–1451`）；`boxWiseDelete` 原地删除子树或压缩叶段（`1704–1821`）。必须调用并保留这些机制，不能改成以历史数组或临时 KNN 代替地图。
- 许可事实：仓库 README 明示 GPLv2、仓库 `LICENSE` 是 GPLv2 原文；`octree2/Octree.h:4–22` 自身包含完整 MIT 许可文字与作者署名。导入时保留头文件全部原始声明及仓库 GPLv2 LICENSE，`SOURCE.md` 如实同时记录，不自作法律结论或静默删除任一声明。仓库 `examples` 还依赖独立的 GPLv2 ikd-Tree；不复制该示例依赖。源内调试例程含作者机器的绝对路径（例如 `Octree.h:443–456,940–954`），提取核心时移除写本机路径的调试功能并记录差异，不复制这些路径。

## 阻断：浮点树盒不一定包住其点

`initialize` 的根中心/半宽使用 float（`Octree.h:543–592`），`createOctant` 继续以 float 计算子中心/半宽（`1321–1354`），`update` 以 float 扩根（`653–697`）；半径整子树快速收集/近邻剪枝依赖这些盒（`1453–1475,1505–1531,1610–1656,1823–1900`）。静态反例：x 为 `{16777216.0f,16777218.0f}`、y=z=0，按 IEEE round-to-nearest，根 half-extent=1，而 `16777216+1` 舍回 `16777216`；第二点距中心 2，大于根 half-extent。以第一点、r=1.9 查询时，`contains` 可认为半宽为 1 的根立方体被球包含而整叶返回，错误包含真实距离 2 的第二点。**这由固定源代码直接推导，尚未执行复现。**

实现和发布精确 kNN/radius 前，须由主 agent 组织定向数学/工程审查：证明根、子盒及扩根后对所有存储点的保守包围；确保重叠裁剪、`inside` 提前结束和 `contains` 整树收集对边界舍入均安全，且更新后不破坏不变量。仅检测 finite/float 可表示或声称“小范围坐标足够”都不能解决一般缺口。

候选方向，由该审查选定后落地，不预判哪一种最小/已经正确：

1. **保留 float 点、将树几何/包围盒及裁剪计算改为 double 或保守区间。** 必须覆盖初建、递归子盒、扩根、插入/删除、box 查询、根和叶所有接口；比较时仍明确 native 点坐标及输出距离的 float 舍入。注意 double 自身也有有限精度，float 数据经过 midpoint、递归缩放和扩根时，不能仅凭改类型就宣称证明完成。量化额外节点内存、速度和实际树形差异。
2. **每节点维护真实数据的保守 AABB，搜索剪枝及盒删除的整子树包含/跳过均使用该 AABB；必要时禁用不能证明安全的提前结束/整树收集，叶端逐点筛选。** 保留八分路由和连续叶段，但增加每节点内存及插删维护；注意兄弟包围盒可能重叠，`inside` 不能假定其他兄弟不相交。不得用全量点集重建或线性 KNN 冒充该结构。
3. **保守表示条件 + 明确失败**：只在能证明所有子划分及扩根覆盖的情况下使用路径，否则拒绝操作且拒绝前不改变原树，或使用经过证明的局部修复。需说明有效坐标域、故障报告及性能；不能静默缩小输入域后继续宣称一般输入精确。

无论选哪条，浮点上溢、下溢、深度、扩根不终止、搜索度量一致性均需显式检查；相关树不变量与穷举/随机 oracle 通过前不接真实数据回放。

## 最小后端落地方案

独立 worktree 以整合后的主线为基准。建议增 `native/i_octree/Octree.h` 和 `SOURCE.md`，将作者头适度提取、命名空间隔离、补自包含标准头；在 `MapManager` 增枚举 `IOctree`、selector `i_octree`（别名 `ioctree`/`i-octree`）和 `unique_ptr`，仅加私有 `initialize/update/search/erase` 分支。`Mapping` 保持公共传播、预处理、IKFoM 循环，走 `ObservationModelPointBackend`；若主线已增加 hVox 的可选上下文，本后端忽略上下文，不另造公共地图接口。

- 生命周期：先完整验证批次坐标及所需数值域，再以 world XYZ 的 float 值 `initialize`/`update`；空初始化返回可用空树，空更新为 no-op。有效初始化标记沿用公共契约；`clear` 必须释放树并重置全部计数，允许空图重复 clear、删光后 update、重新配置。地图只保存原生树中的 XYZ 值；别累积 `map_points_` 历史。`snapshot()` 从当前叶值复制，`size()`/`point_count()` 定义为当前物理存储点数，要求 O(1) 正确，供计时 CSV 使用。
- 默认建议沿作者头的 bucket=32、`min_extent=0.01` 米、`downsample=false`（`Octree.h:485–507`，作者比较示例 `examples/time_compare.cpp:308,322,357` 使用默认 update），提供 `mapping/i_octree_bucket_size`、`mapping/i_octree_min_extent`、`mapping/i_octree_on_tree_downsample`，检查正数、有限及实现所需上限。`min_extent` 是算法内部 half-extent 参数，不承诺统一固定栅格边长。`nearest_point_count`、`nearest_max_range`（米，默认5）、`plane_fit_threshold` 沿用公共参数；需额外验证 radius 转 float 后及其平方仍有限、非零。
- **树上降采样不是外部体素均值滤波**：作者仅在既有叶且 `extent <= 2*min_extent`、原叶点数 `> bucket/8` 时整体拒绝该次发往此叶的新点（`1405–1409`）；单批可超过阈值，初建、空树 update 及新建子树不执行同样拒绝（`601–607,1437–1444`），`update` 的参数还会覆盖 setter 状态（`609`）。每次 update 显式传配置开关；保留上述真实语义并记录，测试开关而非只测关闭。
- 核心计数必须修补：`last_pts_num` 在 update 路由前按尝试数增长（`617–630,699–703`），丢弃的降采样点不计入 `pts_num_deleted`；`clear` 只删 root 而未清计数（`711–715,891–894`），删光后重新 initialize 还可能使无符号 `size()` 下溢。建议使递归 update 返回本次实际接收/丢弃数，物理删除返回真实删除数；分别维护尝试 ID 与 live count，`size()` 返回 live count，任何 clear/reinit 一致复位。测试核对 live count = 叶段中坐标多重集大小。先证明计数再使用 O(1) telemetry。
- 输入约束只影响新分支：拒绝 NaN/Inf 和不能安全转换的坐标、参数，拒绝修改前完成校验；查询的无效点按 no-match 处理，不能将非有限 plane、h/H/R 传进滤波。`UpdateMap` 的 covariance 排序在调用 MapManager 前执行（`mapping.cpp:820–834`），新分支须在排序前剔除/拒绝非有限输入，避免 NaN comparator；不要修改旧后端。
- 新结果 stem `pv_lio_i_octree`，脚本只登记 backend/aliases，原默认六项保持不变。记录原生节点个数与占用点数的区别，输出不能声称完整作者 LIO 或精度提升。

## 查询、身份、盒删除与窗口契约

- `knnNeighbors` 的结果集 `dist_index_` 长度是容量 k，实际条数为 `heap.size()`（`Octree.h:199–255`）；三个公开重载正确只复制实际条数，却返回 `data.size()` 即 k（`776–861`）。非空图少于 k 时返回值错误；空图直接返回 0 且**不清已有输出**（`779–780`）；k=0 在 `KNNSimpleResultSet::clear` 下标为 `capacity-1` 越界，负 k 可转超大无符号。修补公开重载：先清输出，k=0 返回空，k<0 报错，返回实际有效数量并确保点/距离数组长度相等；大 k 可按准确 live count 限制分配。仅使用“返回坐标值”的重载，复制 float XYZ，不缓存内部 `float*`。距离为 float 平方米、非递减，等距项不保证身份次序。
- 点面适配执行全树 native kNN（不是受邻格限制的近似 KNN），使用前 `NUM_MATCH_POINTS=5` 个有效近邻；第 5 个距离满足 `d² <= nearest_max_range²` 才拟合，即公开 kNN 门槛含边界，与当前 ikd 适配一致，先平方以保持米语义。少于5、零匹配返回 no-match；`nearest_point_count` 至少5。调用现有 `fit_point_plane` 的共同适配，不冒充作者原生观测或附加 ikd/iVox 专有 score gate。`common_lib.h:342–373` 的退化 QR 解可产生 NaN，而 `fabs(NaN)>threshold` 为 false，导致返回 true；仅在 i-Octree 分支检查输出法向、d、distance、center、查询及所需协方差均 finite 且法向非零，失败 no-match。观测权重仍用共享 point backend，需单独说明。
- 独立 radius 接口按 native **严格** `d² < r²`（`Octree.h:1483–1491,1533–1549`），参数为正的米，距离平方、输出未排序；这与上面的 kNN 拟合门槛含边界是两个明确契约。无效半径/非有限输入不得隐式变成正半径或默认为单位值。`contains` 快路径须先完成上节不变量审计。
- `clear_data=false` 禁用：部分相交叶被整叶标 inactive（`1740–1747`），并非精确点删除；`get_leaf_nodes`/`get_data` 不过滤 inactive（`920–971`），radius 的 contains 分支也可把失活后代返回，后续 `updateOctant` 又会激活节点（`1375`）。仅开放 `boxWiseDelete(...,true)`，对原生盒内 `[min,max]` 含边界物理删除；保留空盒、整树、局部叶压缩的路径。
- 公共 `erase_outside_window`/`Inside` 保留窗口闭区间（主线 `map_manager.cpp:1587–1590`）。首选使用六次原生物理删除覆盖窗口补集，而非复制/重建全图。树中坐标已量化为 float，按这些存储值与 double window 精确比较：下侧上界取“严格小于 window.min 的最大有限 float”，上侧下界取“严格大于 window.max 的最小有限 float”，可根据 cast 后与 double 的比较及 `nextafterf` 得到；跨 float 极值的空侧直接跳过、全域外窗口删光；另两轴用覆盖有效 float 域的端点，验证 finite、min<=max。六盒可重叠，计数必须按每次实际删除数更新，不能将窗口边界点删掉。
- 备选：增加 native 单次 `keep-window` 树上递归复用其整节点释放和局部叶段压缩；若安全剪枝的盒不变量尚未证明，则遍历叶并用原始 float 点与 double 边界比较，保留树结构、原地修改，不以整图重建实现。对比六盒的实现复杂度和性能，由主 agent / 数学审查确定；两者都依赖安全所有权与正确 live count。
- 作者把连续点存为 `[x,y,z,float(id)]`（`Octree.h:551–556,625–630`）。过滤 NaN 后 id 是压缩序号，删除不重排，clear/重建可重用，float 超过 `2^24` 不再逐整数精确；**禁止把它当持久索引、禁止 ID 重载作为生产查询接口**。固定源中的 ID 可用于小规模原生回归验证和多重集诊断，唯一身份若未来确需，则另立设计。任一 update/删盒/clear 可释放或移动叶连续块：无裸指针、引用或 native 点 ID 跨调用；点值和快照复制拥有自身生命周期。
- native 树没有互斥锁；共享主循环当前先串行 `map_manager_->search`，再并行写独立观测行（`mapping.cpp:453–483`）。禁止与插删/clear 并行，初版也不对外承诺并发只读；未来需要并发时另做锁/快照和 TSAN 审查。

## 独立验证与交付门槛

1. 原生确定性及随机参考集：保留以 float 量化的 `std::vector` 多重集作测试 oracle；逐操作（构建、批量插入、重复点、跨八分面、扩根、物理盒删除、clear/reinit、降采样开关）比较叶点坐标多重集、live count、全部祖先保守包围、每叶连续段。固定 RNG seed 并输出失败 seed；oracle 禁止复用树的分区、剪枝或计数实现。降采样启动与整批接受/拒绝的真实语义用专门手工序列断言。
2. kNN 与 radius 对独立暴力距离排序/过滤：空图预填哨兵输出、k=0/k<0、k=1/少k/等k/多k、非常大k、重复与等距、负坐标、分割平面、根/子盒边界、两轴零宽。对等距只约束距离与可接受多重集而不假设排序；边界使用精确表示的坐标及 `nextafter`。分别核验 kNN 含半径界、native radius 严格界、距离单位、实际数量、尾部不读、旧输出清空。数值 oracle 先对输入按 float 量化；非边界点可用独立 double 参考并合理设公差，避免把被测遍历复制成 oracle。
3. 明确复现上节 `2^24` 根盒反例及正负大平移、相邻 float ULP、根生长至多个方向、深树、近切半径与裁剪盒接触；验证所有存储点真正被祖先盒保守包含。只有选定的修复及极值/随机 oracle 均通过才解除阻断；额外审查 `inside` / `contains` 的数值比较及所有权。
4. 原生闭盒/公共窗口：窗上/下边界点保留，严格外侧点删除，窄窗无可表示 float、原点与负坐标、空图/删全部/重复删/删后重插；校验树快照而非历史 `map_points_`。ID 测试标明仅对 <=`2^24` 的原生临时序号，构造过滤/删除/重建和重复坐标，不把 ID 绑定为持久身份；追加 float 精度边界反例说明。
5. MapManager 空/少五/无匹配/五点平面、退化全零与原点平面、范围边界、非有限和重复初始化，比较共享观测与已知几何，保持旧后端回归（含 R 和 hVox）不变。所有 native 物理删除及局部重分配路径启用 Eigen 断言和 ASan/UBSan；LSan 单独报告环境支持状况，不把关闭 LSan 称为通过。不会声称已验证线程安全。
6. 隔离 build/devel、`-j1`，固定所选 Eigen 头路径与 ROS overlay，真实 node 构建、合成回归、唯一输出 stem 检查通过后，主 agent 再做输入覆盖/输出进度可审计的同段 ROS 回放及性能比较。保留 source diff/patch 清单与运行日志；浮点树盒安全或退化观测未解决时不得合并为可用后端。

## 与 i-Octree feature 分离的既有并发缺陷

主线 `mapping.cpp:479–528` 的 `ObservationModelPointBackend`（ikd-tree/iVox）、`:578–612` 的 OctVox、`:665–704` 的 C3P 在 OpenMP parallel for 中对成员 `total_residual_ += ...` 无同步，构成数据竞争。`res_mean_last_` 当前在此文件只被赋值，未发现影响滤波的读取，但竞争本身不能忽略。VoxelMap / VoxelMapPlus 的并行循环不累加该成员，其均值当前恒零，是另一个诊断现象，不应顺带变更；R-VoxelMap 的累加为串行（`:711–755`）。此审计基于当前主线，不覆盖尚未合入的其他 feature。

建议独立 fix 提交：三个受影响分支均移除循环内成员累加，并在并行循环后按行序串行求 `sum(abs(ekfom_data.h(i)))`，再赋 `total_residual_`/`res_mean_last_`。这样 h/H/R 的每行计算不变，诊断求和顺序稳定；不同于 OpenMP reduction 的合并顺序可能改变浮点末位。构造确定残差向量比较串行均值、1/多线程产物及原 h/H/R，视工具链支持进行定向线程检查；必要短回放仍须核对输入覆盖。此修复由主 agent 单独派发，不进入 i-Octree feature。

## 可分派包

- 高阶实现包：确定并证明浮点 bbox/裁剪修法；实现上述 native 内存、计数、查询、物理删除、共同 MapManager/观测接线、接口行为和针对风险的核心 oracle。明确所选安全域、修改与原生行为差异。
- 可交超级牛马：在接口和算法冻结后，按固定测试矩阵补可重复 fixtures/脚本、登记 selector/stem、隔离构建/日志归档、跑旧后端和受控回放、同步来源与结果文档；发现 bbox/所有权/并发新问题时停止并回传，不自行决定结构改动。
