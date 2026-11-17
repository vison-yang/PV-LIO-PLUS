# iG-LIO / incremental VoxelMap

论文：[iG-LIO.pdf](../iG-LIO.pdf)，IEEE RA-L 2024 作者 early-access
版本（[DOI](https://doi.org/10.1109/LRA.2024.3349915)），*iG-LIO: An Incremental
GICP-based Tightly
Coupled LiDAR-Inertial
Odometry*。作者项目：[zijiechenrobotics/ig_lio](https://github.com/zijiechenrobotics/ig_lio)；作者固定提交为 `2d79259e3aaa5bbb10d6b13234398760bbacf8f5`。当前完整 `src/ig_lio` checkout 已缺失；本包基于固定作者源码重实现地图核心于 [`include/map_manager/native/ig_lio`](../../include/map_manager/native/ig_lio)，不是逐字节导入整个上游包。适配和来源见 [`SOURCE.md`](../../include/map_manager/native/ig_lio/SOURCE.md) 与 [`UPSTREAM_MANIFEST.json`](../../include/map_manager/native/ig_lio/UPSTREAM_MANIFEST.json)。

## 问题与完整系统

iG-LIO 在紧耦合 LiDAR-IMU 估计中使用 Generalized ICP，以 LiDAR
点及局部表面协方差建模扫描约束。作者也提出增量体素地图，目标是降低点云近邻/表面统计的计算和内存开销（pp. 1–2）。完整方法的
GICP、VSCE 及状态更新不等同于 PV 的标量点面观测。

## 增量体素地图

地图按体素哈希组织。当前导入并接入的 `ig_lio_plane` 保留每格增量充分统计
`N`、`S = Σp` 和 `Q = Σppᵀ`，并独立保留用于查询/快照的点数组。对 `N ≥ 6`，
令 `μ = S/N`，按 `C = (Q − S μᵀ)/(N − 1)` 形成样本矩阵，作 SVD
`C = UΣVᵀ`，再构造 `C_t = U diag(1, 1, 1e−3)Vᵀ` 及其逆。这个修正矩阵在有限精度下不保证总是 SPD 概率协方差；适配不把它写进公共 `plane_cov`。

批更新规则保留了作者地图的行为：只有旧 `N < 50` 时才批量更新统计，因此一批可跨过 50 并使后续统计冻结。已有格仅在更新后的 `N < grid_max_points` 时追加整批点；新格会完整保留首批点。因此 `grid_max_points` 不是严格的单格点数上限。

查询支持 1/7/19/26 邻域模式，其中 26 表示中心格加 26 个相邻格，共 27 格。候选点以严格 `dist² < range²` 筛选，排序后取前五点交给共享五点平面拟合与标量点面观测。iG 地图的 query 不刷新 LRU；update 会刷新。插入后若 `size() >= capacity` 即逐出一格，所以稳态活跃格数可能为 `capacity−1`。此适配拒绝几何 `local_window`。

Cell key 保留完整有符号 32 位三元组；哈希使用定义良好的无符号回绕，避免有符号溢出。批处理的确定顺序为旧回绕哈希模容量、tuple、输入下标。插入坐标为 float，查询与 key 检查使用受限 double 精度；整批输入在修改前预检，clear/reinitialize 先构造替换状态，单格失败提供 basic allocation guarantee。完整实现和来源见上述导入文件及其来源清单。

完整 iG-LIO 还用采样点和表面协方差形成 GICP 约束。PV 的 `ig_lio_plane`
选择器刻意保留地图、但采用公共平面拟合器和标量点面残差；它不是原生 GICP。Grid
的散点几何协方差不能直接解释为点面参数不确定性，`C_t` 也没有发布为
`plane_cov`。

## 作者实验

作者比较每帧各处理模块耗时，并在 Avia、GDUT、NCLT 等数据上报告轨迹误差、运行时间、鲁棒性及地图分辨率/约束形式消融（pp. 5–7，Tables
II–IV、Figs.
3–6）。指标包括运行时、ATE/RMSE
类轨迹误差和不同场景的完成情况。作者报告该完整 GICP 系统相较 Faster-LIO 在其配置下效率较高、精度与已发表 LIO 相当；这不能归因于孤立点图。

## 2026-11-16 对 PV-LIO-PLUS 的范围说明

- 当前完整作者 checkout `src/ig_lio` 不存在；固定提交、历史完整包构建与当前重实现文件分别记录于[工作空间来源清单](../../doc/workspace/SOURCE_MANIFEST.json)、[本地来源说明](../../include/map_manager/native/ig_lio/SOURCE.md)和[上游文件清单](../../include/map_manager/native/ig_lio/UPSTREAM_MANIFEST.json)。2026-11-14 的共享 ROS1 build 是旧完整 checkout 的证据，不表示当前完整作者包仍在本地或仍可构建。
- `ig_lio_plane` 保留 iG 增量充分统计和批次/容量维护规则，使用共享五点 float QR 标量点面观测。Round 2 node build/link、九个指定构建目标、CTest 9/9（65 个 GTest 条目，含 11 个新 iG 用例，ASan 关闭）及配置合成测试通过；该 11-16 Round 2 当时没有回放。Round 1 初次 make 因目标列表陈旧未识别新目标，单目标重试又发现测试缺少 GTest `main`；修正后 round 2 通过。新增 config 成员与 `unique_ptr` 改变类布局，验收必须全量重编所有消费者。结果与失败轮次记录见[验收结果](../../doc/workspace/evidence/IG_LIO_RESULTS.json)。
- GICP 是独立开放的观测设计，不因其残差不同而排除；需检查两侧表面协方差、3 维残差、完整信息矩阵和 Jacobian。`Grid::cov_` 不能直接解释为平面参数协方差。
- 原 Cauchy 权重包含 `rho[2]` 二阶项；相关 3×3 信息和鲁棒二阶项不能直接压成一个标量对角 `R` 并声称等价。白化方案属于未来设计，须保留二次型并验证。完整 iG 优化循环与自适应扫描另行处理。

当前 `config/backends/ig_lio_plane.yaml` 默认使用 `nearest_point_count=5`、
`nearest_max_range=5 m`、`plane_fit_threshold=0.1`、`ig_lio_resolution=0.5 m`、
`ig_lio_search_method=7`、`ig_lio_capacity=5,000,000` 和
`ig_lio_grid_max_points=20`。配置覆盖顺序、参数约束和 launch / batch 用法见
[配置说明](../../config/README.md)。

## 局限

体素尺寸、格容量和邻域配置共同决定查询覆盖和细节保留；GICP
的表面协方差在该论文中服务分布匹配，脱离该目标不能直接复用。真实数据覆盖和指标仍受论文传感器与平台限制。

2026-11-17 另已通过 30 秒 iG-LIO/iG-LIO+GM 回归，以及默认权重 197.190 秒支持窗回归（1959 帧）；完整 GICP 仍未接入，GM 长窗亦未执行。见[最新验收](../../doc/workspace/evidence/MAP_ACCEPTANCE_20261117.json)及[短回归](../../doc/workspace/evidence/INTEGRATION_PROGRESS_20261117.json)。
