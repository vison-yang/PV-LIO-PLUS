# iG-LIO / incremental VoxelMap

论文：[iG-LIO.pdf](../iG-LIO.pdf)，IEEE RA-L 2024 作者 early-access
版本（[DOI](https://doi.org/10.1109/LRA.2024.3349915)），*iG-LIO: An Incremental
GICP-based Tightly
Coupled LiDAR-Inertial
Odometry*。作者项目：[zijiechenrobotics/ig_lio](https://github.com/zijiechenrobotics/ig_lio)；本地实现：[src/ig_lio](../../../../src/ig_lio)，地图接口
[voxel_map.h](../../../../src/ig_lio/include/ig_lio/voxel_map.h)。

## 问题与完整系统

iG-LIO 在紧耦合 LiDAR-IMU 估计中使用 Generalized ICP，以 LiDAR
点及局部表面协方差建模扫描约束。作者也提出增量体素地图，目标是降低点云近邻/表面统计的计算和内存开销（pp. 1–2）。完整方法的
GICP、VSCE 及状态更新不等同于 PV 的标量点面观测。

## 增量体素地图

地图按体素哈希组织。每个 Grid 持有有限容量的点数组并可保存该格几何协方差；AddCloud
按输入点坐标定位格并追加/更新点集。KNNByCondition
遍历查询位置附近配置的体素，从满足距离条件的格内点中选择候选近邻（pp.
2、5；本地符号见上述头文件和 [voxel_map.cpp](../../../../src/ig_lio/src/voxel_map.cpp)）。

完整 iG-LIO 还用采样点和表面协方差形成 GICP 约束。PV 侧若仅抽地图，不能将 Grid
的散点几何协方差直接解释成点面参数不确定性；需要检查近邻数量、排序并用公共平面拟合适配器生成法向和标量残差。

## 作者实验

作者比较每帧各处理模块耗时，并在 Avia、GDUT、NCLT 等数据上报告轨迹误差、运行时间、鲁棒性及地图分辨率/约束形式消融（pp. 5–7，Tables
II–IV、Figs.
3–6）。指标包括运行时、ATE/RMSE
类轨迹误差和不同场景的完成情况。作者报告该完整 GICP 系统相较 Faster-LIO 在其配置下效率较高、精度与已发表 LIO 相当；这不能归因于孤立点图。

## 2026-11-15 对 PV-LIO-PLUS 的范围说明

- 作者源码在 `src/ig_lio`，ROS1 共享包有编译证据；地图尚未接入 `MapManager`。移植须保留增量充分统计和原生地图维护；可先评估相应点面观测变体，不能只抽取普通 hash 点列表。
- GICP 是独立开放的观测设计，不因其残差不同而排除；需检查两侧表面协方差、3 维残差、完整信息矩阵和 Jacobian。`Grid::cov_` 不能直接解释为平面参数协方差。
- 原 Cauchy 权重包含 `rho[2]` 二阶项；相关 3×3 信息和鲁棒二阶项不能直接压成一个标量对角 `R` 并声称等价。白化方案属于未来设计，须保留二次型并验证。完整 iG 优化循环与自适应扫描另行处理。

## 局限

体素尺寸、格容量和邻域配置共同决定查询覆盖和细节保留；GICP
的表面协方差在该论文中服务分布匹配，脱离该目标不能直接复用。真实数据覆盖和指标仍受论文传感器与平台限制。
