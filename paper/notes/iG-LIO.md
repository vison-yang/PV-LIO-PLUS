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

## 对 PV-LIO-PLUS 的迁移边界

- 范围审查把 iG-LIO 的 incremental VoxelMap 列为新增点图候选；可抽取有界体素点集、增量 AddCloud 和邻域候选查询。
- 排除 VSCE、GICP 分布匹配、iG-LIO 的 IMU/状态更新和自适应扫描处理；统一使用 PV 点面适配器、观测残差及权重。
- 本地 KNN 查询结果可能不足 k 且只部分排序；适配器应验证数量并统一排序。地图协方差不能直接套到平面参数协方差。
- 本地上游包可供阅读和局部抽取，但尚未形成 PV 后端；笔记不声称本地已有该后端。

## 局限

体素尺寸、格容量和邻域配置共同决定查询覆盖和细节保留；GICP
的表面协方差在该论文中服务分布匹配，脱离该目标不能直接复用。真实数据覆盖和指标仍受论文传感器与平台限制。
