# KISS-ICP / VoxelHashMap

论文：[KISS-ICP.pdf](../KISS-ICP.pdf)，IEEE RA-L 2023
作者预印本（[DOI](https://doi.org/10.1109/LRA.2023.3236571)），*KISS-ICP: In
Defense of Point-to-Point ICP — Simple, Accurate, and Robust Registration if
Done the Right
Way*。作者项目：[PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp)；本地代码：[VoxelHashMap.cpp](../../../../upstream/kiss-icp/cpp/kiss_icp/core/VoxelHashMap.cpp)。

## 问题与完整流程

论文讨论点到点 ICP
在合理运动预测、扫描降采样、局部地图、鲁棒核和自适应数据关联阈值配合下，也能形成准确稳健的纯激光里程计。系统用恒速模型预测扫描间运动，点云 deskew
后进行体素采样，再用
point-to-point ICP
估计位姿（pp. 1–5）。这不是 LIO 系统。

## VoxelHashMap 地图操作

哈希地图以体素坐标作为键，每格维护有限数量的地图点。匹配时只检查查询点所在格周围固定邻域（27
个体素），由候选返回最近点和距离。每帧注册完成后，将新点按格加入地图；已达到容量的格不继续增长。地图通过当前位置和最大范围裁剪远处体素（p.
4）。

本地实现的核心操作为 **AddPoints、RemovePointsFarFromLocation 和
GetClosestNeighbor**。最近邻接口返回单个最近点，不提供 k
个邻点、平面估计或协方差。因此该接口本身不包含点面拟合所需的邻域点集；此处记录接口差异，不提出当前集成任务。论文讨论的仍是点点 ICP。

## 作者实验

论文在 KITTI Odometry、MulRan、Newer College 和 NCLT 等数据上测试，以相对/绝对轨迹误差、运行速度及 CPU
处理时间评价精度与实时性；另有自适应阈值、采样和运动补偿消融（pp. 6–7，Tables
II–VI）。作者报告同一组参数覆盖多种传感器/运动条件，并在所测数据集上超过传感器帧率运行。各数据集和指标不同，不能把数字横向当作统一基准。

## 2026-11-15 对 PV-LIO-PLUS 的范围说明

- VoxelHashMap 源码已收集并有 C++ 静态库构建记录；论文与源码作为系统资料归档，不进入当前地图候选或移植排期。
- 论文使用既有 voxel hash，最近点/kNN 查询或点面/点点残差改动不构成新地图结构创新。KISS 的 ICP 优化器、恒速运动/去畸变和自适应阈值仅按完整系统方法理解。
- 27 格查询只覆盖有限邻域，淘汰按体素首点距离判定；不称为全图精确 kNN 或精确球形裁剪。构建不表示 LIO 或精度复现。

## 局限

地图只提供最近点，局部候选受到固定体素邻域限制；点面适配会改变原生点点目标和查询负载。运动预测与自适应阈值是 KISS
完整系统结果的重要部分，抽取地图后不能继承完整系统的论文指标。
