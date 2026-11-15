# FAST-LIO

论文：[FAST-LIO.pdf](../FAST-LIO.pdf)，arXiv v3（2010.08196v3），*FAST-LIO: A Fast, Robust LiDAR-Inertial
Odometry Package by
Tightly-Coupled Iterated Kalman
Filter*。作者项目：[hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)；该仓库后续主线以
FAST-LIO2 为主。

## 问题与核心方法

FAST-LIO 面向高速运动、非重复扫描和退化几何下的 LiDAR-inertial odometry。其核心是紧耦合迭代误差状态 Kalman filter：IMU
提供高频传播和扫描内运动补偿，LiDAR
几何特征直接参与滤波更新，减少松耦合方法中先独立求位姿、再融合 IMU 的误差（pp. 1–2）。

### 点级去畸变和滤波更新

扫描点按采样时刻记录。作者从扫描末端状态和 IMU 轨迹向后传播，估计每个点采样时的相对位姿，再把该点变换到统一扫描时刻（pp. 3–4）。论文使用 18
维状态，包含姿态、位置、速度、陀螺仪/加速度计偏置和重力；LiDAR-IMU
外参已知，不作为在线估计状态。迭代时对非线性 LiDAR 观测重新线性化。

LiDAR 约束由局部平面/特征匹配产生，构成观测残差及 Jacobian。作者针对测量数量远大于状态维度的情形，推导迭代 Kalman
增益的等价计算式，使主要矩阵运算维度由状态维数决定，而不是直接对巨大的测量维矩阵求逆（pp.
3–5）。

## 地图与更新

本文的重点是估计器、点级反向传播和高效更新公式，不是一个独立的新地图表示。系统逐帧把估计后的特征点并入地图，并用地图特征支持后续扫描匹配；不能把 FAST-LIO 与 FAST-LIO2 的
raw-point ikd-Tree
设计混为一谈。FAST-LIO2 的地图论文另见 [FAST-LIO2 笔记](FAST-LIO2.md)。

## 作者实验

论文报告计算复杂度/处理时间，以及无人机飞行、室内和室外实测；主要指标包括运行时间、轨迹/定位误差和高速运动下的稳定性（pp. 6–8，Tables II–III、Figs.
4–11）。实验用于验证紧耦合迭代滤波、运动补偿和计算公式在实际
LiDAR/IMU 数据上的表现。原论文没有把某一种可独立替换的地图后端作为主要消融对象。

## 2026-11-15 对 PV-LIO-PLUS 的范围说明

- FAST-LIO 论文主要描述滤波、IMU 传播、点级去畸变和特征匹配；其完整系统可作为独立端到端方法理解。
- PV-LIO-PLUS 的主路线共享自己的同步、去畸变、状态与滤波循环，比较地图及对应观测；完整 FAST-LIO 流程不作为隐藏的 `MapManager` 后端。
- FAST-LIO 与 FAST-LIO2/ikd-tree 的方法和地图来源不可混称；本笔记不表示新接入了 FAST-LIO 估计器。

## 局限

基于特征的匹配需要可靠的平面/边缘筛选；密集原始点的直接匹配和动态增量树结构属于后续 FAST-LIO2 的设计。论文中的时延和精度绑定其传感器、采样和平台，不能直接与不同论文的数字横比。
