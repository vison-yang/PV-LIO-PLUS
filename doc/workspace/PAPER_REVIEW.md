> 历史文献审校记录：以下保留条目已在对应笔记修复并复核。位置与行号描述审校时的文件；当前框架接入状态见[集成状态](../INTEGRATION_STATUS.md)。

修复文件：[VoxelMap++](../../paper/notes/VoxelMap++.md)、[ikd-Tree](../../paper/notes/ikd-Tree.md)；i-Octree 的精确 kNN 补注也已复核。

# 论文笔记实质审校

原审校日期：2026-11-14。以下保留条目依据工作空间 `output/maintenance-20261114/paper-texts/*.txt` 中对应 PDF 的逐页文本核对；该次审校未编译、未联网。

以下记录当前资料中已修复的两组实质问题，行号以原审校时为准。

## 审校修复记录

1. **VoxelMap++：3 参数的具体表示及合并公式需要写准。**
   - 位置：`src/PV_LIO_PLUS/paper/notes/VoxelMap++.md:16–23`。
   - 问题：论文不是以单位法向的二维切空间加偏移直接作为参数；其主轴归一化参数为 `[a,b,d]`，例如 `ax+by+z+d=0`，法向为未归一化的 `[a,b,1]`。合并也不是把子格原始点的累积求和统计直接汇成父平面。
   - 准确替代：**“论文以主轴归一化的 `[a,b,d]` 表示平面（例如 `ax+by+z+d=0`）；必要时更换主轴，查询时再将法向与距离归一化。单格拟合可增量缓存求和量。收敛后，以相邻平面的 3 参数估计及其协方差作为父平面的测量，经 Mahalanobis/χ² 检验决定是否合并，再按协方差迹构造加权平均及合成协方差；union-find 组织共享父平面，并将最大深度剪枝至 2。”**
   - 证据：`paper-texts/VoxelMap++.txt`，PAGE 3，§III-B、Eq.(4)、(6)–(8)；PAGE 4，§III-C3、Eq.(9)–(11)、Algorithm 1；PAGE 5，Eq.(13)。

2. **ikd-Tree：作者实验混入了 FAST-LIO2 的传感器和误差评估。**
   - 位置：`src/PV_LIO_PLUS/paper/notes/ikd-Tree.md:31–33`。
   - 问题：固定的 arXiv v1 仅有 Table I–II。其真实应用是单个 Livox Avia 100 Hz 室外 FAST-LIO 建图实验，没有机械式 LiDAR 对比，也没有 Tables II–V、轨迹误差/端点误差表。“长时间建图”也未在该版本作为独立实验验证。
   - 准确替代：**“随机点实验检验持续插入/盒删除时的更新与查询时间，以及稀疏、紧凑增量分布的影响；室外应用将 FAST-LIO 的静态 k-d tree 替换为 ikd-Tree，使用 Livox Avia 100 Hz 数据，在 i7-8550U / 8 GB 的 DJI Manifold 2-C 上报告扫描融合、近邻查询、增量更新与重平衡耗时，并展示建图结果（pp. 6–8，Table II、Figs. 4–7）。该版本未给轨迹/端点误差量化。”**
   - 证据：`paper-texts/ikd-Tree.txt`，PAGE 6，§V-A、Table II；PAGE 7–8，§V-B、Figs. 6–7。全文表名检索只有 Table I、II。

## 已核验通过的重点

- **FAST-LIO**：当前笔记正确标明 v3 的 18 维状态与已知 LiDAR–IMU 外参，未混入 FAST-LIO2 的在线外参状态；依据 PAGE 3，Eq.(1)–(3)。
- **Faster-LIO**：当前论文标题已与 PDF 首页一致；6/18/26 邻格、近似 kNN、LRU 和 PHC 的归属正确。
- **VoxelMap、FAST-LIO2、i-Octree、iG-LIO**：概率平面/动态点索引/GICP 的主要归属及迁移排除关系未发现阻断问题。i-Octree 原文 §II-C 明确是保留点集上的精确 kNN；其分辨率或树内下采样会改变保留点集，宜避免把这一影响理解成 iVox 式有限邻域近似。

本报告仅核验上述笔记的实质表述与本地论文来源，不构成实现正确性或论文性能复现结论。
