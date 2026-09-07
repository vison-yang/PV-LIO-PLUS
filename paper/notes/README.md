# PV-LIO-PLUS 相关论文笔记

## 说明与阅读边界

本文整理 `paper/` 中当前收录的 12 篇论文，重点关注对 PV-LIO-PLUS 可能有参考价值的三个层面：

1. 地图怎样表示：点、平面、Gaussian、surfel、连续的高度图，或语义增强的混合表示。
2. 观测怎样进入滤波器：点到平面、点到点、分布到分布、光度残差以及它们的协同加权。
3. 怎样处理困难场景和实时性：退化、运动畸变、稀疏/密集切换、内存上限、邻域搜索和迭代开销。

除特别说明外，`ATE/RMSE` 越小越好，时间越小越好，FPS 越大越好。以下实验数字是论文作者报告的结果，不是本仓库的编译、数据回放或复现实验。

- 关键图表按照论文页码记录，并在各篇笔记中附少量代表性结果页图片；同时优先转录有明确数值的表格。
- “最好”“实时”“稳定”等结论只表示原论文在其测试条件下的报告结果。
- 表格数值来自论文原文，未在 PV-LIO-PLUS 中重放；不同论文的传感器、预处理、硬件和评价指标不同，不能直接当作统一 benchmark。
- 论文中的 FAST-LIO2、Faster-LIO、VoxelMap/VoxelMap++ 等作为对比方法出现时保留在表格中，但没有另建经典算法笔记，符合项目的排除要求。
- `Environment-Adaptive Solid-State LiDAR-Inertial Odometry` 不在当前 `paper/` 收录范围内：没有对应 PDF，因此没有根据标题或外部摘要臆造笔记。

## 论文清单和每篇笔记/PDF入口

每篇论文的背景、相关研究、核心算法与推导、实验流程、结果表格和精简总结记录在对应的独立笔记中；以下条目同时提供笔记与原文 PDF 入口：

1. [C3P-VoxelMap 笔记](C3P-VoxelMap.md) / [PDF](../C3P-VoxelMap.pdf)：紧凑、累积、可合并的概率体素地图。
2. [Super-LIO 笔记](Super-LIO.md) / [PDF](../Super-LIO.pdf)：OctVox + HKNN 的紧凑高效 LIO。
3. [LIO-GVM 笔记](LIO-GVM.md) / [PDF](../LIO-GVM.pdf)：Gaussian Voxel Map 与分布到分布匹配。
4. [R-VoxelMap 笔记](R-VoxelMap.md) / [PDF](../R-VoxelMap.pdf)：递归平面拟合与平面有效性检查。
5. [CT-VoxelMap 笔记](CT-VoxelMap.md) / [PDF](../CT-VoxelMap.pdf)：连续时间 B-spline、混合特征和重估策略。
6. [FR-LIO 笔记](FR-LIO.md) / [PDF](../FR-LIO.pdf)：自适应子帧、ESKS 和 Robocentric Voxel。
7. [GenZ-LIO 笔记](GenZ-LIO.md) / [PDF](../GenZ-LIO.pdf)：空间尺度自适应体素化、混合度量和剪枝搜索。
8. [Hybrid-VoxelMap 笔记](Hybrid-VoxelMap.md) / [PDF](../Hybrid-VoxelMap.pdf)：语义驱动的平面/Gaussian 混合体素。
9. [BIEVR-LIO 笔记](BIEVR-LIO.md) / [PDF](../BIEVR-LIO.pdf)：高分辨率 bump image 与地图感知采样。
10. [Surfel-LIO 笔记](Surfel-LIO.md) / [PDF](../Surfel-LIO.pdf)：预计算 surfel 与分层 Z-order 哈希。
11. [SA-LIVO 笔记](SA-LIVO.md) / [PDF](../SA-LIVO.pdf)：子空间感知的 LiDAR-视觉信息融合。
12. [SV-LIO 笔记](SV-LIO.md) / [PDF](../SV-LIO.pdf)：概率自适应语义体素地图。

## 横向比较表

| 论文 | 地图/特征核心 | 观测核心 | 主要解决的问题 | 主要代价或边界 |
| --- | --- | --- | --- | --- |
| C3P-VoxelMap | 概率平面、累积统计、共面体素合并 | 概率点到平面 | 平面不确定性重复计算、地图膨胀 | 仍以平面为主，合并条件需要调节 |
| Super-LIO | 每个体素固定 8 个子体素代表点的 OctVox | HKNN + 点到平面 | KNN、PCA 和内存开销 | 几何表达较紧凑，复杂曲面表达能力有限 |
| LIO-GVM | 每体素均值与协方差 Gaussian | 相似度加权的 Gaussian 分布到分布 | 错配、不同 IMU 噪声和稀疏采样 | Gaussian 拟合和更多输入点增加估计时间 |
| R-VoxelMap | RANSAC 递归八叉树和有效性检查 | 概率平面匹配 | 离群点、误合并、过分割 | RANSAC 带来随机性和参数敏感性 |
| CT-VoxelMap | 连续时间 B-spline + 自适应平面/体素 | 连续时间观测与重估 | 扫描不同步、强运动和连续轨迹拟合误差 | 状态和重估流程更复杂 |
| FR-LIO | Robocentric 两层数组 RC-Vox | 子帧 + ESKS 平滑 | 高角速度下的畸变和子帧约束不足 | 需要维护多子帧和历史/未来约束 |
| GenZ-LIO | 尺度自适应体素化 | 点到平面 + 点到点混合 | confined/open 场景切换 | PD 参数、尺度阈值和两类协方差需配置 |
| Hybrid-VoxelMap | 语义决定平面或 Gaussian | 统一尺度的混合残差 | 平面和非平面环境泛化 | 依赖语义网络，推理成本和误分类风险 |
| BIEVR-LIO | 体素定向 bump height image | 高度图残差 + LM | 隧道、草地等微小几何退化 | 需要足够像素填充和较好的初始位姿 |
| Surfel-LIO | L0 子体素 + L1 预计算 surfel | O(1) 点到平面 | 迭代期间重复 KNN/PCA | 固定分辨率、局部平面假设 |
| SA-LIVO | 多尺度平面 + LiDAR 锚定视觉点 | 子空间感知信息融合 | LiDAR/视觉按方向分别退化 | 只在外感知 6-DoF 子空间做谱门控 |
| SV-LIO | 带类别概率的自适应语义平面 | 语义引导匹配和加权残差 | 动态物体、几何歧义 | 实验中的语义标签离线预计算，端到端实时性未证实 |

## 方法演化与对 PV-LIO-PLUS 的启发

### 方法演化主线

这些论文大致沿着下面的路线演进：

```text
原始点/固定体素
    -> 概率平面（C3P、R-VoxelMap、SV-LIO）
    -> 高斯分布地图（LIO-GVM）
    -> 预计算 surfel（Surfel-LIO）
    -> 高程/图像化地图（BIEVR-LIO）
    -> 混合平面-高斯和语义地图（Hybrid-VoxelMap、SV-LIO）
    -> 连续时间、子空间退化和自适应查询（CT、SA、FR、GenZ、Super）
```

### 对 PV-LIO-PLUS 的直接启发

- **地图表示：** 固定点云体素的瓶颈通常不只是查找，而是邻域点收集、PCA、法向估计和重复内存访问；C3P/R-VoxelMap 的递推统计量、Surfel-LIO 的预计算、LIO-GVM 的高斯统计是三种不同的“把计算前移”方案。
- **残差模型：** 点到平面仍是主流，但 Gaussian 的分布到分布残差、BIEVR 的高度残差、Hybrid 的混合残差说明地图表示和残差应共同设计。
- **查询策略：** GenZ-LIO 的双分辨率与下界剪枝、Super-LIO 的层次候选和提前终止、FR-LIO 的固定邻域缓存，都在减少无效候选与每帧动态分配。
- **退化处理：** R-VoxelMap 解决平面约束不足，SA-LIVO 则进一步在信息子空间逐方向衰减；这比简单地报告一个总退化标志更适合诊断。
- **动态/语义：** Hybrid-VoxelMap 和 SV-LIO 表明语义可同时进入地图、数据关联和权重，但离线标签会把“前端实时性”与“完整系统实时性”分开。
- **时间因素：** CT-VoxelMap 和 FR-LIO 提醒：高速运动下，单帧刚体模型与固定更新频率可能成为主要误差源；连续时间插值和自适应子帧是另一条优化维度。

### 后续阅读与移植时的验证边界

论文表格是各自数据集、参数、硬件和成功序列定义下的结果，不能直接横向宣称某算法在 PV-LIO-PLUS 上一定更快或更准。若后续移植，建议为每项改动单独记录有效点数、平面接受率、邻域查询次数、协方差权重、地图大小、队列长度和模块耗时，并保持同一传感器、数据集、编译选项和局部地图尺寸。CT/语义/随机采样/信息子空间模块还应保留显式开关，避免把论文中的特定假设未经验证地替换现有路径。

## 精简总结

这批工作共同说明：LiDAR-inertial odometry 的下一步优化重点，正在从“更快地找最近邻”转向“用更有信息的地图统计量、更可信的残差和更有方向性的退化处理减少无效计算”。对 PV-LIO-PLUS 最稳妥的工程路线，是先在保持旧路径可回退的前提下，分别验证预计算几何、递推协方差、候选剪枝和语义/退化加权，再用统一的精度、耗时、内存和失败率指标比较，而不是直接叠加全部机制。
