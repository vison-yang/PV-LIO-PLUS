# PV-LIO-PLUS 相关论文笔记

这里索引 22 篇 canonical 论文的独立笔记与本地 PDF。PDF 版本、来源链接和作者代码入口见[论文资料清单](../README.md)；地图是否可作为替换层，以[地图替换范围审查](MAP_BACKEND_SCOPE_REVIEW.md)为准。作者仓库或论文链接不表示对应方法已移植。

## 可迁移地图层与地图结构

下表只表示适配范围或已有地图分支，不代表新后端已经实现。点图候选均需经过 PV 点面适配器，固定公共滤波、残差与权重合同。

| 论文 | 独立笔记 / 本地 PDF | 范围说明 |
| --- | --- | --- |
| VoxelMap | [笔记](VoxelMap.md) / [PDF](../VoxelMap.pdf) | 现有概率平面地图分支 |
| VoxelMap++ | [笔记](VoxelMap++.md) / [PDF](../VoxelMap++.pdf) | 现有平面合并分支；观测权重/Jacobian 有已知差异 |
| ikd-Tree | [笔记](ikd-Tree.md) / [PDF](../ikd-Tree.pdf) | 现有增量点图；由 PV 点面适配器拟合 |
| Faster-LIO / iVox | [笔记](Faster-LIO.md) / [PDF](../Faster-LIO.pdf) | 现有稀疏增量点图；有限邻域查询 |
| C3P-VoxelMap | [笔记](C3P-VoxelMap.md) / [PDF](../C3P-VoxelMap.pdf) | 现有概率体素及共面合并分支 |
| Super-LIO / OctVox | [笔记](Super-LIO.md) / [PDF](../Super-LIO.pdf) | 现有代表点地图；保留原匹配/观测差异 |
| R-VoxelMap | [笔记](R-VoxelMap.md) / [PDF](../R-VoxelMap.pdf) | 范围审查优先保留的地图层候选 |
| Surfel-LIO / hVox | [笔记](Surfel-LIO.md) / [PDF](../Surfel-LIO.pdf) | 保留本地 73348c1 预计算 L1 surfel 地图 |
| GenZ-LIO 纯平面地图 | [笔记](GenZ-LIO.md) / [v3 PDF](../GenZ-LIO.pdf) | 仅纯平面子模块；不含自适应采样/点点残差 |
| i-Octree | [笔记](i-Octree.md) / [PDF](../i-Octree.pdf) | 动态点八叉树候选 |
| KISS-ICP VoxelHashMap | [笔记](KISS-ICP.md) / [PDF](../KISS-ICP.pdf) | 有界点图候选；不含点点 ICP 流程 |
| iG-LIO incremental VoxelMap | [笔记](iG-LIO.md) / [PDF](../iG-LIO.pdf) | 增量点图候选；不含 VSCE/GICP |

## 相关方法资料

以下论文帮助理解估计、连续时间、退化、语义或混合残差问题；按范围审查，不作为当前直接地图替换实现。

| 论文 | 独立笔记 / 本地 PDF | 主要资料边界 |
| --- | --- | --- |
| FAST-LIO | [笔记](FAST-LIO.md) / [PDF](../FAST-LIO.pdf) | 紧耦合迭代滤波与点级去畸变 |
| FAST-LIO2 | [笔记](FAST-LIO2.md) / [PDF](../FAST-LIO2.pdf) | 直接点面 LIO 系统；地图结构另记 ikd-Tree |
| Environment-Adaptive Solid-State LIO | [笔记](Environment-Adaptive-Solid-State-LIO.md) / [PDF](../Environment-Adaptive-Solid-State-LIO.pdf) | 局部角度约束和退化感知更新耦合 |
| CT-VoxelMap | [笔记](CT-VoxelMap.md) / [PDF](../CT-VoxelMap.pdf) | 连续时间状态、时间观测与重估 |
| FR-LIO | [笔记](FR-LIO.md) / [PDF](../FR-LIO.pdf) | 自适应子帧、ESKS；RC-Vox 尚未核验作者实现 |
| Hybrid-VoxelMap | [笔记](Hybrid-VoxelMap.md) / [PDF](../Hybrid-VoxelMap.pdf) | 语义驱动平面/Gaussian 混合 |
| BIEVR-LIO | [笔记](BIEVR-LIO.md) / [PDF](../BIEVR-LIO.pdf) | bump image、高度图残差和地图感知采样 |
| LIO-GVM | [笔记](LIO-GVM.md) / [PDF](../LIO-GVM.pdf) | Gaussian 地图与分布到分布残差 |
| SA-LIVO | [笔记](SA-LIVO.md) / [v3 PDF](../SA-LIVO.pdf) | 视觉误差建模与子空间信息融合 |
| SV-LIO | [笔记](SV-LIO.md) / [PDF](../SV-LIO.pdf) | 语义概率体素、关联和残差加权 |

## 版本与实验阅读说明

- GenZ-LIO 笔记前半部的推导、实验数字和图片标签专属 [v2 PDF](../GenZ-LIO-v2.pdf)；文末的独立摘要只依据默认 [v3 PDF](../GenZ-LIO.pdf)。两版结果不混用。
- SA-LIVO 笔记前半部的推导、表格和图片标签专属 [v2 PDF](../SA-LIVO-v2.pdf)；文末的独立摘要只依据默认 [v3 PDF](../SA-LIVO.pdf)。v3 的信息误差模型和实验结果不回标为 v2。
- 其余笔记按每条列出的 PDF 版本整理。实验指标、设置和结论均来自论文作者报告，不是本工作区的编译、数据回放或复现实验。
- 各论文的传感器、预处理、硬件、序列和成功定义不同，表中数字不是统一 benchmark，不能据此跨论文直接排序。
- 地图替换的共同边界是固定 IMU 传播、去畸变、滤波、扫描采样与点面观测合同；地图协方差必须保持同义，有限邻域查询不能称为全图精确 kNN。详见[范围审查](MAP_BACKEND_SCOPE_REVIEW.md)。
