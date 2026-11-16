> 历史记录：本报告审校时的 SA-LIVO 笔记包含 v2 主体与 v3 摘要；现仅在独立工作空间保留 v3。此处旧行号和版本比较描述当时的文档状态，不代表当前笔记仍混用版本。
>
> 最终状态：以下五组问题均已在笔记正文修复，并由同一审校 agent 逐项复核通过；i-Octree 精确 kNN 的补注也已通过。本报告下文记录修复前的问题及原文依据。

修复文件：[VoxelMap++](../../paper/notes/VoxelMap++.md)、[Environment-Adaptive](../../paper/notes/Environment-Adaptive-Solid-State-LIO.md)、[ikd-Tree](../../paper/notes/ikd-Tree.md)、[KISS-ICP](../../paper/notes/KISS-ICP.md)、[SA-LIVO](../../../../../livo_src/paper/notes/SA-LIVO.md)、[i-Octree](../../paper/notes/i-Octree.md)。

# 论文笔记实质审校

审校日期：2026-11-14。仅审阅十篇新增笔记、GenZ-LIO/SA-LIVO 末尾 v3 摘要及既定地图替换边界；证据为工作空间 `output/maintenance-20261114/paper-texts/*.txt` 中对应 PDF 的逐页文本。未改笔记、未编译、未联网。

结论：发现 **5 组可局部修复的实质问题**。未发现候选保留/排除结论与 `MAP_BACKEND_SCOPE_REVIEW.md` 的新增冲突。以下行号以本次读取时为准。

## 必修短列表

1. **VoxelMap++：3 参数的具体表示及合并公式需要写准。**
   - 位置：`src/PV_LIO_PLUS/paper/notes/VoxelMap++.md:16–23`。
   - 问题：论文不是以单位法向的二维切空间加偏移直接作为参数；其主轴归一化参数为 `[a,b,d]`，例如 `ax+by+z+d=0`，法向为未归一化的 `[a,b,1]`。合并也不是把子格原始点的累积求和统计直接汇成父平面。
   - 准确替代：**“论文以主轴归一化的 `[a,b,d]` 表示平面（例如 `ax+by+z+d=0`）；必要时更换主轴，查询时再将法向与距离归一化。单格拟合可增量缓存求和量。收敛后，以相邻平面的 3 参数估计及其协方差作为父平面的测量，经 Mahalanobis/χ² 检验决定是否合并，再按协方差迹构造加权平均及合成协方差；union-find 组织共享父平面，并将最大深度剪枝至 2。”**
   - 证据：`paper-texts/VoxelMap++.txt`，PAGE 3，§III-B、Eq.(4)、(6)–(8)；PAGE 4，§III-C3、Eq.(9)–(11)、Algorithm 1；PAGE 5，Eq.(13)。

2. **Environment-Adaptive：把角度残差写成了两组局部法向之间的夹角。**
   - 位置：`src/PV_LIO_PLUS/paper/notes/Environment-Adaptive-Solid-State-LIO.md:8–15`。
   - 问题：按正文明确公式，`d1` 是当前变换点与地图对应点的连线，`d2` 是拟合平面的参考法向，残差为 `1 − (d1/||d1||)·d2`；不是“相邻局部结构的法向之间的角度”。论文引言的法向描述与此公式并不完全一致，笔记应据公式说明而不另造法向对。
   - 准确替代：**“作者将角度项加入完整 MAP 目标。按 §III-B 的定义，令 `d1=q_i^G−Q`，以其归一化方向和拟合平面法向 `d2` 构造 `eθ=1−(d1/||d1||)·d2`。完整目标还包括点面残差、GICP 残差和 IMU 先验，因此不是仅在通用点面观测上追加两法向夹角。”** 迁移排除结论可保持。
   - 证据：`paper-texts/Environment-Adaptive-Solid-State-LIO.txt`，PAGE 2，§III-A；PAGE 3，Eq.(1)–(7)。

3. **ikd-Tree：作者实验混入了 FAST-LIO2 的传感器和误差评估。**
   - 位置：`src/PV_LIO_PLUS/paper/notes/ikd-Tree.md:31–33`。
   - 问题：固定的 arXiv v1 仅有 Table I–II。其真实应用是单个 Livox Avia 100 Hz 室外 FAST-LIO 建图实验，没有机械式 LiDAR 对比，也没有 Tables II–V、轨迹误差/端点误差表。“长时间建图”也未在该版本作为独立实验验证。
   - 准确替代：**“随机点实验检验持续插入/盒删除时的更新与查询时间，以及稀疏、紧凑增量分布的影响；室外应用将 FAST-LIO 的静态 k-d tree 替换为 ikd-Tree，使用 Livox Avia 100 Hz 数据，在 i7-8550U / 8 GB 的 DJI Manifold 2-C 上报告扫描融合、近邻查询、增量更新与重平衡耗时，并展示建图结果（pp. 6–8，Table II、Figs. 4–7）。该版本未给轨迹/端点误差量化。”**
   - 证据：`paper-texts/ikd-Tree.txt`，PAGE 6，§V-A、Table II；PAGE 7–8，§V-B、Figs. 6–7。全文表名检索只有 Table I、II。

4. **KISS-ICP：鲁棒核名称写错。**
   - 位置：`src/PV_LIO_PLUS/paper/notes/KISS-ICP.md:37`。
   - 准确替代：将 **“Huber/自适应匹配阈值”** 改为 **“Geman–McClure 鲁棒核及自适应匹配阈值”**。原生点点 ICP 与 PV 点面适配的边界说明正确。
   - 证据：`paper-texts/KISS-ICP.txt`，PAGE 5，§III-E、Eq.(12)–(13)。

5. **SA-LIVO v3：内存指标及时间比较的统计范围需要区分。**
   - 位置：`src/PV_LIO_PLUS/paper/notes/SA-LIVO.md:122`。
   - 问题：v3 测的是序列结束时 RSS（final memory），不是 peak memory；“峰值”来自 v2 的评估口径。13.4 ms 出现在 v3 摘要与时间分解中，对 Table IV 的全部 15 个 HILTI 序列均值复算为 13.4 ms；但所有比较方法均完成的 **7 序列共同子集**上，正文报告 **17.2 ms**，2.0–6.2 倍的内存比较也在该共同子集上。不能把这些范围合为一次整体比较。
   - 准确替代：**“v3 在 HILTI’22 的 15 个手持序列和 MARS-LVIG 的 9 个空中序列上评估，共 24 个序列，在 22 个序列最优或并列最优。摘要与时间分解报告 HILTI’22 平均 13.4 ms/frame；在所有比较方法均成功的 7 个 HILTI’22 序列上，笔记本 CPU 平均为 17.2 ms/frame，序列结束时 RSS 相较 FAST-LIVO2、R3LIVE、SR-LIVO 分别低 2.0、4.1、6.2 倍。这里的内存不是峰值，且不同统计子集应分别标明。”**
   - 证据：`paper-texts/SA-LIVO.txt`，PAGE 1 摘要；PAGE 8，§VIII-A；PAGE 10，§VIII-E、Table IV；PAGE 11 结论。按 Table IV 已四舍五入的单序列时间复算，15/7 序列均值分别为 13.4/17.1571 ms，与正文保留一位小数的 13.4/17.2 ms 一致。

## 已核验通过的重点

- **FAST-LIO**：当前笔记正确标明 v3 的 18 维状态与已知 LiDAR–IMU 外参，未混入 FAST-LIO2 的在线外参状态；依据 PAGE 3，Eq.(1)–(3)。
- **Faster-LIO**：当前论文标题已与 PDF 首页一致；6/18/26 邻格、近似 kNN、LRU 和 PHC 的归属正确。
- **GenZ-LIO v3 摘要**：9 个公开数据集加 NarrowWide、42 个序列、ATE RMSE 超过 200 m 定义发散，均来自 v3；未把主体 v2 的具体 ATE、rank 或 λ 参数改标为 v3。纯平面地图分支与排除尺度自适应/点点 fallback 的边界一致。
- **SA-LIVO 版本隔离**：主体明确标为 v2，末尾独立 v3 摘要；v3 的帧级噪声估计、Schmidt 相关性、视觉信息上限和联合 InEKF 描述来自对应 v3。除上述实验口径外，未发现把 v2 的弱方向谱门控当成 v3 机制。
- **VoxelMap、FAST-LIO2、i-Octree、iG-LIO**：概率平面/动态点索引/GICP 的主要归属及迁移排除关系未发现阻断问题。i-Octree 原文 §II-C 明确是保留点集上的精确 kNN；其分辨率或树内下采样会改变保留点集，宜避免把这一影响理解成 iVox 式有限邻域近似。
- **地图范围**：十篇新增笔记及两个 v3 摘要均未提议将 IMU、deskew、滤波、光度/GICP/混合残差随地图 selector 一起替换；现有观测分支差异也未被误称为严格相同观测。

本报告仅核验上述笔记的实质表述与本地论文来源，不构成实现正确性或论文性能复现结论。可下派的执行包就是上述五处局部文字修复；无需再研究后端架构。
