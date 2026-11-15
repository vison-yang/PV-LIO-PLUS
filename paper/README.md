# 论文与官方源码资料

论文与源码资料核验日期：2026-11-14；PV-LIO-PLUS 集成范围更新日期：2026-11-15。本目录收录 22 篇论文及 2 份供既有笔记使用的旧版归档，覆盖主 README 的算法清单，并补充 FAST-LIO 和 ikd-Tree 原始论文。

2026-11-14 的资料整理核验了全部本地 PDF 的页数与 SHA-256，并收录 i-Octree、KISS-ICP 和 iG-LIO 的作者论文及对应源码快照。已有论文版本继续固定在表列版本，不代表全部都核查为最新版本。2026-11-15 更新的集成范围是：主路线比较共同框架中具备新或实质改进地图数据结构的候选及其对应观测；仅改变残差、采样、查询算法或估计流程不构成结构准入。严格统一观测是可选消融；完整上游系统作为独立端到端基线。当前资格证据和实施条件见[地图与观测范围审查](notes/MAP_BACKEND_SCOPE_REVIEW.md)、[笔记索引](notes/README.md)和[集成计划](../PLAN.md)。

表中的源码链接指向作者项目，不表示该算法已接入 PV-LIO-PLUS，也不表示已在本机完成编译或数据集复现。源码已收集、上游构建、框架集成和数据集精度复现是不同状态；本地路径、作者提交和构建证据见[源码清单](../doc/workspace/SOURCE_MANIFEST.json)与[构建报告](../doc/workspace/BUILD_REPORT.md)。未确认官方源码的项目保留论文入口，不以同名项目、相关算法或独立重实现替代；“本地未确认”不表示远程不存在。PDF 的具体版本、页数及 SHA-256 记录在 [catalog.json](catalog.json)。

## 全量资料存档

下表包含已移出候选的方法及框架来源，不代表当前集成清单；合格地图、论文储备与归档方法分别列在[笔记索引](notes/README.md)。

| 方法 | 本地论文 / 版本 | 作者论文或出版社入口 | 官方源码 |
| --- | --- | --- | --- |
| PV-LIO | 上游 README 未提供独立论文链接；相关基础论文见 VoxelMap、FAST-LIO | [项目说明](https://github.com/HViktorTsoi/PV-LIO) | [PV-LIO](https://github.com/HViktorTsoi/PV-LIO) |
| VoxelMap | [PDF](VoxelMap.pdf)，arXiv v5，8 页 | [2109.07082v5](https://arxiv.org/abs/2109.07082v5) | [VoxelMap](https://github.com/hku-mars/VoxelMap) |
| VoxelMap++ | [PDF](VoxelMap++.pdf)，arXiv v1，8 页 | [2308.02799v1](https://arxiv.org/abs/2308.02799v1) | [VoxelMapPlus_Public](https://github.com/uestc-icsp/VoxelMapPlus_Public) |
| FAST-LIO | [PDF](FAST-LIO.pdf)，arXiv v3，8 页 | [2010.08196v3](https://arxiv.org/abs/2010.08196v3) | [FAST_LIO](https://github.com/hku-mars/FAST_LIO)，当前主分支为 FAST-LIO2 |
| FAST-LIO2 | [PDF](FAST-LIO2.pdf)，arXiv v1，19 页 | [2107.06829v1](https://arxiv.org/abs/2107.06829v1) | [FAST_LIO](https://github.com/hku-mars/FAST_LIO) |
| ikd-Tree | [PDF](ikd-Tree.pdf)，arXiv v1，8 页 | [2102.10808v1](https://arxiv.org/abs/2102.10808v1) | [ikd-Tree](https://github.com/hku-mars/ikd-Tree)；也是 FAST-LIO2 的内部地图 |
| Faster-LIO / iVox | [PDF](Faster-LIO.pdf)，IEEE RA-L 2022，8 页 | [10.1109/LRA.2022.3152830](https://doi.org/10.1109/LRA.2022.3152830) | [faster-lio](https://github.com/gaoxiang12/faster-lio)；PDF 来自该作者仓库的 `doc/faster-lio.pdf` |
| C3P-VoxelMap | [PDF](C3P-VoxelMap.pdf)，arXiv v2，8 页 | [2406.01195v2](https://arxiv.org/abs/2406.01195v2) | [c3p-voxelmap](https://github.com/deptrum/c3p-voxelmap) |
| Super-LIO / OctVox | [PDF](Super-LIO.pdf)，arXiv v2，8 页 | [2509.05723v2](https://arxiv.org/abs/2509.05723v2) | [Super-LIO，ROS1 分支](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1) |
| Hybrid-VoxelMap | [PDF](Hybrid-VoxelMap.pdf)，IEEE RA-L 正式版，8 页 | [10.1109/LRA.2025.3632730](https://doi.org/10.1109/LRA.2025.3632730) | [Hybrid-VoxelMap](https://github.com/haiyang2022/Hybrid-VoxelMap) |
| R-VoxelMap | [PDF](R-VoxelMap.pdf)，arXiv v1，8 页 | [2601.12377v1](https://arxiv.org/abs/2601.12377v1) | [R-VoxelMap](https://github.com/NKU-MobFly-Robotics/R-VoxelMap) |
| BIEVR-LIO | [PDF](BIEVR-LIO.pdf)，arXiv v2，12 页 | [2604.14421v2](https://arxiv.org/abs/2604.14421v2) | [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO) |
| Surfel-LIO / hVox | [PDF](Surfel-LIO.pdf)，arXiv v2，8 页 | [2512.03397v2](https://arxiv.org/abs/2512.03397v2) | [lidar_inertial_odometry](https://github.com/93won/lidar_inertial_odometry) |
| LIO-GVM | [PDF](LIO-GVM.pdf)，arXiv v3，8 页 | [2306.17436v3](https://arxiv.org/abs/2306.17436v3) | [lio_gvm](https://github.com/Ji1Xingyu/lio_gvm)，上游发布缺少编译必需的定制头文件 |
| FR-LIO / RC-Vox | [PDF](FR-LIO.pdf)，arXiv v3，8 页 | [2302.04031v3](https://arxiv.org/abs/2302.04031v3) | 本地收集记录未确认作者官方源码；RC-Vox 是论文中的地图结构 |
| CT-VoxelMap | [PDF](CT-VoxelMap.pdf)，arXiv v1，16 页 | [2604.03747v1](https://arxiv.org/abs/2604.03747v1) | 本地收集记录未确认作者源码；归档为连续时间估计研究，当前证据未确认独立地图结构改进 |
| GenZ-LIO | [PDF](GenZ-LIO.pdf)，arXiv v3，29 页 | [2603.16273v3](https://arxiv.org/abs/2603.16273v3) | [genz-lio](https://github.com/cocel-postech/genz-lio)；源码与构建记录留档，v3 沿用 VoxelMap，不列当前地图候选 |
| SA-LIVO | [PDF](SA-LIVO.pdf)，arXiv v3，17 页 | [2606.25699v3](https://arxiv.org/abs/2606.25699v3) | 本地收集记录未确认独立作者地图源码；作为相关视觉/估计研究归档 |
| SV-LIO | [PDF](SV-LIO.pdf)，Electronics 2026 正式版，22 页 | [10.3390/electronics15081744](https://doi.org/10.3390/electronics15081744) | 本地收集记录未确认作者官方源码 |
| Environment-Adaptive Solid-State LIO | [PDF](Environment-Adaptive-Solid-State-LIO.pdf)，arXiv v2，6 页 | [2604.15864v2](https://arxiv.org/abs/2604.15864v2) | 本地收集记录未确认作者源码；作为地图维护研究归档，尚无空间/层级/几何结构改进证据 |
| i-Octree | [PDF](i-Octree.pdf)，arXiv v2，7 页 | [2309.08315v2](https://arxiv.org/abs/2309.08315v2) | [i-octree](https://github.com/zhujun3753/i-octree)，仅抽动态八叉树；独立 CMake 示例 |
| KISS-ICP / VoxelHashMap | [PDF](KISS-ICP.pdf)，RA-L 2023 作者预印本，8 页 | [10.1109/LRA.2023.3236571](https://doi.org/10.1109/LRA.2023.3236571) | [kiss-icp](https://github.com/PRBonn/kiss-icp)；沿用既有 voxel hash，源码/构建留作归档，不列当前地图候选 |
| iG-LIO / incremental VoxelMap | [PDF](iG-LIO.pdf)，RA-L 2024 作者 early access，8 页 | [10.1109/LRA.2024.3349915](https://doi.org/10.1109/LRA.2024.3349915) | [ig_lio](https://github.com/zijiechenrobotics/ig_lio)，增量点图与 GICP 观测分阶段评估；不移植完整状态优化循环 |

## 旧版归档与笔记对应关系

- [GenZ-LIO v2](GenZ-LIO-v2.pdf)，21 页，是 [GenZ-LIO 笔记](notes/GenZ-LIO.md) 中原 v2 推导与图表的依据；默认 `GenZ-LIO.pdf` 为 v3，笔记另列 v3 补充。
- [SA-LIVO v2](SA-LIVO-v2.pdf)，18 页，是 [SA-LIVO 笔记](notes/SA-LIVO.md) 中原 v2 推导与图表的依据；默认 `SA-LIVO.pdf` 为 v3，笔记另列 v3 补充。v3 标题为 *SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with Subspace-Aware Information Fusion*，不应把 v2 的推导或实验数字当作 v3 的结论。
- 其余既有专题笔记入口见 [notes/README.md](notes/README.md)。22 篇正式论文的独立笔记及版本差异整理均已补齐；笔记中的结果属于作者报告，不是本工作空间的数据回放结果。

## 本项目的集成与复现状态

- 当前六个 selector 均集成地图及对应观测。2026-11-14 的共享工作区构建和独立上游构建结果见[构建报告](../doc/workspace/BUILD_REPORT.md)；该报告明确记录本轮没有 rosbag 回放。
- 另有七项满足结构准入条件的本地作者源码待评估，其中 LIO-GVM 因作者发布缺少定制头文件而阻塞；两项结构合格论文（FR-LIO、SV-LIO）本地收集记录尚未确认作者源码。KISS-ICP、GenZ-LIO、CT-VoxelMap、SA-LIVO 和 Environment-Adaptive 的论文、源码或历史构建证据保留归档，不列当前候选。逐项状态见[集成计划](../PLAN.md)。
- 本地构建退出 0 不等于数据集精度复现。作者论文中的精度、耗时和运行条件不代表本工作区的实验结果。

## 源码核查边界

- GenZ-LIO v3 明确链接 `cocel-postech/genz-lio`；GenZ-ICP 是相关的另一项目。
- 本清单中的 FR-LIO 是 2023 年 *Fast and Robust LiDAR-Inertial Odometry by Tightly-Coupled Iterated Kalman Smoother and Robocentric Voxels*。同名的 Flight-ready Drift-Aware 项目不对应这篇论文。
- CT-VoxelMap 和 SA-LIVO 的独立论文重实现不视为作者官方发布。
- LIO-GVM 已公开仓库，但作者的 [Project Update](https://github.com/Ji1Xingyu/lio_gvm#project-update) 仍注明 `.hpp` 待更新；[issue #8](https://github.com/Ji1Xingyu/lio_gvm/issues/8) 与 [issue #13](https://github.com/Ji1Xingyu/lio_gvm/issues/13) 也报告缺少定制 Fast-VGICP 和 IKFoM 接口。通用依赖不能补足这些作者定制接口，因此“官方源码已获取”不等于其完整节点目前可编译。
- “本地收集记录未确认”仅描述本地来源清单的状态；不代表远程不存在任何公开实现。本地源码状态按已固定快照和当前 manifest 记录，未拉取远程更新。
- [IKFoM](https://github.com/hku-mars/IKFoM) 是 PV-LIO 使用的滤波工具库，其源码随项目的 `include/IKFoM_toolkit/` 保留，不另计为一个完整 LIO 基线。
