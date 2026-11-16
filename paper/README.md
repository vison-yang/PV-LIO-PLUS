# 论文与官方源码资料

论文与源码资料核验日期：2026-11-14；PV-LIO-PLUS 集成范围更新日期：2026-11-16。本目录收录 17 篇论文，覆盖主 README 的算法清单，并补充 FAST-LIO 和 ikd-Tree 原始论文。

2026-11-14 的资料整理核验了本地 PDF 的页数与 SHA-256，并收录 i-Octree 和 iG-LIO 的作者论文及对应源码快照。已有论文版本继续固定在表列版本，不代表全部都核查为最新版本。2026-11-16 更新的集成范围是：主路线比较共同框架中具备新或实质改进地图数据结构的候选及其对应观测；仅改变残差、采样、查询算法或估计流程不构成结构准入。严格统一观测是可选消融；完整上游系统作为独立端到端基线。当前资格证据和实施条件见[地图与观测范围审查](notes/MAP_BACKEND_SCOPE_REVIEW.md)、[笔记索引](notes/README.md)和[集成计划](../PLAN.md)。

表中的源码链接指向作者项目，不表示该算法已接入 PV-LIO-PLUS，也不表示已在本机完成编译或数据集复现。源码已收集、上游构建、框架集成和数据集精度复现是不同状态；本地路径、作者提交和构建证据见[源码清单](../doc/workspace/SOURCE_MANIFEST.json)与[构建报告](../doc/workspace/BUILD_REPORT.md)。未确认官方源码的项目保留论文入口，不以同名项目、相关算法或独立重实现替代；“本地未确认”不表示远程不存在。PDF 的具体版本、页数及 SHA-256 记录在 [catalog.json](catalog.json)。

## 全量资料索引

下表汇总 17 篇本地论文对应的方法、固定版本和作者源码入口，包含已集成地图、当前候选、论文储备及框架来源；它是资料索引，不单独代表当前实施状态。合格地图和论文储备分别列在[笔记索引](notes/README.md)。

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
| SV-LIO | [PDF](SV-LIO.pdf)，Electronics 2026 正式版，22 页 | [10.3390/electronics15081744](https://doi.org/10.3390/electronics15081744) | 本地收集记录未确认作者官方源码 |
| i-Octree | [PDF](i-Octree.pdf)，arXiv v2，7 页 | [2309.08315v2](https://arxiv.org/abs/2309.08315v2) | [i-octree](https://github.com/zhujun3753/i-octree)；已按结构保真方式适配到 `i_octree`，不是未修改上游性能复现 |
| iG-LIO / incremental VoxelMap | [PDF](iG-LIO.pdf)，RA-L 2024 作者 early access，8 页 | [10.1109/LRA.2024.3349915](https://doi.org/10.1109/LRA.2024.3349915) | [ig_lio](https://github.com/zijiechenrobotics/ig_lio)，固定作者提交 `2d79259e`；原完整 `src/ig_lio` checkout 当前缺失，本地地图核心是基于固定源文件的安全重实现，并接入 `ig_lio_plane` 点面变体；作者 GICP 仍独立待设计；旧构建记录不代表当前完整包可用 |

其余既有专题笔记入口见 [notes/README.md](notes/README.md)。17 篇论文的独立笔记及版本差异整理均已补齐；笔记中的结果属于作者报告，不是本工作空间的数据回放结果。

## 本项目的集成与复现状态

- 当前有十个地图 selector；`ig_lio_plane` 已接入，round 2 node build/link 与九个指定构建目标通过，CTest 9/9（65 个 GTest 条目，ASan 关闭）及配置合成测试通过。本轮尚无回放或精度/性能验证，详见[iG-LIO 结果记录](../doc/workspace/evidence/IG_LIO_RESULTS.json)。i-Octree feature `5c27a79` 已以 `6876a8e` 合入主线；feature 隔离测试、数值审查和 30/141 秒回放通过，主线构建/链接、CTest 7/7（49 个 GTest 条目，ASan 关闭）及 30 秒完整输入回放通过。详见[i-Octree 结果记录](../doc/workspace/evidence/I_OCTREE_RESULTS.json)。R-VoxelMap 与 hVox 的主线状态和证据见[集成状态](../doc/INTEGRATION_STATUS.md)、[R 结果记录](../doc/workspace/evidence/R_VOXELMAP_RESULTS.json)和[hVox 结果记录](../doc/workspace/evidence/HVOX_RESULTS.json)。历史回归不构成独立真值精度或受控性能结论。
- 另有三项满足结构准入条件的地图候选（Hybrid-VoxelMap、BIEVR-LIO、LIO-GVM）。BIEVR-LIO 未集成，仅有移植设计草稿；作者上游独立构建已通过，PV-LIO-PLUS 框架适配尚未实现。LIO-GVM 因作者发布缺少定制头文件而阻塞；两项结构合格论文（FR-LIO、SV-LIO）本地收集记录尚未确认作者源码。完整 iG-LIO GICP 观测属于独立后续设计，不计作尚未接入的新地图。逐项状态见[集成计划](../PLAN.md)。
- 本地构建退出 0 不等于数据集精度复现。作者论文中的精度、耗时和运行条件不代表本工作区的实验结果。

## 源码核查边界

- 本清单中的 FR-LIO 是 2023 年 *Fast and Robust LiDAR-Inertial Odometry by Tightly-Coupled Iterated Kalman Smoother and Robocentric Voxels*。同名的 Flight-ready Drift-Aware 项目不对应这篇论文。
- LIO-GVM 已公开仓库，但作者的 [Project Update](https://github.com/Ji1Xingyu/lio_gvm#project-update) 仍注明 `.hpp` 待更新；[issue #8](https://github.com/Ji1Xingyu/lio_gvm/issues/8) 与 [issue #13](https://github.com/Ji1Xingyu/lio_gvm/issues/13) 也报告缺少定制 Fast-VGICP 和 IKFoM 接口。通用依赖不能补足这些作者定制接口，因此“官方源码已获取”不等于其完整节点目前可编译。
- “本地收集记录未确认”仅描述本地来源清单的状态；不代表远程不存在任何公开实现。本地源码状态按已固定快照和当前 manifest 记录，未拉取远程更新。
- [IKFoM](https://github.com/hku-mars/IKFoM) 是 PV-LIO 使用的滤波工具库，其源码随项目的 `include/IKFoM_toolkit/` 保留，不另计为一个完整 LIO 基线。
