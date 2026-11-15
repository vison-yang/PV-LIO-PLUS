# 分阶段提交记录

日期：2026-11-14。所有提交均为本地提交，未 push。工作空间根目录没有有效 Git 元数据，以下分别记录框架仓库和各个上游工作树。

2026-11-15 范围更新：下表保留当时的阶段记录；其中“仅地图替换”和移出候选的判定已由当前[集成计划](../../PLAN.md)及[地图与残差范围审查](../../paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)更新。当前清单先要求算法具有地图数据结构创新，再以共同框架中的地图及对应残差为比较对象；严格统一观测的索引/表示消融是可选实验。本次仅更新文档与清单分类，没有新增后端或重跑构建、回放。

## PV_LIO_PLUS 仓库

| 阶段 | 提交 | 内容 |
| --- | --- | --- |
| 1. 候选筛选与资料 | `c577b75` | 仅地图替换边界；新增三项候选、移除七项越界方法；固定源码和22篇论文/2份旧版本。 |
| 2. 论文笔记 | `4f63136` | 补齐10篇独立笔记、12篇移植边界；GenZ/SA v3单列；原文机制审校修正。 |
| 3. 源码与编译 | `c9b0264` | 本地依赖/构建脚本、固定上游提交及完整补丁、真实编译与测试证据。 |

后续按用户请求单独清理无用 `research/`，未改变上述三阶段的源码与构建结果；核验、保留记录及删除后验证见 [RESEARCH_CLEANUP.json](RESEARCH_CLEANUP.json)。

## 各上游仓库的兼容性提交

本轮对已有未提交的共享工作空间接入修改进行了检查并提交；新改动主要为统一 Eigen 搜索顺序、iG-LIO 依赖、Super-LIO 的 basic manifest 依赖、i-Octree/Surfel 外部构建路径。其它已存在的节点/目标名称与消息命名空间修复一并保留。PV_LIO_PLUS 的核心估计算法未修改。

| 路径 | 本地提交 | 摘要 |
| --- | --- | --- |
| `src/GenZ-LIO` | `24dce4f` | build(ros1): align Eigen headers and Livox message dependencies |
| `src/ig_lio` | `a75b362` | build(ros1): declare direct ROS PCL and message dependencies |
| `src/Super-LIO` | `9bd033b` | build(ros1): order workspace dependencies and distinguish node names |
| `src/Hybrid-VoxelMap` | `917ff80` | build(ros1): avoid target and generated message namespace conflicts |
| `src/R-VoxelMap` | `5166480` | build(ros1): distinguish mapping targets and order generated messages |
| `src/faster_lio` | `1273553` | build(ros1): reuse the workspace Livox driver dependency |
| `src/PV-LIO` | `4a22b93` | build(ros1): declare direct dependencies and message generation order |
| `src/VoxelMapPlus_Public` | `d0cf184` | build(ros1): declare dependencies and link OpenCV |
| `src/c3p_voxelmap` | `69f84b8` | fix(ros1): give the mapping node a distinct name |
| `upstream/i-Octree` | `25903a3` | build: allow external ikd-Tree comparison dependency |
| `upstream/Surfel-LIO` | `2ed0410` | build: allow out-of-tree Pangolin build directory |

精确父基线、当前本地提交和可重放补丁见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。补丁是相对固定作者版本的完整工作树差异，包含本轮开始前已存在且与共享构建有关的修改。

## 验证范围

- 14包共享构建退出0，GenZ CTest 21/21，23个ELF链接校验通过。
- upstream按顺序独立构建：4项通过；LIO-GVM因作者发布缺源码退出1。
- 未回放rosbag，未修改已有地图或轨迹结果；当前六后端仍保留不同原生观测策略，详见范围审查。
