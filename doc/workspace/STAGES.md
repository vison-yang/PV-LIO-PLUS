# 分阶段提交记录

历史记录日期：2026-11-14；当时的提交均为本地提交。2026-11-15 的推送状态另见末节。工作空间根目录没有有效 Git 元数据，以下分别记录框架仓库和各个上游工作树。

2026-11-15 范围更新：下表保留当时的阶段记录；其中“仅地图替换”和移出候选的判定已由当前[集成计划](../../PLAN.md)及[地图与残差范围审查](../../paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)更新。当前清单先要求算法具有地图数据结构创新，再以共同框架中的地图及对应残差为比较对象；严格统一观测的索引/表示消融是可选实验。该范围更新最初仅涉及文档与清单分类；随后完成的核心修改、后端集成和回放另见末节。

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

## 2026-11-15 分阶段集成

| 阶段 | 提交 / 证据 | 结果 |
| --- | --- | --- |
| 文档与结构资格 | `084af65` | 已推送；项目定位、演进和22篇结构资格审查。 |
| 阶段1契约核心 | `9fa3111` | 外参雅可比、物理半径及合成测试；两目标/11用例、真实节点构建及链接检查通过。 |
| 阶段1运行记录 | `73aa72c` | 已推送；原有六后端30秒同输入完整回归各288帧及契约记录。 |
| 单调时钟 | `f86ebd6` | 使用 steady_clock 修复系统时钟回拨产生负耗时；六后端30秒和141秒、0.5米完整回归通过，各288/1402帧，全部8个时长字段有限非负。早先完整长回放的轨迹有效，耗时无效，见[阶段1结果](evidence/STAGE1_RESULTS.json)。 |
| R-VoxelMap 核心 | `b820b08`（feature `af44fc0`） | 原生递归地图、概率观测、安全修补及专项测试合入；隔离 CTest 4/4、23用例（含正向 ASan），主线常规 CTest 3/3、22用例通过。 |
| R-VoxelMap 运行验收 | [结果记录](evidence/R_VOXELMAP_RESULTS.json) | 30秒/141秒完整输入回放通过，分别288/1402帧且每帧非零约束；主线构建/链接及30秒复测通过。 |
| hVox 核心 | `c81f12f`（feature `997c00a`） | 双层预计算 surfel、原生移动盒和 PKO 观测接入；隔离6/6 CTest、34用例及数学复核通过。 |
| hVox 运行验收 | [结果记录](evidence/HVOX_RESULTS.json) | 30秒/141秒完整回放通过；主线构建/链接、5/5 CTest（33用例）及30秒复测通过。 |
| i-Octree 设计 | `8d5c040` | 单独记录结构接入审计、浮点包围盒反例及实点AABB/隔离距离核方案；尚未作为主线后端验收。 |
| 诊断累加竞争 | `f6403f0`（feature `1672ee0`） | 三个观测分支的诊断求和移到并行循环后；真实节点编译及3分支30秒完整输入回归通过，见[结果](evidence/DIAGNOSTIC_RACE_RESULTS.json)。 |

代码和运行记录分别提交；源代码/二进制/配置及输入身份均记录。R-VoxelMap 已成为主线第七个后端。hVox 在独立工作树通过数学审查、6/6 CTest（34用例）及30/141秒完整回放后，以 `c81f12f` 合入主线；主线节点构建/链接及30秒复测通过，主线常规 CTest 5/5（33用例）通过。hVox 是第八个 selector。ASan 泄漏检测未启用且有一次原因未知的挂起后重试通过；这些限制以及无真值/非受控计时的边界详见结果文件。

## 工作树与仓库体积审查（2026-11-15）

工作空间根目录保持不建仓。对远端 `d2d17bc` 的41个可达提交逐树核验，均无 `experiments/`、build/devel/output、bag/PCD或编译产物。分阶段验证目录是共享对象库的 linked worktree；R-VoxelMap 与 hVox 的两个已完成检出已移除，11份未提交文档均先以SHA核验备份。当前仅保留正在验收的i-Octree检出。本轮相对 `084af65` 的当前树净增401736字节；已有论文及GIF合计约250MB。原始数据、构建和运行产物保留在仓库外；精确口径和清理记录见[审查结果](evidence/WORKTREE_AUDIT.json)。
