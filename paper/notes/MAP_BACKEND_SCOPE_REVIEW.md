# 局部地图候选范围审核

核验日期：**2026-11-14**。这是移植范围与源码静态审核，未编译或复现实验；“移出”仅指直接地图候选表，论文与研究笔记仍可保留。

结论：优先保留 **R-VoxelMap 地图层**和本地固定版本的 **Surfel-LIO / hVox**；GenZ-LIO 仅保留其纯平面地图分支。新增限定为 **i-Octree、KISS-ICP VoxelHashMap、iG-LIO VoxelMap** 三项。不能把完整系统的准确度或速度归因于这些被抽出的地图。

## 替换边界

- 固定 IMU 传播、点级去畸变、滤波状态与外参估计开关、扫描预处理/降采样、迭代与收敛逻辑。只替换地图存储、增删/淘汰、索引、近邻或平面查询；地图内的点筛选、层级与平面拟合属于被比较的表示策略，必须记录。
- 输出仍是单位法向和中心/偏移的标量点面观测：`r = nᵀ(Rp+t-c)`。保持公共 Jacobian、信息权重构造、鲁棒核与接受门限的既定策略；地图协方差可以随表示改变，但必须同义，不能把散点几何协方差直接充当平面参数不确定性。缺失协方差的处理应统一且显式记录。
- 点点/GICP/分布匹配、高度图残差、光度/语义残差与加权、连续时间控制点、平滑器、子帧重构和自适应扫描采样均超出本轮边界。允许抽取这些系统中独立的点/平面地图，但名称应明确为“某系统的地图层 + PV-LIO 点面适配器”。

纯索引对照还需固定保留点集与平面适配器；改变体素分辨率、容量/LRU、平面模型或点集时，只能称为地图策略对照。有限体素邻域内的 kNN 也不能宣称是全图精确 kNN。

## 现有六后端：公共循环相同，观测并非完全相同

[公共循环](../../src/mapping.cpp#L896)确实共用滤波初始化、`imu_process_->Process` 与 `update_iterated_dyn_share_diagonal`；但 [ObservationModel 分派](../../src/mapping.cpp#L711)保留了不同观测实现。下表 `w` 表示代码中 `ekfom_data.R` 保存的逆方差/信息量。

| 后端 | 权重和残差证据 | 对“其它不变”的限制 |
| --- | --- | --- |
| VoxelMap | [ObservationModelVoxelMap](../../src/mapping.cpp#L246)：`w=1/(JΣplaneJᵀ+nᵀΣpoint n)`。 | 原生概率平面、匹配门限及 6 维平面协方差。 |
| VoxelMap++ | [ObservationModelVoxelMapPlus](../../src/mapping.cpp#L346)：3 参数平面协方差经 `J_abd` 传播；残差强制转 `float`。 | 外参 Jacobian 的后 6 列始终置零，未遵循其它分支的外参开关；不能视为严格相同观测。 |
| ikd-tree | [ObservationModelPointBackend](../../src/mapping.cpp#L449)及 [fit_point_plane](../../src/map_manager/map_manager.cpp#L388)。 | `plane_cov=0`，因此只保留点协方差项，并将总方差下限设为 `1e-9`。 |
| iVox | 同上，共用点图观测构造。 | 与 ikd-tree 的距离门限在实数运算下等价为 `range>81*r²`；近邻覆盖/点集仍不同。 |
| C3P-VoxelMap | [ObservationModelC3P](../../src/mapping.cpp#L619)：6 维平面协方差与点协方差相加取逆。 | 保留原生平面匹配/合并；未使用点图分支的方差下限。 |
| OctVox | [ObservationModelOctVox](../../src/mapping.cpp#L537)：`w=1000`；[search_octvox](../../src/map_manager/map_manager.cpp#L1281)。 | 固定信息量、4/5 点拟合、0.1 m 门限及容量/LRU；不是点图协方差权重。 |

另外，[calc_lidar_cov_for_backend](../../src/mapping.cpp#L227)为 VoxelMap++ 选择单独的传感器协方差实现，其余分支共用 VoxelMap 实现。建议表述为：**“共用 IMU、去畸变与滤波主循环，比较各地图及其已保留的原生点面匹配/观测策略。”** 若要声称严格地图消融，需另设统一观测策略；本审核不修改算法。

## 现有 11 项逐项判定

本轮补充实读的本地版本为：`upstream/Surfel-LIO@73348c1e5560eccd2df1ff9f87683b2326a396d4`、`src/Hybrid-VoxelMap@89ed71c2bff47b11a78dabbf244afbed6d311c5d`、`upstream/BIEVR-LIO@38f98b900dad0e562cddfff16f85f681288a95ed`。前者地图与 Estimator 文件、后者地图文件均无本地修改；Hybrid 地图头无本地修改，节点另有工作区修改，以下判定以地图头为证。其它条目的源码符号来自各链接所固定的远程版本，不声明与本地副本相同。

| 条目 | 判定 | 可以保留的独立范围 / 排除原因与一手证据 |
| --- | --- | --- |
| R-VoxelMap | **保留地图层，优先** | 递归平面拟合、有效性检查与地图更新可以独立；[作者代码](https://github.com/NKU-MobFly-Robotics/R-VoxelMap/blob/50ffbbab3626e384d77f0da91a7e51d5e4b14ff7/include/voxel_map_util.hpp)有 `OctoTree::build_single_residual`、`build`、`update`、`build_residual_omp`，输出平面及 6 维协方差，与现有概率点面合同接近；[论文](https://arxiv.org/abs/2601.12377v1)。 |
| Surfel-LIO / hVox | **保留地图层，固定本地 73348c1** | 本地 [UpdateVoxelMap](../../../../upstream/Surfel-LIO/src/core/VoxelMap.cpp#L281)在更新期收集 L0 质心、SVD 并保存 L1 法向/中心；[Estimator::FindCorrespondences](../../../../upstream/Surfel-LIO/src/core/Estimator.cpp#L822)调用 [GetSurfelAtPoint](../../../../upstream/Surfel-LIO/src/core/VoxelMap.cpp#L472)，按 L1 key 查表后读取已保存平面，查询期不做 kNN/PCA。符合[论文 v2](https://arxiv.org/abs/2512.03397v2)预计算地图方向，可接标量点面观测；排除原 Estimator/预处理，协方差仍须公共适配。 |
| GenZ-LIO | **降级为纯平面子模块，保留** | [VoxelMap.hpp](https://github.com/cocel-postech/genz-lio/blob/bf444f7a747ed3da8548b56b49c14f207e8d49bb/cpp/genz_lio/core/VoxelMap.hpp)含 `hybrid_metric_enabled_`；[VoxelPrunedSearch.hpp](https://github.com/cocel-postech/genz-lio/blob/bf444f7a747ed3da8548b56b49c14f207e8d49bb/cpp/genz_lio/core/VoxelPrunedSearch.hpp)已有 `buildPlaneResidualList`。关闭混合度量，可抽平面地图；排除 `ScaleAwareVoxelizer`、点点 fallback 和完整系统的混合更新。纯平面版本不代表[论文 v3](https://arxiv.org/abs/2603.16273v3)完整贡献。 |
| Hybrid-VoxelMap | **移出直接候选** | 本地 89ed71c 的[地图头](../../../Hybrid-VoxelMap/include/voxelmapplus_util.hpp#L841)由 `BuildResidualListOMP` 按 `is_plane/is_NDT` 调用 `BuildSingleResidual` 和 `BuildSingleResidual_NDT`，并以 `semantic_categary` 决定表示；混合残差超出固定点面观测。强制提取纯平面统计只可作为另命名的派生实验。远程 fb60275 的[无语义时 Gaussian-only 行为](https://github.com/haiyang2022/Hybrid-VoxelMap/blob/fb60275e78e010ef85f5c7574fa330774f714373/README.md)属于另一版本观察，未据此推断本地缺字段行为。 |
| BIEVR-LIO | **移出直接候选** | 本地 38f98b9 的 [BIEVRMap](../../../../upstream/BIEVR-LIO/BIEVR/include/bievr_lio/bievr_map.h#L14)含 `Voxel::bump_img_`、`integratePoints`、`nearestVoxel`；[更新实现](../../../../upstream/BIEVR-LIO/BIEVR/src/bievr_map.cpp#L123)在体素成熟时清除 `pending_points_`，不保留通用 kNN 点集。定向高度图、采样及注册目标越界；[论文](https://arxiv.org/abs/2604.14421v2)。此前远程 2306022 的观察与此地图边界相符，但两者不是同一提交。 |
| LIO-GVM | **移出直接候选 / 暂停地图抽取** | [论文](https://arxiv.org/abs/2306.17436v3)使用新的 Gaussian 残差。当前[laserMapping.cpp](https://github.com/Ji1Xingyu/lio_gvm/blob/c80c968ed7ffe61af0a02a1b3c72fe03149c5c3e/src/laserMapping.cpp#L62)依赖 `fast_vgicp/fast_vgicp.hpp`，作者发布树缺该定制地图头；不能把可访问仓库等同可独立获取地图实现。即使补齐，Gaussian-to-plane 也是新适配实验。 |
| FR-LIO / RC-Vox | **降级：仅 RC-Vox 理论候补** | [论文](https://arxiv.org/abs/2302.04031v3)的两层固定数组、机器人中心滑动与邻域索引理论上可抽出；尚未核验作者源码，无法给实现符号。完整自适应子帧与 ESKS 不纳入本轮。 |
| CT-VoxelMap | **移出直接候选** | [论文](https://arxiv.org/abs/2604.03747v1)改变 B-spline 状态、时间观测和重估策略；独立地图代码未核验，不能仅凭名称中的 VoxelMap 视作可替换后端。 |
| SA-LIVO | **移出直接候选** | [v3 论文](https://arxiv.org/abs/2606.25699v3)涉及视觉误差模型、Schmidt 相关性与不变滤波融合，不是新地图索引；作者地图源码入口未核验。第三方复现不能标作作者代码，v2 的谱门控描述也不能直接当作 v3。 |
| SV-LIO | **移出直接候选** | [正式论文](https://www.mdpi.com/2079-9292/15/8/1744)的 §4.3–4.5 将语义用于关联和残差权重；实验语义标签离线生成，未给可核验作者源码。去掉语义后只剩 VoxelMap 派生几何层，需单独命名。 |
| Environment-Adaptive Solid-State LIO | **移出直接候选** | [v2 论文](https://arxiv.org/abs/2604.15864v2)增加局部法向约束和退化感知更新；作者源码未核验。可研究独立地图维护规则，但新增约束与估计侧退化指标不属于固定点面前端。 |

“源码未核验”表示本次未找到可确认的作者实现，不能据此断言不存在源码或独立复现。

**Surfel 版本注意：** 本地调用链是 `Estimator::UpdateWithLidar → FindCorrespondences → VoxelMap::GetSurfelAtPoint`；`GetL1Surfels` 是另一个供 viewer 遍历全部 surfel 的接口，不是估计查询入口。本地没有 `GetClosestSurfel/KNearest`。本次远程 API 所见 HEAD 为另一版本 2cbfad9，其[头文件](https://github.com/93won/lidar_inertial_odometry/blob/2cbfad923583b44d04d3ab078888bd9fe55450db/src/core/VoxelMap.h#L99)及 [GetClosestSurfel](https://github.com/93won/lidar_inertial_odometry/blob/2cbfad923583b44d04d3ab078888bd9fe55450db/src/core/VoxelMap.cpp#L533)改用增量 KD-tree 后现场 PCA；这只能作为后续更新时的复核提示，不能否定本地 73348c1 的预计算 L1 实现。

## 新增三项及工程边界

| 优先级 / 候选地图 | 论文与作者代码 | 最小抽取范围与必须补齐的适配 |
| --- | --- | --- |
| **P1：i-Octree** | [ICRA 2024 / arXiv v2](https://arxiv.org/abs/2309.08315v2)；[octree2/Octree.h](https://github.com/zhujun3753/i-octree/blob/5b640e05dab23230207dd849eb1549b5074d1a0e/octree2/Octree.h)。 | 独立 `thuni::Octree` 的 `initialize`、`update`、`knnNeighbors`、`boxWiseDelete`，复用 PV 点面适配器。与 OctVox 的每格八代表点不同。无需引入 Python/Torch 演示层；注意空树输出清理、返回计数与结果长度、空输入、盒边界及删除后的点 ID。 |
| **P2：KISS-ICP / VoxelHashMap** | [RA-L 2023 作者论文](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2023ral.pdf)；[VoxelHashMap.hpp/.cpp](https://github.com/PRBonn/kiss-icp/blob/1ffa7d7512f10bfc8b1185095011fa31184019e3/cpp/kiss_icp/core/VoxelHashMap.cpp)。 | 抽 `AddPoints`、`RemovePointsFarFromLocation` 与 `map_` 的有界点集/27 格查询。原生 `GetClosestNeighbor` 只返 1 点，需在相同候选格中取 k 点后复用点面适配器。其淘汰按体素首点距离判定，不能称精确球形裁剪。排除点点 ICP、恒速去畸变和自适应匹配阈值。 |
| **P3：iG-LIO / incremental VoxelMap** | [RA-L 2024 DOI](https://doi.org/10.1109/LRA.2024.3349915)；[voxel_map.h](https://github.com/zijiechenrobotics/ig_lio/blob/2d79259e3aaa5bbb10d6b13234398760bbacf8f5/include/ig_lio/voxel_map.h)、[voxel_map.cpp](https://github.com/zijiechenrobotics/ig_lio/blob/2d79259e3aaa5bbb10d6b13234398760bbacf8f5/src/voxel_map.cpp)。 | `Grid::points_array_`、`VoxelMap::AddCloud`、`KNNByCondition` 可作独立点图。kNN 结果仅部分排序，可能不足 k 个；适配器需检查数量并统一排序。保留容量策略与邻域配置，排除 VSCE/GICP 状态更新，不把 `Grid::cov_` 直接代入平面参数协方差。 |

## 验证与可下派任务

本次核对了本地接口/观测分支、上文明确列出的三个本地上游版本，以及其余条目的远程固定版本与原论文。远程链接中的提交号只标识远程证据；本地版本以上文 `git rev-parse HEAD` 和实读文件为准。本 agent 未下载源码、修改 README 或进行性能/精度背书。

- **关键验收 oracle：** 给所有后端输入相同的已去畸变点、地图快照和迭代状态；点图的查询结果在其声明的搜索域内对照暴力距离排序，覆盖空图、少于 k 点、等距点、负坐标、体素/盒边界、插入删除与淘汰。固定匹配数据时比较公共 `h/H/w`，才能验证观测没有随 selector 偷换。
- **可下派批量笔记包：** 三篇新增论文各补“表示/增删/查询符号、可抽取范围、排除模块、论文实验条件”；另补七篇基础笔记与 GenZ/SA v3 的版本差异索引。仅摘录可核验图表，不混用跨论文实验数字。
- **需高阶实现/验证的后续：** 统一观测策略、协方差合同、KISS kNN 适配与 i-Octree 生命周期；Surfel-LIO 可直接以本地 73348c1 的预计算地图设计适配，后续更新时重新核验查询路径。
