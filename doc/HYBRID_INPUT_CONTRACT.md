# Hybrid-VoxelMap 逐点语义输入契约

更新：2026-11-17。**输入待确认，当前不排代码集成。** BIEVR 与 `ig_lio_plane` 本轮短回归已通过，见[验收记录](workspace/evidence/INTEGRATION_PROGRESS_20261117.json)；在取得逐点标签来源并确认下列策略前，不得将无标签或 Gaussian-only 路径称为完整 Hybrid。

## 当前源码边界

- 静态核验 checkout：`src/Hybrid-VoxelMap` HEAD `b57ce03eaf8461d3c9a8f15db6df2e82048de9ff`。当前有 36 个 tracked 文件修改及 6 个 untracked 文件；历史对象 `89ed71c2` 不可读取。不转用旧构建或旧结果，也不覆盖当前修改。
- 地图是 hash voxel 中的 `UnionFindNode`：原格 Gaussian/NDT 统计加上跨格合并 Plane；不是八叉树。审查位置：`include/voxelmapplus_util.hpp` 的 `UnionFindNode`（约 162 行）、`InitPlane_NDT`（217）、`InitUnionFindNode`（443）、`UpdatePlane`（471）、语义更新（574）、`UpdateVoxelMap`（703/762）、残差构造（805）和查询（995）；`src/voxelMapping.cpp` 的标签转换（596）、Plane（1973）、NDT（1997）消费与地图更新。
- 当前 KITTI `lidar_type==6` 路径从 `PointCloud2.semantic`（`UINT32`）读标签并暂存于 `normal_x`（`src/preprocess.cpp` 约 254–310 行）。这是既有字段搬运，不是通用逐点语义 API，也没有为过滤后的点保留稳定原始点索引。
- `scripts/generic_semantic_field_bridge.py`（约 169 行）能接收标签并写 semantic 字段；这不等于当前完整 mapper 已有可复现的逐点标签管线。RandLA `Inf_scan_ros.py`（约 366 行）输出的是带类别的体素中心点云；当前 mapper 未消费该 topic，不能把它当作源点逐点标签。

## 接入前必须固定的输入

输入必须能为**原始帧中的每个点**提供一个离散类别，并能将其稳定关联到 mapper 实际使用的点。一个运行档案至少记录：标签生产器/模型版本、原始与标签 topic 和消息类型、坐标系、frame 标识、时间戳、点数与字段类型、类别映射和源数据哈希。若用旁路标签流，需定义同步键与允许时间差；MultiArray 没有 header 时若按序配对，必须明确记录其顺序约束与丢帧处理，不得静默退化为顺序同步。

候选最小类别编码如下，仍需由数据提供方确认，尤其是类别 3：

| ID | 候选语义 | 必须决定的地图行为 |
| --- | --- | --- |
| 0 | unknown / 无标签 | 保留为 unknown、降级、拒绝，三者选一；不得暗中改成 plane 或 Gaussian。 |
| 1 | plane | 定义 plane 资格、何时形成/合并 Plane，以及未知历史点如何参与。 |
| 2 | Gaussian/NDT | 定义协方差累计、正定性和语义类别变化后的状态。 |
| 3 | object | 当前跳过 object 的语句已注释；必须决定保留到 Gaussian、独立处理或排除，并规定帧间类别变化时旧地图点如何处理。 |

代码当前将 `1..3` 当作有效语义；类别 `2/3` 都可能被切到 NDT，类别 `3` 的地图更新跳过语句当前被注释，并非实际过滤。不能把现状误写成已确认的语义含义。

## 点身份、过滤与降采样

1. 入口为每个源点建立 `(frame_id, stamp, source_point_index)` 身份；标签必须按同一身份对齐。过滤、排序、变换和降采样后保留来源索引或可审计的贡献集合。
2. 对 blind/range/scan 过滤、非有限点及任何预采样，明示标签与点共同保留/删除规则；不得先独立过滤 XYZ 和标签数组再假定顺序仍相同。
3. 若降采样将多个源点合成一个注册点，必须定义类别投票规则：哪些类别参与计数、unknown 是否参与、平票规则、类别 3 的优先级，以及点云多层/跨帧时的贡献边界。全量地图更新点与注册降采样点分别记清使用哪套标签。
4. 当前 `transferDownsampledSemantics` 按 voxel 对有效 `1..3` 投票、忽略其他值，并因类别升序且只在 `>` 时更新而在平票时偏向较小 ID。此为当前 checkout 行为，不是已批准契约；实现前需明确保留还是改成另一条具名规则并写 oracle。

## 匹配与数值契约

- 增加独立 `HybridMatch`，显式标记 `plane` 或 `ndt` 并保存源索引、LiDAR 点及对应地图模型参数。不要用一个 `pp_res` 布尔量隐含类型，也不要把 NDT 对应塞入只代表平面的 `PlaneMatch`。
- NDT 作者残差与对源点的梯度为

  \[
  d=y-\mu,\quad m=\sqrt{d^T\Sigma^{-1}d},\quad
  s=\sqrt{\operatorname{tr}(\Sigma)/3},\quad
  r=s\,m,\quad g=s\,\Sigma^{-1}d/m.
  \]

  `Σ` 是 voxel Gaussian 协方差，均值为 `μ`。先用分解求解，检查有限值与正定性；奇异/非正定协方差拒绝该对应，不直接调用无保护逆矩阵。`m==0` 时该范数残差不可直接求导，必须先定义失效或保留完整信息的观测策略；不能将 `g=0` 当作与原二次型等价的处理。阈值、Jacobian 映射与信息权重须按最终变体单独验证。
- Plane 消费者对 `main_direction==0` 的两个平移 Jacobian 列乘 `1e-4`，这不是其声明残差的真实导数。主实现需明确命名为 corrected-Jacobian 变体，使用真实导数并通过状态 `boxplus` 有限差分 oracle；若要复刻旧缩放，只能单列 legacy 变体，不能称为 Jacobian。
- 保留原 hash/UnionFind 地图、Gaussian 统计与跨格 Plane 合并结构。语义输入不能被一个无标签通用 voxel 容器替代，否则不再具备完整 Hybrid 资格。

## 当前状态与下一步

1. BIEVR 和 `ig_lio_plane` 已通过本轮独立 SOLID 回归记录 01 短回归；该证据不覆盖 Hybrid 标签流或其地图/观测路径。
2. Hybrid 前置门：锁定逐点标签生产器与可用数据；确认类别 0/3 行为、同步键/容差、原点索引贯穿过滤与降采样投票。任一缺项时保持待输入，不以合成语义数组代替真实来源。
3. 门通过后先固定 checkout 与本地差异清单，再实现标签接入、状态映射与 typed match；为 plane/Gaussian 统计更新、合并后类别变化、奇异协方差、`m=0`、Plane Jacobian 和各类失败回滚分别建立合成 oracle。
4. 合成结果、构建、mapper 对真实标签流的消费、回放完整性和地图/轨迹效果分开报告。无标签 Gaussian-only 路径如需保留，使用独立名称和结果目录，不计为完整 Hybrid。
