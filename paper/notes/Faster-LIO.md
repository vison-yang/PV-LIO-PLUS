# Faster-LIO / iVox

论文：[Faster-LIO.pdf](../Faster-LIO.pdf)，IEEE RA-L 2022（DOI:
10.1109/LRA.2022.3152830），*Faster-LIO:
Lightweight Tightly
Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental
Voxels*。作者实现：[gaoxiang12/faster-lio](https://github.com/gaoxiang12/faster-lio)；论文
PDF 来自该作者仓库。

## 问题与 iVox 地图

FAST-LIO2 的 ikd-Tree 查询和动态更新适合直接点匹配，但高点数扫描下树维护与访问仍有开销。Faster-LIO 提出稀疏增量体素
iVox：用稀疏哈希体素组织点，减少一般树操作；查询时根据邻近体素收集候选，并以近似 kNN
换取固定、局部访问（pp. 1–3）。

### 查询和管理

每个体素容纳一组地图点。查询点落入体素后，按邻域模式检查有限个相邻格，论文实现支持 6、18 或 26
个邻居配置；然后从候选点中维护所需近邻。搜索因此有界且易于并行，但不是整个地图上的严格精确
kNN（p. 3）。

iVox 以增量方式加入新扫描的点，并结合局部时间窗/LRU 淘汰较旧体素，控制地图规模。论文另提出 PHC
底层布局，试图改善体素及点的空间访问局部性；它仍使用近似查询（pp. 3–4）。

Faster-LIO 的整体系统把 iVox 配到紧耦合激光惯性估计器中；其扫描处理、状态更新和地图查询时间不可简单等同于 iVox 单项耗时。

## 作者实验

论文分别测量 iVox/iVox-PHC 的插入、查询和召回率，比较 ikd-Tree、nanoFlann 等；随后在 NCLT、UTBM
等数据上报告轨迹精度、处理时间、FPS
和系统集成结果（pp. 5–7，Tables
I–III、Figs. 6–11）。作者报告 iVox 在多种扫描模式下提高地图操作速度，且在测试序列的精度接近
FAST-LIO2；同时实验展示近邻召回率和性能之间的折衷。结果限于文中硬件、邻域和体素配置。

## 2026-11-15 对 PV-LIO-PLUS 的范围说明

- iVox 点图与 PV 点面适配观测已集成。主路线记录地图、有限邻域搜索和配套观测的组合。
- 邻域数、容量和淘汰规则属于地图策略；若做统一索引消融，还需固定输入点集和点面观测。
- `nearest_max_range` 现以米表示，iVox 原生查询仍采用严格 `<r`，并限于配置的附近体素域；ikd-tree 则以 `r²` 比较第五近邻且包含边界。iVox 的旧配置值已经是米，保持不变；旧 ikd-tree 平方距离阈值 `v` 需用新半径 `sqrt(v)` 米复现。详见[集成状态](../../doc/INTEGRATION_STATUS.md)。

## 局限

固定邻域可能漏掉较远但更近的欧氏近邻；邻域大小会带来召回率和耗时的折衷。LRU 体素淘汰按空间/时序策略影响局部地图覆盖，性能依赖传感器视场和运动方式（pp.
3–7）。
