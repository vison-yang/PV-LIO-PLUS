# i-Octree

论文：[i-Octree.pdf](../i-Octree.pdf)，arXiv v2（2309.08315v2），*i-Octree: A Fast,
Lightweight, and
Dynamic Octree for
Proximity
Search*。作者代码：[zhujun3753/i-octree](https://github.com/zhujun3753/i-octree)；工作区副本
[upstream/i-Octree](../../../../upstream/i-Octree)，核心头文件
[Octree.h](../../../../upstream/i-Octree/octree2/Octree.h)。

## 问题与结构

固定八叉树在逐帧增量插入和删除中可能反复重建；指针式节点也会带来内存开销。i-Octree
是支持近邻查询和动态更新的叶节点八叉树，重点放在局部数据移动、连续内存布局及树上更新（pp. 1–2）。

八叉树仅创建有点的子八分体；节点保留点索引与坐标。叶节点的点在一段连续内存中存放，局部重分配可维持空间连续性，并让盒删除直接作用于相交叶段（p. 2）。

## 插入、删除和查询

批量构建时先过滤无效点、求包围盒，再按八分递归划分；满足停止条件后分配连续段保存叶数据。增量插入把新点定位到叶；超过容量或触发细分时只更新相关路径，不重建整棵树。更新逻辑支持树内降采样（pp.
2–3）。

盒删除递归遍历与查询盒相交的节点，局部移除命中的点；kNN 和半径近邻查询利用八叉树空间界定候选区域（pp. 3–4）。工作区公开接口包括
initialize、update、knnNeighbors
和 boxWiseDelete；要用于
PV 点面匹配，还需在查询返回后按公共适配器拟合局部平面。

## 作者实验

随机点实验比较树规模变化时的更新时间、查询耗时和内存；真实数据实验包括激光里程计地图更新、每扫描插入/重建时间以及轨迹误差（pp. 4–6，Tables
I–VI、Figs.
5–7）。摘要报告作者测试中相对同类结构平均运行时间减少；正文还讨论盒删除开销和不同数据序列上的精度/速度折衷。论文的 LIO
应用实验并不等于本地适配后的表现。

## 2026-11-15 对 PV-LIO-PLUS 的范围说明

- 当前 selector 为 `i_octree`，feature `5c27a79` 已以 `6876a8e` 合入主线。该适配保留动态八叉树、叶端连续点块、增删和树上降采样，并以实点 AABB、binary64 查询/距离及受控浮点模式检查修正数值边界；不能将它描述为未修改上游的性能复现。
- 适配中的 kNN 按声明的 binary64 度量遍历全树；生产第五近邻门限接受 `d² <= R²`，native radius API 则使用严格 `< r²`。闭合局部窗口通过树上保留点实现，不重建整棵树；`size()`/`point_count()` 报告当前存活点，snapshot 为 XYZ。生产语义、异常保证及限制见[本地源码说明](../../include/map_manager/native/i_octree/SOURCE.md)。
- Feature 隔离 CTest 9/9（49 个常规用例与 11 个 sanitizer 重复执行，共 60 次执行），独立数值审查、正向 ASan/UBSan、30/141 秒完整输入回归均通过；LeakSanitizer 未启用。主线节点/链接、CTest 7/7（49 个常规用例，ASan 关闭）及 30 秒完整输入回归通过。机器统计与验证限制见[集成状态](../../doc/INTEGRATION_STATUS.md)和[结果记录](../../doc/workspace/evidence/I_OCTREE_RESULTS.json)。这些回归不代表精度或受控性能结论。

## 局限

叶数据连续存储改善局部访问，但插入后扩展/搬移仍有更新代价。桶容量与叶尺寸影响树形和耗时；树内降采样改变保留点集，但论文的 kNN 在该保留点集上是精确查询，不能与有限体素邻域的近似 kNN 混淆。移植时仍须保持查询和树生命周期语义（pp. 2–6，§II-C）。
