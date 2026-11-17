# 第二类：观测与权重组件集成方案

更新：2026-11-17。原计划 `7a83f0c` 对应改写后历史 `81b136a`；源码交付于 `acc3279`。GM 首包已接入，仍只有 `ig_lio_plane`、`c=0.1 m` 的 30 秒固定输入 A/B；GenZ 尚未实现，`observation/model=genz_hybrid` 启动时明确拒绝。第一类本轮补齐 i-Octree adapter 30 秒专项和 BIEVR/iG-LIO 默认权重 197.190 秒支持窗回归，见[验收记录](workspace/evidence/MAP_ACCEPTANCE_20261117.json)。这些长窗没有启用 GM，不能扩大其 A/B 覆盖。完整优先级见[计划](../PLAN.md)，GenZ 变体见[设计](GENZ_HYBRID_DESIGN.md)。

## 准入与边界

第二类不要求创新地图数据结构；要求核心改进位于观测残差、Jacobian、协方差/信息、鲁棒权重或轻量关联门限，能够在共同状态与 IKFoM 更新内独立实现。保留现有同步、IMU 传播、去畸变、点云采样和输出；不导入上游 ICP 求解器、另一套滤波、平滑窗口或后端优化。局部几何统计属于关联输入，须说明点集、坐标、单位与搜索域。

| 候选组件 | 拟纳入范围 | 实施状态与条件 |
| --- | --- | --- |
| KISS-ICP 启发的 Geman–McClure 权重 | 对已有标量点面残差进行固定尺度鲁棒加权；仅移植权重思想，明确为点面组件变体。 | 已接入。为四个共享点面后端提供默认关闭的选项；不改变已有匹配门限。完整 KISS-ICP 的点点关联和自适应尺度另作后续设计。 |
| GenZ-ICP 混合观测 | 局部几何分类、点面/点点残差及按匹配点类别计数的混合信息；映射至共同 12 列右扰动。 | 待实现。作者 checkout 固定为 `b19b2455e9b1272a68990969271685af54126b0d`；按本地变体设计增加 typed 对应与原始邻域接口，替换为 ikdtree 原生 KNN。不是完整 GenZ 复现；不导入上游自适应体素化或注册求解器。 |

来源：[KISS-ICP 作者论文](https://arxiv.org/abs/2209.15397)、[固定权重源码](https://github.com/PRBonn/kiss-icp/blob/1ffa7d7512f10bfc8b1185095011fa31184019e3/cpp/kiss_icp/core/Registration.cpp)、[GenZ-ICP 作者论文](https://arxiv.org/abs/2411.06766)、[作者源码](https://github.com/cocel-postech/genz-icp)。固定 GenZ checkout 的核心目录干净，仅 `ros/CMakeLists.txt` 有本地改动；固定提交与脏文件不构成已移植或可沿用旧构建结果的证明。作者参数 `k=σ/3` 的长度单位与被加项 `r^2` 的平方长度不一致，本地实现不沿用该尺度写法。

## 选项与兼容关系

| 选项 | 设计值 | 首包支持与默认 |
| --- | --- | --- |
| `mapping/map_type` | 现有地图名称，以及将来新增的结构后端 | 第一类选择器；不因第二类选项改变。 |
| `observation/weight_model` | `backend_default`、`geman_mcclure` | 首包实现，默认 `backend_default`；GM 只允许 `ikdtree`、`ivox`、`i_octree`、`ig_lio_plane`。 |
| `observation/gm_scale_m` | 正有限长度，单位米 | GM 显式尺度；profile 提供起始值，需按数据校准，不是作者自适应尺度复现。 |
| `observation/model` | `backend_default`、`genz_hybrid` | 当前支持 `backend_default`；`genz_hybrid` 尚未实现，并在节点启动时拒绝。 |
| 自适应关联门限 | 独立布尔开关与尺度状态 | 后续设计；首包不改变搜索范围或匹配点集。 |

GenZ 启用后须将 `observation/weight_model` 设为 `backend_default`，由混合观测本身构造信息，避免叠加第二层 GM。GenZ 观测行数按 1 行/点面、3 行/点点计数；混合系数则按两类匹配点数计算，二者不能混用。

观测配置独立放入 `config/observations/`，launch 在 sensor、backend 配置之后按需加载，未选择时保留传感器配置值及旧行为。兼容 launch 透传选项，runner 须保存选项、实际 profile 内容与 SHA-256。非法名称、不支持的地图组合、非有限或非正尺度须在节点初始化时拒绝，不能静默回退。默认 profile 应显式恢复后端权重，便于做开关对照。

## 首包数学契约

对固定匹配的米制标量残差 \(r\)，GM 尺度 \(c>0\)，采用零残差归一化权重：

\[
q(r;c)=\left(\frac{c^2}{c^2+r^2}\right)^2,
\qquad W_i=q(r_i;c)W_{i,\mathrm{backend}}.
\]

`dyn_share.R` 在本项目存信息（逆方差）\(W\)，不是协方差；保持 \(h=-r\) 与原 Jacobian \(H=\partial r/\partial\delta x\)，只乘一次 \(q\)，不再缩放 `h/H`。固定尺度、点面残差和零点归一化是本地组件的显式定义；其信息绝对尺度与上游 LiDAR-only 最小二乘可能不同，不能忽略相对 IMU 先验的尺度。

## 验收与排期

1. BIEVR、iG-LIO 与 iG-LIO+GM 当前集成变体已使用同一份 SOLID 逐点时标数据（回归记录 01）完成 30 秒短回归，并通过输入/输出完整性检查；按实际输入、配置和产物检查结论见[本轮记录](workspace/evidence/INTEGRATION_PROGRESS_20261117.json)。BIEVR 保留定向高度图、像素融合/重投影/LRU 与独立高度观测，不带入作者 LM/Ceres，也不转用旧快照构建证据。BIEVR/iG-LIO 默认权重又通过 197.190 秒支持窗，各 1959 帧；末尾两帧缺 IMU 支持而被裁出，见[长窗证据](workspace/evidence/MAP_ACCEPTANCE_20261117.json)。高度噪声标定、退化与精度仍待后续执行。
2. GM helper、启动组合校验、可选 profile 和 runner 归档已接入；权重极值、非法参数、行数和回滚测试通过。固定输入 GM on/off 仅在 `ig_lio_plane`、`c=0.1 m` 这一组合完成 287 帧配对回归；其余三个兼容地图组合尚未做 A/B。该轨迹差异不是精度改善证据，亦无受控时延对比。
3. 按[GenZ 设计](GENZ_HYBRID_DESIGN.md)实现独立 typed 匹配和原始 KNN 邻域 API；用真实状态 `boxplus` 对 12 列 Jacobian 做中心差分，并核对混合二次型、梯度和信息块、尺度单位、类别计数、退化回退及 23 行门控/失败回滚后，再接共享滤波。
4. 分别记录静态、编译、合成测试和回放结果；源码接入、合成通过、实际回放与精度效果分开写。固定数据缺失时继续可独立实施的工作，保留回放待验状态。

GM 开关验证包括实际 Noetic 离线 launch 覆盖与 runner profile 归档、启动组合纯函数测试、权重极值和回滚边界测试；默认路径只作静态保持与原后端合成回归，不宣称逐位 A/B。GenZ 固定源码中的 `k=σ/3` 被加到平方残差存在量纲不一致；本地混合观测采用单独定义的米制固定尺度 `c`，不将上游该写法当成本首包固定 GM 定义。
