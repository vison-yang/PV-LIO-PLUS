# BIEVR 高度图到共同 IKFoM 的设计与实施记录

初始设计日期：2026-11-15；当前实施状态：2026-11-16。

## 当前实施

`bievr` 已接入本项目，依据当前可读取的干净作者快照 `2306022e341e98ce84244f92ba7d26f047b41dbc` 安全重实现 C++17 地图核心。旧 `38f98b9` 在当前浅克隆不可读取，旧独立构建不转用于新版本。来源、逐文件 hash 和本地差异见 [SOURCE](../include/map_manager/native/bievr/SOURCE.md)；本轮构建与合成测试见[结果记录](workspace/evidence/TWO_TRACK_RESULTS.json)。尚未回放或标定高度噪声，不是作者完整 LIO 复现。

实现保留定向图像、n−1 统计、pending 成熟投影、加权融合、掩码平滑、法向重投影及 LRU；修复单体素分组，检查 Morton/像素整数域与图像安全上限。公共完整去畸变云以 double 变换后写图，真实 IMU 系量程按当前外参逐点计算；注册使用公共降采样点。每次 callback 重新查询，采用精确 masked-bilinear 导数、独立 12 列观测和 Huber/σ² 信息；没有作者 LM/Ceres 或 MID 采样。

新观测少于 23 行、任意迭代拒绝或最终状态/P 非有限时，恢复预测 x/P 并跳过本帧写图。此行数门限不是秩保证，稀疏初图可能停滞；原生 n≥4 成熟规则也不解决重合/共线退化。参数预检拒绝不写图，积分中的资源/数值异常可能部分提交并传播。σ 为本地显式噪声尺度，需标定；方程依据当前源码与真实 boxplus oracle，未逐式复核 PDF。

## 历史设计稿

以下为 2026-11-15 按旧快照形成的设计记录；“拟议/待实现/前置项”描述当时的规划，当前实现状态与差异以上节及 SOURCE 为准，历史行号不能直接当作新作者快照行号。

## 1. 固定依据与可分离边界

SOURCE_MANIFEST 指向工作空间 `upstream/BIEVR-LIO`，本次核对 HEAD 为
`38f98b900dad0e562cddfff16f85f681288a95ed`，工作树无修改。
以下作者路径均相对该目录；`map.cpp` 指 `BIEVR/src/bievr_map.cpp`，
`ls.cpp/h` 指 `BIEVR/src/ls_optimizer.cpp` / `BIEVR/include/bievr_lio/ls_optimizer.h`。
论文身份为仓库 `paper/BIEVR-LIO.pdf`，范围说明见 `paper/notes/BIEVR-LIO.md`。
本轮未重新逐式核定 PDF 及论文页码；论文公式的版本对应仍是设计前置项，
下列可执行判定以固定作者源码为准，不把既有笔记公式当作已复核原文。

| 已核定事实 | 一手源码证据 |
| --- | --- |
| 原系统先把 LiDAR 点变为 IMU 点，再预积分、去畸变和 LiDAR 位姿注册 | `BIEVR/src/pipeline.cpp:74,97–139` |
| 原注册是 6 维 Huber LM；右乘 `xi=x0*delta`，增量顺序为旋转、局部平移 | `ls.cpp:163–195`，不是公共状态的世界平移列 |
| 注册后另开 Ceres 惯性窗口，固定两端姿态/位置，求速度、偏置和重力并加重力先验 | `pipeline.cpp:285–338`；窗口清理 `253–269` |
| 结构为 Morton 键体素、定向像素高度/融合权重、累计统计、未成熟点缓冲和 LRU | `BIEVR/include/bievr_lio/bievr_map.h:14–38,96–103` |
| 成熟体素清掉 pending 原始点，不保留通用 kNN 点集 | `map.cpp:120–135`；不能退化成临时 KNN 平面 |

最小抽取范围：`bievr_map.h/.cpp`、必要 `common.h` 点云/变换类型、Morton helper、
`utils` 中邻格偏移及 Gaussian kernel、`unordered_dense` 与原许可/出处。
将 `ls.h:27–152` 的纯像素采样分离为私有数值 helper；保留源对照和本地修改清单。
不导入 `LsqRegistration::computeTransformation/stepLm`、Pipeline、IMU integrator、
inertial/prior factor、Ceres 窗口、同步器或去畸变器；新依赖先审 TBB/Eigen 边界。

## 2. 地图生命周期与不能丢失的结构

`MapType::BIEVR` / `bievr` 只是后续命名建议；设计拟由公共主循环串行执行地图构建、更新和清理。
适配时拟以世界系 XYZ 作为地图更新输入；每点还应保留真实 IMU 系量程，不能从世界系原点距离推测。
作者实际调用是 `calculateRanges(points_undistorted_I)`（`pipeline.cpp:121–122`；
`BIEVR/src/common.cpp:6–12`），因此是 `||R_IL p_L+t_IL||`，不是笔记所写的 LiDAR 系量程。
原地图以 `sum/outer_sum/n` 更新法向，至少 4 点成熟；实际 scatter 分母为 **n−1**
（`map.cpp:158–198`），既有笔记的 n+1 不能作为实现依据。
适配时应保留 `T_O_W` 法向坐标、带像素原点的 `T_C_W`、法向变化阈值及图像重投影，
以及 `round` 像素写入、范围权重、可选有效掩码 Gaussian 平滑与 MID（`map.cpp:203–397`）。
写入权重为 `weighted ? min(0.5,1/range) : 1`，与配置像素融合语义绑定。
`config/params.yaml` 明设 smooth/weighted=true；结构体和 loader 缺参默认 false，必须记录实际值。
成熟前 pending 点须在首次成熟时投影一次并释放；LRU 按更新触碰，容量按体素数。
拟提供的 `clear/configure/reinitialize` 应销毁并重建 native 地图、递增 generation，不保留旧像素指针。
建议将 `size()` 定义为当前体素数；快照应从有效像素提升为世界点并标明是表面像素快照，
不是历史输入点数量。作者没有共同几何窗口 API；若首版处理 common local_window，设计应明确拒绝该模式。

原系统用完整去畸变云更新地图（`pipeline.cpp:143–146`），注册点集另选。
后续适配应显式登记 `map_update_cloud=common_undistorted_full`，保持点与 range 顺序一一对应；
若先用公共降采样云更新，须另命名/记录这一地图分辨率变体，不假称完整原生数据策略。

## 3. 查询、像素和线性化边界

先查所在体素；不存在/未成熟时才以邻格质心距离找 observed 体素
（`ls.cpp:66–79`，`map.cpp:399–438`）。所在体素存在但像素无效时，作者不再搜索别的体素。
不能用全局最近点/平面替代该路径，也不能在无匹配时补造零高度观测。
拟议观测沿用作者的 `p_C=T_CW p_W` 坐标变换，以 `u=p_C.x/px_size`、`v=p_C.y/px_size` 查询；高度 `I` 和残差均为米。
`getSubPixelValue` 不是要求四角全有效：它用 `weight>0` 的二值掩码重归一化双线性权重；
中心 2×2 超图像边界或有效权重和为零才拒绝（`ls.h:27–57`）。
`sampleValueAndGradient` 返回相同中心高度，但梯度由 `I(u±1,v)`、`I(u,v±1)` 中心差分，
外侧 stencil 缺失/越界时该轴梯度置零（`ls.h:60–152`）。
因此 native gradient 对一般非仿射或缺角图像只是近似，不能要求它普遍通过残差精确 FD。

第一版建议每次公共观测 callback 按当前状态重新查询/插值；整个滤波更新期间地图只读。
拟定的线性化边界是：一次局部线性化固定 voxel、`T_CW`、图像、有效掩码、floor 选定的 stencil 和去畸变原始点，
仅对 stencil 内连续坐标求导；不对 hash/floor、邻格选择、重投影或历史地图融合求导。
跨 voxel/像素边界重新选支，不对不连续边界做中心差分验收。
如以后缓存，须绑定 frame+map generation，缓存拥有值或受控只读句柄，
不得跨 LRU 更新保存裸 `Voxel*`；每次仍重算当前 world/C 点、r、梯度和鲁棒权重。

## 4. 残差到共同状态的映射

建议先审 **`bievr_height_ikfom_bilinear`**：保留作者地图、masked-bilinear 高度和 Huber，
对实际插值函数求精确分段导数；这是明确更换 Jacobian 的共同滤波变体。
另可研究 `bievr_height_ikfom_nativegrad`，保留原中心差分梯度，明确其 Jacobian 近似。
两者均不是作者 LM 求解器的复现，也不承诺与原系统估计结果相同。

拟议适配中记 `A=R_IL`、`t=t_IL`、`R=R_WI`、`p=p_WI`；公共观测使用去畸变 LiDAR 点 `p_L`：
`a=A p_L+t`，`p_W=R a+p`，`p_C=Q p_W+c`，其中 `Q=R_CW` 为固定地图变换。
`r=p_C.z-I(u,v)`，提交滤波器的 innovation 为 **h=−r**。
固定掩码 `m_j∈{0,1}`，双线性基函数 `b_j`，令 `N=Σm_j b_j I_j`、`S=Σm_j b_j`：
`I=N/S`，`I_u=(N_u S−N S_u)/S²`，`I_v=(N_v S−N S_v)/S²`。
该式直接来自已核定的采样代码；只在 S>0、同一 stencil、有限输入的连续分支成立。
`g_C=[−I_u/px_size, −I_v/px_size, 1]ᵀ`，`g_W=Qᵀg_C`，`C=Rᵀg_W`。
拟采用的公共前 12 列顺序为 pos、右扰动 rot、右扰动 extrinsic rot、IMU 系 extrinsic translation：

```text
H = dr/dδ = [ g_Wᵀ, (a×C)ᵀ, (p_L×AᵀC)ᵀ, Cᵀ ]
```

若 `extrinsic_est_en=false`，拟将最后六列置零，前六列不变。

`include/use-ikfom.hpp:12–13` 给出状态顺序；SO3 右乘见
`include/IKFoM_toolkit/mtk/types/SOn.hpp` 的 `boxplus`；对比作者 `ls.cpp:101–112,174–180`。
拟议的 LiDAR 行不直接填写速度、bias、gravity 列；其更新通过公共协方差交叉项传递。
**不得归一化 g_W**：其长度包含高度图切向梯度信息；不能塞进会正规化 normal 的通用点面路径。
应设计独立 `HeightMatch/HeightObservation` 记录与分派，保留 `p_L`、voxel generation、图像坐标及梯度语义。
不要把作者六列 `[rot,local translation]` 原样复制到公共 `[world pos,rot,...]`。

## 5. 鲁棒权重、不确定性和近似

作者 `Accumulator::add`（`ls.h:162–174`）使用原始米制 r：
`w=1` 当 `|r|<=delta`，否则 `w=delta/|r|`；累计 `w JᵀJ`、`w Jᵀr`。
δ 来自 registration config：随包 params 显式为 0.1 m；loader 缺参默认为 100，
不能取 Accumulator 的 0.2 或任意缺参值冒充运行配置（`ls.cpp:58–59`，`config_loader.h:242–247`）。
原 LiDAR LM 没有独立测量方差/残差协方差，像素融合 weight 也没有被用作 LM information。
首版建议显式配置 `sigma_height_m>0`，公共对角 information 为 `w/sigma_height_m²`，
接口 `dyn_share.R` 存逆方差，见 `esekfom.hpp:2105–2142`；不要再次平方 w。
这是相对公共 IMU 先验的新噪声尺度选择，必须先定标/消融；不声称来自作者概率模型。
scatter 是坐标分布（m²），像素 weight 是融合权重，均不是已标定的高度残差逆方差。
拟议信息模型不重复加入共享状态 P 形成的 pose covariance，也不导入作者重力 prior/惯性窗口。
未来若加入点传感器协方差或像素相关性，须另做信息模型；本设计的对角接口会忽略共享像素相关性。
零残差 w=1；非有限 r/H、非有限或非正 sigma²/info、无观测分支须显式拒绝，不能虚构约束。

## 6. 导入前必须关闭的具体缺口

1. **单体素批次分组缺陷**：`map.cpp:79–92` 把 prev_hash 初始化为末项，
   所有点同 hash 时不产生组起点，整批不积分却返回 true。先建立独立分组 oracle，最小修补后验证并记录差异。
2. **梯度关闭分支未初始化**：`ls.cpp:105–112` 的 I_Jac 仅在 img_jacobian=true 时赋值；
   第一版要求 img_residual/img_jacobian=true，抽取 helper 时显式初始化；消融不能沿用未定义值。
3. 输入有限性、range 数组长度/正值、floor/round 到 int 的域、像素写入边界须在写图前验证。
   Morton 仅编码每轴 21 位且直接作 map key（`common.h:231–244`）；须定义无别名索引域，
   同时覆盖邻格偏移，不能只检查 int 可表示而允许周期键碰撞。
4. 零/少点、共线或奇异 scatter、法向夹角 acos 输入、重投影落同像素的覆盖次序和空图尺寸，
   须核定原行为及最小有限性防护。不得把未论证的特征值门槛静默包装成作者规则。
5. 补核固定 PDF 的方程、像素坐标约定及原论文梯度描述；sigma、极小 S 的数值策略、
   生命周期快照 seam 和本地修改清单评审通过后，才能派实现。

## 7. 最小阶段包与验收

- A：只抽地图/数值采样，保留出处；单/多体素积分、4 点成熟、pending 清空、n−1统计、
  pixel weighted/smooth 开关、法向旋转/重投影、LRU、负坐标及 Morton 域测试通过。
- B：冻结两种命名 profile 和 sigma；对仿射高度图、一般双线性面、缺一/多角掩码，
  用独立高度值函数和实际 MTK boxplus 做 12 列中心差分，覆盖非单位姿态及外参开关。
  仿射完整 stencil 应同时核对 nativegrad；一般图像只核对其原中心差分数值，不冒充精确 FD。
- C：验证 Huber 在 0、±δ、阈值两侧的 information，单行/多行 `ΣHᵀWH` 和 `ΣHᵀW(−r)`；
  验证像素边/角、零权重和外侧梯度 stencil 缺失、无匹配、非有限、更新后的缓存失效。
  平面图像切向弱约束、斜坡及 bump 的梯度符号、法向翻转/重投影须有合成几何 oracle。
- D：单独接 MapManager+高度观测分派，检查 native 求解器/第二 IMU 状态未链接入执行路径；
  旧后端回归后，再以已核准标定/单位/时间戳的短 bag 验证输入排空、末 stamp、有效像素/行数、
  有限 h/H/info、地图容量与配置/source/binary 身份。失败显式记录，不用非空轨迹当完整成功。

注册采样拟在首版沿用公共采样并记录分辨率；`sampleInformed`（`BIEVR/src/preprocess.cpp:68–152`）
的 MID 选区/双分辨率采样为独立开关、独立阶段和独立 run manifest，不能由地图 selector 暗启。
完整作者系统只能作为独立基线；共同 KF 的 IMU 先验、迭代/对应点刷新及信息尺度会改变估计行为。
