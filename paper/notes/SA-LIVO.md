# SA-LIVO

本文的推导、实验数字和配图基于论文 **v2**：[`SA-LIVO-v2.pdf`](../SA-LIVO-v2.pdf)，题目为 *SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with Subspace-Aware Degeneracy Handling*。

最新论文为 **v3**：[`SA-LIVO.pdf`](../SA-LIVO.pdf)，题目已改为 *SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with Subspace-Aware Information Fusion*。v3 的方法与实验内容均有变化；下文主体总结 v2，文末另附 v3 独立摘要；旧推导、表格和配图不代表 v3，两个版本的结论不可混用。

## 背景和相关研究

LiDAR 在走廊、隧道、平面墙面等环境中会出现几何退化；视觉在暗光、运动模糊和低纹理场景中也会失效。常见的“整块关闭某一传感器”策略没有利用退化只发生在部分状态方向这一事实，可能丢掉仍然可靠的互补约束。SA-LIVO 的目标是把 LiDAR、视觉的退化判断统一到信息矩阵的子空间中。

## 核心算法和推导

**LiDAR 约束。** LiDAR 点先变换到 IMU 和世界系：

$$
\begin{aligned}
p_{\mathrm{I}} &= R_{\mathrm{LI}} p_{\mathrm{b}} + p_{\mathrm{LI}} \\
p_{\mathrm{w}} &= R p_{\mathrm{I}} + p
\end{aligned}
$$

局部平面用中心 $p_{\mathrm{bar}}$、法向 $n$ 和最小特征值描述，点到平面残差为：

$$
\begin{aligned}
r_{\mathrm{i}} &= n_{\mathrm{i}}^\top p_{\mathrm{w}} + d_{\mathrm{i}}
\end{aligned}
$$

右扰动下，外参已知时的 6-DoF 位姿 Jacobian 写成：

$$
\begin{aligned}
h_i &=
\begin{bmatrix}
[R p_I]_\times n_i\\
n_i
\end{bmatrix}
\end{aligned}
$$

论文用无量纲的平面可观测性指标筛选平面：

$$
\begin{aligned}
\kappa = \lambda_{\min} / \operatorname{tr}(\Sigma_{\mathrm{pts}})
\end{aligned}
$$

当 $\kappa < 0.05$ 时接受为平面。邻域搜索采用自适应 Chebyshev 壳层，最多取 25 个点；同时根据 LiDAR 的距离、入射角和离散化误差构造点噪声。平面协方差近似为 $\lambda_{\min} / N_{\mathrm{eff}} * I_{\mathrm{6}}$，从而形成 LiDAR 信息：

$$
\begin{aligned}
\Lambda_{\mathrm{L}} = \sum_i (1 / \sigma_{\mathrm{i}}^{2}) h_{\mathrm{i}}^\top h_{\mathrm{i}} \\
b_{\mathrm{L}} &= -\sum_i (1 / \sigma_{\mathrm{i}}^{2}) h_{\mathrm{i}}^\top r_{\mathrm{i}}
\end{aligned}
$$

**视觉约束。** 视觉使用 LiDAR 提供的深度锚点进行直接法 patch 光度残差。深度相关噪声、梯度和 patch 质量会影响权重；Jacobian 在一次 InEKF 迭代开始前冻结，避免迭代期间重复计算。5 帧滑窗中，重复使用同一 LiDAR 锚点时用 $w_{\mathrm{dec}} = 1 / n_{\mathrm{used}}$ 降低过度计数。质量因子由 RMS、有效 patch 数和拒绝比例组成：

$$
\begin{aligned}
q &= q_{\mathrm{rms}} q_{\mathrm{cnt}} q_{\mathrm{rej}} \\
\Lambda_{\mathrm{V}} = \sum_j w_{\mathrm{j}} J_{\mathrm{j}}^\top J_{\mathrm{j}} \\
b_{\mathrm{V}} &= -\sum_j w_{\mathrm{j}} J_{\mathrm{j}}^\top r_{\mathrm{j}}
\end{aligned}
$$

**SAIF 子空间融合。** LiDAR 和视觉信息先相加，再对 6-DoF 外部位姿块做特征分解：

$$
\begin{aligned}
\Lambda_{\Sigma} &= \Lambda_{\mathrm{L}} + q \Lambda_{\mathrm{V}} = U \operatorname{diag}(\lambda_{\mathrm{k}}) U^\top \\
b' &= U^\top (b_{\mathrm{L}} + q b_{\mathrm{V}}) \\
\gamma_{\mathrm{k}}    = \min(\sqrt{\lambda_{\mathrm{k}}} / \sigma_{\min}, 1) \\
\tilde{\lambda}_{k} &= \gamma_{\mathrm{k}} \lambda_{\mathrm{k}} \\
\tilde{b}'_{k} &= \gamma_{\mathrm{k}} b'_k \\
\Lambda_{\mathrm{f}}    = U \operatorname{diag}(\tilde{\lambda}_{k}) U^\top \\
b_{\mathrm{f}} &= U \tilde{b}'
\end{aligned}
$$

小特征值方向被连续衰减而不是整块清零；较强的方向保持原有信息。$\Lambda_{\mathrm{f}}$ 仍为半正定矩阵，因此可以直接送入 InEKF。算法只处理外部传感器的 6-DoF 位姿子块，不对完整 18 状态做无量纲混合；实现时必须注意平移和旋转信息量纲不同，$\sigma_{\min}$ 的设定不能脱离单位。

## 实验流程和结果

实验覆盖 HILTI’22、Newer College 和 Oxford Spires，共 29 个序列，并额外构造视觉或 LiDAR 退化场景；平台为 i9-13900HX 和 Jetson Orin，推理不使用 GPU。代表性 ATE（m）如下，数值为论文 Table II：

| 场景 | FAST-LIVO2 | SA-LIVO |
| --- | ---: | ---: |
| HILTI exp10 | 0.218 | **0.075** |
| HILTI exp15 | 0.192 | **0.122** |
| Newer College Quad Hard | 0.080 | **0.055** |
| Newer College Stairs | 0.057 | **0.036** |
| Oxford observatory-quarter-01 | 0.297 | **0.055** |

HILTI 的 15 个序列中，只有 SA-LIVO 和 FAST-LIVO2 完成全部序列；在共同成功子集上，SA-LIVO 平均约 12.3 ms/frame，论文报告约为 FAST-LIVO2 的 2.2 倍。Jetson Orin 上 SA-LIVO 平均约 26.8 ms/frame；峰值内存约为基线的 3.6--6.3 倍更低。Table III 给出 HILTI 各序列约 6.9--14.2 ms/frame 的耗时，Table IV 中联合 InEKF 更新耗时占比分别约为 HILTI 46.9%、Newer College 33.0%、Oxford 39.4%。第 5 页 Fig. 2 展示 SAIF 的信息椭球衰减，第 9 页 Fig. 5 对比退化场景轨迹，第 10 页 Fig. 9/10 展示子空间选择和运行时间。

![SA-LIVO 代表性实验结果页](assets/sa-livo-results.png)

## 总结

SA-LIVO 的关键贡献不是再设计一个视觉或 LiDAR 前端，而是把“传感器是否可靠”细化为“哪些状态方向可靠”。这比模态级开关更充分地利用互补信息，但实现必须维护信息矩阵的单位、块结构和数值稳定性；论文结果主要验证了所选数据集和平台，尚不能直接等价为所有传感器组合下的实时性。

---

## 2026-11-14 地图替换边界

- 范围审查将 SA-LIVO 移出直接地图候选：主要贡献在视觉观测误差模型、重用观测相关性和子空间信息融合。
- 多尺度平面地图服务 LiDAR-视觉联合估计，单独复用地图不能带入 SAIF、Schmidt 过滤或论文完整系统表现。
- 本轮固定 LiDAR 点面适配器和滤波观测策略，不引入图像残差或联合信息权重。
- 本文原有推导、数字和图片仍属 v2；v3 独立摘要在后文。详见[范围审查](MAP_BACKEND_SCOPE_REVIEW.md)。

## SA-LIVO v3 独立摘要（2026-11-14）

本节依据默认 [SA-LIVO.pdf](../SA-LIVO.pdf)，arXiv v3（2606.25699v3），标题为 *SA-LIVO: Efficient LiDAR-Inertial-Visual Odometry with Subspace-Aware Information Fusion*。上文推导、数字和图片标签仍仅属于 SA-LIVO v2，不能与本节结果合并。

v3 将视觉异常从简单的弱信息/阈值门控，改为在线建模的观测误差。帧级噪声尺度通过协方差匹配估计，用于吸收整帧相关误差；Schmidt 过滤处理滑动窗口中重复使用视觉观测造成的相关性。子空间信息融合把一帧共享误差表示为位姿偏移，并按方向限制可接纳的视觉信息量；当视觉误差模型一致时，融合退化为普通信息相加（摘要 p. 1，方法 pp. 5–8）。

LiDAR 路径仍使用平面点面残差和体素地图；视觉路径以 LiDAR 地图点作为深度锚点构造直接光度残差。两类信息进入统一的不变 EKF 更新，而不是先后分开的两次更新（pp. 3–8）。补充材料给出点面/平面不确定性、视觉残差、相关模板处理和联合更新细节（pp. 12–15）。

v3 在 HILTI’22 的 15 个手持序列和 MARS-LVIG 的 9 个空中序列上评估，共 24 个序列，在 22 个序列最优或并列最优。摘要与时间分解报告 HILTI’22 平均 13.4 ms/frame；在所有比较方法均成功的 **7 个 HILTI’22 序列共同子集**上，笔记本 CPU 平均为 **17.2 ms/frame**，序列结束时 RSS 相较 FAST-LIVO2、R3LIVE、SR-LIVO 分别低 2.0、4.1、6.2 倍。该内存指标是结束时 RSS，**不是峰值**，不同统计子集不能合成一次整体比较（p. 1 摘要；p. 8 §VIII-A；p. 10 §VIII-E、Table IV）。这些均为 v3 作者结果，不替代上文 v2 表格。

**版本和移植边界：** v3 的噪声估计、视觉锚点、Schmidt 相关性和子空间融合与 v2 方法及数字不同。按[范围审查](MAP_BACKEND_SCOPE_REVIEW.md)，SA-LIVO 属于视觉/估计方法资料，不是本轮地图替换候选；不移植光度残差、视觉权重或联合滤波流程。
