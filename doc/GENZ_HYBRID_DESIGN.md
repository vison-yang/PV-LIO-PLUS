# GenZ 混合观测：PV-LIO-PLUS 适配设计

更新：2026-11-17。**设计稿，尚未实现；** `observation/model=genz_hybrid` 当前仍会拒绝启用。BIEVR 与 `ig_lio_plane` 本轮 30 秒短回归及 iG-LIO 的 GM 开关回归已通过，见[验收记录](workspace/evidence/INTEGRATION_PROGRESS_20261117.json)；本设计作为下一实现包，长序列与精度验收仍单列。

## 来源与声明范围

- 固定作者源码：工作区旁的 `src/GenZ-ICP`，HEAD `b19b2455e9b1272a68990969271685af54126b0d`。核心代码干净，只有 `ros/CMakeLists.txt` 有未提交修改。该源码状态不代表本包已移植，也不授权复用旧构建结果。
- 本地适配保留 GenZ 的局部 PCA 分类、点面/点点观测和按匹配点类别数混合信息；**不**导入其 27 格查询实现、自适应扫描体素化、ICP 注册求解器、另一套状态估计器或后端优化器。
- 上游 `k=σ/3` 以长度量进入平方残差，量纲不一致。本地版本改用明确的米制固定尺度 `c`；该移植是命名适配变体，不宣称完整 GenZ 复现。

## 匹配输入与分类

新增独立的强类型对应关系，不从已经丢弃非平面匹配的 `PlaneMatch` 反向拼装：

```text
GenzMatch {
  sourceIndex, pointLidar, targetWorld, normalWorld, kind  // kind: plane | point
}
```

另提供原始地图邻域接口：按当前位姿将注册点变换到世界系后，对 ikdtree 做原生 KNN，`K=max(nearest_point_count, 5)`，只保留严格满足 `d<dmax` 的邻点。`targetWorld` 是邻点中的最近地图点，不是邻域质心。无邻点时没有对应关系；有最近点但有效邻点不足 5 个时生成 point 对应。邻点重复或退化到共线、使 `lambda1-lambda0<=epsC` 时也归 point；非有限协方差、特征分解失败或输出非有限时拒绝本次更新，由帧级保护恢复预测状态。

对有效邻域 (x_j) 使用总体协方差

\[
\bar{x}=\frac1N\sum_jx_j,\qquad
C=\frac1N\sum_j(x_j-\bar{x})(x_j-\bar{x})^T,
\]

并按升序记特征值 \(\lambda_0\le\lambda_1\le\lambda_2\)。令

\[
\epsilon_C=64\,\epsilon_{\rm machine}\max(\operatorname{tr}C,d_{\max}^2).
\]

若 `tr(C)` 非有限则拒绝更新；若 `tr(C)<=epsC` 或 `lambda1-lambda0<=epsC`，点退化到 point 对应。其余情况在

\[
\eta=\lambda_0/\operatorname{tr}C<0.1
\]

时分类为 plane，并取最小特征值对应向量为 `normalWorld`。协方差单位为 m²，`dmax` 单位为 m。KNN 的严格距离边界与退化分类均须由合成用例锁定。

## 共同状态观测

注册点 \(l\) 在世界系为

\[
y=R(E l+t)+p,\qquad u=El+t,
\]

其中 \(p,R\) 是 IMU 世界位姿，\(E,t\) 是 LiDAR 到 IMU 外参。对右扰动列序
\(\delta x=(\delta p_W,\delta\theta_I,\delta\phi_L,\delta t_I)\)，其中各块均为 3 维，令

\[
B=\begin{bmatrix}I&-R[u]_\times&-RE[l]_\times&R\end{bmatrix}.
\]

固定当前对应、PCA 和权重计算时，残差与 IKFoM 测量量为：

| kind | 残差 | Jacobian | 测量量 |
| --- | --- | --- | --- |
| plane | \(r=n^T(y-q)\) | \(H=n^TB\)，1×12 | \(h=-r\)，1 行 |
| point | \(e=y-q\) | \(H=B\)，3×12 | \(h=-e\)，3 行 |

\(q\) 是固定的最近地图点；plane 的 \(n\) 是固定世界法向。外参估计关闭时，最后 6 列置零。更新必须映射到公共状态同一右扰动列，不增加或隐藏另一套求解器。

## 混合信息与失败边界

设本次有效匹配中 plane、point 数分别为 \(N_{plane},N_{point}\)，以匹配点数而非展开后的观测行数计算

\[
\alpha=\frac{N_{plane}}{N_{plane}+N_{point}}.
\]

`N_plane+N_point=0` 时跳过更新。固定尺度 \(c>0\) 与测量噪声尺度 \(\sigma_m>0\) 均以米计，令

\[
q(s;c)=\left(\frac{c^2}{c^2+s}\right)^2,
\]

并构造信息（逆方差）

\[
W_{plane}=\alpha\,q(r^2;c)/\sigma_m^2,\qquad
W_{point}=\frac{1-\alpha}{\sigma_m^2}q(\|e\|^2;c)I_3.
\]

点点的 3 行共用一个向量范数权重，不除以 3，也不按类别再归一化。每项信息单位为 m⁻²。现有 `dyn_share.R` 存信息；堆叠 `H/h` 时直接提供此信息，不再白化一次。关联、PCA、`alpha`、`c`、`sigma_m` 和 IRLS 权重在一次更新线性化内固定；不把含状态协方差的 `cov_world` 再加进观测噪声。

展开行数

\[
m=N_{plane}+3N_{point}
\]

须满足现有 `RunGuardedUpdate` 的 `m>=23` 门槛。少行、非有限信息或更新失败时恢复预测前的 `x/P`，并抑制本帧地图写入。

## 实施与验收顺序

1. 加入独立原始邻域/KNN API 与 `GenzMatch`，用 0、4、5 个邻点及 `d=dmax` 边界验证对应分类；增加重复点、共线、零迹、非有限值和特征分解失败用例。
2. 加入点面 1 行、点点 3 行观测构造；以真实 `boxplus` 对 12 列分别中心差分，检查 plane 与 point 的符号、单位及外参关闭零列。
3. 独立 oracle 核验 \(H^TWh\)、\(H^TWH\) 和按定义堆叠后的梯度/信息块；验证 `alpha` 以类别匹配点计数（例如 1 个 plane 与 1 个 point 对应仍是 \(1/2\)，虽产生 4 行），并测旋转不变性和尺度变化。
4. 测试至少 23 行、少行回滚、非有限权重回滚及失败帧不写图；检查模型兼容校验要求 `weight_model=backend_default`，避免 GM 二次加权。
5. 合成 oracle 和真实配置归档通过后，才运行固定输入回归；分别报告源码/静态检查、构建、测试和回放证据。
