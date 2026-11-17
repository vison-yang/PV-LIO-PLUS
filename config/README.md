# 配置文件

配置按职责分成传感器配置、地图后端配置和可选观测配置。11 个后端的地图参数各自只归属 `backends/<backend>.yaml`；没有 `common.yaml` 或 `sensors/` 子目录。四份主传感器 YAML 的 `mapping` 下仅保留五项：`down_sample_size`、`max_iteration`、`extrinsic_est_en`、`extrinsic_T`、`extrinsic_R`。

| 位置 | 内容 |
| --- | --- |
| `avia.yaml`、`mid360_indoor.yaml`、`solid.yaml`、`solid_point_time.yaml` | topic、时间、预处理、噪声、外参和传感器采样设置；`mapping` 下仅有上述五项。 |
| `backends/<backend>.yaml` | 该后端实际使用的地图几何、查询、容量、合并等默认参数；包含 selector。 |
| `observations/<profile>.yaml` | 可选观测/权重组件参数。 |

后端专属参数只写入所属后端文件。例如 C3P 的六个 `c3p_*` 参数只在 `backends/c3p_voxelmap.yaml` 中定义。体素尺寸等同名键可以分别出现在不同后端文件中，因为每次只加载一个后端。

其中 `det_range=100` 只配置在 `voxelmap`、`voxelmap_plus`、`c3p_voxelmap`、`ikdtree`、`ivox`、`i_octree` 六个后端文件中；`point_map_downsample_size=0.1` 只配置在 `ikdtree` 和 `ivox` 文件中。后端选择通过 launch 的 `backend` 参数完成。

相对拆分前的 Solid 基线，当前默认 iVox 配置会采用后端值：`nearest_max_range` 从 1.75 m 变为 5 m，`ivox_resolution` 从 0.5 m 变为 0.2 m，`ivox_capacity` 从 200,000 变为 1,000,000，`point_map_downsample_size` 从 0.5 变为 0.1。选择 OctVox 时，`octvox_capacity` 从 200,000 变为 1,000,000。上述六个后端的 `det_range` 从旧 Solid 值 70 变为 100；Solid 默认 `local_window_en=false`，所以该变化本身不会启用窗口裁剪。这些是实际基线变化，不保证与拆分前行为相同。

## 后端选择

| 后端名称 | 配置文件 | 地图参数范围 |
| --- | --- | --- |
| `voxelmap` | `backends/voxelmap.yaml` | 概率体素几何、层级、点数、平面阈值和 sigma 门限。 |
| `voxelmap_plus` | `backends/voxelmap_plus.yaml` | 体素几何、点数、平面阈值、sigma 和更新门限。 |
| `c3p_voxelmap` | `backends/c3p_voxelmap.yaml` | 体素层级、平面阈值、sigma 及 C3P 合并参数。 |
| `r_voxelmap` | `backends/r_voxelmap.yaml` | 体素层级、点数及原生 RANSAC、重建和 LRU 参数。 |
| `ikdtree` | `backends/ikdtree.yaml` | 点面邻域、物理搜索半径、拟合阈值和树维护参数。 |
| `ivox` | `backends/ivox.yaml` | 点面邻域、搜索半径、拟合阈值、体素分辨率、容量和存储实现。 |
| `octvox` | `backends/octvox.yaml` | 原生分辨率和容量。 |
| `hvox` | `backends/hvox.yaml` | 层级、原生移动盒、平面性和噪声尺度。 |
| `i_octree` | `backends/i_octree.yaml` | 点面邻域和原生树参数。 |
| `ig_lio_plane` | `backends/ig_lio_plane.yaml` | 点面邻域、增量统计格、查询模式和 LRU 参数。 |
| `bievr` | `backends/bievr.yaml` | 定向高度图、像素、融合、LRU、Huber 和高度噪声尺度。 |

`nearest_max_range` 单位是米；ikd-tree 包含半径边界，iVox 保留其严格半径与附近格域语义。`mapping/plannar_threshold` 保留节点实际读取的历史拼写。各后端的窗口、计数与观测限制见[契约说明](../doc/INTEGRATION_STATUS.md)。所有配置数字都是明确的运行基线，不表示已经完成传感器噪声标定或达到最优性能。

## 启动与覆盖顺序

固定顺序为：**传感器 YAML → 选中后端 YAML → 可选观测 YAML → 强制设置 `mapping/map_type`**。后加载的后端参数会覆盖同名传感器参数；最后由 `backend` 参数固定选择，避免其它配置中的 selector 改变后端。路径不依赖内置传感器名称，也不推断其它配置文件。

```bash
roslaunch pv_lio_plus mapping.launch backend:=c3p_voxelmap
roslaunch pv_lio_plus mapping_solid.launch backend:=ivox rviz:=false
roslaunch pv_lio_plus mapping.launch sensor_config:=/data/my_sensor.yaml backend:=i_octree observation:=geman_mcclure rviz:=false
```

三个传感器入口默认选择 Avia/Mid360 的 `voxelmap_plus` 和 Solid 的 `ivox`，默认也会加载对应后端文件。`backend` 必须是表中的名称，不能留空或使用 `none`。`observation` 可省略或设为 `none`，表示不加载观测预设；`backend_default` 显式选择后端默认权重，`geman_mcclure` 只适用于四个共享点面后端。该可选层不负责清理已有 master 上的其它观测参数，切换实验应使用明确的观测配置或干净的运行环境。

单独加载传感器文件不包含完整地图参数。手动加载时也按传感器、后端、可选观测的顺序执行。例如：

```bash
rosparam load "$(rospack find pv_lio_plus)/config/solid.yaml"
rosparam load "$(rospack find pv_lio_plus)/config/backends/ivox.yaml"
```

## 批量测试

runner 与 launch 使用相同的传感器后端顺序；`backend_config_mode` 记录为 `sensor_then_backend`。manifest 不再声明 `map_configs`，不按文件路径猜测配置层。

```bash
python3 scripts/run_backend_tests.py --manifest scripts/datasets.example.json --algorithms c3p_voxelmap
python3 scripts/run_backend_tests.py --manifest scripts/datasets.example.json --algorithms ivox --observation-profile geman_mcclure
```

`--backend-presets` 仅作为兼容旧命令的 no-op 保留，当前总是加载所选后端文件。每次结果归档 backend、sensor 和可选 observation 的副本及 SHA-256；生成的 launch 引用归档副本。

## SOLID 逐点时标数据

`preprocess/solid_timestamp_mode` 只控制 SOLID 在缺少 `time`、提供 `timestamp:FLOAT64` 时的解释；已有 `time:FLOAT32` 相对秒路径保持原约定。

| 模式 | 输入契约 |
| --- | --- |
| `frame_absolute`（默认） | 旧转换器将相同帧时间写入所有点；时间跨度不得超过 1 微秒，输出点内偏移为零。真实逐点变化会拒绝整帧，并提示切换模式。 |
| `point_absolute` | 每点绝对秒。以 `header.stamp` 为原点，保留逐点相对时间并排序；允许最多 1 微秒的负舍入并钳为零，正偏移不超过 `2 / scan_rate` 秒。 |

点模式使用 `curvature = 1000 × (timestamp - header.stamp)`，内部单位为毫秒，供公共同步及 IMU 去畸变消费。字段布局、有限性、越界与端序检查失败时拒绝整帧；不合成固定子扫描组数。`scan_rate` 必须为正，时间模式名称在节点初始化时校验。

`solid_point_time.yaml` 仅固定 SOLID 回归记录 01（采集时间 `2026-09-24_17-29-45`）的外参，不能直接推广至其他记录。标定文件说明保存在项目外归档，逻辑引用为 `external://record_01/calibration.yaml`；该 URI 不是可直接访问的物理路径。SHA-256 为 `f579332586fdc26a266574c28eff1eda9797098c6cbe2407c19b98c205142267`。按相机到 IMU、相机到 LiDAR 的变换组合：

\[
R_{IL}=R_{IC}R_{LC}^{\mathsf T},\qquad
t_{IL}=t_{IC}-R_{IL}t_{LC},\qquad p_I=R_{IL}p_L+t_{IL}.
\]

下标表示从右侧坐标系变换到左侧坐标系，平移单位为米；结果直接写入该配置的外参。配置使用 30 m 量程、0.2 m 扫描降采样，以及只排除后向点的宽角度范围；这与旧传感器配置是独立输入基线。噪声字段沿用 SOLID 方差基线，尚未重新标定。

```bash
roslaunch pv_lio_plus mapping.launch sensor_config:=$(rospack find pv_lio_plus)/config/solid_point_time.yaml backend:=bievr rviz:=false
```

`output/root_dir` 可显式指定本次地图、轨迹和默认 timing CSV 的输出目录；未指定时仍使用编译时目录。应为每次实验准备独立目录。runner 的 `--output-root` 同时写入这个运行参数；其常规成功标记仅检查输出文件，不代表完整输入或 EOF 排空验收。

timing CSV 末尾新增 `observation_rejected` 与 `map_revision`：前者明确记录受保护观测路径是否回滚，后者记录地图修订状态。未收敛不等于观测被拒绝；修订号增长表示地图更新提交，不等于新增点数。2026-11-17 的 BIEVR、iG-LIO 和 iG-LIO+GM 独立 SOLID 回归记录 01 短回归均通过，具体范围见[验收记录](../doc/workspace/evidence/INTEGRATION_PROGRESS_20261117.json)。

最新第一类地图验收采用同一传感器 profile：i-Octree 专项 30 秒 / 287 帧，BIEVR 与 iG-LIO 默认权重各 197.190 秒 / 1959 帧。后者的末尾两帧 LiDAR 超出 IMU 覆盖而裁去，不能仅以 bag 总时长推断可处理帧数。实际配置 SHA、选窗及输出复核见[验收记录](../doc/workspace/evidence/MAP_ACCEPTANCE_20261117.json)；噪声值没有因回归通过而成为已标定参数。
