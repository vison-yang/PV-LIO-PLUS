# 配置文件

配置按职责分成传感器配置、地图后端配置和可选观测配置。11 个后端的地图参数各自只归属 `backends/<backend>.yaml`；没有 `common.yaml` 或 `sensors/` 子目录。三份主传感器 YAML 的 `mapping` 下仅保留五项：`down_sample_size`、`max_iteration`、`extrinsic_est_en`、`extrinsic_T`、`extrinsic_R`。

| 位置 | 内容 |
| --- | --- |
| `avia.yaml`、`mid360_indoor.yaml`、`solid.yaml` | topic、时间、预处理、噪声、外参和传感器采样设置；`mapping` 下仅有上述五项。 |
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
