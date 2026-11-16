# 配置文件

## 传感器配置与后端预设

`avia.yaml`、`mid360_indoor.yaml` 和 `solid.yaml` 是保留的传感器配置，也是旧版直接 `rosparam load` 的完整配置。历史上增加地图后端时，参数结构体和 `Config::Load` 已经扩展，但这三份文件仍以传感器为单位，没有新增后端预设；缺少的值由 C++ 参数默认值补齐。这就是集成后端数量增加、配置文件数量却没有增加的原因。

现在可用 `backends/<name>.yaml` 为后端提供显式基线。通用 launch 先载入传感器 YAML，再载入后端 YAML；Noetic `roslaunch` 会把 YAML 字典展开成叶参数，后载入的同名叶键覆盖先前值，其余参数保留。因此 extrinsic、topic、noise、preprocess 和传感器采样值只需留在传感器文件。后端 preset 不含这些字段，也不改变 `point_map_downsample_size`，它继续继承传感器配置（Avia/Mid360 为 0.1 m，Solid 为 0.5 m）。

每个后端 YAML 是可复现的显式基线，不表示该参数对所有数据集都最优。没被 preset 覆盖的公共设置继续从传感器配置继承；直接加载旧 YAML 时，原有 selector 和参数仍保持不变。

| 后端 | 预设文件 | 主要参数与默认值 | 约束及语义 |
| --- | --- | --- | --- |
| VoxelMap | `backends/voxelmap.yaml` | 体素尺寸、层级和点数沿用传感器配置 | `local_window_en` 默认关闭；共享体素几何应为正值。 |
| VoxelMap++ | `backends/voxelmap_plus.yaml` | `update_size_threshold=5`、`sigma_num=3` | 继承公共体素几何。 |
| ikd-tree | `backends/ikdtree.yaml` | 最近点数 5、搜索半径 5 m、平面阈值 0.1、删除/平衡参数 0.5/0.7、盒长 0.2 m | 搜索半径须为正有限值且平方仍有限；边界点可被接受。 |
| iVox | `backends/ivox.yaml` | 最近点数 5、搜索半径 5 m、平面阈值 0.1、分辨率 0.2 m、邻域 18、容量 1,000,000、优化关闭 | 邻域选项为 0/6/18/26；其它值按实现回退到 18。搜索严格小于半径且受邻域格范围限制；非正容量会钳制为 1。 |
| OctVox | `backends/octvox.yaml` | 分辨率 0.5 m、LRU 容量 1,000,000 | 不支持 `local_window_en=true`；容量不大于 0 时实现钳制为 1。 |
| C3P-VoxelMap | `backends/c3p_voxelmap.yaml` | 合并关闭；角度/距离/最小特征值阈值 0.05/0.05/0.002；x/y 最大距离 5/5 | 体素几何沿用传感器配置。 |
| R-VoxelMap | `backends/r_voxelmap.yaml` | 根点数 1000、更新阈值 5、LRU 容量 1,000,000、RANSAC 10 轮/4 点、内点距 0.1、平面概率 0.8、seed 42、有效性检查层/点数/分辨率 0/10/5；重建阈值 `[100,200,300,500,700,1000]` | 不支持通用滑窗；公共体素尺寸和阈值须为正、层数为 0–32、点数限制为正。重建阈值须严格递增且为正，RANSAC 采样数至少 3、概率在 (0,1)，有效性检查层数至少 -1、点数至少 3、分辨率为 1–1,000,000；`layer_point_size` 项须至少为 2。 |
| hVox | `backends/hvox.yaml` | 分辨率 0.4 m、层级倍率 3、平面性 0.01、最大距离 50 m、盒倍率 2、LiDAR 噪声标准差 0.05 | 不支持通用滑窗；层级倍率必须为正奇数，距离/倍率/体素尺寸须为正有限值且乘积、比值可表示，平面性与噪声标准差须有限且非负。 |
| i-Octree | `backends/i_octree.yaml` | 共用最近点数 5、半径 5 m、平面阈值 0.1；桶容量 32、最小范围 0.01 m、树内降采样关闭 | 半径须为正有限值且平方仍有限；桶容量为正，平面阈值有限且非负，最小范围须落在原生浮点范围内。 |
| iG-LIO 点面变体 | `backends/ig_lio_plane.yaml` | 共用最近点数 5、半径 5 m、平面阈值 0.1；分辨率 0.5 m、搜索方法 7、容量 5,000,000、批追加门限 20 | 不支持通用滑窗；使用严格半径判断。搜索方法仅 1/7/19/26；26 对应实际查询中心周围 27 个网格。容量至少为 2，原生地图最多保留 `capacity-1` 个网格；`grid_max_points` 是批追加门限，不是硬上限。该模式是增量地图加共享五点 float QR 点面观测，不是原生 GICP 或原生至少三点 double 平面模型。 |
| BIEVR | `backends/bievr.yaml` | 体素 0.5 m、像素 0.05 m、容量 1,000,000；加权和平滑开启；法向容差 3°；高度噪声初值 0.1 m、Huber 阈值 0.1 m | `bievr_height_sigma_m` 表示高度观测噪声模型的初始标准差，不表示传感器、外参或地图的校准精度。`bievr_max_image_pixels=1,048,576` 是本地实现的图像像素安全上限，不是作者算法参数。 |

共享 `nearest_max_range` 的物理单位是米，不是平方值；iVox 对半径边界使用严格小于判定，ikd-tree 接受等于边界的点。`plane_fit_threshold` 使用现有共享观测实现的残差阈值语义。`mapping/plannar_threshold` 保留历史拼写，因为节点实际读取的参数名就是该键。

## 启动方式

通用入口支持传感器配置路径、可选后端预设和 RViz 开关：

```bash
roslaunch pv_lio_plus mapping.launch backend:=i_octree
roslaunch pv_lio_plus mapping.launch sensor_config:=$(rospack find pv_lio_plus)/config/solid.yaml backend:=ig_lio_plane rviz:=false
```

`backend` 留空时不载入预设，直接沿用传感器 YAML 中的 selector。兼容入口 `mapping_avia.launch`、`mapping_mid360.launch`、`mapping_solid.launch` 默认都将 `backend` 留空，并分别继续使用 Avia 的 `voxelmap_plus`、Mid360 的 `voxelmap_plus`、Solid 的 `ivox`；三个入口仍默认启动 RViz，也可传 `backend:=i_octree rviz:=false` 选择其它预设。

`observation` 是独立可选项，默认空字符串，不载入观测预设，因此传感器 YAML 已设置的观测参数保持原值。指定后，配置按传感器 YAML、后端 YAML、观测 YAML 的顺序加载；三个兼容入口均透传该参数。`backend_default` 只显式选择后端默认权重；`geman_mcclure` 提供固定尺度 `gm_scale_m=0.1` m，作为首轮配置值而非传感器噪声标定结果或作者自适应尺度复现。该权重目前只允许用于 `ikdtree`、`ivox`、`i_octree` 和 `ig_lio_plane`，不改变原有匹配门限。

```bash
roslaunch pv_lio_plus mapping.launch sensor_config:=$(rospack find pv_lio_plus)/config/solid.yaml backend:=ivox observation:=geman_mcclure rviz:=false
```

未知且非空的 `backend` 会因对应 YAML 文件不存在而使 `roslaunch` 配置解析失败。可用 `roslaunch --dump-params` 检查叶参数覆盖，不启动节点或 master。

## 批量后端测试

`scripts/run_backend_tests.py` 默认使用 manifest 指定的完整数据集 YAML 和 runner 选择的后端 selector，不额外载入后端或观测预设；这样保留历史实验参数。若希望 batch 参数与上面的通用 launch 一样采用显式后端基线，必须主动启用 `--backend-presets`。观测预设通过独立的 `--observation-profile` 选择：

```bash
python3 scripts/run_backend_tests.py --manifest <...> --algorithms ig_lio_plane --backend-presets
python3 scripts/run_backend_tests.py --manifest <...> --algorithms ivox --backend-presets --observation-profile geman_mcclure
```

runner 将后端与观测 preset 依次叠加到数据集 YAML，并在每次结果中归档所选 YAML、副本路径和 SHA-256；`dataset.json` 记录 `backend_config_mode`、观测选项和基础数据集 YAML 的 SHA-256。默认未指定观测预设时不生成观测 YAML 副本，旧的六后端默认列表和数据集参数流程保持不变。
