# PV-LIO-PLUS

[中文版本](README.zh-CN.md)

PV-LIO-PLUS is a ROS1 LiDAR-inertial odometry (LIO) framework for comparing
local-map models with the same point-to-plane estimation front end. It is
mainly inherited from [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git),
which follows the probabilistic voxel-map idea of
[VoxelMap](https://github.com/hku-mars/VoxelMap.git) and the tightly coupled
iterated-filter structure of FAST-LIO.

The project keeps the original PV-LIO, VoxelMap, and VoxelMap++ fixes and
optimizations, and adds a `MapManager` for selecting and comparing five local
maps from one configuration file. All integrated map sources are kept inside
this package; the full external LIO packages are not linked into PV-LIO-PLUS.

## Main work

- Preserve PV-LIO's tightly coupled LiDAR-IMU estimation and IKFoM iterated
  filter.
- Preserve the original PV-LIO observation-flow and residual-weighting fixes.
- Retain VoxelMap's probabilistic adaptive voxel planes and VoxelMap++'s local
  map and residual optimizations.
- Add `MapManager` with common search, insertion, deletion, local-window, and
  publication interfaces while preserving backend-specific behavior.
- Integrate VoxelMap, VoxelMap++, FAST-LIO2 ikd-tree, Faster-LIO iVox, and
  C3P-VoxelMap as selectable backends.
- Add explicit trajectory and world-frame map-cloud outputs for comparison.
- Use C++17 and keep the integrated local-map implementations in this package.

## Algorithm inventory

The inventory distinguishes a local-map backend from a complete LIO package.
PV-LIO-PLUS integrates the map layer into its own estimation loop; it does not
embed another package's complete state-estimation node.

### Integrated local-map backends

| Algorithm / map | Workspace source or reference | Selector | Note |
| --- | --- | --- | --- |
| VoxelMap | `src/VoxelMap`; native copy in `include/map_manager/native/voxelmap` | `voxelmap` | Probabilistic adaptive voxel planes; the first core local map in the PV-LIO lineage. |
| VoxelMap++ | Native copy in `include/map_manager/native/voxelmap_plus` | `voxelmap_plus` | VoxelMap-derived map and residual optimizations; its standalone clone is not kept. |
| FAST-LIO2 / ikd-tree | `src/FAST_LIO`; native copy in `include/map_manager/native/ikdtree` | `ikdtree` | FAST-LIO2's incremental KD-tree map is integrated; the complete `fast_lio` node remains an independent package. |
| Faster-LIO / iVox | `src/faster_lio`; native copy in `include/map_manager/native/ivox` | `ivox` | Faster-LIO's sparse incremental voxel map is integrated; the complete `faster_lio` node remains an independent package. |
| C3P-VoxelMap | `src/c3p_voxelmap`; native copy in `include/map_manager/native/c3p_voxelmap` | `c3p_voxelmap` | Compact probabilistic voxel map with native plane matching and merging. |

Therefore FAST-LIO2 and Faster-LIO are already represented in the integrated
map-backend list through ikd-tree and iVox. Their complete standalone nodes
remain available for end-to-end comparison, but are not a second pending
`MapManager` backend task.

### Other algorithms pending integration

These algorithms are present in the workspace but their map and/or residual
logic has not been merged into `MapManager`.

| Algorithm | Location | Current status / characteristic |
| --- | --- | --- |
| Hybrid-VoxelMap | `src/Hybrid-VoxelMap` | Pending; hybrid voxel/plane representation. |
| R-VoxelMap | `src/R-VoxelMap` | Pending; robust recursive voxel-plane estimation. |
| Super-LIO / OctVox | `src/Super-LIO` | Pending; OctVox is an internal map structure of Super-LIO, not a separate package here. |
| BIEVR-LIO | `research/BIEVR-LIO` | Pending; voxelized map with map-informed sampling and geometric statistics. |
| Surfel-LIO / hVox | `research/Surfel-LIO` | Pending; hierarchical voxel hash with precomputed surfels. |
| LIO-GVM | `research/lio_gvm` | Pending; Gaussian voxel map and Gaussian-based scan matching. |

The complete `fast_lio` and `faster_lio` nodes remain independently buildable
at `src/FAST_LIO` and `src/faster_lio` for end-to-end comparison. They are not
listed as pending `MapManager` backends because their local-map layers are
already integrated above.

`src/livox_ros_driver` is a sensor driver and is not counted as an algorithm.
`research/build` contains build artifacts and is not counted. CT-VoxelMap and
RC-Vox have no confirmed source directory in the current workspace; they are
tracked in the workspace [TODO.md](../../TODO.md).

## Architecture

```text
LiDAR / IMU
    |
    v
PV-LIO state estimation and point-to-plane residuals
    |
    v
MapManager
    |-- VoxelMap
    |-- VoxelMap++
    |-- ikd-tree
    |-- iVox
    `-- C3P-VoxelMap
```

VoxelMap and VoxelMap++ keep native plane-map matching. ikd-tree and iVox keep
their point-map search and use the manager's plane-fitting adapter. C3P-VoxelMap
uses its native plane-map implementation. This separation makes map-level
comparisons possible without changing the LIO state-estimation loop.

## Requirements and build

- Ubuntu 20.04
- ROS Noetic
- PCL, Eigen, Sophus, YAML-CPP, and the dependencies of the ROS workspace

```bash
cd ~/src/lio_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Select a local map

Set the backend in the PV-LIO-PLUS YAML configuration:

```yaml
mapping:
  map_type: ikdtree   # voxelmap, voxelmap_plus, ikdtree, ivox, c3p_voxelmap
```

The local-window mode is selected with the existing `mapping` window
parameters. Backend-specific parameters remain in the same configuration file
and are ignored when another backend is selected.

## Run

```bash
source /opt/ros/noetic/setup.bash
source ~/src/lio_ws/devel/setup.bash
roscore
roslaunch pv_lio_plus mapping_avia.launch
rosbag play --clock /path/to/your.bag
```

Use the launch file and sensor parameters that match the input dataset.

## ROS interfaces and saved results

The node publishes odometry, trajectory/path, registered scans, and the local
map through the topics configured by the launch/YAML files. The final result
files are written under the configured output directory:

| Backend | Trajectory | Final map |
| --- | --- | --- |
| `voxelmap` | `pv_lio_pos.txt` | `pv_lio.pcd` |
| `voxelmap_plus` | `pv_lio_plus_pos.txt` | `pv_lio_plus.pcd` |
| `ikdtree` | `pv_lio_ikdtree_pos.txt` | `pv_lio_ikdtree.pcd` |
| `ivox` | `pv_lio_ivox_pos.txt` | `pv_lio_ivox.pcd` |
| `c3p_voxelmap` | `pv_lio_c3p_voxelmap_pos.txt` | `pv_lio_c3p_voxelmap.pcd` |

The trajectory is stored as timestamped pose records. Point-cloud chunks may
also be written under `output/PCD/` according to the configured scan-saving
options. When comparing maps, use the same bag segment, sensor parameters,
window settings, and output directory layout for every backend.

## References

- [PV-LIO](https://github.com/HViktorTsoi/PV-LIO)
- [VoxelMap](https://github.com/hku-mars/VoxelMap)
- [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public)
- [FAST-LIO / ikd-tree](https://github.com/hku-mars/FAST_LIO)
- [Faster-LIO / iVox](https://github.com/gaoxiang12/faster-lio)
- [C3P-VoxelMap](https://github.com/deptrum/c3p-voxelmap)
- [Hybrid-VoxelMap](https://github.com/haiyang2022/Hybrid-VoxelMap)
- [R-VoxelMap](https://github.com/NKU-MobFly-Robotics/R-VoxelMap)
- [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1)
- [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO)
- [Surfel-LIO](https://github.com/93won/lidar_inertial_odometry)
- [LIO-GVM](https://github.com/Ji1Xingyu/lio_gvm)
