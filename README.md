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
embed another package's complete state-estimation node. This repository only
bundles the local-map layers under `include/map_manager/native/`.

### Integrated local-map backends

| Algorithm / map | Source included in this repository | Selector | Note |
| --- | --- | --- | --- |
| VoxelMap | `include/map_manager/native/voxelmap` | `voxelmap` | Probabilistic adaptive voxel planes; the first core local map in the PV-LIO lineage. |
| VoxelMap++ | `include/map_manager/native/voxelmap_plus` | `voxelmap_plus` | VoxelMap-derived map and residual optimizations. |
| FAST-LIO2 / ikd-tree | `include/map_manager/native/ikdtree` | `ikdtree` | FAST-LIO2's incremental KD-tree map. |
| Faster-LIO / iVox | `include/map_manager/native/ivox` | `ivox` | Faster-LIO's sparse incremental voxel map. |
| C3P-VoxelMap | `include/map_manager/native/c3p_voxelmap` | `c3p_voxelmap` | Compact probabilistic voxel map with native plane matching and merging. |

FAST-LIO2 and Faster-LIO are therefore represented through ikd-tree and iVox.
Their complete nodes are not included in this repository; obtain and build the
upstream projects separately when an end-to-end baseline is needed.

### Candidate local-map backends

The following public projects are candidates for future `MapManager` backends.
They are neither bundled with PV-LIO-PLUS nor required to build it.

| Algorithm | Upstream project | Current status / characteristic |
| --- | --- | --- |
| Hybrid-VoxelMap | [Hybrid-VoxelMap](https://github.com/haiyang2022/Hybrid-VoxelMap) | Candidate; hybrid voxel/plane representation. |
| R-VoxelMap | [R-VoxelMap](https://github.com/NKU-MobFly-Robotics/R-VoxelMap) | Candidate; robust recursive voxel-plane estimation. |
| Super-LIO / OctVox | [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1) | Candidate; OctVox is an internal map structure of Super-LIO. |
| BIEVR-LIO | [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO) | Candidate; voxelized map with map-informed sampling and geometric statistics. |
| Surfel-LIO / hVox | [Surfel-LIO](https://github.com/93won/lidar_inertial_odometry) | Candidate; hierarchical voxel hash with precomputed surfels. |
| LIO-GVM | [LIO-GVM](https://github.com/Ji1Xingyu/lio_gvm) | Candidate; Gaussian voxel map and Gaussian-based scan matching. |
| [FR-LIO](https://arxiv.org/abs/2302.04031) | — | Paper available; no public source found currently, awaiting release or independent implementation. |
| [CT-VoxelMap](https://arxiv.org/abs/2604.03747) | — | Paper available; no public source found currently, awaiting release or independent implementation. |
| [GenZ-LIO](https://arxiv.org/abs/2603.16273) | [GenZ-ICP](https://github.com/cocel-postech/genz-icp) (related project) | Paper available; the related repository does not currently release GenZ-LIO source, awaiting release or independent implementation. |
| [SA-LIVO](https://arxiv.org/abs/2606.25699) | — | Paper available; the authors state that code will be open-sourced, but it is not released yet; awaiting release or independent implementation. |
| [SV-LIO](https://doi.org/10.3390/electronics15081744) | — | Paper available; no public source found currently, awaiting release or independent implementation. |
| [Environment-Adaptive Solid-State LiDAR-Inertial Odometry](https://arxiv.org/abs/2604.15864) | — | Paper available; no public source found currently, awaiting release or independent implementation. |

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

- Ubuntu 20.04 and ROS Noetic
- PCL 1.8 or later, Eigen3, Boost (Timer), and Python development headers
- `livox_ros_driver` in the same catkin workspace (provides
  `livox_ros_driver/CustomMsg.h`)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/vison-yang/PV-LIO-PLUS.git
# Run this only when livox_ros_driver is not already available in the workspace.
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
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
source ~/catkin_ws/devel/setup.bash
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

## License and third-party notices

PV-LIO-PLUS is distributed under the GNU General Public License, version 2 or
any later version. See [LICENSE](LICENSE) for the complete license text and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the provenance and license
status of the local map implementations and other bundled third-party code.
The VoxelMap++ and C3P-VoxelMap entries in that notice require upstream license
clarification before public redistribution of those local copies.

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
