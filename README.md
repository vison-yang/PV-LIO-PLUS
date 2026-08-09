# PV-LIO-PLUS

[中文版本](README.zh-CN.md)

PV-LIO-PLUS is a ROS1 LiDAR-inertial odometry (LIO) framework for evaluating
different local-map models under one experiment and point-to-plane matching
framework. The project is chiefly inherited from
[PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git),
and keeps its tightly coupled LiDAR-IMU estimation structure and IKFoM-based
iterated filter. PV-LIO itself is built on the probabilistic voxel-map idea of
[VoxelMap](https://github.com/hku-mars/VoxelMap.git) and is inspired by
[FAST-LIO2](https://github.com/hku-mars/FAST_LIO.git).

VoxelMap is the first and core local-map algorithm in this project lineage. It
contributes an efficient probabilistic adaptive voxel map: local surfaces are
represented by planes, and plane parameters and uncertainty are updated as
scans arrive. On top of this foundation, PV-LIO-PLUS brings four additional
local-map backends into the same experiment framework: VoxelMap++, ikd-tree,
iVox, and C3P-VoxelMap.

Researchers can select a backend from a configuration file and compare map
search, insertion, deletion/local-window maintenance, and point-to-plane
matching in the same LIO front end. All bundled local-map sources are kept
inside PV-LIO-PLUS; the node does not link to another LIO package's source
tree.

## Project lineage and map roles

| Role | Component | Contribution or characteristic retained here |
| --- | --- | --- |
| LIO foundation | [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git) | Tightly coupled LiDAR-IMU odometry, IKFoM iterated filtering, and the original PV observation flow. |
| First/core local map | [VoxelMap](https://github.com/hku-mars/VoxelMap.git) | Probabilistic adaptive coarse-to-fine voxel mapping with planes and plane uncertainty as the map representation. |
| Additional backend | [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git) | VoxelMap-derived local-map manager and residual-calculation optimizations, with native voxel-plane matching. |
| Additional backend | [FAST-LIO2 ikd-tree](https://github.com/hku-mars/FAST_LIO.git) | Dynamic incremental KD-tree point map with efficient nearest-neighbor search and native point deletion. |
| Additional backend | [Faster-LIO iVox](https://github.com/gaoxiang12/faster-lio) | Sparse incremental voxel point map designed for high-throughput nearby-point search. |
| Additional backend | [C3P-VoxelMap](https://github.com/deptrum/c3p-voxelmap) | Compact, cumulative, and coalescible probabilistic voxel mapping with optional on-demand plane merging. |
| Integration layer | `MapManager` | Common initialization, search, insertion, snapshot, deletion/window, and publication interfaces while preserving backend-specific logic. |

## Main work

The project keeps the original README's PV-LIO/VoxelMap/VoxelMap++ repair and
optimization record, while making the scope of each change explicit. These are
engineering integration and numerical-hardening changes; they do not by
themselves establish a new covariance model or general improvements in
stability, efficiency, or accuracy:

- **PV-LIO residual weighting:** retained the historical adjustment to the
  error-propagation path used to form residual weights, as documented by the
  original project.
- **VoxelMap integration:** retained the native probabilistic plane-map logic
  and made the point/covariance data flow explicit in the common manager,
  including the LiDAR-to-world covariance convention.
- **VoxelMap++ integration:** retained its upstream local-map-manager and
  residual-calculation optimizations, made it selectable from configuration,
  carried the correct point-covariance frame into the residual, and guarded
  observed invalid or near-zero residual-variance cases.
- **Unified local-map evaluation:** added `MapManager` and integrated the four
  additional backends above. Native plane backends keep their native matching;
  point-map backends search nearby points and fit a plane for the common
  residual contract.
- **Reproducible outputs:** added explicit trajectory and world-frame scan-cloud
  output semantics, with backend-specific filenames documented below.
- **Build and packaging:** upgraded the package to C++17 and kept the local-map
  implementations inside PV-LIO-PLUS so backend comparison does not require
  linking external LIO source trees.

## 1. Architecture

PV-LIO-PLUS separates the LIO state-estimation loop from local-map storage and
search. The observation model selects the native matching implementation from
`mapping/map_type`, while `MapManager` provides a common lifecycle and match
contract.

```text
LiDAR / IMU
    │
    ▼
PV-LIO state estimation and point-to-plane residuals
    │
    ▼
MapManager
    ├── VoxelMap       ─ probabilistic adaptive voxel planes
    ├── VoxelMap++     ─ enhanced voxel planes and residual logic
    ├── ikd-tree       ─ incremental point map and plane fitting
    ├── iVox           ─ voxelized point map and plane fitting
    └── C3P-VoxelMap   ─ compact cumulative probabilistic voxels
```

The manager preserves the native algorithmic behavior as far as each API
allows. Backend-specific differences are isolated in the manager adapters:

- VoxelMap and VoxelMap++ use their native voxel-plane search and update logic;
  VoxelMap++ additionally retains its native residual formulation.
- ikd-tree uses an incremental KD-tree point map and its native deletion path.
- iVox uses a sparse voxel point map for nearby-point search; the adapter fits a
  local plane before creating the common point-to-plane match.
- C3P-VoxelMap uses its compact probabilistic plane representation and optional
  on-demand plane merging.
- When local-window mode is enabled, ikd-tree uses native point deletion.
  iVox and C3P-VoxelMap rebuild from the manager's retained points because
  their native APIs do not expose a safe erase-and-iterate operation.

### Source layout

```text
PV_LIO_PLUS/
├── include/map_manager/map_manager.h
├── src/map_manager/map_manager.cpp
└── include/map_manager/native/
    ├── voxelmap/       # PV-LIO VoxelMap implementation
    ├── voxelmap_plus/  # VoxelMap++ implementation
    ├── ikdtree/        # FAST-LIO2 ikd-tree implementation
    ├── ivox/           # Faster-LIO iVox implementation
    └── c3p_voxelmap/   # C3P-VoxelMap implementation
```

## 2. Requirements

The workspace is tested with Ubuntu 20.04, ROS Noetic, C++17, PCL, Eigen, and
Livox ROS messages. Other ROS1 distributions may work when their compiler and
dependency versions provide C++17 support.

- ROS1 Melodic or later; ROS Noetic is recommended.
- PCL >= 1.8.
- Eigen >= 3.3.4.
- A sourced `livox_ros_driver` workspace providing `livox_ros_driver/CustomMsg.h`.
- A C++17-capable compiler.

The bundled local-map backends do not require Ceres. PV-LIO-PLUS builds them
as part of the package and does not link against the source trees of the other
LIO packages.

## 3. Build

From the catkin workspace root:

```bash
cd ~/src/lio_ws
source /opt/ros/noetic/setup.bash
# Source the workspace that provides livox_ros_driver, if it is separate.
source <livox_driver_ws>/devel/setup.bash
catkin_make
source devel/setup.bash
```

The normal workspace build with `catkin_make` is the recommended build check.
`catkin_make --pkg pv_lio_plus` is useful for quick iteration, but does not
replace a complete workspace build.

## 4. Select a local-map backend

Set `mapping/map_type` in
[`config/mid360_indoor.yaml`](config/mid360_indoor.yaml):

| `map_type` | Local-map model | Matching strategy |
| --- | --- | --- |
| `voxelmap` | PV-LIO's original VoxelMap | Native adaptive voxel-plane matching |
| `voxelmap_plus` | VoxelMap++ | Native voxel-plane and residual logic |
| `ikdtree` | FAST-LIO2 ikd-tree | Nearby point search followed by plane fitting |
| `ivox` | Faster-LIO iVox | Voxelized nearby-point search followed by plane fitting |
| `c3p_voxelmap` | C3P-VoxelMap | Compact probabilistic plane matching with optional merging |

Example:

```yaml
mapping:
    map_type: ikdtree
    local_window_en: false
    det_range: 100.0
```

`mapping/map_type` takes priority over the legacy
`mapping/b_use_voxelmap_plus` boolean. If `map_type` is absent, the legacy
boolean selects VoxelMap++ or the original VoxelMap.

The common point-map parameters include `nearest_point_count`,
`nearest_max_range`, `plane_fit_threshold`, and
`point_map_downsample_size`. Backend-specific parameters include `ikd_*`,
`ivox_*`, and `c3p_*`; the native-compatible defaults are used when they are
omitted.

### Local-window mode

Set `mapping/local_window_en: true` to enable a moving local map. The window is
centered at the current LiDAR position and uses `mapping/det_range` as its half
extent. The manager applies the backend-appropriate deletion or rebuild policy
described in the architecture section above.

## 5. Run

Start ROS and launch the included Mid-360 configuration:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
roslaunch pv_lio_plus mapping_mid360.launch rviz:=false
```

In another terminal, play a ROS1 bag with simulated time enabled:

```bash
source /opt/ros/noetic/setup.bash
source ~/src/lio_ws/devel/setup.bash
rosbag play --clock /home/yxy/data/outdoor_Mainbuilding_10hz_2020-12-24-16-38-00.bag
```

The launch file loads the YAML configuration and subscribes to the topics
specified by `common/lid_topic` and `common/imu_topic`. Make sure the LiDAR and
IMU timestamps are synchronized. The warning `Failed to find match for field
'time'.` indicates that per-point LiDAR timestamps are missing and can affect
the motion compensation and propagation result.

## 6. ROS interfaces and saved results

| Interface | Meaning |
| --- | --- |
| `/Odometry` | Estimated LiDAR/IMU odometry |
| `/path` | `nav_msgs/Path` trajectory for visualization |
| `/Laser_map` | Snapshot of the selected local map |
| `/planes` | Plane markers for voxel-plane backends |
| `/cloud_registered` | Registered world-frame cloud |
| `/cloud_registered_body` | Registered body-frame cloud |
| `/cloud_registered_lidar` | Registered LiDAR-frame cloud |

On shutdown, the node writes results under the workspace `output/` directory:

| Backend | Trajectory | Backend-named PCD when enabled |
| --- | --- | --- |
| `voxelmap` | `pv_lio_pos.txt` | `pv_lio.pcd` |
| `voxelmap_plus` | `pv_lio_plus_pos.txt` | `pv_lio_plus.pcd` |
| `ikdtree` | `pv_lio_ikdtree_pos.txt` | `pv_lio_ikdtree.pcd` |
| `ivox` | `pv_lio_ivox_pos.txt` | `pv_lio_ivox.pcd` |
| `c3p_voxelmap` | `pv_lio_c3p_voxelmap_pos.txt` | `pv_lio_c3p_voxelmap.pcd` |

Each trajectory row contains:

```text
timestamp  tx  ty  tz  qw  qx  qy  qz
```

The saved PCD is an accumulated undistorted world-frame scan cloud. It is
different from `/Laser_map`, which is the current selected local-map snapshot
and not a standardized saved-map file. With `pcd_save/interval: -1`, one final
complete PCD is written. With a positive interval, completed chunks are written
under `output/PCD/`; the backend-named PCD written at shutdown, if any, only
contains the remaining unflushed tail and may not be present.

The result stem is selected from `mapping/map_type`; therefore different map
types intentionally save different final filenames. Do not assume that every
run produces `pv_lio_pos.txt` and `pv_lio.pcd`. Intermediate chunks use the
shared names `output/PCD/scans_N.pcd`, so move or rename that directory after
each run when comparing backends.

## 7. Recommended backend comparison procedure

For a meaningful comparison:

1. Use the same bag, sensor topics, initial state, noise parameters, and
   preprocessing settings for every backend.
2. Change only `mapping/map_type` between runs; do not change the LIO residual
   or state-estimation code.
3. Stop each run cleanly with `Ctrl-C` so the final trajectory and PCD are
   flushed.
4. Move the generated files into a separate result directory, for example:

   ```bash
   mkdir -p output/map_comparison/ikdtree
   mv output/pv_lio_ikdtree_pos.txt output/map_comparison/ikdtree/
   mv output/pv_lio_ikdtree.pcd output/map_comparison/ikdtree/
   ```

5. Check trajectory length, runtime, map size, and finite numeric values before
   comparing accuracy or scene-dependent failure cases.

This procedure keeps the source tree unchanged while preserving independent
outputs for `voxelmap`, `voxelmap_plus`, `ikdtree`, `ivox`, and `c3p_voxelmap`.

## 8. Related work and upstream implementations

- [PV-LIO](https://github.com/HViktorTsoi/PV-LIO.git)
- [VoxelMap](https://github.com/hku-mars/VoxelMap.git)
- [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public.git)
- [FAST-LIO2 / ikd-tree](https://github.com/hku-mars/FAST_LIO.git)
- [Faster-LIO / iVox](https://github.com/gaoxiang12/faster-lio)
- [C3P-VoxelMap](https://github.com/deptrum/c3p-voxelmap)
- [IKFoM](https://github.com/hku-mars/IKFoM)

The upstream implementations are locally integrated under
`include/map_manager/native/` to keep backend selection inside PV-LIO-PLUS.
Please retain the corresponding upstream licenses and notices when
redistributing the project. See [`LICENSE`](LICENSE).

## 9. Acknowledgements

Thanks to the authors and contributors of PV-LIO, VoxelMap, VoxelMap++,
FAST-LIO2, Faster-LIO, C3P-VoxelMap, and IKFoM.
