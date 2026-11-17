# PV-LIO-PLUS

[中文版本](README.zh-CN.md)

PV-LIO-PLUS is a ROS1 LiDAR-inertial odometry (LIO) framework that integrates
multiple map structures and their corresponding residual models to evaluate
different maps within the same framework. LiDAR/IMU processing, state
propagation, the iterated-filter loop, and result outputs are shared; each
backend retains its map representation, matching rules, and observation model.

The project originated from [PV-LIO](https://github.com/HViktorTsoi/PV-LIO),
which replaced ikd-tree with the probabilistic
[VoxelMap](https://github.com/hku-mars/VoxelMap) map in the **FAST-LIO2 source
framework**. PV-LIO-PLUS first integrated
[VoxelMapPlus (VoxelMap++)](https://github.com/uestc-icsp/VoxelMapPlus_Public),
then substantially refactored and optimized the code framework, and subsequently
incorporated more map structures and their corresponding residual models.

`MapManager` currently selects eleven map backends from a common
configuration. Their map implementations are bundled in this package. Official
upstream projects collected in the surrounding workspace supply provenance and
independent baselines; their complete estimation nodes are built separately.

## Main work

- Share PV-LIO's tightly coupled LiDAR-IMU estimation and IKFoM iterated filter.
- Integrate VoxelMap and VoxelMap++ maps and their probabilistic plane residuals.
- Refactor configuration, map management, observation dispatch, and result output
  into a common C++17 framework.
- Add ikd-tree, iVox, C3P-VoxelMap, OctVox, R-VoxelMap, hVox, i-Octree,
  iG-LIO's incremental voxel map, and a BIEVR-derived oriented height map with
  their corresponding matching and observation paths, giving eleven
  selectable backends.
- Provide trajectory, world-frame map-cloud, and per-frame timing outputs for
  reproducible comparisons.

The [integration plan](PLAN.md) records source availability, extraction scope,
implementation order, and validation gates. The
[map and residual review](paper/notes/MAP_BACKEND_SCOPE_REVIEW.md) records the
current observation differences and the boundaries of future adapters.

## Algorithm inventory

An integrated backend comprises a map and its corresponding matching/residual
path inside PV-LIO-PLUS's estimation loop. The upstream map implementations
live under `include/map_manager/native/`; observation dispatch lives in
`src/mapping.cpp`. Source collection, upstream compilation, and integration are
separate milestones.

The active inventory requires a new map data structure or a substantive
improvement to one to be among the method's stated contributions. Using an
existing map, or changing only residuals, sampling, search policies, or
estimation, does not meet this criterion. The [eligibility review](paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)
records paper pages and structural evidence for each decision. An adapter must
preserve the qualifying structural contribution; a generic container extracted
after removing that contribution cannot stand in for the original map.

### Integrated local-map backends

All eleven map families below meet the structural-contribution criterion.
FAST-LIO2 is represented by its ikd-tree contribution, not by FAST-LIO1.

| Algorithm / map | Bundled map source | Selector | Map and residual model |
| --- | --- | --- | --- |
| VoxelMap | `include/map_manager/native/voxelmap` | `voxelmap` | Adaptive probabilistic planes; point-to-plane residual with plane and point uncertainty. |
| VoxelMap++ | `include/map_manager/native/voxelmap_plus` | `voxelmap_plus` | VoxelMapPlus planes and merging; native plane parameterization and uncertainty propagation. |
| FAST-LIO2 / ikd-tree | `include/map_manager/native/ikdtree` | `ikdtree` | Incremental KD-tree point map; neighborhood plane-fitting and point-uncertainty adapter. |
| Faster-LIO / iVox | `include/map_manager/native/ivox` | `ivox` | Sparse incremental voxel point map; the same point-to-plane adapter as ikd-tree. |
| C3P-VoxelMap | `include/map_manager/native/c3p_voxelmap` | `c3p_voxelmap` | Compact probabilistic planes and merging; native matching and uncertainty-weighted residual. |
| Super-LIO / OctVox | `include/map_manager/native/octvox` | `octvox` | LRU octant-aggregated point map; ordered TopK, 4/5-point plane fitting, and fixed observation information. |
| R-VoxelMap | `include/map_manager/native/r_voxelmap` | `r_voxelmap` | Recursive internal/child planes with outlier reuse, native root LRU, and covariance-aware probabilistic plane observation. |
| Surfel-LIO / hVox | `include/map_manager/native/hvox` | `hvox` | Two-level L0-centroid/L1-precomputed-surfel map with native moving-box maintenance and a PKO-based signed plane observation adapted to the shared IKFoM loop. |
| i-Octree | `include/map_manager/native/i_octree` | `i_octree` | Dynamic octree with contiguous RAII leaf blocks, data-AABB-pruned full-tree kNN, and the shared point-plane observation adapter. |
| iG-LIO / incremental VoxelMap | `include/map_manager/native/ig_lio` | `ig_lio_plane` | Incremental voxel sufficient statistics and native retained-point maintenance; bounded local search feeds the shared five-point float-QR scalar plane observation. This is not the author GICP observation. |
| BIEVR-LIO-derived height map | `include/map_manager/native/bievr` | `bievr` | Oriented voxel height images with weighted pixel fusion and a bilinear height residual; this local adaptation uses the shared 12-column IKFoM update, not the author LM/Ceres pipeline. |

FAST-LIO2 and Faster-LIO are represented through ikd-tree and iVox, using this
framework's observation adapter. Official full-system copies are collected in
workspace `src/FAST_LIO` and `src/faster_lio` for independent baselines.

### Collected sources and planned backends

The original source snapshot was checked on **2026-11-15**. Paths are relative
to the workspace root. Pinned author commits and compatibility patches are in
the [source manifest](doc/workspace/SOURCE_MANIFEST.json). iG-LIO's full
`src/ig_lio` checkout is absent from the current workspace; the bundled native
map core is a safety-focused reimplementation based on author commit
`2d79259e3aaa5bbb10d6b13234398760bbacf8f5`. Its 2026-11-14 shared-build record
describes the earlier complete checkout only. The `ig_lio_plane` selector is
source-integrated; its prior build and synthetic-test evidence is recorded
separately, while runtime replay remains pending. See the
[iG-LIO result record](doc/workspace/evidence/IG_LIO_RESULTS.json). BIEVR's
local height-map backend is source-integrated from author baseline
`2306022e341e98ce84244f92ba7d26f047b41dbc`; its current build and synthetic-test evidence is recorded in the
[two-track results](doc/workspace/evidence/TWO_TRACK_RESULTS.json); replay remains pending. The old `38f98b9` source/build record is
historical. See [`doc/BIEVR_IKFOM_PORT.md`](doc/BIEVR_IKFOM_PORT.md).
Two map candidates remain without selectors; full iG-LIO GICP is a separate
observation-design task.

| Map / method | Collected official source | Recorded build | Integration work |
| --- | --- | --- | --- |
| Hybrid-VoxelMap | `src/Hybrid-VoxelMap` | Earlier shared ROS1 build is historical; the current checkout is dirty and the pinned object is unavailable | Conditional candidate: define semantic-label inputs and typed Plane/NDT matches, then adapt both residuals. Identify plane-only or label-free variants explicitly. |
| LIO-GVM | `upstream/lio_gvm` | Blocked by incomplete author release | Incremental Gaussian voxel map storing centroids/covariances; obtain missing author-specific Fast-VGICP/IKFoM and map source before implementing its map/residual adapter. |

FR-LIO/RC-Vox and SV-LIO also have paper evidence of map-structure contributions,
but the local collection records do not establish official implementations.
They remain paper reserves, pending source verification.

A second integration track covers observation and weighting components, independently of `mapping/map_type`, with existing backend behavior as the default. The fixed-scale KISS-ICP-inspired Geman–McClure weight is source-integrated for `ikdtree`, `ivox`, `i_octree`, and `ig_lio_plane`; it changes information weight only and keeps each backend's matches, residual, Jacobian, and gates. `observation/model: backend_default` is supported; `genz_hybrid` is explicitly rejected until typed hybrid observations are implemented. Map structures take priority; see the [component plan and option status](doc/OBSERVATION_INTEGRATION_PLAN.md).

### Comparison controls

The main comparison evaluates each **map together with its corresponding
residual model** in the shared framework. Use the same sensor calibration,
input segment, IMU processing, state definition, filter schedule, and evaluation
method. Record residual type, covariance/information, robust kernel, matching
gates, scan sampling, and map budget/eviction with each result. Semantic inputs
or map-guided sampling need explicit experiment profiles and ablations.

The existing point-to-plane paths use scalar observations, with different
parameterizations, uncertainty models, and gates; OctVox uses fixed information.
R-VoxelMap uses its recursive probabilistic planes and native 3-sigma matching.
hVox uses fixed L1 surfels within a correspondence group and PKO weighting
adapted to the shared IKFoM loop. `ig_lio_plane` retains the incremental voxel
map but uses the shared five-point float-QR scalar plane fit; it is not the author's
GICP residual or complete iG-LIO estimator.
The eleventh path, `bievr`, uses an oriented height map and bilinear height
observation with Huber information in the common filter. It requires at least
23 valid scalar rows for an update, but that count does not guarantee
observability; mature map cells also require four supporting points. The
configured height sigma is an initial model value that needs calibration, not a
calibration-accuracy claim. VoxelMap++ now follows `extrinsic_est_en`: when enabled, its analytic Jacobian
includes the right-perturbation extrinsic rotation and translation columns;
when disabled, those columns are zero. Its `h = -float(dist)`, native weights,
and plane gates are unchanged. Details and test limits are in the
[integration status](doc/INTEGRATION_STATUS.md) and
[review](paper/notes/MAP_BACKEND_SCOPE_REVIEW.md).

`nearest_max_range` is now specified in metres for ikd-tree and iVox. ikd-tree
compares the fifth-neighbor squared distance with `r²` and accepts the boundary;
iVox passes `r` to its native query, which uses a strict `< r` test and searches
only the configured nearby voxel domain. Equal values therefore share a
physical unit but do not imply identical boundary or search-domain behavior.
To reproduce an old ikd-tree threshold configured as squared-distance value
`v`, set the new radius to `sqrt(v)` metres; existing iVox values are unchanged.

The example dataset manifest still lists the previous ten-selector baseline;
add `bievr` to the selected algorithms to include the new backend. The prior eight-selector
baseline passed main-line build/link, CTest 5/5 (33 GTest cases), and the hVox
30-second replay. i-Octree's feature build/link, independent numerical review,
isolated tests, and complete-input 30/141-second replays passed. On main-line
commit `6876a8e`, the node build/link, CTest 7/7 (49 regular GTest cases, ASan
off), and 30-second complete-input replay passed. See the
[integration status](doc/INTEGRATION_STATUS.md) for test counts, replay limits,
and evidence.

A stricter index/map-representation ablation is an optional experiment: it
must additionally align residuals, Jacobians, weights, gates, retained points,
and map budgets. Complete upstream systems are independent end-to-end
baselines. Report each experiment according to what it varies.

After the monotonic-clock fix, the original six backends completed shared
30-second and 141-second, 0.5 m input profiles. Each long run wrote 1402
trajectory/timing rows with nonzero constraints in every frame and finite,
nonnegative timing values. R-VoxelMap, hVox, and i-Octree also passed their
separate 30/141-second complete-input checks in their recorded feature runs.
The i-Octree main-line node, regular tests, and short replay also passed.
These are workflow regressions; no
ground-truth accuracy or controlled performance ranking is claimed. See the
[stage record](doc/INTEGRATION_STATUS.md), [R result record](doc/workspace/evidence/R_VOXELMAP_RESULTS.json), and [hVox result record](doc/workspace/evidence/HVOX_RESULTS.json).

## Architecture

```text
LiDAR / IMU
    |
    v
Shared preprocessing, deskew, and IMU state propagation
    |
    v
MapManager: map storage, updates, matching
    |-- VoxelMap       --> probabilistic plane observation
    |-- VoxelMap++     --> native plane observation
    |-- ikd-tree       --> shared point-to-plane adapter
    |-- iVox           --> shared point-to-plane adapter
    |-- C3P-VoxelMap   --> native probabilistic plane observation
    |-- OctVox         --> plane observation with fixed information
    |-- R-VoxelMap     --> recursive probabilistic plane observation
    |-- hVox          --> PKO-weighted precomputed-surfel observation
    |-- i-Octree      --> shared point-plane observation
    |-- iG-LIO        --> shared five-point float-QR scalar plane observation
    `-- BIEVR         --> oriented height-map observation
    |
    v
Backend observation dispatch --> shared IKFoM update
    |
    v
Map update, trajectory / map / timing outputs
```

Map storage and query are managed by `MapManager`; residuals, Jacobians, and
information are constructed by the corresponding observation paths. `hvox`
uses a dedicated surfel-to-`PlaneMatch` adapter, and `bievr` has a dedicated
height-sample path. General typed `Surfel`, `Gaussian`, and reusable
`HeightImage` observation interfaces remain future work.

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

Select the backend with the launch `backend` argument. Its YAML declares the corresponding map type:

```yaml
mapping:
  map_type: octvox    # voxelmap, voxelmap_plus, ikdtree, ivox, c3p_voxelmap, octvox, r_voxelmap, hvox, i_octree, ig_lio_plane, bievr
```

Each of the 11 backend YAML files owns that backend's map parameters. The
three main sensor YAML files keep sensor settings and only five `mapping` keys:
`down_sample_size`, `max_iteration`, `extrinsic_est_en`, `extrinsic_T`, and
`extrinsic_R`. Loading order is **sensor → selected backend → optional
observation**, then launch forces `mapping/map_type` from `backend`. The later
backend layer wins on overlapping keys; there is no `common.yaml` or
`sensors/` layer. See the [configuration guide](config/README.md) for the
resulting Solid baseline changes. Configure local-window behavior through the
existing `mapping` window parameters.

```bash
roslaunch pv_lio_plus mapping_avia.launch backend:=ig_lio_plane rviz:=false
roslaunch pv_lio_plus mapping_solid.launch backend:=ivox observation:=geman_mcclure rviz:=false
```

The batch runner follows the same sensor-then-backend order, recorded as
`backend_config_mode=sensor_then_backend`. The old `--backend-presets` flag is
now a compatibility no-op because the selected backend is always loaded.
Backend, sensor and optional observation YAML files are archived with SHA-256
values, and the generated launch reads those copies. `gm_scale_m=0.1` remains
a starting robust-weight scale, not a calibration result.

`octvox` retains Super-LIO's
capacity/LRU eviction and rejects geometric local-window mode. `hvox` uses its
native moving box and rejects the shared geometric-window option. `i_octree`
physically retains points inside the closed common window by pruning its tree.
`ig_lio_plane` rejects geometric local-window mode and uses its native capacity
and LRU behavior.

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
| `octvox` | `pv_lio_octvox_pos.txt` | `pv_lio_octvox.pcd` |
| `r_voxelmap` | `pv_lio_r_voxelmap_pos.txt` | `pv_lio_r_voxelmap.pcd` |
| `hvox` | `pv_lio_hvox_pos.txt` | `pv_lio_hvox.pcd` |
| `i_octree` | `pv_lio_i_octree_pos.txt` | `pv_lio_i_octree.pcd` |
| `ig_lio_plane` | `pv_lio_ig_lio_plane_pos.txt` | `pv_lio_ig_lio_plane.pcd` |
| `bievr` | `pv_lio_bievr_pos.txt` | `pv_lio_bievr.pcd` |

The trajectory is stored as timestamped pose records. Point-cloud chunks may
also be written under `output/PCD/` according to the configured scan-saving
options. When comparing maps, use the same bag segment, sensor parameters,
window settings, and output directory layout for every backend.

### Per-frame timing CSV

Set `logging/timing_en: true` to print each corrected pose together with the
current and running-average `search`, `ikf`, `map_update`, and `total` times
(milliseconds). It also writes a comma-separated file. `logging/timing_path`
selects an explicit file; an empty path writes
`output/<backend>_timing.csv`, for example `pv_lio_ivox_timing.csv`.

The CSV has one header and one row for each frame completing an EKF update and
map update. Its stable fields are `frame`, sensor `stamp_sec`, pose
(`pos_*_m`, `quat_*`), and `<stage>_ms` / `<stage>_avg_ms`. `search_ms` is the
sum of map-search work requested by all iterated-EKF observation evaluations;
`ikf_update_ms` therefore includes that search time. `total_ms` covers the
whole processed frame through ROS publication/map snapshot, but excludes the
timing log write itself. The stream is buffered and closed on node shutdown;
rows are not flushed per frame.

## License and third-party notices

The package metadata declares the GNU General Public License, version 2 or any
later version. See [LICENSE](LICENSE) for the root license text and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the provenance and license
status of the local map implementations and other bundled third-party code.
The imported iG-LIO source text is GPL version 2; no later-version grant or
compatibility conclusion is inferred, and the package metadata's scope for this
component remains under review. VoxelMap++ and C3P-VoxelMap also require upstream
license clarification before public redistribution of those local copies.

## References

- [PV-LIO](https://github.com/HViktorTsoi/PV-LIO)
- [VoxelMap](https://github.com/hku-mars/VoxelMap)
- [VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public)
- [FAST-LIO / ikd-tree](https://github.com/hku-mars/FAST_LIO)
- [Faster-LIO / iVox](https://github.com/gaoxiang12/faster-lio)
- [C3P-VoxelMap](https://github.com/deptrum/c3p-voxelmap)
- [Super-LIO / OctVox](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1)
- [Hybrid-VoxelMap](https://github.com/haiyang2022/Hybrid-VoxelMap)
- [R-VoxelMap](https://github.com/NKU-MobFly-Robotics/R-VoxelMap)
- [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO)
- [Surfel-LIO](https://github.com/93won/lidar_inertial_odometry)
- [LIO-GVM](https://github.com/Ji1Xingyu/lio_gvm)
- [i-Octree](https://github.com/zhujun3753/i-octree)
- [iG-LIO](https://github.com/zijiechenrobotics/ig_lio)

## Workspace source and build records

For this multi-repository workspace, see the pinned
[source manifest](doc/workspace/SOURCE_MANIFEST.json) and
[build report](doc/workspace/BUILD_REPORT.md). ROS1 packages that can be built
together live in workspace `src/`; standalone CMake, ROS2/mixed builds, and
conflicting or incomplete source releases live in workspace `upstream/`.
The latter are compiled separately. Collected sources support both backend
development and independent full-system baselines. Dependency prefixes stay
inside the workspace.

For the existing multi-project workspace, build with the local Eigen/Ceres
prefix and explicit include ordering:

```bash
bash src/PV_LIO_PLUS/scripts/prepare_workspace_deps.sh
bash src/PV_LIO_PLUS/scripts/build_workspace.sh \
  --build-dir build/maintenance-20261114-unified \
  --devel-dir devel/maintenance-20261114-unified --jobs 2
source devel/maintenance-20261114-unified/setup.bash
python3 src/PV_LIO_PLUS/scripts/build_upstream.py --jobs 2
```

The upstream script records and continues individual failures; LIO-GVM still
requires missing author source. These commands validate builds, not map quality.

The current two-track increment passed all 12 build targets, node dynamic-link checking and CTest 12/12 (89 GTest cases and 9 Python configuration tests), with ASan disabled. No replay or accuracy/performance A/B was run; see [the evidence](doc/workspace/evidence/TWO_TRACK_RESULTS.json).
