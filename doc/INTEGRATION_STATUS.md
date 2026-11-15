# Current Backend Contracts and Verification Status

Updated: 2026-11-15. This document records eight integrated `MapManager`
backends, contract fixes, separate synthetic test suites, and complete-input
replays. hVox feature commit `997c00a` is cherry-picked into main as `c81f12f`;
the main-line production node built and passed `ldd -r`, CTest 5/5 with 33 GTest
cases, and a 30-second complete-input replay. At the earlier R-VoxelMap integration stage, the main-line node built successfully and
passed a dynamic-link check; its three main-worktree CTest targets passed 3/3
with 22 GTest cases (ASan was off in this main configuration). Paper eligibility and
future adapter boundaries remain in the
[map/residual review](../paper/notes/MAP_BACKEND_SCOPE_REVIEW.md).

## Contract fixes

- **VoxelMap++ extrinsic Jacobian:** `ObservationModelVoxelMapPlus` now follows
  `extrinsic_est_en_`. When enabled, `VoxelMapPlusPointPlaneJacobian` fills the
  extrinsic rotation and translation columns for the right-perturbation
  convention; when disabled, those six columns are zero. The residual remains
  `h = -float(dist)`. Native plane gates and weights are unchanged.
- **Point-map search radius:** `nearest_max_range` is specified in metres for
  ikd-tree and iVox. ikd-tree compares its fifth-neighbor squared distance with
  `r²` and accepts equality. iVox passes `r` to its native search, uses a strict
  `< r` comparison, and only searches the configured nearby-grid domain.
- **Configuration safety:** for ikd-tree/iVox, the radius must be positive and
  finite; its square must also be finite and nonzero. `MapManager::configure`
  validates before clearing an initialized map. The three shipped YAML comments
  describe metres and preserve their numeric values. The example dataset
  example manifest lists all eight selectors, including `octvox`,
  `r_voxelmap`, and `hvox`.
- **Migration:** an old ikd-tree setting `v` was compared as squared distance;
  to reproduce that threshold, set the new radius to `sqrt(v)` metres. Existing
  iVox values already represented metres and remain unchanged.

The later diagnostic fix `f6403f0` moves residual-sum accumulation outside
the point-map, OctVox and C3P OpenMP loops. Observation `h/H/R` expressions
are unchanged. The rebuilt node passed complete-input 30-second regressions
for those three paths (288 output rows each); see
[`DIAGNOSTIC_RACE_RESULTS.json`](workspace/evidence/DIAGNOSTIC_RACE_RESULTS.json).
This check does not establish general thread safety.

## Eight backend contracts

These entries describe current `MapManager` behavior, not equivalence between
backends. `size()` intentionally reports native backend-dependent units;
`point_count()` reports retained points and is not a cross-backend replacement
for `size()`.

| Backend | Stored map and update | Query and observation | `size()` | Geometric local window |
| --- | --- | --- | --- | --- |
| VoxelMap | Hash from voxel location to native `OctoTree`; native build/update maintains the hierarchical probabilistic plane map. | Native OMP residual builder supplies point-plane matches; the observation propagates plane and point uncertainty. | Number of root voxel-map entries. | Supported. Removes voxel entries using each native voxel center against the axis-aligned bounds. |
| VoxelMap++ | Hash from voxel location to native `UnionFindNode`; native build/update maintains and merges plane nodes. | Native OMP residual builder supplies the 3-parameter plane match; the observation uses native plane covariance and residual. | Number of voxel-map entries. | Supported. Removes nodes using their voxel centers against the axis-aligned bounds. |
| FAST-LIO2 / ikd-tree | Incremental KD-tree of retained world points; updates apply the configured point-map downsampling policy. | Requests at least five neighbors, checks the fifth-neighbor distance against `r²` inclusively, fits a local plane, then uses the common point-plane observation adapter. | Valid KD-tree point count. | Supported. Deletes tree points outside the world-frame axis-aligned bounds. |
| Faster-LIO / iVox | Sparse voxel point map with configured resolution, nearby-voxel type, capacity, and optional optimized storage/query path. | Native query searches its configured nearby grid domain under strict `< r`; accepted neighbors feed the common point-plane adapter. | Number of valid grids. | Supported. Filters the retained point snapshot and rebuilds the iVox map. |
| C3P-VoxelMap | Native hierarchical voxel `OctoTree` map, with separate plane and optional merge metadata. | Native OMP residual builder supplies C3P plane matches; the observation propagates C3P plane and point uncertainty. | Number of voxel-tree entries. | Supported. Filters retained points and rebuilds the C3P map. |
| Super-LIO / OctVox | Native LRU grid map containing up to eight sub-voxel representatives per grid, bounded by its configured capacity. | Ordered TopK query; accepts four-point fallback or five-point plane fitting, with the native range/residual gate and fixed observation information. | Number of active LRU grid entries. | Unsupported. Configuration and explicit geometric deletion are rejected; OctVox owns capacity/LRU eviction. |
| R-VoxelMap | Native recursive voxel map with planes at internal/child nodes, linked-list outlier pushdown/rebuild, and root LRU retention. | Queries the current root and, conditionally, one adjacent root; matching uses full world-point covariance with a native 3-sigma gate. The filter observation uses plane covariance plus rotated LiDAR measurement covariance, without adding state covariance a second time. | Number of root voxels. | Unsupported. `local_window_en` and explicit geometric erase/window are rejected; native batch-then-freeze root budgeting is not a strict upper bound. |
| Surfel-LIO / hVox | Native L0 centroid / L1 precomputed-surfel hierarchy; updates use current world-frame IMU position and keyframe flag, with upstream moving-box maintenance. | Exact containing-L1-key lookup via `GetSurfelAtPoint`; fixed correspondences within an observation group feed signed plane residuals and PKO-adapted robust weights. Failed lookups are appended to `non_match`. | Native L0 voxel count. | Unsupported. Uses its native moving box; public geometric window operations and `local_window_en` are rejected. |

The source of these behaviors is `MapManager` in
[`map_manager.cpp`](../src/map_manager/map_manager.cpp), its public types and
configuration in [`map_manager.h`](../include/map_manager/map_manager.h), and
the eight observation paths in [`mapping.cpp`](../src/mapping.cpp), plus
[`hvox_observation.cpp`](../src/map_manager/hvox_observation.cpp). In
particular, `size()` counts voxel entries, KD-tree points, iVox grids, or
OctVox LRU grids depending on the selected backend; R-VoxelMap `size()` counts
root voxels while its `point_count()` traverses retained native points; hVox
`size()` is its L0 voxel count, `point_count()` uses native retained-point
count, and `snapshot()` contains L0 centroids. hVox `clear()` reconstructs the
native map because upstream `Clear()` does not reset its window center. Do not
compare these values as if they all counted points.

### Unmatched-query outputs

The common `MapManager::search(MapPointList, ...)` entry point clears `matches`
and `non_match` before dispatch. ikd-tree, iVox, OctVox, and R-VoxelMap append
individual queries that fail their backend-specific search to `non_match`;
hVox also appends failed exact-key surfel lookups. The native OMP residual
builders for VoxelMap, VoxelMap++, and C3P do not guarantee
that list contains one entry per failed query. The common guarantee is only
that `matches` is empty when no valid match was produced; callers must not
assume that `non_match` is a complete per-query complement.

### Window and size semantics

When enabled, the geometric window is an axis-aligned world-frame box. VoxelMap
and VoxelMap++ decide retention using voxel centers; ikd-tree removes points
outside the box; iVox and C3P rebuild from retained world points. OctVox rejects
geometric-window mode and retains its own capacity/LRU policy. R-VoxelMap also
rejects geometric-window mode and uses its native root LRU; its `clear()`
destroys the native map so it can be initialized again. `snapshot()` and
`point_count()` generally use common retained point records; ikd-tree snapshots
are flattened from its native tree, and R-VoxelMap snapshots traverse its
native recursive map. hVox uses a native moving box instead of the public
geometric window, snapshots L0 centroids, and reconstructs its native map on
clear. Neither `size()` nor `point_count()` is a normalized
cross-backend map-cell count.

## Synthetic verification

The `pv_lio_plus_map_contract_tests` and `pv_lio_plus_observation_tests`
targets were built and run: CTest **2/2 passed**, with **11 GTest cases**
total. Their sources are [`map_contract_tests.cpp`](../tests/map_contract_tests.cpp)
and [`observation_tests.cpp`](../tests/observation_tests.cpp).

- Six parameterized lifecycle cases cover empty maps, plane initialization,
  accepted matches, updates, clear, and reinitialization for the original six
  selectors. This suite does not parameterize R-VoxelMap.
- Point-map tests cover radii 4.99, 5.00, and 5.01 m; both iVox query paths;
  selected nearby-grid modes and finite-domain behavior; invalid radii rejected
  without clearing an initialized map; and rejection of OctVox geometric
  windows.
- The VoxelMap++ Jacobian helper is checked by central differences over all 12
  state columns for three non-identity poses and non-axis-aligned normals, with
  the extrinsic switch both enabled and disabled.

The tests do not specifically cover sub-metre radius thresholds, negative
world coordinates, or queries with only one through four neighbors. The production residual sign
and `float` cast, native weights, and plane gates were checked statically; the
synthetic tests do not exercise those production observation branches end to
end. Actual build and runtime records are in
[`STAGE1_RESULTS.json`](../doc/workspace/evidence/STAGE1_RESULTS.json).

R-VoxelMap's separate isolated suite passed CTest **4/4** with **23 GTest
cases**. The positive ASan fixture passed 30 repetitions. One full-suite ASan
invocation hung and was interrupted; later individual, repeated, and full-suite
retries passed, but the cause is unknown. Leak checking was disabled because
ptrace is unavailable in this environment, so no leak-free claim is made.

The R-VoxelMap selector/observation feature `af44fc0` was cherry-picked into
main as `b820b08`. The main-line production node built with exit code 0 and
passed `ldd -r` with no missing or unresolved libraries. Its three regular
CTest targets passed **3/3** with **22 GTest cases**; ASan was off in the main
configuration. This is separate from the isolated ASan-enabled suite above.
Detailed counts and limitations are in
[`R_VOXELMAP_RESULTS.json`](../doc/workspace/evidence/R_VOXELMAP_RESULTS.json).

## hVox verification

Feature `997c00a` imports the fixed Surfel-LIO author map at `73348c1e` and
adapts its two-level map and PKO observation to the shared `MapManager` and
IKFoM loop. The feature node built with exit code 0 and passed `ldd -r`.
Independent math/source review passed. Its combined isolated suite passed CTest
**6/6**, with **34 GTest cases**. This does not parameterize hVox through the
original six-backend lifecycle suite. The machine record is
[`HVOX_RESULTS.json`](workspace/evidence/HVOX_RESULTS.json).

The 30-second, 0.5 m regression received the full **301 LiDAR / 6088 IMU**
input and wrote **288 trajectory/timing rows**. Every frame had effective
constraints (minimum 374, median 1455, maximum 2044); timing columns were finite
and nonnegative, and the final map cloud was nonempty.

The 141-second, 0.5 m regression received **1414 LiDAR / 28595 IMU** messages
and wrote **1402 trajectory/timing rows**. Every frame had effective
constraints (minimum 243, median 1115, maximum 2193); native L0 map size ranged
from 3341 to 112385, all timing columns were finite and nonnegative, and the
maximum quaternion norm error was below `7.0e-7`. These are complete-input
regression checks, not ground-truth accuracy or controlled-performance
results. Both replays used non-isolated workload. The main-line cherry-pick `c81f12f` built and passed `ldd -r`, CTest 5/5
with 33 GTest cases, and a separate 30-second complete-input replay (288 rows).
All seven main build targets use Eigen 3.4.0; ASan is off in that configuration.

## Complete-input runtime regression

Code commit `9fa3111` built the real node and passed `ldd -r` with no missing
libraries or unresolved symbols. All six backends completed the same 30-second
Avia input window: **301 LiDAR / 6088 IMU messages received**, with the exact
per-topic header sequence checked against the bag. Of these scans, 300 have
full IMU support; 12 startup/initialization scans yield no corrected output.
Every backend saved **288 finite trajectory rows**, identical first/last sensor
timestamps, and a positive-point PCD. Monotonic output and row counts were
checked after normal shutdown.

This profile uses `mapping/down_sample_size: 0.1`, disables per-frame map/cloud
visualization equally, enables timing CSV, and retains final PCD saving. The
first batch directory mistakenly included “coarse” in its name: an unused
`filter_size_surf: 0.5` overlay did not change the effective 0.1 m setting.
The generated configuration and ROS parameter dump were audited; the complete
input/output checks remain valid. The original artifact-only harness stopped
at bag EOF and truncated queued work, so those earlier runs are excluded from
complete-window claims. The accepted run waits for connected sensor transports,
checks input header hashes, and drains to the final IMU-supported scan.

Stage 1's six original backends passed both the corrected-clock 30-second,
0.5 m profile (288 rows each) and the corrected-clock 141-second profile.
Each long run received 1414 LiDAR / 28595 IMU messages, wrote 1402 trajectory
and timing rows, had nonzero effective constraints in every frame, and finite,
nonnegative values in all eight timing columns. The earlier `9fa3111` long run
had complete input/trajectory records but negative durations from wall-clock
rollback; it remains historical and is not used as timing evidence. An earlier
directory named “coarse” actually used the 0.1 m configuration, not the 0.5 m
profile. The phase-1 record is
[`STAGE1_RESULTS.json`](../doc/workspace/evidence/STAGE1_RESULTS.json).

R-VoxelMap's 30-second and 141-second replays passed complete-input checks,
with 288 and 1402 output rows respectively. The 141-second record matched
1414 LiDAR and 28595 IMU messages; all 1402 frames had nonzero effective
constraints (minimum 473, median 2418, maximum 4850), and native root voxels
peaked at 122755. Monotonic durations were finite and nonnegative, pose
quaternion norm error stayed below `5e-6`, and the final PCD contained points.
The replay has no independent ground truth and does not establish accuracy
improvement. Timings were collected under non-isolated CPU load and are not a
controlled performance ranking. Full measurements are in
[`R_VOXELMAP_RESULTS.json`](../doc/workspace/evidence/R_VOXELMAP_RESULTS.json).

The R ASan limitation above remains: one invocation hung, its cause is unknown,
and leak checking was disabled for ptrace incompatibility. Stage 1 flow
regression is complete; it does not establish ground-truth accuracy or a
controlled performance comparison. hVox main-line build, conventional tests, and short replay passed.
