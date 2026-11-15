# Current Backend Contracts and Verification Status

Updated: 2026-11-15. This document records seven integrated `MapManager`
backends, contract fixes, separate synthetic test suites, and complete-input
replays. The R-VoxelMap main-line production node built successfully and passed
a dynamic-link check; its three main-worktree CTest targets passed 3/3 with 22
GTest cases (ASan was off in this main configuration). Paper eligibility and
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
  manifest lists all seven selectors, including `octvox` and `r_voxelmap`.
- **Migration:** an old ikd-tree setting `v` was compared as squared distance;
  to reproduce that threshold, set the new radius to `sqrt(v)` metres. Existing
  iVox values already represented metres and remain unchanged.

## Seven backend contracts

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

The source of these behaviors is `MapManager` in
[`map_manager.cpp`](../src/map_manager/map_manager.cpp), its public types and
configuration in [`map_manager.h`](../include/map_manager/map_manager.h), and
the seven observation paths in [`mapping.cpp`](../src/mapping.cpp). In
particular, `size()` counts voxel entries, KD-tree points, iVox grids, or
OctVox LRU grids depending on the selected backend; R-VoxelMap `size()` counts
root voxels while its `point_count()` traverses retained native points. Do not
compare these values as if they all counted points.

### Unmatched-query outputs

The common `MapManager::search(MapPointList, ...)` entry point clears `matches`
and `non_match` before dispatch. ikd-tree, iVox, OctVox, and R-VoxelMap append
individual queries that fail their backend-specific search to `non_match`. The
native OMP residual builders for VoxelMap, VoxelMap++, and C3P do not guarantee
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
native recursive map. Neither `size()` nor `point_count()` is a normalized
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
[`STAGE1_RESULTS.json`](workspace/evidence/STAGE1_RESULTS.json).

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
[`R_VOXELMAP_RESULTS.json`](workspace/evidence/R_VOXELMAP_RESULTS.json).

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

Stage 1's six original backends were rerun after the monotonic-clock fix on the
same 30-second, 0.5 m profile; each produced 288 rows with finite, nonnegative
durations. An earlier 141-second `9fa3111` run completed all input and
trajectory checks with 1402 rows per backend, but wall-clock rollback yielded
negative durations; rerun that long profile with the clock fix before using it
as a timing baseline. An earlier directory named “coarse” actually used the
0.1 m configuration and is not the 0.5 m profile. These phase-1 facts are
tracked separately in [`STAGE1_RESULTS.json`](workspace/evidence/STAGE1_RESULTS.json).

R-VoxelMap's 30-second and 141-second replays passed complete-input checks,
with 288 and 1402 output rows respectively. The 141-second record matched
1414 LiDAR and 28595 IMU messages; all 1402 frames had nonzero effective
constraints (minimum 473, median 2418, maximum 4850), and native root voxels
peaked at 122755. Monotonic durations were finite and nonnegative, pose
quaternion norm error stayed below `5e-6`, and the final PCD contained points.
The replay has no independent ground truth and does not establish accuracy
improvement. Timings were collected under non-isolated CPU load and are not a
controlled performance ranking. Full measurements are in
[`R_VOXELMAP_RESULTS.json`](workspace/evidence/R_VOXELMAP_RESULTS.json).

The R ASan limitation above remains: one invocation hung, its cause is unknown,
and leak checking was disabled for ptrace incompatibility. Stage 1 remains
partially complete pending a corrected-clock long run. The current R node
build/link check and main worktree conventional tests have passed.
