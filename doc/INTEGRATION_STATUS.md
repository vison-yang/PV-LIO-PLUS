# Current Backend Contracts and Verification Status

Updated: 2026-11-15. This document records the six existing `MapManager`
backends, contract fixes, synthetic tests, and complete-input smoke replay.
The production node built successfully and passed a dynamic-link check. Paper eligibility and
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
  manifest lists all six selectors, including `octvox`.
- **Migration:** an old ikd-tree setting `v` was compared as squared distance;
  to reproduce that threshold, set the new radius to `sqrt(v)` metres. Existing
  iVox values already represented metres and remain unchanged.

## Six backend contracts

These entries describe current `MapManager` behavior, not equivalence between
backends. `size()` intentionally reports native backend-dependent units;
`point_count()` is the retained common point-record count and is not a
cross-backend replacement for `size()`.

| Backend | Stored map and update | Query and observation | `size()` | Geometric local window |
| --- | --- | --- | --- | --- |
| VoxelMap | Hash from voxel location to native `OctoTree`; native build/update maintains the hierarchical probabilistic plane map. | Native OMP residual builder supplies point-plane matches; the observation propagates plane and point uncertainty. | Number of root voxel-map entries. | Supported. Removes voxel entries using each native voxel center against the axis-aligned bounds. |
| VoxelMap++ | Hash from voxel location to native `UnionFindNode`; native build/update maintains and merges plane nodes. | Native OMP residual builder supplies the 3-parameter plane match; the observation uses native plane covariance and residual. | Number of voxel-map entries. | Supported. Removes nodes using their voxel centers against the axis-aligned bounds. |
| FAST-LIO2 / ikd-tree | Incremental KD-tree of retained world points; updates apply the configured point-map downsampling policy. | Requests at least five neighbors, checks the fifth-neighbor distance against `r²` inclusively, fits a local plane, then uses the common point-plane observation adapter. | Valid KD-tree point count. | Supported. Deletes tree points outside the world-frame axis-aligned bounds. |
| Faster-LIO / iVox | Sparse voxel point map with configured resolution, nearby-voxel type, capacity, and optional optimized storage/query path. | Native query searches its configured nearby grid domain under strict `< r`; accepted neighbors feed the common point-plane adapter. | Number of valid grids. | Supported. Filters the retained point snapshot and rebuilds the iVox map. |
| C3P-VoxelMap | Native hierarchical voxel `OctoTree` map, with separate plane and optional merge metadata. | Native OMP residual builder supplies C3P plane matches; the observation propagates C3P plane and point uncertainty. | Number of voxel-tree entries. | Supported. Filters retained points and rebuilds the C3P map. |
| Super-LIO / OctVox | Native LRU grid map containing up to eight sub-voxel representatives per grid, bounded by its configured capacity. | Ordered TopK query; accepts four-point fallback or five-point plane fitting, with the native range/residual gate and fixed observation information. | Number of active LRU grid entries. | Unsupported. Configuration and explicit geometric deletion are rejected; OctVox owns capacity/LRU eviction. |

The source of these behaviors is `MapManager` in
[`map_manager.cpp`](../src/map_manager/map_manager.cpp), its public types and
configuration in [`map_manager.h`](../include/map_manager/map_manager.h), and
the six observation paths in [`mapping.cpp`](../src/mapping.cpp). In
particular, common `size()` values count voxel entries, KD-tree points, iVox
grids, or OctVox LRU grids depending on the selected backend. Do not compare
these values as if they all counted points.

### Unmatched-query outputs

The common `MapManager::search(MapPointList, ...)` entry point clears `matches`
and `non_match` before dispatch. ikd-tree, iVox, and OctVox append individual failed queries to
`non_match`. The native OMP residual builders for VoxelMap, VoxelMap++, and
C3P do not guarantee that list contains one entry per failed query. The common
guarantee is only that `matches` is empty when no valid match was produced;
callers must not assume that `non_match` is a complete per-query complement.

### Window and size semantics

When enabled, the geometric window is an axis-aligned world-frame box. VoxelMap
and VoxelMap++ decide retention using voxel centers; ikd-tree removes points
outside the box; iVox and C3P rebuild from retained world points. OctVox rejects
geometric-window mode and retains its own capacity/LRU policy. `snapshot()` and
`point_count()` use the common retained point records except that ikd-tree
snapshots are flattened from the native tree; neither is a normalized count of
native map cells.

## Synthetic verification

The `pv_lio_plus_map_contract_tests` and `pv_lio_plus_observation_tests`
targets were built and run: CTest **2/2 passed**, with **11 GTest cases**
total. Their sources are [`map_contract_tests.cpp`](../tests/map_contract_tests.cpp)
and [`observation_tests.cpp`](../tests/observation_tests.cpp).

- Six parameterized lifecycle cases cover empty maps, plane initialization,
  accepted matches, updates, clear, and reinitialization for every selector.
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

A complete 141-second sequence at a correctly configured 0.5 m resolution is
still running. No ground truth or accuracy improvement is claimed. Build work
may overlap regression runs, so timing CSVs are diagnostic and are not a
controlled performance ranking. Stage 1 remains partially complete. R-VoxelMap
is being implemented and reviewed in an isolated worktree; it has not added a
selector to the main branch, and hVox remains planned work.
