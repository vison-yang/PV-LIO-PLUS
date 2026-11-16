# Current Backend Contracts and Verification Status

Updated: 2026-11-16. The backend replay evidence below is from 2026-11-15;
the current refactor's validation is recorded separately below. This document records ten `MapManager`
backend paths, contract fixes, separate synthetic test suites, and complete-input
replays. The `ig_lio_plane` source integration is complete; the round-2
specified build targets and synthetic tests passed, while runtime acceptance
is pending. i-Octree feature `5c27a79` is cherry-picked into main as `6876a8e`;
the main node/link, CTest 7/7 (49 regular GTest cases, ASan off), and a
30-second complete-input replay passed. Its isolated sanitizer/numerical tests
and feature 30/141-second replays also passed. hVox feature commit `997c00a` is cherry-picked into main as `c81f12f`;
the main-line production node built and passed `ldd -r`, CTest 5/5 with 33 GTest
cases, and a 30-second complete-input replay. At the earlier R-VoxelMap integration stage, the main-line node built successfully and
passed a dynamic-link check; its three main-worktree CTest targets passed 3/3
with 22 GTest cases (ASan was off in this main configuration). Paper eligibility and
future adapter boundaries remain in the
[map/residual review](../paper/notes/MAP_BACKEND_SCOPE_REVIEW.md).

## BIEVR-LIO status

BIEVR-LIO is not integrated: this checkout has no BIEVR selector or height-map observation path. The [integration plan](../PLAN.md) lists it as pending; the [IKFoM port document](BIEVR_IKFOM_PORT.md) is a design draft only. Its standalone upstream ROS1 build in the [workspace build report](workspace/BUILD_REPORT.md) is not PV-LIO-PLUS integration evidence.

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
  describe metres and preserve their numeric values. The 2026-11-15 check below
  records the then-current nine-selector example manifest, including `octvox`,
  `r_voxelmap`, `hvox`, and `i_octree`.
- **Migration:** an old ikd-tree setting `v` was compared as squared distance;
  to reproduce that threshold, set the new radius to `sqrt(v)` metres. Existing
  iVox values already represented metres and remain unchanged.

The later diagnostic fix `f6403f0` moves residual-sum accumulation outside
the point-map, OctVox and C3P OpenMP loops. Observation `h/H/R` expressions
are unchanged. The rebuilt node passed complete-input 30-second regressions
for those three paths (288 output rows each); see
[`DIAGNOSTIC_RACE_RESULTS.json`](workspace/evidence/DIAGNOSTIC_RACE_RESULTS.json).
This check does not establish general thread safety.

## iG-LIO point-plane adapter: build and synthetic tests passed; runtime pending

The new selector `ig_lio_plane` retains the iG-LIO incremental voxel map and
uses PV-LIO-PLUS's shared five-point float-QR scalar point-to-plane observation.
It is an explicit map/observation variant, not the author's GICP observation
or full iG-LIO estimator. The complete author checkout `src/ig_lio` is absent
from the current workspace. The local map core is a safety-focused
reimplementation based on the pinned author sources at
`2d79259e3aaa5bbb10d6b13234398760bbacf8f5`; the 2026-11-14 shared ROS1 build
record refers to the earlier complete checkout and does not establish that the
current checkout contains or builds that complete system. The round-2 node
build exited 0 and `ldd -r` reported no missing libraries or undefined symbols;
the node SHA-256 and binary identity are in the result record. The node and
eight GTest targets exited 0. CTest passed 9/9: eight GTest CTest
entries with 65 GTest cases (including 11 new iG cases), plus the configuration
profile test; there were no failures, errors, or skips. The config suite's six
Python tests covered 30 sensor/backend launch combinations and 30 runner-preset
comparisons. ASan was off. This round's build and synthetic evidence does not
establish replay, GUI/device behavior, ATE, or performance. The Avia and S3 bag
paths in the current run manifests were checked and are missing, so no replay
was performed; see
[`IG_LIO_RESULTS.json`](workspace/evidence/IG_LIO_RESULTS.json) and the current
source snapshot in [`SOURCE_MANIFEST.json`](workspace/SOURCE_MANIFEST.json).

The new configuration fields and `unique_ptr` member change the `MapManager`
class layout. Acceptance requires rebuilding all consumers from source; an
ABI check or previously built consumer is insufficient. The native iG map
implementation is compiled with each manager consumer rather than as a shared
Release static library, avoiding a mixed Eigen-macro definition across
production and test builds. The recorded round-2 source fingerprint matches
the 99-file pre-build snapshot; the six consumers' 24 implementation-source
compile-command comparisons matched.

Round 1 did not pass: the first multi-target make invocation used a stale goal
list that omitted the newly generated iG target, and a direct retry exposed a
missing GoogleTest `main`. After adding the project's standard test entry point,
the complete round-2 target list built and the tests above passed. The recorded
failures and correction remain in the result archive.

### Per-cell statistics and retained points

Each active cell tracks the sufficient statistics `N`, `S = Σp`, and
`Q = Σppᵀ`. For `N ≥ 6`, the sample matrix is formed as
`C = (Q − S μᵀ)/(N − 1)`, where `μ = S/N`, and decomposed as `C = UΣVᵀ`.
The map forms the adjusted matrix
`C_t = U diag(1, 1, 1e−3) Vᵀ` and its inverse. This adjusted matrix is not
unconditionally guaranteed to be an SPD probability covariance under finite
precision. The adapter does not publish it through `plane_cov`; the shared
scalar point-plane path uses the common point-uncertainty information instead.

The author-style batch rules are retained: when the old `N < 50`, a batch may
update the sufficient statistics and cross 50, after which those statistics
can freeze. Points are appended to an existing cell only if its updated `N` is
below `grid_max_points`; a new cell stores its initial batch in full. Therefore
`grid_max_points` is not a strict per-cell point cap. `size()` counts active
cells; `point_count()` and `snapshot()` count/return the separately retained
point arrays, not the statistical `N` values.

### Search, capacity, and numerical boundaries

Search supports the configured 1/7/19/26-neighborhood stencils (the 26 mode
contains the center plus 26 adjacent cells). Candidate points must satisfy the
strict `dist² < range²` gate; accepted candidates are sorted before the first
five are passed to the shared plane fit. Query does not update LRU recency;
map updates do. Insertion followed by `size() >= capacity` eviction means the
steady active-cell count can be `capacity - 1`, not `capacity`. Geometric
`local_window` operations are rejected.

Cells use their full signed 32-bit coordinate tuple as the key and a defined
unsigned-wrapped hash. Deterministic insertion order is based on the legacy
wrapped-hash modulo capacity, then tuple and input index. Point insertion uses
float coordinates while query and key checks use guarded double precision.
Input batches are checked before mutation for finite, float-representable
coordinates and valid int32 cell keys; invalid-input rejection leaves the
whole batch unchanged. Clear/reinitialize constructs replacement state first.
Allocation/resource failure has only the basic guarantee across the whole
batch: cells committed before the failure may remain. `MapManager` advances its
revision conservatively before rethrowing that exception; callers must not
swallow it and continue as though the operation had no effect. These
compatibility and boundary choices do not constitute a successful build or
runtime validation.

## Ten backend contracts

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
| i-Octree | Dynamic octree with actual-point AABBs, contiguous RAII leaf blocks, and dynamic local insertion/deletion/downsampling; numeric adaptation uses double queries/distances and checks supported rounding/denormal modes. | Full-tree kNN under the declared binary64 metric; production requests at least five neighbors and accepts when fifth squared distance is `<= R²`, then fits a shared local plane. Native radius query uses strict `< r²`. Failed queries are appended to `non_match`. | Live retained point count (`size()` and `point_count()`). | Supported. Tree-side retain keeps points inside the closed world-frame box without rebuilding the whole tree. |
| iG-LIO / `ig_lio_plane` | Incremental voxel hash with per-cell `N/S/Q` statistics and a separately retained point array; author batch/freeze and insertion-based LRU rules. | Selects the configured 1/7/19/26 stencil, applies strict `dist² < range²`, sorts and sends up to five points to the shared float-QR scalar plane fit; not author GICP. | Active cell count; `point_count()` and snapshot describe retained points, not statistical `N`. | Unsupported. Geometric `local_window` is rejected. |

The source of these behaviors is `MapManager` in
[`map_manager.cpp`](../src/map_manager/map_manager.cpp), its public types and
configuration in [`map_types.h`](../include/map_manager/map_types.h) and
[`map_config.h`](../include/map_manager/map_config.h), re-exported by
[`map_manager.h`](../include/map_manager/map_manager.h), and
the ten observation paths in [`mapping.cpp`](../src/mapping.cpp), plus
[`hvox_observation.cpp`](../src/map_manager/hvox_observation.cpp), the iG map
implementation in [`voxel_map.cpp`](../include/map_manager/native/ig_lio/voxel_map.cpp),
and its [adaptation notes](../include/map_manager/native/ig_lio/SOURCE.md). In
particular, `size()` counts voxel entries, KD-tree points, iVox grids, or
OctVox LRU grids depending on the selected backend; R-VoxelMap `size()` counts
root voxels while its `point_count()` traverses retained native points; hVox
`size()` is its L0 voxel count, `point_count()` uses native retained-point
count, and `snapshot()` contains L0 centroids. hVox `clear()` reconstructs the
native map because upstream `Clear()` does not reset its window center. Do not
compare these values as if they all counted points. i-Octree reports live
retained points, snapshots XYZ, and applies the public closed window through
native tree retention; it does not rebuild the tree for that operation.

The 2026-11-16 source refactor moves i-Octree operations to an internal
non-owning adapter and extracts the shared point-plane fit. This pilot still
leaves storage and lifecycle state in `MapManager`. The current `lio_map_ws`
node and seven test targets built, `ldd -r` passed, and CTest passed 7/7
(54 GTest cases, including five new cases; ASan off). See the
[current results](workspace/evidence/MAP_MANAGER_ADAPTER_RESULTS.json) and
[refactor progress](MAP_MANAGER_REFACTOR_PLAN.md) for source and binary identity.
Replay, bitwise equivalence, and performance A/B validation remain pending;
earlier backend replays do not validate this refactor.

### Unmatched-query outputs

The common `MapManager::search(MapPointList, ...)` entry point clears `matches`
and `non_match` before dispatch. ikd-tree, iVox, OctVox, R-VoxelMap, and i-Octree append
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

## i-Octree verification

Feature `5c27a79` adapts the pinned author tree at `5b640e05` while retaining
its dynamic octree organization, contiguous leaf data, insertion/deletion, and
tree-side downsampling. The local implementation uses actual-point AABBs for
pruning, binary64 query/distance calculations with supported RN/FTZ/DAZ mode
checks, and RAII contiguous leaf blocks. This is a numerically guarded,
structure-preserving adaptation, not an unchanged-upstream performance run.
Detailed source behavior and limitations are recorded in
[`SOURCE.md`](../include/map_manager/native/i_octree/SOURCE.md).

The feature node built with exit code 0 and passed `ldd -r`. Its isolated suite
passed CTest **9/9**. The record contains **49 normal GTest cases plus 11
sanitizer repetitions (60 executions total)**, not 60 unique cases. Positive
ASan/UBSan checks passed and the independent numeric review passed, including
closure of C1/C2. LeakSanitizer was disabled. Main-line commit `6876a8e` built
and passed `ldd -r`; its regular CTest targets passed **7/7** with **49 GTest
cases** (ASan off).

On both feature and main line, the 30-second, 0.5 m profile received **301
LiDAR / 6088 IMU** messages and wrote **288 rows** with no zero-constraint
frames. The feature 141-second profile received **1414 LiDAR / 28595 IMU** and
wrote **1402 rows**; effective constraints ranged from 513 to 5620 (median
2792), the live native map held 15,763–3,918,943 points, quaternion norm error
was below `7.07e-7`, all eight duration columns were finite and nonnegative,
and the final PCD contained 30,411,349 points. Main-line short replay also
passed complete-input checks and wrote 288 rows. These are regression checks,
not ground-truth accuracy or controlled-performance results. The main-line
node/test/replay and feature evidence are recorded in
[`I_OCTREE_RESULTS.json`](workspace/evidence/I_OCTREE_RESULTS.json).

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
