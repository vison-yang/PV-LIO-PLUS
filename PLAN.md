# PV-LIO-PLUS Project Plan

## 0. Current status (2026-08-20)

Legend: `[x]` verified completed, `[ ]` not completed, and `[-]` planned
but only partially evidenced. A checked item means that the stated scope was
actually inspected, built, or replayed; it does not imply that all deeper
accuracy or deployment validation is complete.

### Completed

- [x] Five selectable backends are integrated: VoxelMap, VoxelMap++, ikd-tree,
  iVox, and C3P-VoxelMap.
- [x] `MapManager` provides common backend lifecycle and configuration entry
  points.
- [x] C3P has an independent observation-model path; ikd-tree and iVox share
  the point-neighborhood plane-fitting path.
- [x] The automated runner builds first with the `.vscode/tasks.json` command,
  validates bag topic/message types, runs combinations, and archives logs and
  outputs.
- [x] Avia and S3/SOLID bags completed all five backend replays: 10/10 process
  runs passed. This proves compile/startup/replay/output execution, not
  accuracy superiority.
- [x] The test runner preserves generated launch files, configurations,
  commands, logs, status, trajectories, and PCD results in per-run folders.

### Not completed yet

- [ ] A formal per-backend contract for frames, units, covariance layout,
  residual sign, distance semantics, and ownership has not been published.
- [ ] Native residual/Jacobian/weight equivalence checks against every source
  backend have not been automated.
- [ ] Numerical synthetic-plane tests, NaN/Inf checks, non-empty-output gates,
  resource metrics, ATE/RPE, and regression tolerances are not implemented.
- [ ] No Candidate local-map backend beyond the current five has been
  integrated.
- [ ] Candidate source/license feasibility and redistribution review remain
  open for future imports.
- [ ] Map-only versus Native-stack experiment profiles have not yet been
  separated in the runner and result schema.

Evidence for the completed replay is stored outside the source tree under
`results/backend_tests_20260820/` and the repaired Avia/`voxelmap` run under
`results/backend_tests_20260820_repair/`; generated data manifests are local
and intentionally ignored by Git.

## 1. Project objective

PV-LIO-PLUS is a common tightly coupled LiDAR-inertial estimation framework
for comparing local-map backends under the same sensor preprocessing, IMU
propagation, IKFoM state update, and experiment protocol.

The long-term objective is:

> Integrate representative open-source local-map methods into PV-LIO-PLUS
> while preserving each method's native map representation, matching logic,
> residual model, uncertainty model, and update semantics wherever they are
> part of the published method.

The project is a local-map integration framework, not a collection of complete
external LIO nodes. External projects contribute map layers and their required
adapters; PV-LIO-PLUS remains the owner of sensor synchronization, deskewing,
state propagation, iterated filtering, output, and experiment orchestration.

## 2. Non-negotiable design principles

### 2.1 Keep the estimator loop common

The following path remains shared by all backends:

```text
LiDAR / IMU messages
        |
        v
preprocess and deskew
        |
        v
predicted PV-LIO state
        |
        v
backend-specific map search and observation construction
        |
        v
IKFoM iterated update
        |
        v
world-frame insertion and publication
```

A backend must not introduce a second pose estimator, synchronization policy,
or hidden state machine into this path.

### 2.2 Unify lifecycle, not necessarily observation mathematics

`MapManager` owns the common backend lifecycle:

- configure and select a backend;
- initialize, update, clear, and snapshot the map;
- search or dispatch native matches;
- maintain optional local-window state;
- expose configuration and result metadata.

The observation model remains backend-specific when the native method requires
it. The intended grouping is:

```text
VoxelMap       -> native VoxelMap observation model
VoxelMap++     -> native VoxelMap++ observation model
C3P-VoxelMap   -> native C3P observation model
ikd-tree/iVox  -> shared point-neighborhood-to-plane observation model
```

Future Gaussian, surfel, or statistical-voxel methods must not be forced into a
point-to-plane `PlaneMatch` if their published residual is different. They may
need a separate observation type or a backend-provided residual/Jacobian/
information interface.

### 2.3 Treat data and uncertainty as contracts

Every backend adapter must explicitly define:

- LiDAR-frame and world-frame point semantics;
- transform direction and units;
- point covariance frame and units;
- map-plane, surfel, or Gaussian uncertainty;
- residual sign and normalization;
- neighbor distance convention (metres versus squared metres);
- insertion/downsampling/deletion behavior;
- self-match and current-frame visibility rules;
- snapshot and map-size semantics.

Variable names are not sufficient evidence for any of these contracts.

### 2.4 Make comparisons reproducible

Backend comparisons must use the same:

- bag segment and playback policy;
- sensor topics and message-type validation;
- preprocessing and extrinsic parameters;
- initialisation period;
- filter iteration settings;
- output collection rules;
- evaluation metrics and reporting format.

Changing a backend-specific parameter is an experiment variable and must be
recorded with the result.

## 3. Current baseline

### 3.1 Integrated backends

The current selectable backends are:

- `voxelmap`: PV-LIO/VoxelMap probabilistic adaptive voxel planes;
- `voxelmap_plus`: VoxelMap++ native residual and update logic;
- `ikdtree`: FAST-LIO2 incremental ikd-tree with manager-side plane fitting;
- `ivox`: Faster-LIO sparse incremental voxel map with manager-side plane fitting;
- `c3p_voxelmap`: C3P-VoxelMap native probabilistic voxel-plane matching.

### 3.2 Current integration infrastructure

- `MapManager` provides runtime backend selection and common map lifecycle;
- `PlaneMatch` is used only where a backend can truthfully expose a plane
  observation contract;
- C3P has an independent observation-model path;
- ikd-tree and iVox share the point-neighbor plane-fitting path;
- `scripts/run_backend_tests.py` builds first using the command from
  `.vscode/tasks.json`, validates bag topics/types, runs dataset/backend
  combinations, and archives logs and outputs;
- `scripts/datasets.example.json` is the committed manifest template.

## 4. Roadmap

### Phase 0 - Freeze the baseline contract

- [x] Record the five current backend names and output-file stems.
- [x] Record LiDAR/IMU topic and message-type requirements in the test
  manifest format.
- [x] Separate C3P observation construction from ikd-tree/iVox observation
  construction.
- [ ] Add a concise backend contract table to the developer documentation,
  including coordinate frames, residuals, covariance, and distance units.
- [ ] Define a versioned experiment metadata schema for configuration,
  commit, bag identity, playback rate, and output paths.

### Phase 1 - Harden the five-backend baseline

- [x] Compile with the repository's standard VS Code task.
- [x] Run Avia and SOLID/S3 data through all five backends.
- [x] Archive per-combination logs, generated launch files, configurations,
  trajectories, and PCD outputs.
- [ ] Add automated checks that every successful run produced a non-empty
  trajectory and expected map artifact.
- [ ] Add numerical residual/Jacobian checks for VoxelMap, VoxelMap++, C3P,
  ikd-tree, and iVox using synthetic planes.
- [ ] Audit backend-specific numerical assumptions before changing them,
  especially covariance floors, normal normalization, distance units, and
  external-calibration Jacobians.
- [ ] Establish baseline runtime, memory, map-size, valid-match count, and
  convergence statistics for every dataset/backend pair.

### Phase 2 - Stabilize the backend adapter boundary

- [x] Keep native backend data structures isolated under
  `include/map_manager/native/`.
- [ ] Make initialization, update, search, clear, snapshot, and local-window
  behavior explicit for every backend.
- [ ] Distinguish `map_size`, retained point count, valid grid count, and
  published point count in diagnostics.
- [ ] Add backend-specific result metadata rather than inferring semantics
  from a generic point-cloud snapshot.
- [ ] Define how empty maps, insufficient matches, failed plane fitting, and
  unsupported local-window operations are reported.
- [ ] Prevent stale output files from being mistaken for a successful run.

### Phase 3 - Integrate open-source candidate local maps

Candidates should be integrated one at a time, in this order of increasing
observation-model risk:

1. **Hybrid-VoxelMap and R-VoxelMap**
   - first assess whether their native output is still a probabilistic plane;
   - preserve robust/recursive plane estimation and rejection rules;
   - add a dedicated observation path if their weighting differs from
     VoxelMap/C3P.

2. **Super-LIO / OctVox, BIEVR-LIO, and Surfel-LIO / hVox**
   - identify whether the native representation is a point neighborhood,
     plane, surfel, or hierarchical statistical element;
   - preserve map-informed sampling and hierarchy rules;
   - do not replace a surfel/statistical query with arbitrary KNN plane fitting
     merely to satisfy an adapter shape.

3. **LIO-GVM and other Gaussian/statistical maps**
   - derive the native residual and information model first;
   - define a separate observation interface when the residual is not a
     signed point-to-plane distance;
   - keep Gaussian covariance, likelihood, and degeneracy semantics intact.

For each candidate, the required workflow is:

1. verify public source availability, version, and license;
2. identify the smallest local-map source subset, excluding the full LIO node;
3. trace map creation, insertion, query, deletion, export, and statistics;
4. write the native-to-PV-LIO data and coordinate-frame contract;
5. implement the adapter and native observation path in a disjoint backend
   directory;
6. add configuration and one manifest entry;
7. compile the complete workspace;
8. run a short synthetic/smoke bag before long datasets;
9. compare residual counts, weights, trajectory, map, runtime, and memory;
10. only then classify the backend as integrated.

Methods with papers but no released source remain documented candidates and
must not be represented as implemented backends.

### Phase 4 - Make the experiment runner an evaluation tool

- [x] Support multiple datasets and multiple backends from one manifest.
- [x] Validate required bag topics and ROS message types.
- [x] Build before replay and record the build command/result.
- [x] Isolate ROS home/log directories per run.
- [x] Preserve generated launch, commands, logs, status, trajectory, and PCD.
- [ ] Add a dry-run mode that prints the complete test matrix without starting
  ROS or modifying output.
- [ ] Add a run identifier and source commit to every result metadata file.
- [ ] Add automatic trajectory/map sanity checks and a machine-readable
  failure reason.
- [ ] Add optional playback rate, segment selection, and resource limits to
  the manifest while keeping the default comparison protocol explicit.
- [ ] Add an evaluation stage for ATE/RPE, map completeness, effective-match
  ratio, runtime, memory, and convergence behavior.

### Phase 5 - Benchmark, document, and release

- [ ] Freeze a small smoke-test dataset and a full benchmark dataset set.
- [ ] Produce backend comparison tables with identical preprocessing and
  clearly separated static, build, replay, and metric evidence.
- [ ] Document known backend limitations and unsupported parameter combinations.
- [ ] Complete third-party license and redistribution review before publishing
  imported source, especially for VoxelMap++ and C3P-VoxelMap.
- [ ] Update English and Chinese README sections from the verified baseline.
- [ ] Publish only reproducible configurations and result metadata; do not
  publish local data paths or machine-specific manifests.

## 5. Definition of done for a backend

A backend is considered integrated only when all of the following are true:

- its source provenance and license are recorded;
- its map lifecycle is implemented and scoped to `MapManager`;
- its native matching and observation model are preserved or mathematically
  justified in the adapter documentation;
- all coordinate frames, units, covariance semantics, and thresholds are
  verified from producer/consumer code;
- the complete workspace builds with the backend enabled;
- a smoke bag starts, processes data, and shuts down cleanly;
- non-empty trajectory and map artifacts are archived;
- at least one long-sequence comparison is available against the five-backend
  baseline;
- failures are distinguishable from missing data, startup errors, runtime
  crashes, empty matches, and poor estimation quality.

## 6. Explicit non-goals

- Do not embed complete external LIO applications into this package.
- Do not replace a native residual model only to obtain a common C++ type.
- Do not claim accuracy improvement from compilation or successful replay
  alone.
- Do not treat `git diff --check` as runtime or numerical validation.
- Do not use physical/device validation claims for bag-only experiments.
- Do not commit machine-specific data manifests or generated result files.

## 7. Immediate next actions

1. Add the backend contract and experiment metadata schema.
2. Add output-existence and numerical smoke checks to the test runner.
3. Establish the five-backend baseline metrics on Avia and S3.
4. Audit and document current covariance/distance/Jacobian assumptions.
5. Select Hybrid-VoxelMap or R-VoxelMap as the first new open-source
   candidate, subject to source and license verification.

## 8. Engineering conventions

- [x] Track this plan in Git and keep completed items marked with `[x]`.
- [x] Use the project VS Code C/C++ clang-format style for modified C++ code:
  Google base style, 120-column limit, four-space indentation, no tabs,
  aligned assignments/operands, one maximum empty line, and custom braces
  after functions, control statements, classes, structs, enums, and namespaces.
