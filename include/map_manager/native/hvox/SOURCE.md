# hVox and probabilistic kernel source

Source: Surfel-LIO, author revision `73348c1e5560eccd2df1ff9f87683b2326a396d4`;
the locally available compatibility revision `2ed04104a3e166582e9c7560640c919c4e7bb93b`
has no differences in the imported source files. Copyright and full MIT terms
for Surfel-LIO are retained in `LICENSE`; the vendored `unordered_dense.h` and
`stl.h` carry their own author and MIT notices.

`VoxelMap.h/.cpp` and `ProbabilisticKernelOptimizer.h/.cpp` are imported from
`src/core`. `PointCloudUtils.h` is the point type and point-cloud section of the
same upstream header, stopping before the independent utility functions.
`unordered_dense.h` and `stl.h` are the upstream third-party headers. In
`VoxelMap.cpp`, spdlog calls are redirected to the local ROS logging shim
`hvox_log.h`; only messages are changed. Trailing whitespace was removed from
the imported Surfel-LIO files. The native map and PKO algorithms are otherwise
unchanged. The adapter validates finite, representable voxel coordinates
before float-to-integer conversion and rejects unsupported public window
operations.

Each update calls native `UpdateVoxelMap` with the current world-frame IMU
position and `is_keyframe=true`. L1 surfels are precomputed from equally
weighted L0 centroids. Upstream intentionally skips surfel refresh when the
number of children is unchanged, even if their centroids move. Nonplanar L1
cells and their children may be removed by upstream. The native moving box has
half extent `max_distance * box_multiplier` and is recentered only after
movement strictly greater than `max_distance / box_multiplier`. The adapter
reconstructs the native object on clear because upstream `Clear` does not reset
its window center. A snapshot contains current L0 centroids, and map size is
its current L0 voxel count.

The `hvox_pko_ikfom` observation profile preserves signed PKO inputs and
upstream Estimator PKO parameter values. It adapts matching to the shared IKFoM
callback timing: each new correspondence group fixes its residual scale and
resets PKO graduation, while subsequent callbacks in that group update residuals
and weights at the current state. Frame and map-generation changes invalidate
the group. This is not the author's Estimator dual-loop implementation, nor a
claim of full Surfel-LIO reproduction.
