# BIEVR native height map source and adaptation notes

This native map is based on the fixed official `ethz-asl/BIEVR-LIO` revision
`2306022e341e98ce84244f92ba7d26f047b41dbc` from
<https://github.com/ethz-asl/BIEVR-LIO>. The upstream worktree was clean at that
revision. Per-file Git blob IDs and SHA-256 values are recorded in
[UPSTREAM_MANIFEST.json](UPSTREAM_MANIFEST.json). The BSD 3-Clause notice from
that revision is copied verbatim to [LICENSE](LICENSE).

`voxel_map.h/.cpp` are a C++17 reimplementation of the map and pixel update
path, not copied author files. They use Eigen and the C++ standard library;
TBB, `unordered_dense`, Ceres, the author's registration optimizer, the
inertial pipeline, and the common-window optimizer are not imported. The map
updates and observations are serialized by the caller. A common local-window
mode is not supported by this first native map.

## Preserved source behavior

- Voxel indices use `floor(world * (1 / voxelSize))`; Morton keys interleave
  three 21-bit axes with the non-aliasing signed domain
  `[-2^20, 2^20 - 1]`. Conversion is checked before integer casts. Neighbor
  offsets are checked as signed indices, so the key cannot wrap at a boundary.
- Point moments retain `n`, `sum(p)`, and `sum(p p^T)`. For `n >= 4`, the
  covariance passed to Eigen's self-adjoint eigensolver is
  `(outerSum - sum * mean^T) / (n - 1)`. Its smallest-eigenvalue vector defines
  the image normal. Solver failures and non-finite frames fail explicitly;
  no additional rank or singular-value cutoff is imposed.
- `objectFromWorld_` retains the oriented plane frame and `cameraFromWorld_`
  includes the projected voxel-corner image origin. The image bounds cover the
  oriented projection of all eight voxel corners. `maxImagePixels_` defaults
  to 1,048,576 pixels and rejects an oversized image before allocating its
  matrices.
- Valid old pixels are lifted to 3D, transformed into the new camera, rounded
  to the nearest pixel, and written in old row-major order. A collision is
  overwritten by the later old pixel, matching the source loop. Reprojected
  height is the transformed point's new z coordinate. New world points use
  the same checked `round` pixel conversion and weighted mean. The optional
  mask-aware Gaussian uses the source radius-1, sigma-0.8 kernel; unchanged
  smoothed pixels retain their previous values.
- Fusion weight is `weighted ? min(0.5, 1 / range) : 1`. Ranges are required,
  length-matched, finite, and positive even when weighting is disabled.
- Fewer than four accumulated points remain pending in world XYZ plus range.
  When the voxel first matures, the current batch is projected first, then
  pending points are projected once and their storage is released. Image
  pixels are not retained as a general KNN point set.
- Updates touch LRU; reads do not. The query first returns a mature voxel in
  the query's own cell. If absent or immature, it scans self then
  `+x, -x, +y, -y, +z, -z`, choosing the smallest centroid distance and the
  later candidate on exact ties. A mature local cell is returned as-is so a
  later pixel miss does not trigger cross-cell fallback.

## Native interface and differences

- `Map::Integrate` prechecks the complete point/range batch, key domain, and
  accumulated double moments before changing the map. The batch is then
  `stable_sort`ed by Morton key and x coordinate and processed serially. Group
  boundaries are built as `{0}`, every key transition, and the `N` sentinel;
  this explicitly handles an all-one-voxel batch that the author source's
  last-key initialization could omit.
- Invalid batches are rejected before mutation. Allocation failures and
  failures arising during image projection are explicit exceptions but do not
  promise transaction rollback after geometry/statistic writes have begun.
- `GetVoxel` reads the containing cell whether mature or not. `FindVoxel`
  applies the mature-cell association behavior described above. Both return
  const pointers valid only until the next non-const map operation, and neither
  touches LRU.
- `Size()` counts currently retained voxels. `PointCount()` sums each retained
  voxel's accumulated input `numPoints_` and excludes evicted history. This is
  not a historical all-input point count. `Snapshot()` instead lifts every
  currently valid raw image pixel back to world XYZ; it is a surface-pixel
  snapshot, not an input-point snapshot.
- Storage uses `std::unordered_map` with complete 63-bit Morton keys and a
  separate `std::list` LRU. TBB parallel sorting/updating and the author's
  `unordered_dense` map are replaced with stable single-threaded updates.

The map-only implementation is not the full BIEVR-LIO system and does not claim
the author's end-to-end estimate or registration behavior.
