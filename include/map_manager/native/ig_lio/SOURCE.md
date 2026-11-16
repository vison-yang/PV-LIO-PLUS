# iG-LIO source and adaptation notes

This directory derives its incremental voxel-map behavior from
[`zijiechenrobotics/ig_lio`](https://github.com/zijiechenrobotics/ig_lio) at the fixed
commit `2d79259e3aaa5bbb10d6b13234398760bbacf8f5`. The `voxel_map.h` and
`voxel_map.cpp` files here are a safety-focused reimplementation of that source,
not byte-for-byte copies. `point_type.h` was inspected for the upstream point
representation; this native implementation uses Eigen vectors directly and
does not import the upstream PCL type header.

The upstream GPL v2 license text is copied verbatim to [LICENSE](LICENSE).
The upstream source grants the permission stated by that fixed license; this
file does not assume a later-version grant. Per-file source paths, Git blob IDs,
text SHA-256 values, and retrieval URLs are recorded in
[UPSTREAM_MANIFEST.json](UPSTREAM_MANIFEST.json).

## Preserved map behavior

- Each full int32 cell key stores the sample count `N`, first moment
  `S = sum(p)`, second moment `Q = sum(p p^T)`, centroid, shape matrix, and the
  retained XYZ sequence.
- For `N >= 6`, the raw sample matrix is
  `C = (Q - S mu^T) / (N - 1)`. The upstream spectral replacement is preserved
  as `U diag(1, 1, 1e-3) V^T`, followed by its inverse. The raw `C` and this
  replacement matrix are exposed separately for diagnostics. They are not
  treated as a plane covariance or guaranteed to be a symmetric positive
  definite probability covariance.
- A pre-existing cell updates its statistics only when its old `N < 50`; the
  whole incoming cell batch is applied, so an update may cross 50. Retained
  points are appended only when the updated `N < gridMaxPoints`. A new cell
  retains its complete first batch even when that batch exceeds the threshold.
- Updates touch LRU; queries do not. Insertion evicts the tail after map size
  reaches capacity, so the steady-state map contains at most `capacity - 1`
  cells. `Size`, `PointCount`, the statistical count, and snapshot point count
  therefore remain distinct quantities.
- Search methods 1, 7, 19, and 26 retain their center/axis/face/corner
  neighborhoods; mode 26 checks 27 cells including the center. Radius
  comparison is strict (`distanceSquared < rangeSquared`).

## Safety and observable differences

- Storage is keyed by the complete `(x, y, z)` int32 tuple. The legacy
  capacity-reduced hash only orders batch groups; it never identifies a cell.
  Its multiplication and XOR are explicitly uint32 operations. The result is
  sign-extended as int32, converted to the platform `size_t` width, then reduced
  modulo capacity. This matches the old expression where its signed arithmetic
  was representable and defines the formerly overflowing cases.
- Every input batch is checked before mutation. Coordinates must be finite,
  representable as float, and map to a floored int32 cell index with a one-cell
  margin using `point * (1 / resolution)`, matching the upstream reciprocal
  multiplication. Stored XYZ values are quantized once to float and represented
  as double. Query coordinates remain double and use the same reciprocal
  multiplication; for example, `0.3 * (1 / 0.1)` maps to key 3 even though
  `0.3 / 0.1` can round below 3. Query neighbors are formed by checked integer
  key offsets instead of adding floating resolution offsets.
- Batch sorting is deterministic by `(legacyHashIndex, fullCellKey, inputIndex)`.
  Same-cell points keep input order. This replaces TBB parallel sort's
  unspecified ordering for equal hashes. Full-key separation removes the old
  hash-collision cell replacement behavior. Queries sort by distance, XYZ,
  cell key, then stored index for deterministic ties.
- The upstream PCL, TBB, Boost spreadsort, and glog code paths are not imported;
  this implementation uses Eigen and the standard library. The public
  `ig_lio_plane` selector applies the shared five-point point-plane fit to the
  retained XYZ output. It is a point-plane observation variant, not the
  upstream GICP/three-point-double-plane observation model. The replaced
  spectral matrix is never passed as `PlaneMatch::plane_cov`.
