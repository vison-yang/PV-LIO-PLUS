# R-VoxelMap source and integration changes

Upstream: <https://github.com/NKU-MobFly-Robotics/R-VoxelMap>.
Original `include/voxel_map_util.hpp` SHA-256: `fb32284f599c07c1dd0d5d31bd4013bcf4c7c69a63bce50abe8756abde14e100`.

R-VoxelMap core is extracted from `include/voxel_map_util.hpp` in the local
author repository `src/R-VoxelMap`, author revision
`50ffbbab3626e384d77f0da91a7e51d5e4b14ff7` (the local build-compatibility
revision is `5166480ebec04e348fe3d2aad80a72450ac60ffc`). The author
package declares `MIT` in `package.xml`; that repository snapshot has no
separate LICENSE file. The upstream comments in the extracted algorithms are
retained. ROS node, deskew, visualization and publication code are omitted.

Integration changes:

- Place the point/plane definitions, OctoTree and VoxelMap in a private
  namespace, with local hash functions and a read-only traversal of live
  point lists. RANSAC, split inliers/outliers, internal and child planes,
  incremental updates, subtree rebuilds and root LRU remain the native model.
- Bound-check the reinitialization threshold index before every read. Make
  matching serial to avoid the upstream parallel `vector<bool>` data race.
- Use the solver's sorted eigenvalue indices and reject failed/nonfinite or
  numerically degenerate plane spectra before any eigenvector access or
  covariance denominator. Failed fits retain and recursively distribute their
  points; incremental degeneracy rebuilds from the retained point lists.
- Process a frame's cached nodes in descending layer order. Rebuilding retains
  the current node but may delete descendants, so descendants must have been
  processed first. A future rebuild that deletes ancestors or other branches
  would invalidate this assumption.
- Use floor for voxel and plane-grid coordinates, including exact negative
  boundaries. Compare neighbor decisions against world coordinates in metres
  rather than voxel indices. Reject nonfinite covariance/positions and invalid
  or unrepresentable voxel indices; do not emit invalid or nonpositive
  variance matches. Clamp roundoff below zero before a geometric square root.
- Enforce native LRU after initial build as well as updates so snapshots only
  expose retained root voxels. Native matches omit the source point; the
  adapter restores it and both covariance frames using `useful_index`.
- Matching keeps the native 3-sigma gate and full world covariance. The
  filter weight follows the author's `src/voxelMapping.cpp` consumer around
  lines 1111–1124: plane covariance plus LiDAR sensor covariance rotated into
  the world frame, without adding pose covariance a second time.

Retained native limitations:

- Root and plane point-count thresholds freeze after a batch update. They are
  not strict hard point budgets: a batch may exceed the threshold.
- Matching tries the current root and, after a failed match in an existing
  root, one selected adjacent root. It is not a complete-neighborhood search.
- Invalid/degenerate spectra are rejected using a scale-aware tolerance that
  accounts for raw-second-moment cancellation at translated world positions;
  normal, well-conditioned plane statistics retain the author's formulas.
