# i-Octree source and local adaptation

The source baseline is `upstream/i-Octree` revision
`5b640e05` (`octree2/Octree.h`), by Jun Zhu, Tsinghua University. The local
upstream compatibility revision `25903a33` modifies only the comparison
example's CMake configuration. The repository README and `LICENSE` identify
GPLv2; the original core header itself includes a complete MIT permission
notice, preserved in both local source files. The repository GPLv2 LICENSE is
also retained here. No Python/Torch wrapper, independent node or benchmark
ikd-Tree is imported.

This is a structure-preserving adaptation with numerical fixes, not an
unchanged binary or claim of upstream performance. It keeps octant routing,
local leaf splitting, root growth, incremental insertion, contiguous leaf
storage, native batch rejection for on-tree downsampling, AABB-pruned kNN and
radius searches, and physical subtree/leaf deletion. Each leaf now owns an
RAII `vector<Point3f>` contiguous block rather than an array of float records
plus an auxiliary pointer vector. The original fourth float ID and public ID
searches are removed: IDs were rounded beyond 2^24 and reused after rebuild.
No pointers into leaf storage are exposed across a mutation. Debug file writers
with author-machine paths and separate comparison/instrumentation code are not
included.

This adaptation promotes the author's routing center and extent to binary64;
they are used only for routing/splitting/growing. Every node also stores the tight float
min/max of its actual descendant points, plus exact live count and subtree
height. Searching, full-subtree radius inclusion, and physical deletion use
these data AABBs, never the routing cube. Search never propagates the original
`inside` early-exit; sibling data AABBs may overlap. This adds six floats per
node, promotes four routing scalars to double, and adds live-count/height
counters. As in the author structure, eight child-pointer slots are allocated
only for internal nodes; a leaf retains one null child-array pointer. Actual
memory also depends on alignment and vector capacity. Visited-node
counts and performance can differ; no speedup is implied.

On the validation host (x86-64, GCC 9.4), the adapted node is 112 bytes and a
field-only mirror of the author's node is 56 bytes; the optional internal child
array occupies 64 bytes in either layout. Per point at exact vector capacity,
the original four-float block plus pointer costs 24 bytes, while the adapted
XYZ element costs 12 bytes. These exclude allocator and vector-capacity
overhead and are not a measured end-to-end memory benchmark.

Map input is validated and converted once from double to binary32 under
round-to-nearest/ties-to-even. Queries remain double. Distance is computed as
`D = RN(RN(RN(dx*dx)+RN(dy*dy))+RN(dz*dz))`, with each `d` the absolute value
of the corresponding RN binary64 subtraction. Radius has the strict `D <
RN(r*r)` boundary; the common fifth-neighbor point-plane gate uses `D <=
RN(R*R)`. The isolated translation unit is built without fast-math,
reassociation, FMA contraction or LTO. It checks the calling thread has RN,
FTZ and DAZ off; architectures without the implemented mode check are rejected.
Coordinates and query components must be finite and within binary32's finite
range; radius must be finite and in `(0,4*FLT_MAX]`, with a nonzero finite
binary64 square. The minimum extent is restricted to `[float_denorm_min,
FLT_MAX]`; the maximum tree depth is at most 512. Numeric/coordinate/expansion
and actual split-depth checks run on a complete batch before mutating the old
tree, and reject impossible operations. The check does not promise that an
out-of-memory exception midway through insertion is transactional; remaining
nodes retain valid AABBs and counts if such an exception occurs.

Initialization is not downsampled, and updating an empty tree first initializes
it. As in the author code, when an existing leaf cannot split and its extent is
at most twice the configured minimum, an already populated leaf rejects the
whole incoming batch only when its prior size is greater than `bucket/8`.
Initial/new subtrees and a batch already accepted into a leaf can exceed that
threshold. The default is no on-tree downsampling; when enabled it is passed on
every update. Live count reflects accepted points, resets on clear/reinitialize,
and falls on physical deletion. Empty builds and queries are valid; k=0 returns
an empty result and negative k is rejected. Query results contain point values
and the actual count, never uninitialized capacity entries.

Only physical deletion is available. Box deletion removes stored points inside
closed double bounds; a separate tree traversal retains a closed common local
window without rebuilding the entire tree. Snapshots copy only current leaf
points; the common map does not keep a parallel historical point vector.
The shared point-plane observation is a PV-LIO-PLUS adapter, not the original
i-Octree paper's end-to-end estimator.
