# iVox implementation switch

The iVox backend has two internal implementations selected by
`mapping/ivox_optimized`:

- `false` (default): use the imported Faster-LIO-compatible implementation.
- `true`: use the optimized hash, query scratch space, and update path.

The key is read for every configuration but has an effect only when
`mapping/map_type` is `ivox`. Keeping the key absent or setting it to `false`
preserves the previous default behavior.

## Preserved map and matching semantics

The optimized path deliberately keeps the following behavior shared with the
legacy path:

- voxel coordinates use `round(point / resolution)`;
- CENTER/6/18/26 neighborhood contents and traversal order are unchanged;
- each voxel stores every inserted point in insertion order;
- per-voxel distance filtering and KNN truncation are unchanged;
- the final candidate truncation and nearest-first placement are unchanged;
- capacity is enforced by the same exact, per-point LRU policy.

It does not use small_gicp's floor-based coordinates, point cap, minimum point
spacing, age horizon, or periodic eviction. Those choices would change map
contents or eviction timing.

## Optimized operations

The opt-in path changes only storage and maintenance mechanics:

- reserve the voxel hash table for the configured capacity to avoid rehashing;
- use defined unsigned hash arithmetic and retain the full hash instead of
  applying the legacy fixed modulus;
- keep the default 5-neighbor candidates for all 27 possible surrounding
  voxels in inline scratch storage, falling back to the heap for larger custom
  requests;
- process insertion in input order instead of applying an unsequenced policy
  to shared hash/list containers;
- preserve exact LRU order while skipping a no-op head splice and the redundant
  second hash lookup/write after `std::list::splice`.

The upfront hash reservation trades startup memory for stable lookup/update
cost. The point container remains `std::vector<PointXYZINormal>`: without an
observed voxel-occupancy distribution, embedding several large PCL points in
every list node could increase memory rather than reduce it.

## Validation boundary

Static review can verify the configuration default and the preserved control
flow, but it cannot establish runtime speedup or numerical equivalence. A later
A/B replay should use the same bag and parameters, compare trajectory/map
outputs, and measure search/update time plus peak memory. This change was
prepared without compiling or replaying the node.
