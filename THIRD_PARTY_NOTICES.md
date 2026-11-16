# Third-party notices

This file records third-party source bundled in `PV-LIO-PLUS`. It supplements
the root [`LICENSE`](LICENSE); it does not replace any upstream license or
copyright notice.

## Package license

The original PV-LIO-PLUS work is distributed under the GNU General Public
License, version 2 or any later version (`GPL-2.0-or-later`). The complete
license text is in [`LICENSE`](LICENSE), and the package manifest uses the same
SPDX identifier.

The local map implementations are compiled from the source tree under
`include/map_manager/native`; they are not linked from an external checkout.
For components that retain upstream implementation files, local changes
include file organization, include/path adaptation, map-manager integration,
and warning handling. The iG-LIO map is a safety-focused reimplementation from
pinned source files, not a byte-for-byte copy of the author package; its exact
provenance and observable differences are recorded below.

## Local modification record

| Backend or component | Local changes | Relevant dates |
| --- | --- | --- |
| VoxelMap | `voxel_map_ns` isolation; PV point/covariance and match adaptation; MapManager-facing calls | 2025-06-09, 2026-08-09; notice 2026-08-16 |
| VoxelMap++ | `voxel_map_plus_ns` isolation; shared point/covariance and MapManager-facing call adaptation | 2025-06-09, 2026-08-09; notice 2026-08-16 |
| ikd-Tree | Native path/header adaptation and scoped compiler-warning handling | 2026-08-09; notice 2026-08-16 |
| Faster-LIO iVox | Native path adaptation; iVox KNN safety fixes and removal of the glog dependency | 2026-08-09; notice 2026-08-16 |
| C3P-VoxelMap | Distinct namespace/guard; hash and match-data adaptation; scoped compiler-warning handling | 2026-08-09; notice 2026-08-16 |
| Super-LIO OctVox | Isolated native namespace; retained octant aggregation, capacity/LRU, and ordered TopK table; MapManager and native observation integration | 2026-08-23 |
| Surfel-LIO hVox / PKO | Native L0/L1 map and probabilistic kernel optimizer extraction; ROS logging shim, finite coordinate checks, shared IKFoM observation scheduling and native moving-box context | 2026-11-15 |
| iG-LIO incremental voxel map | Safety-focused local reimplementation from pinned voxel-map sources; full-key cells, checked updates, deterministic ordering, PV map lifecycle integration and shared point-plane adapter | 2026-11-16 |
| MapManager | New common backend lifecycle, search, update, local-window, and map-publication interface | 2026-08-09; notice 2026-08-16 |

Each modified native source file carries a corresponding source-level notice.

## Bundled components

| Component | Local files | Upstream source | License/status |
| --- | --- | --- | --- |
| PV-LIO | PV-LIO-derived package code | <https://github.com/HViktorTsoi/PV-LIO> | GPLv2; see the root `LICENSE` |
| VoxelMap | `include/map_manager/native/voxelmap/voxel_map_util.hpp` | <https://github.com/hku-mars/VoxelMap> | GPLv2; developers credited upstream: Chongjian Yuan and Wei Xu |
| ikd-Tree | `include/map_manager/native/ikdtree/` | <https://github.com/hku-mars/ikd-Tree> | GPLv2; upstream implementation by Yixi Cai, with FAST-LIO2 integration credited to Wei Xu |
| Faster-LIO iVox | `include/map_manager/native/ivox/` except `hilbert.hpp` | <https://github.com/gaoxiang12/faster-lio> | GPLv2; authors credited upstream: Chunge Bai, Tao Xiang, Yajie Chen, Haoqian Wang, Fang Zhang, and Xiang Gao |
| Super-LIO OctVox | `include/map_manager/native/octvox/` | <https://github.com/Liansheng-Wang/Super-LIO/tree/ros1> | GPL-3.0-or-later; source retained under `LICENSES/GPL-3.0.txt`; OctVox is used without the full Super-LIO node |
| R-VoxelMap | `include/map_manager/native/r_voxelmap/` | <https://github.com/NKU-MobFly-Robotics/R-VoxelMap> at `50ffbbab` | The pinned package manifest declares MIT; the snapshot has no standalone LICENSE. Original comments and the exact source provenance/patch record are retained in `native/r_voxelmap/SOURCE.md`. |
| i-Octree | `include/map_manager/native/i_octree/` | <https://github.com/zhujun3753/i-octree> at `5b640e05` | The repository README and LICENSE identify GPLv2, while the imported core header carries its own MIT notice. Both notices and adaptation details are retained locally. |
| Surfel-LIO hVox / PKO | `include/map_manager/native/hvox/` | <https://github.com/93won/lidar_inertial_odometry> at `73348c1e` | MIT; complete upstream license retained in `native/hvox/LICENSE`. Included unordered_dense/stl headers retain their own MIT notices. Source extraction and adaptation are recorded in `native/hvox/SOURCE.md`. |
| iG-LIO incremental voxel-map core | `include/map_manager/native/ig_lio/` | <https://github.com/zijiechenrobotics/ig_lio> at `2d79259e3aaa5bbb10d6b13234398760bbacf8f5` | The fixed upstream root `LICENSE` text is GNU GPL version 2 and is retained verbatim in the native directory. No later-version grant is inferred. The retrieved upstream manifest does not include author `package.xml` metadata; the local package's `GPL-2.0-or-later` declaration needs separate review for this component. This notice makes no legal-compatibility or redistribution-clearance claim. The local code is a reimplementation, not a byte-for-byte copy of the full author package; provenance and differences are recorded in `native/ig_lio/UPSTREAM_MANIFEST.json` and `native/ig_lio/SOURCE.md`. |
| Hilbert curve helper | `include/map_manager/native/ivox/hilbert.hpp` | <https://github.com/spectral3d/hilbert_hpp> | MIT; the complete upstream notice is retained in the file |
| IKFoM toolkit | `include/IKFoM_toolkit/` | <https://github.com/hku-mars/IKFoM> | BSD-3-Clause; the original notices are retained in the source files |
| LOAM/Livox mapping base | `src/mapping.cpp` | LOAM and Livox contributions | BSD-3-Clause; the complete notice is retained at the top of the source file |

## Components requiring upstream clarification

### VoxelMap++

The local copy is `include/map_manager/native/voxelmap_plus/voxelmapplus_util.hpp`.
The upstream repository is <https://github.com/uestc-icsp/VoxelMapPlus_Public>.
Its ROS package metadata declares BSD, but the repository does not provide a
standalone license text, and its README describes the project as an extension
of GPLv2 VoxelMap. PV-LIO-PLUS therefore does not reinterpret the metadata as a
complete license grant. Obtain written upstream clarification and preserve the
corresponding license text before publicly redistributing this local copy.

### C3P-VoxelMap

The local copy is `include/map_manager/native/c3p_voxelmap/` and the upstream
repository is <https://github.com/deptrum/c3p-voxelmap>. The upstream repository
does not provide a standalone license text; its README states that the work is
based on VoxelMap, while its ROS package metadata declares BSD. PV-LIO-PLUS
does not infer a license from those conflicting signals. Obtain written
upstream clarification and preserve the corresponding license text before
publicly redistributing this local copy.

Until these two upstream statuses are clarified, treat the VoxelMap++ and
C3P-VoxelMap copies as research-only material and do not claim that the whole
PV-LIO-PLUS tree is cleared for unrestricted redistribution.

The iG-LIO source text is retained under the fixed upstream GPL version 2
license file. Review the local package's SPDX declaration and the component's
license scope before making any redistribution or compatibility claim; this
notice does not resolve that review.

## Redistribution checklist

When distributing PV-LIO-PLUS or a binary built from it:

1. Keep `LICENSE`, this file, all source files, and the build instructions.
2. Preserve the copyright, license, and warranty notices in the bundled source.
3. For GPL-covered code, provide the corresponding source and build scripts as
   required by the GPL.
4. Resolve the VoxelMap++ and C3P-VoxelMap entries before distributing those
   two backends outside a research checkout.
