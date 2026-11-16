# MapManager 类和后端职责拆分评估与试点

## 2026-11-16 i-Octree 试点

当前工作区为 `/home/yxy/src/lio_map_ws/src/PV-LIO-PLUS`，分支 `master`，实施基线为 `2774293`。

- 公共类型和配置分别移至 `include/map_manager/map_types.h`、`include/map_manager/map_config.h`；旧 `map_manager.h` 继续提供原有入口与原生传递包含。`MapManager` 的方法、成员类型、顺序和默认值保持；没有把存储所有权移给 adapter。
- `src/map_manager/backends/i_octree_backend.{h,cpp}` 使用无状态窄函数，每次显式接收原生树及配置，承接校验、构建、更新、搜索、窗口保留和快照。replacement 构建完成后的 `clear`、替换、initialized/revision 更新仍由 facade 按原顺序执行。
- 公共点面拟合移至 `src/map_manager/point_plane.{h,cpp}`，原私有函数保留薄包装；保持 float QR、前五点拟合和中心、各后端原门限及 i-Octree 专有有限性拒绝。未修改原生树或滤波观测算法。
- adapter 和拟合源文件分别加入生产节点及四个 manager 测试目标的源列表，而非共用新静态库。这样各 consumer 继续使用自己的 `-UNDEBUG/-UEIGEN_NO_DEBUG` 和 Eigen 包含顺序；i-Octree native 的严格浮点目标和 native-only sanitizer 目标保持原边界。此项细化了初始方案中“链接相同对象”的建议，避免测试与节点的宏配置发生变化。分编译单元后的数值等价仍需运行核对。
- `tests/map_contract_tests.cpp` 新增五项黑盒用例，覆盖跨后端切换、非法 i-Octree 配置保留旧图、空树/空查询仍检查浮点环境、两套 PV native typed 搜索字段和错误后端拒绝。VoxelMap++ 使用斜平面，分别核对非单位 `omega` 和归一化残差；LiDAR/world 协方差取不同值以捕获坐标系误传。

遗留边界：新编译单元仍通过公共头依赖 `common_lib.h`，其中 inline `g2R` 引用 TU-local 的 `static R2ypr/ypr2R`，存在既有的跨编译单元 ODR 风险；新 adapter/helper 不调用这些函数，本次未修改其链接属性。这也不解决 PV/Plus/C3P 的既有原生头限制。公共拟合跨编译单元后可能改变内联与 Eigen 实例化边界；没有 A/B 证据前，不宣称逐位数值或性能等价。

验证结果见[本轮证据摘要](workspace/evidence/MAP_MANAGER_ADAPTER_RESULTS.json)，完整日志、源码差异/新增文件副本、哈希和测试 XML 保存在 `/home/yxy/output/agent/pv_lio_plus_map_adapter/20261116_round1/`。本轮是在用户授权后对当前 checkout 实际执行的结果：

| 验证层级 | 本轮结果及边界 |
| --- | --- |
| 静态检查 | 七个公共定义的声明 token、别名、半径校验运算和原生 include 顺序保持；公共拟合及 i-Octree 搜索除更名/委托外运算 token 一致；19 个源码文件哈希在构建前后相同。 |
| 编译与链接 | 当前工作区节点和七个测试目标增量构建退出 0，节点 `ldd -r` 无缺失库或未定义符号；日志仍有原生 R-VoxelMap 既有符号类型比较警告。 |
| 合成测试 | CTest 7/7，七份新生成 XML 共 54 用例，0 失败、错误或跳过；包含五项新增契约用例。ASan 未启用。 |
| 编译单元边界 | 实际 compile database 中五个 consumer 的十条新增源编译命令均与本目标原始 TU 一致（除源路径及输出路径），原生内核保留四项严格浮点参数。仅验证命令配置，不证明完整 ODR 或 ABI 等价。 |
| 回放与运行效果 | 本轮未执行回放、GUI/设备验证、逐位数值或性能 A/B，未给出精度/性能结论。清单引用的 Avia 和 S3 bag 在当前环境均不存在，路径见证据摘要；历史二进制和回放不能替代本轮运行验收。 |

下一步先补齐可用数据路径，以固定输入核对匹配、有效约束、轨迹及地图/窗口行为，再决定是否迁移其他后端。当前 `scripts/run_backend_tests.py` 还会无条件构建，不应直接作为免重编的回放命令。该试点仍不是存储所有权拆分的完成判据；成员声明不变也不能单独证明 ABI 等价。

## 初始评估（历史记录）

初始评估对象：当时的 `src/PV_LIO_PLUS` 主线 `6876a8ec42bc8a1497b5f6cb249a7d4dff85d11e`。下文的目录、行号和构建示例属于该评估时的工作区；形成评估时只读源码与现有构建配置，没有修改源码、编译或回放。当前实施和验证状态以上节为准，旧构建目录不能直接作为当前 checkout 的证据。

## 判断

值得分阶段拆分，但不宜在本次 23:20 前的收尾中启动大重构。当前 `src/map_manager/map_manager.cpp` 2,241 行、`include/map_manager/map_manager.h` 450 行：前者同时负责选择与生命周期、九种存储/查询/删除、数据转换、点面拟合、窗口和发布，新增后端要在多段相距很远的 dispatch 中同步修改；后者还直接持有不同原生实现。建议最终保留一个 `MapManager` 公共 facade，并以各后端 adapter 隔离各自生命周期、数据、窗口和查询规则；先迁一个可验证的小后端，再决定是否推广。**仅按文件移动现有成员函数不是类/所有权拆分，也不能宣称解决 ODR 或编译问题。**

短期严格保留 `MapManager` 的签名、头文件兼容、字段/默认值/对象布局和运行行为。首个试点采用不持有存储的内部 adapter：引用现有成员，由 facade 继续拥有和管理状态；这样能分离实现职责，但要明确它还没有完成存储所有权隔离。若之后需要 adapter 真正拥有存储、减少 `MapManager` 私有成员，应单列方案，评估私有对象布局和 ABI 变化、完整重编及测试，不把它混进试点。

## 本轮决定

本轮完成类职责评估，暂不实施源码重构。下一阶段按后端职责逐个迁移，保留 `MapManager` 统一入口；长期目标是让后端明确拥有其原生状态和操作。下文的非拥有型 i-Octree adapter 仅是控制风险、验证编译单元边界的过渡试点，不作为完成类拆分的判据。短期保留私有布局是迁移选择，不是长期架构约束；真正迁移存储所有权时重新评审并完整重编。

## 现状和必要边界

- Facade 的 `configure`/`clear`/`initialize`/`update`/`search` 分别在 `map_manager.cpp:287/449/663/963/1395`；`erase_outside_window`、`size`、`point_count`、`snapshot`、`publish_planes` 在 `:1886/2094/2139/2157/2225`。相同后端逻辑分布在多处，不能只搬 `search_*` 就称迁移完成。
- `mapping.cpp:296,394` 分别调用 VoxelMap 和 VoxelMap++ 的两个**原生类型** `search` 重载，后续直接使用各自 `ptpl` 字段计算不同观测；其余路径使用 `MapPointList`、`PlaneMatchList`。没有暴露原生地图 getter。应保留两套 typed 重载及原生字段/量化/残差/权重，不强行转换为单一平面结果或引入通用观测接口。通用数据类型可抽到独立头，但这只是给内部 adapter 建立无原生实现的依赖，不能消除全部耦合。
- `MapManager` 当前有共用 `config_`、`initialized_`、`revision_`、`window_` 和部分后端使用的 `map_points_`，及各自原生容器/指针（`map_manager.h:409–445`）。hVox 的 IMU 位置上下文和 observation cache 依赖 revision；R/hVox/i-Octree 的 snapshot 必须来自当前原生存储，其他后端的 `map_points_` 刷新/过滤方式不同。`size()` 单位依后端不同。不要统一生命周期时改变这些含义。
- `configure` 对不同后端的校验时机不同；hVox/i-Octree 的非法输入须在替换旧图前拒绝；i-Octree 的 `initialize` 先构造 replacement 再 `clear`。`clear`、`configure`、hVox/i-Octree 的修改会递增 revision；不能因架构整理随意增加/省略次数或改变异常顺序。R/hVox/OctVox 公共窗口拒绝的时机、i-Octree 的树上 retain、点图的范围边界也须原样保留。
- 保持地图数据/匹配/输出同现状，观测和滤波仍在共享 `Mapping` 路径；不合并各后端的观测数学、缓存策略、线程模型或输入精度。

## 原生头、ODR 和构建阻断

`mapping.cpp:92,98` 直接包含 `map_manager.cpp`、`config.cpp`；四个地图测试也直接包含 `map_manager.cpp`。`map_manager.h:39–44` 包含六个原生头，`voxel_map_util.hpp`、`voxelmapplus_util.hpp` 等定义了非 inline 的自由函数；将带这些头的 `mapping.cpp`、多个 adapter `.cpp` 和独立 manager `.cpp` 直接链接会产生多重定义。仅删除 `#include "*.cpp"` 或增加 CMake source 不可靠。

更重要的是 `voxelmapplus_util.hpp:43–49` 在头中声明 `static` 配置变量及 `plane_id`，且 `configure` 目前在原翻译单元写入这些变量；C3P 还有 `plane_id`/`merged_plane_idx`，R 也有 `static plane_id`。简单把函数批量改为 inline 并将 header 引入多个翻译单元，可能产生每个 TU 不同的状态，inline 函数引用 TU 局部实体也有 ODR 风险；把 static 直接变为全局/inline 变量也可能改变原有多实例语义。应先明确各变量、函数和原生类型的唯一实现归属，测单进程多实例及 plane ID/合并行为。无证据前不要声称原生头可安全多 TU 共用。

可行路径分两层：

1. 当前保留旧 `map_manager.h` 的原生传递包含与 `mapping.cpp` 的单 TU 方案；独立 adapter 只包含窄的公共数据/config 头和自己需要的安全原生声明，不能包含原 `map_manager.h`。这样可先试点不会引入 PV 实现头的后端，但旧 manager 的 ODR 限制仍在。
2. 真正把所有后端编译为独立目标前，对 PV/Plus/C3P 分别做 declarations/types 与单一实现的整理，审计静态状态并保证调用者可取得 native typed 数据；然后再让节点和测试链接同一个 manager/backend 库。保留旧 include 的源码兼容需要同步处理其传递包含：若旧头仍携带非 inline 定义，外部使用者即便不再 include `.cpp` 仍会和库冲突。此项是独立的高风险改造，不属于简单拆文件。

`pv_lio_plus_hvox_native` 与 `pv_lio_plus_i_octree_native` 现已是独立静态库。i-Octree native 标量内核的 `-fno-fast-math -fno-associative-math -ffp-contract=off -fno-lto` 和独立目标边界须原样保留；不要把 native `.cpp` 编进另一目标或重复链接生成实现。Eigen 优先级、ROS/Livox overlay 和各目标依赖继续使用现有 CMake。R 的头内静态 plane ID 使其不应未经专门 ODR 审计就作为多 TU 试点。

## 推荐最小迁移包

1. 新增内部使用的 `map_types.h`、`map_config.h`：机械搬迁现有 `MapType`、`MapPoint`、`MapUpdateContext`、`PlaneMatch`、`MapWindow` 和 `MapManagerConfig`、校验 helper，保留字段顺序、类型、初值、命名空间及 `NUM_MATCH_POINTS` 语义。旧 `map_manager.h` 包含它们，并保留原本原生头的兼容性。`common_lib.h` 仍含 PCL/ROS/Eigen，因此这些头只减少对**原生地图实现头**的依赖，不是完全轻量。原 450 行约 211 行可搬出，但这只是试点前提，不是本次重构目标本身。
2. 首个 adapter 选 i-Octree：新增私有 `src/map_manager/backends/i_octree_backend.{h,cpp}`，仅包含窄类型、配置、i-Octree native 声明；将 `MakeIOctreeConfig`/点转换和 i-Octree 专属查询、更新、snapshot、retain 等相关逻辑逐项迁入。现有 `std::unique_ptr<i_octree_native::Octree>` 及 manager 的 initialized/revision/window 状态暂不迁移；adapter 以明确的引用/参数使用存储，不缓存引用跨调用。针对初始化，仍在 facade 内先构建并验证 replacement，再按原顺序 `clear`、替换、设置状态和更新 revision。现有公共 API 与布局保持。
3. `fit_point_plane` 目前由多种点图共同调用（`:603`），可机械提取到无原生地图头依赖的内部函数，传入原配置/后端参数，保持浮点顺序、原门限和 i-Octree 专有退化检查；其他点图调用薄包装或同一函数。勿在试点中更换算法或统一各后端阈值。由 i-Octree adapter 查 kNN 并调用此 helper；不复制全图、不改变 native 内核编译参数。
4. CMake 只为窄依赖 adapter 加独立源/目标，并让生产节点及引用 manager 的四个测试目标链接相同对象；本阶段仍保留它们原有的单 TU manager 包含，避免 PV native ODR 问题。确保 `I_OCTREE_NATIVE_ONLY` sanitizer 目标保持只链接原本 native 对象，不能意外引入完整 manager。
5. 通过全部测试及小段回放后，再单独迁 hVox（包含 context、native moving box、revision 和 surfel sign/cache），随后处理 R；最后按明确的类型/实现/状态唯一性方案迁 PV、Plus、C3P、ikd-tree/iVox/OctVox。每次只迁一个 adapter 并独立验收。只有所有状态有明确归属后，才评估拥有状态的 adapter 和真正独立 manager 库；不预设虚基类、Pimpl 或统一所有后端能力。

一个内部 adapter 可只实现真实支持的操作；不要求所有后端静默支持窗口、平面发布或同样的 size 单位。若将来需要公共能力接口，必须保留当前异常及顺序，由独立设计决定，不在试点引入虚拟层。

## 验证包（供后续实施者使用，本次未执行）

- 在相同编译参数下验证旧 `#include "map_manager/map_manager.h"` 能独立编译，并能使用旧类型和两个原生 typed 重载；同时分别单独包含新类型/config 头，检查相反包含顺序。比对公共字段声明、默认值及 `sizeof/alignof(MapManager)`，源码 diff 核对成员顺序；对象大小相同本身不构成完整 ABI 证明。
- 回归原有六后端生命周期/物理半径用例和 R、hVox、i-Octree 专项：保留空图/少点/无匹配、更新、clear/reconfigure、snapshot 与窗口、非有限输入原子性、i-Octree 极值与树结构。增加真正缺失的黑盒覆盖：**已有地图跨后端切换与失败配置不破坏旧图**，以及 PV/Plus typed 搜索字段/错误后端行为；比较语义和字段而非只断言内部调用。hVox 测 revision 驱动的观测组失效，R 保留 ASan 重建安全用例。依赖原有结构/数值 oracle，不复制实现写镜像测试。
- 推荐按已存在主构建树逐个重编，确认旧配置仍指向主线；命令示例（目标和路径来自当前 CMake 配置，实际执行前确认环境和所有者）：

```sh
cd /home/yxy/src/lio_ws
for target in pv_lio_plus_node pv_lio_plus_observation_tests pv_lio_plus_map_contract_tests \
  pv_lio_plus_r_voxelmap_tests pv_lio_plus_hvox_map_tests pv_lio_plus_hvox_observation_tests \
  pv_lio_plus_i_octree_tests pv_lio_plus_i_octree_numeric_oracle_tests; do
  cmake --build build/maintenance-20261114-unified --target "$target" -- -j1 || exit 1
done
(cd build/maintenance-20261114-unified && ctest -R '^_ctest_pv_lio_plus_gtest_' --output-on-failure)
```

- 抽离到独立 TU 的阶段补一个真实多 TU 链接 consumer，确保不会出现重复符号或不同的静态后端状态；如果 native 头仍暴露在老 include 中，consumer 必须覆盖这一包含方式。仅做单个 TU 的编译不能证明 ODR 安全。对已迁后端再运行相同数据和配置的短回放，核对输入覆盖、有效约束、轨迹/地图计数及合理数值容差；实际日志优先，不把编译成功当作回放通过。

可由主 agent 后续分派：工程 owner 实施边界及审查，机械执行者可负责已有目标的构建、CTest、日志归档和固定命令回放；对 native ODR/静态状态与语义等价的判断不下放为机械批量替换。
