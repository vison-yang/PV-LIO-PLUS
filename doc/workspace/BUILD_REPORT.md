# 工作空间整理与构建记录

日期：2026-11-14。工作空间是 `/home/yxy/src/lio_ws`，根目录没有有效 Git 仓库；PV_LIO_PLUS 和各上游源码分别保留自己的 Git 仓库。

## 范围与目录

本轮新增的是地图候选研究、论文笔记和可构建源码，未新增 `MapManager` selector，未修改 IMU 传播、去畸变或残差算法。现有后端的原生权重仍不同，严格地图消融的边界见[范围审查](../../paper/notes/MAP_BACKEND_SCOPE_REVIEW.md)。

- `src/`：14 个 ROS1 Catkin 包，包含新获取的 `ig_lio`；Super-LIO 自带 `basic` 和 `super_lio` 两个包。
- `upstream/`：BIEVR-LIO、Surfel-LIO、i-Octree、kiss-icp、lio_gvm，各自独立构建。BIEVR 是普通 CMake / ROS1 / ROS2 混合源码；Surfel 和 i-Octree 是独立 CMake；KISS 当前包装器是 ROS2；lio_gvm 与 FAST_LIO 包名冲突且缺作者源码。
- `upstream/deps/`：i-Octree 比较示例所需的官方 ikd-Tree，以及 KISS 固定版本的 Sophus 1.24.6、robin-map 1.4.0。
- `.workspace-deps/`：工作空间内的 Eigen 3.4.0 和 Ceres 2.2.0 前缀，不修改系统库。
- `.workspace-tools/`：工作空间内的 CMake 3.28.4；KISS 所用 Sophus 要求 CMake >= 3.24。安装命令：`python3 -m pip install --target .workspace-tools/python cmake==3.28.4`。
- `research/deps/`、`research/build/`：保留的历史依赖与构建产物。本轮未删除它们；其中旧路径不能作为当前源码位置或新构建证据。

固定版本与来源见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。旧 `research/BIEVR-LIO`、`research/Surfel-LIO`、`research/lio_gvm` 已移到 `upstream/`，没有再复制第二份工作树。

## 环境修复与共享构建

系统为 Ubuntu 20.04 / ROS Noetic，GCC 9.4、系统 CMake 3.16.3、PCL 1.10。系统 Eigen 3.3.7 / Ceres 1.14 与 GenZ-LIO / BIEVR 要求不一致。

本地前缀取自先前已验证的 Eigen/Ceres 暂存安装树；`prepare_workspace_deps.sh` 校验版本，复制到工作空间并修正 Eigen pkg-config 前缀。不会将旧 Ceres 构建目录视作统一 Eigen 的安装前缀。

只指定 `Eigen3_DIR` 不够：Noetic 的 PCL 配置会导出普通 `-I/usr/include/eigen3`，优先于本地 `-isystem`。共享构建脚本关闭 CMake 用户包注册表，使用 project hook 将本地 Eigen 普通 include 放在前面；GenZ 节点和测试移除导致顺序失效的 `SYSTEM`。核对实际 `compile_commands.json` 的 87 条命令，未发现旧 Eigen 优先的反例。

兼容性修复包括 iG-LIO 直接 ROS/PCL 依赖和生成消息目标顺序、Super-LIO 对 `basic` 的 manifest 依赖，以及继续保留既有 PV-LIO / VoxelMap++ / GenZ 的 Catkin 接入修复。此前已存在的目标/节点重名、消息命名空间及 launch 接入修改经检查后保留，并一同提交到各自仓库；新旧修改的完整相对上游补丁保存在源码清单中。PV_LIO_PLUS 的传播、去畸变和观测算法未改动。

**最终结果：14 包全量共享构建退出 0；GenZ 现有 CTest 21/21 通过；23 个 ELF 的 `ldd -r` 均无缺库或未解析符号；14 包 `rospack find` 通过。** 另用实际编译参数运行 13 组代表性预处理，均读取本地 Eigen 3.4.0；PV 节点确认包含六组原生后端符号。Ceres consumer 实际链接本地库，输出 `Eigen 3.4.0; Ceres 2.2.0; x=10; usable=1`。

机器可读结果见 [BUILD_RESULTS.json](BUILD_RESULTS.json)。旧根 `build/` / `devel/` 不是本轮验证入口；请使用下面的独立 build/devel 路径。

```bash
cd /home/yxy/src/lio_ws
bash src/PV_LIO_PLUS/scripts/prepare_workspace_deps.sh
bash src/PV_LIO_PLUS/scripts/build_workspace.sh \
  --build-dir build/maintenance-20261114-unified \
  --devel-dir devel/maintenance-20261114-unified --jobs 2
source devel/maintenance-20261114-unified/setup.bash
```

`prepare_workspace_deps.sh` 默认读取本机已有的 `~/srcv/thirdparty/ceres-2.2-build/verification/stage/usr/local`；其它机器须用 `--source-prefix` 指定已安装且相互匹配的 Eigen 3.4.0 / Ceres 2.2.0 前缀。它不是联网安装器。

## upstream 逐个构建

| 项目 | 结果 | 产物或明确阻塞 |
| --- | --- | --- |
| BIEVR-LIO | 通过 | 隔离 ROS1 工作区，选择 core、ros_common、ros1 三包；`process_bag` / `process_topics`。未启用可选 Livox gen1/gen2 包装。 |
| Surfel-LIO | 通过 | 本地固定版本 `73348c1`；重新编译其 bundled Pangolin 及 `lio_player`。 |
| i-Octree | 通过 | `time_compare` C++ 示例；显式指定官方 ikd-Tree 路径。未构建可选 Torch/Python 扩展。 |
| KISS-ICP | 通过 | 原生 C++ core、metrics、pipeline 静态库；使用本地 CMake 和固定依赖。未构建需要 ROS2 的包装器。 |
| LIO-GVM | 阻塞，真实退出码 1 | CMake 缺少 `include/ikd-Tree/ikd_Tree.cpp`；发布树还缺作者定制 FastVGICP/IKFoM 头文件，不能用通用依赖替代算法。 |

Surfel 的 CMake 增加 `PANGOLIN_BUILD_DIR`，使新构建无需复用含旧路径的源内缓存。i-Octree 增加 `IKD_TREE_ROOT` 和 Threads 依赖；其作者示例原本硬编码一个未随仓库提供的 ikd-Tree 子目录。

```bash
cd /home/yxy/src/lio_ws
python3 src/PV_LIO_PLUS/scripts/build_upstream.py --jobs 2 \
  --logs-dir output/maintenance-20261114/upstream-sequential
```

脚本按上表顺序逐项执行并继续记录失败；`results.json` 保存每条命令、目录和退出码。由于 lio_gvm 缺失源码，当前全表执行最终退出 1；仅验证可构建项可用 `--projects BIEVR-LIO Surfel-LIO i-Octree kiss-icp`。

BIEVR 的运行环境可通过 `source upstream/build/bievr_ros1_ws/install_isolated/setup.bash` 激活。配合本地 Ceres/Pangolin 库路径，四个 ELF（BIEVR 两个、Surfel、i-Octree）均通过 `ldd`，没有缺失动态库。KISS 产物为静态库。

## 证据与限制

原始命令日志和最初工作树快照位于 `output/maintenance-20261114/`；该目录在工作空间内保留，关键结果同时写入本仓库 JSON，精简测试、链接和失败证据保存在 [evidence/manifest.json](evidence/manifest.json)。历史维护报告仍保留其原始日期，不能用历史成功代替本轮构建。

本轮没有运行 rosbag 回放，也没有修改既有 `output/` 轨迹或地图文件；编译和链接通过不代表轨迹精度、地图质量或所有输入边界已验证。LIO-GVM 上游当前 HEAD 仍为 `c80c968ed7ffe61af0a02a1b3c72fe03149c5c3e`，缺失源码仍需作者补齐。

## 提交与重建

三个框架阶段和十一项上游兼容性提交见 [STAGES.md](STAGES.md)。新源码仍保留在各自的 Git 仓库中，主仓库用固定提交与 [patches/](patches/) 保存完整兼容性差异。补丁只应应用于源码清单所列的干净作者基线；不要重复应用到已经包含修复的本地提交。

22 篇独立论文笔记的实质审校和修复依据见 [PAPER_REVIEW.md](PAPER_REVIEW.md)。本轮未改 PV_LIO_PLUS 的估计器或新增 selector，严格地图消融仍需后续统一原生观测策略。
