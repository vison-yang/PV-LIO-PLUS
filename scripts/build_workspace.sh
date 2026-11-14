#!/usr/bin/env bash
# Build the complete ROS 1 catkin source space with workspace-local dependencies.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
workspace=$(cd -- "$script_dir/../../.." && pwd -P)
prefix=${LIO_WORKSPACE_DEPS_PREFIX:-$workspace/.workspace-deps}
build_dir=${LIO_BUILD_DIR:-$workspace/build}
devel_dir=${LIO_DEVEL_DIR:-$workspace/devel}
jobs=${LIO_BUILD_JOBS:-2}

usage() {
  echo "Usage: $0 [--prefix DIR] [--build-dir DIR] [--devel-dir DIR] [--jobs N]"
}
while (($#)); do
  case $1 in
    --prefix) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; prefix=$2; shift 2 ;;
    --build-dir) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; build_dir=$2; shift 2 ;;
    --devel-dir) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; devel_dir=$2; shift 2 ;;
    --jobs) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; jobs=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ $jobs =~ ^[1-9][0-9]*$ ]] || { echo "--jobs must be a positive integer" >&2; exit 2; }
[[ -f "$prefix/share/eigen3/cmake/Eigen3Config.cmake" &&
   -f "$prefix/share/eigen3/cmake/Eigen3ConfigVersion.cmake" &&
   -f "$prefix/lib/cmake/Ceres/CeresConfig.cmake" &&
   -f "$prefix/lib/cmake/Ceres/CeresConfigVersion.cmake" &&
   -f "$prefix/include/eigen3/Eigen/Core" ]] || {
  echo "Workspace dependencies not found at $prefix; run prepare_workspace_deps.sh first." >&2
  exit 1
}
grep -q 'set(PACKAGE_VERSION "3\.4\.0")' "$prefix/share/eigen3/cmake/Eigen3ConfigVersion.cmake" &&
  grep -q 'set(PACKAGE_VERSION "2\.2\.0")' "$prefix/lib/cmake/Ceres/CeresConfigVersion.cmake" &&
  grep -q 'set(CERES_EIGEN_VERSION 3\.4\.0)' "$prefix/lib/cmake/Ceres/CeresConfig.cmake" || {
  echo "Expected Eigen 3.4.0 and Ceres 2.2.0 built against Eigen 3.4.0: $prefix" >&2
  exit 1
}
[[ -f /opt/ros/noetic/setup.bash ]] || { echo "ROS Noetic not found in /opt/ros/noetic" >&2; exit 1; }
prefix=$(cd -- "$prefix" && pwd -P)
mkdir -p -- "$build_dir" "$devel_dir"
build_dir=$(cd -- "$build_dir" && pwd -P)
devel_dir=$(cd -- "$devel_dir" && pwd -P)

# Noetic's pcl_ros exports /usr/include/eigen3. Ensure the selected Eigen comes
# first for every package, even where that legacy path is a regular -I option.
cat > "$build_dir/workspace_eigen_priority.cmake" <<'EOF'
if(NOT IS_DIRECTORY "${LIO_EIGEN_INCLUDE_DIR}/Eigen")
  message(FATAL_ERROR "Workspace Eigen headers missing: ${LIO_EIGEN_INCLUDE_DIR}")
endif()
set(CMAKE_NO_SYSTEM_FROM_IMPORTED ON)
include_directories(BEFORE "${LIO_EIGEN_INCLUDE_DIR}")
EOF
# ROS setup scripts can refer to unset variables.
set +u
source /opt/ros/noetic/setup.bash
set -u
export CMAKE_PREFIX_PATH="$prefix${LIO_EXTRA_CMAKE_PREFIX_PATH:+:$LIO_EXTRA_CMAKE_PREFIX_PATH}:/opt/ros/noetic"
export PKG_CONFIG_PATH="$prefix/share/pkgconfig:$prefix/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"

catkin_make -C "$workspace" --build "$build_dir" --force-cmake -j"$jobs" --cmake-args \
  -DCATKIN_DEVEL_PREFIX="$devel_dir" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
  -DCMAKE_PROJECT_INCLUDE_BEFORE="$build_dir/workspace_eigen_priority.cmake" \
  -DLIO_EIGEN_INCLUDE_DIR="$prefix/include/eigen3" -DCMAKE_NO_SYSTEM_FROM_IMPORTED=ON \
  -DEigen3_DIR="$prefix/share/eigen3/cmake" \
  -DEIGEN_INCLUDE_DIR="$prefix/include/eigen3" \
  -DEIGEN3_INCLUDE_DIR="$prefix/include/eigen3" \
  -DEIGEN3_INCLUDE_DIRS="$prefix/include/eigen3" \
  -DCeres_DIR="$prefix/lib/cmake/Ceres" \
  -U 'PC_EIGEN_*'
