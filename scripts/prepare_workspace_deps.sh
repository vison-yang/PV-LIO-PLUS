#!/usr/bin/env bash
# Copy an existing, staged Eigen 3.4 / Ceres 2.2 installation into the workspace.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
workspace=$(cd -- "$script_dir/../../.." && pwd -P)
source_prefix=${LIO_DEPS_SOURCE_PREFIX:-${HOME}/srcv/thirdparty/ceres-2.2-build/verification/stage/usr/local}
prefix=${LIO_WORKSPACE_DEPS_PREFIX:-$workspace/.workspace-deps}

usage() {
  echo "Usage: $0 [--source-prefix STAGED_PREFIX] [--prefix WORKSPACE_PREFIX]"
}

while (($#)); do
  case $1 in
    --source-prefix) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; source_prefix=$2; shift 2 ;;
    --prefix) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; prefix=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

check_prefix() {
  local candidate=$1
  [[ -f "$candidate/share/eigen3/cmake/Eigen3Config.cmake" &&
     -f "$candidate/share/eigen3/cmake/Eigen3ConfigVersion.cmake" &&
     -f "$candidate/share/pkgconfig/eigen3.pc" &&
     -f "$candidate/include/eigen3/Eigen/Core" &&
     -f "$candidate/lib/cmake/Ceres/CeresConfig.cmake" &&
     -f "$candidate/lib/cmake/Ceres/CeresConfigVersion.cmake" &&
     -e "$candidate/lib/libceres.so" ]] || return 1
  grep -q 'set(PACKAGE_VERSION "3\.4\.0")' "$candidate/share/eigen3/cmake/Eigen3ConfigVersion.cmake" &&
    grep -q 'set(PACKAGE_VERSION "2\.2\.0")' "$candidate/lib/cmake/Ceres/CeresConfigVersion.cmake" &&
    grep -q 'set(CERES_EIGEN_VERSION 3\.4\.0)' "$candidate/lib/cmake/Ceres/CeresConfig.cmake"
}

if ! check_prefix "$source_prefix"; then
  echo "Expected an existing staged Eigen 3.4.0 and Ceres 2.2.0 prefix: $source_prefix" >&2
  exit 1
fi
if [[ -e $prefix ]]; then
  if [[ -f "$prefix/.lio-workspace-deps" ]] && check_prefix "$prefix"; then
    echo "Using existing workspace dependency prefix: $prefix"
    exit 0
  fi
  echo "Refusing to replace an existing unrecognized or incomplete prefix: $prefix" >&2
  exit 1
fi

mkdir -p -- "$(dirname -- "$prefix")"
tmp_prefix=$(mktemp -d "${prefix}.tmp.XXXXXX")
trap 'rm -rf -- "$tmp_prefix"' EXIT
cp -a -- "$source_prefix/." "$tmp_prefix/"
# Eigen's staged .pc file names /usr/local; use its own location after copying.
sed -i 's|^prefix=.*$|prefix=${pcfiledir}/../..|' "$tmp_prefix/share/pkgconfig/eigen3.pc"
printf 'Eigen 3.4.0; Ceres 2.2.0; Ceres Eigen 3.4.0\nsource=%s\n' "$source_prefix" > "$tmp_prefix/.lio-workspace-deps"
check_prefix "$tmp_prefix"
mv -T -- "$tmp_prefix" "$prefix"
trap - EXIT
echo "Prepared workspace dependency prefix: $prefix"
