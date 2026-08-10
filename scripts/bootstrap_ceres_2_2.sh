#!/usr/bin/env bash
set -euo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly project_root="$(cd "${script_dir}/.." && pwd)"
readonly deps_root="${project_root}/.deps"
readonly source_root="${deps_root}/src"
readonly build_root="${deps_root}/build"
readonly install_root="${deps_root}/install"
readonly abseil_tag="20240116.2"
readonly ceres_tag="2.2.0"
readonly jobs="${KALIBR_BUILD_JOBS:-4}"

# ROS overlays, Conda and the Windows CMake package registry can otherwise
# inject ABI-incompatible libraries into this Linux-only dependency prefix.
unset CMAKE_PREFIX_PATH ROS_DISTRO ROS_ROOT ROS_PACKAGE_PATH PYTHONPATH
unset CONDA_PREFIX CUDA_HOME CUDA_PATH

mkdir -p "${source_root}" "${build_root}" "${install_root}"

if [[ ! -d "${source_root}/abseil-cpp/.git" ]]; then
  git clone --depth 1 --branch "${abseil_tag}" \
    https://github.com/abseil/abseil-cpp.git "${source_root}/abseil-cpp"
fi

cmake -S "${source_root}/abseil-cpp" -B "${build_root}/abseil-cpp" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_INSTALL_PREFIX="${install_root}" \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DABSL_BUILD_TESTING=OFF \
  -DABSL_ENABLE_INSTALL=ON \
  -DABSL_PROPAGATE_CXX_STD=ON
cmake --build "${build_root}/abseil-cpp" --parallel "${jobs}"
cmake --install "${build_root}/abseil-cpp"

if [[ ! -d "${source_root}/ceres-solver/.git" ]]; then
  git clone --depth 1 --branch "${ceres_tag}" \
    https://github.com/ceres-solver/ceres-solver.git "${source_root}/ceres-solver"
fi

cmake -S "${source_root}/ceres-solver" -B "${build_root}/ceres-solver" \
  -U "glog_DIR" -U "GLOG_*" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_INSTALL_PREFIX="${install_root}" \
  -DCMAKE_PREFIX_PATH="${install_root}" \
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF \
  -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DMINIGLOG=OFF \
  -DGLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION=OFF \
  -DGLOG_INCLUDE_DIR_HINTS=/usr/include \
  -DGLOG_LIBRARY_DIR_HINTS=/usr/lib/x86_64-linux-gnu \
  -DSUITESPARSE=ON \
  -DEIGENSPARSE=ON \
  -DLAPACK=ON
cmake --build "${build_root}/ceres-solver" --parallel "${jobs}"
cmake --install "${build_root}/ceres-solver"

printf 'Ceres %s installed in %s\n' "${ceres_tag}" "${install_root}"
printf 'Configure kalibr_no_ros with -DCMAKE_PREFIX_PATH=%s\n' "${install_root}"
