#!/usr/bin/env bash
# Build CPU-optimized BLASFEO + HPIPM from the already-pinned colcon source cache.
# The standard workspace installation and OCS2 submodule are left untouched.
set -euo pipefail

if [[ ! -f /.dockerenv && ! -f /run/.containerenv ]]; then
  echo "Run this script inside the MPC Docker container." >&2
  exit 1
fi
if ! [[ " $(< /proc/cpuinfo) " == *avx2* && " $(< /proc/cpuinfo) " == *fma* ]]; then
  echo "The AVX2 backend requires a CPU with AVX2 and FMA support." >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${WB_MPC_WORKSPACE:-/wb_humanoid_mpc_ws}"
BUILD_ROOT="${WB_MPC_FAST_QP_ROOT:-${REPO_ROOT}/.docker_ws/optimized_qp/avx2}"
JOBS="${PARALLEL_JOBS:-4}"
PREFIX="${BUILD_ROOT}/install"
BLASFEO_SOURCE="${WORKSPACE}/build/blasfeo_catkin/download"
HPIPM_SOURCE="${WORKSPACE}/build/hpipm_catkin/download"

for source_dir in "${BLASFEO_SOURCE}" "${HPIPM_SOURCE}"; do
  if [[ ! -f "${source_dir}/CMakeLists.txt" ]]; then
    echo "Missing pinned source cache: ${source_dir}. Run make build-all inside Docker first." >&2
    exit 1
  fi
done

mkdir -p "${BUILD_ROOT}/sources/blasfeo" "${BUILD_ROOT}/sources/hpipm"
touch "${BUILD_ROOT}/COLCON_IGNORE"
# BLASFEO generates headers in its source directory: work on independent copies.
cp -a "${BLASFEO_SOURCE}/." "${BUILD_ROOT}/sources/blasfeo/"
cp -a "${HPIPM_SOURCE}/." "${BUILD_ROOT}/sources/hpipm/"

cmake -S "${BUILD_ROOT}/sources/blasfeo" -B "${BUILD_ROOT}/build/blasfeo" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DTARGET=X64_INTEL_HASWELL -DLA=HIGH_PERFORMANCE -DMF=PANELMAJ \
  -DBUILD_SHARED_LIBS=ON -DBLASFEO_EXAMPLES=OFF -DBLASFEO_TESTING=OFF
cmake --build "${BUILD_ROOT}/build/blasfeo" --parallel "${JOBS}"
cmake --install "${BUILD_ROOT}/build/blasfeo"

cmake -S "${BUILD_ROOT}/sources/hpipm" -B "${BUILD_ROOT}/build/hpipm" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DTARGET=AVX -DBLASFEO_PATH="${PREFIX}" -DBLASFEO_INCLUDE_DIR="${PREFIX}/include" \
  -DBUILD_SHARED_LIBS=ON -DHPIPM_TESTING=OFF
cmake --build "${BUILD_ROOT}/build/hpipm" --parallel "${JOBS}"
cmake --install "${BUILD_ROOT}/build/hpipm"

printf 'BLASFEO target: X64_INTEL_HASWELL\nHPIPM target: AVX\n' > "${PREFIX}/fast_qp_backend.txt"
echo "Optimized QP libraries installed in ${PREFIX}"
echo "Run a command with: bash scripts/with_fast_qp.bash ros2 ..."
