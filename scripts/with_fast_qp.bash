#!/usr/bin/env bash
# Opt-in runtime selection of the separately built AVX2 QP backend.
set -eo pipefail

if [[ ! -f /.dockerenv && ! -f /run/.containerenv ]]; then
  echo "Run this script inside the MPC Docker container." >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${WB_MPC_WORKSPACE:-/wb_humanoid_mpc_ws}"
BUILD_ROOT="${WB_MPC_FAST_QP_ROOT:-${REPO_ROOT}/.docker_ws/optimized_qp/avx2}"
PREFIX="${BUILD_ROOT}/install"
if [[ $# -eq 0 || ! -f "${PREFIX}/fast_qp_backend.txt" || ! -f "${PREFIX}/lib/libblasfeo.so" || ! -f "${PREFIX}/lib/libhpipm.so" ]]; then
  echo "Usage: bash scripts/with_fast_qp.bash COMMAND [ARGS...]" >&2
  echo "Build the backend first: bash scripts/build_fast_qp.bash" >&2
  exit 1
fi
if ! [[ " $(< /proc/cpuinfo) " == *avx2* && " $(< /proc/cpuinfo) " == *fma* ]]; then
  echo "This backend requires AVX2 and FMA support." >&2
  exit 1
fi

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${WORKSPACE}/install/setup.bash"
# Apply after ROS overlay setup, which otherwise prepends the generic library directories.
export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH:-}"
exec "$@"
