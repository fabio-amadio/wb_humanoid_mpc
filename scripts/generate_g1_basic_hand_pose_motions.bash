#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MOTIONS_PER_PRIMITIVE="${MOTIONS_PER_PRIMITIVE:-10}"
EXTRA_GENERATOR_ARGS="${EXTRA_GENERATOR_ARGS:-}"

PRIMITIVES=(
  stand
  walk_forward
  walk_backward
  walk_left
  walk_right
  turn_left
  turn_right
  walk_forward_turn_left
  walk_forward_turn_right
)

cd "${REPO_ROOT}"

for primitive in "${PRIMITIVES[@]}"; do
  args="--basic-primitive ${primitive} --num-motions ${MOTIONS_PER_PRIMITIVE}"
  if [[ -n "${EXTRA_GENERATOR_ARGS}" ]]; then
    args="${args} ${EXTRA_GENERATOR_ARGS}"
  fi

  echo "[generate_basic_hand_pose] ${primitive}: ${MOTIONS_PER_PRIMITIVE} motions"
  make generate-g1-random-hand-pose-mpc-npz GENERATOR_ARGS="${args}"
done
