#!/usr/bin/env bash
#
# Usage:
#
# $ cd ~/your_colcon_ws/src/wb_humanoid_mpc/docker
# $ ./launch_wb_mpc.bash    # Launch the WB Humanoid MPC Docker container
#
# (Cross reference this file with the "run" section of ../.devcontainer/devcontainer.json)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_REPO="$(realpath "${SCRIPT_DIR}/..")"
HOST_WS_CACHE="${HOST_REPO}/.docker_ws"
CONTAINER_WS="/wb_humanoid_mpc_ws"
CONTAINER_REPO="${CONTAINER_WS}/src/wb_humanoid_mpc"

# Allow GUI applications
xhost +SI:localuser:root

# Generate Xauthority file for X11 forwarding
XAUTH=/tmp/.docker.xauth
if [ ! -f "${XAUTH}" ]; then
  touch "${XAUTH}"
  xauth nlist "${DISPLAY}" \
    | sed -e 's/^..../ffff/' \
    | xauth -f "${XAUTH}" nmerge -
  chmod a+r "${XAUTH}"
fi

# Keep the colcon workspace artifacts on the host between container runs.
mkdir -p "${HOST_WS_CACHE}/src" "${HOST_WS_CACHE}/build" "${HOST_WS_CACHE}/install" "${HOST_WS_CACHE}/log" "${HOST_WS_CACHE}/.ccache"

# Run the container, mounting the repo at the expected colcon workspace path
docker run --rm -it \
  --name wb-mpc-dev \
  --net host \
  --privileged \
  -u root \
  -e DISPLAY \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY="${XAUTH}" \
  -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${XAUTH}:${XAUTH}:rw" \
  -v "${HOST_WS_CACHE}:${CONTAINER_WS}:cached" \
  -v "${HOST_REPO}:${CONTAINER_REPO}:cached" \
  --workdir "${CONTAINER_REPO}" \
  wb-humanoid-mpc:dev \
  bash

echo "Done."
