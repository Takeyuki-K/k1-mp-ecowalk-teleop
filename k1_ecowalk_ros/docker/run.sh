#!/usr/bin/env bash
# Run the K1 EcoWalk Jazzy container on an x86 Linux host (e.g. Ubuntu 22.04 + Humble on the host).
#   ./docker/run.sh                     -> sim + browser UI (http://localhost:8080)
#   ./docker/run.sh k1-launch rviz:=true
#   ./docker/run.sh bash                -> shell inside the container
#   ECOWALK_DIR=~/k1-mp-ecowalk-public ./docker/run.sh   -> use your own clone (new policies, edits)
set -e
IMAGE=${IMAGE:-k1-ecowalk:jazzy}
ARGS=(--rm -it --net=host --ipc=host
      -e DISPLAY="${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw
      -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
      -e ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}")
# MuJoCo viewer / RViz: allow X11 from the container, use the GPU if available
command -v xhost >/dev/null && xhost +local:docker >/dev/null 2>&1 || true
if command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi nvidia; then
  ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
elif [ -e /dev/dri ]; then
  ARGS+=(--device /dev/dri)
fi
if [ -n "${ECOWALK_DIR}" ]; then
  ARGS+=(-v "$(realpath "${ECOWALK_DIR}")":/opt/k1-mp-ecowalk-public)
fi
exec docker run "${ARGS[@]}" "${IMAGE}" "$@"
