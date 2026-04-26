#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECTION_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${PROJECTION_DIR}/expleo_nav_stack_yolov7_improve_jazzy"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_BRIDGE_DIR="${ROS_BRIDGE_DIR:-/home/lyukeli/ros-bridge}"
CARLA_ROOT="${CARLA_ROOT:-/home/lyukeli/carla}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -d "${WS_DIR}/src" ]]; then
  echo "Missing workspace source directory: ${WS_DIR}/src" >&2
  exit 1
fi

# The copied workspace contains absolute paths from the old machine in
# build/install/log. Rebuilding from scratch avoids sourcing stale prefixes.
rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"
rm -rf "${WS_DIR}/src/perception2d_interfaces/build" \
       "${WS_DIR}/src/perception2d_interfaces/install" \
       "${WS_DIR}/src/perception2d_interfaces/log"

# A CARLA .whl zip in PYTHONPATH can shadow the pip-installed/extracted package
# and make `import carla` fail with "No module named carla.libcarla".
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$(
    python3 - "${PYTHONPATH}" <<'PY'
import os
import sys

entries = [
    item for item in sys.argv[1].split(os.pathsep)
    if item and not (item.endswith(".whl") and "/PythonAPI/carla/dist/" in item)
]
print(os.pathsep.join(entries))
PY
  )"
fi

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

set +u
source "${ROS_SETUP}"
set -u

if [[ -f "${ROS_BRIDGE_DIR}/install/local_setup.bash" ]]; then
  set +u
  source "${ROS_BRIDGE_DIR}/install/local_setup.bash"
  set -u
else
  echo "Warning: ROS bridge install not found at ${ROS_BRIDGE_DIR}/install/local_setup.bash" >&2
fi

if [[ -d "${CARLA_ROOT}/PythonAPI/carla/dist" ]]; then
  export CARLA_ROOT
fi

cd "${WS_DIR}"
colcon build --symlink-install "$@"

echo
echo "Build complete."
echo "Detected CARLA host: $(python3 "${SCRIPT_DIR}/wsl-bridge.py" --print-host)"
echo
echo "Use in each new terminal:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ${ROS_BRIDGE_DIR}/install/local_setup.bash"
echo "  source ${WS_DIR}/install/local_setup.bash"
echo "  ros2 launch expleo_nav_stack carla_bridge_bringup.launch.py"
