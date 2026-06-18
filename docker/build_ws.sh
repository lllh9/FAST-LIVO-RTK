#!/usr/bin/env bash
# Build the colcon workspace inside the deps container.
# (Runs as the container's default user; the workspace is bind-mounted.)
set -eo pipefail

source /opt/ros/humble/setup.bash
cd /root/ros2_ws

# fast_livo lives in src/; the vendored ROS 2 deps live in thirdparty/.
colcon build --base-paths src thirdparty --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"

echo "=== build complete ==="
ls -l /root/ros2_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping
