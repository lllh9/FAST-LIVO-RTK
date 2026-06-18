#!/usr/bin/env bash
# Run FAST-LIVO2-RTK (ROS 2) headless on a converted ROS 2 bag.
# Results land in ./src/fast_livo/output on the host.
#
#   ./docker/run.sh                       # uses ./bags/HH-LVGO-01-ros2
#   ./docker/run.sh /path/to/ros2_bag_dir
#
# Docker needs sudo on this host (user not in the docker group).
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

DOCKER="docker"
docker info >/dev/null 2>&1 || DOCKER="sudo docker"

DEPS_TAG=fastlivo-rtk-ros2:deps
# Default bag: a converted ROS 2 bag under this repo's (git-ignored) bags/.
BAG_HOST="${1:-$REPO_DIR/bags/HH-LVGO-01-ros2}"
RATE="${RATE:-1.0}"

if [[ ! -e "$BAG_HOST" ]]; then
  echo "[run] ERROR: ROS 2 bag not found: $BAG_HOST" >&2
  echo "      1) Download the example bag HH-LVGO-01.bag (see the upstream README's" >&2
  echo "         Google Drive link), e.g.:  gdown 1RIRcqjaw3x8l-S-Dc655xHi_bKkI7q66 -O HH-LVGO-01.bag" >&2
  echo "      2) Convert it to a ROS 2 bag (Humble metadata):" >&2
  echo "         rosbags-convert --src HH-LVGO-01.bag --dst bags/HH-LVGO-01-ros2 --dst-storage sqlite3 --dst-version 6" >&2
  echo "      3) Re-run:  ./docker/run.sh   (or ./docker/run.sh <ros2_bag_dir>)" >&2
  exit 1
fi
BAG_HOST="$(cd "$BAG_HOST" && pwd)"

echo "[run] bag=$BAG_HOST rate=$RATE"
$DOCKER run --rm -it \
  -v "$REPO_DIR":/root/ros2_ws \
  -v "$BAG_HOST":/bags/ros2_bag:ro \
  -e RATE="$RATE" \
  "$DEPS_TAG" \
  bash /root/ros2_ws/docker/run_demo.sh /bags/ros2_bag
