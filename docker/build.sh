#!/usr/bin/env bash
# Build the FAST-LIVO2-RTK ROS 2 (Humble) images.
#
#   ./docker/build.sh          # build deps image, then build the colcon workspace
#   ./docker/build.sh deps     # build only the deps image
#   ./docker/build.sh image    # also bake a self-contained full image
#
# Docker needs sudo on this host (user not in the docker group).
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

DOCKER="docker"
docker info >/dev/null 2>&1 || DOCKER="sudo docker"

DEPS_TAG=fastlivo-rtk-ros2:deps
FULL_TAG=fastlivo-rtk-ros2:humble

echo "[build] building deps image ($DEPS_TAG)"
$DOCKER build -f docker/Dockerfile.deps -t "$DEPS_TAG" .

case "${1:-ws}" in
  deps) ;;
  image)
    echo "[build] building self-contained full image ($FULL_TAG)"
    $DOCKER build -f docker/Dockerfile -t "$FULL_TAG" .
    ;;
  ws|*)
    echo "[build] building colcon workspace in a deps container (bind-mounted)"
    $DOCKER run --rm -v "$REPO_DIR":/root/ros2_ws "$DEPS_TAG" \
      bash /root/ros2_ws/docker/build_ws.sh
    ;;
esac
echo "[build] done."
