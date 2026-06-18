# Self-contained FAST-LIVO2-RTK ROS 2 (Humble) image.
# FROM the deps image (Sophus + GTSAM + GeographicLib + ROS 2 Humble), copies
# the colcon workspace and builds it. Build context = repo root.
#
#   docker build -f docker/Dockerfile.deps -t fastlivo-rtk-ros2:deps .
#   docker build -f docker/Dockerfile      -t fastlivo-rtk-ros2:humble .
# ---------------------------------------------------------------------------
FROM fastlivo-rtk-ros2:deps AS full

SHELL ["/bin/bash", "-c"]

# Workspace source (reference/, bags/, build/, install/ excluded via .dockerignore).
COPY . /root/ros2_ws
WORKDIR /root/ros2_ws

RUN bash docker/build_ws.sh

RUN echo "source /opt/ros/humble/setup.bash"        >> /root/.bashrc \
 && echo "source /root/ros2_ws/install/setup.bash"  >> /root/.bashrc
CMD ["bash"]
