# FAST-LIVO2-RTK for ROS 2 Humble

FAST-LIVO2-RTK is a ROS 2 Humble mapping and localization system that combines
high-rate LiDAR-inertial-visual odometry with quality-gated RTK measurements and
an incremental GTSAM pose-graph backend.

This repository is a ROS 2 port and extension of
[FAST-LIVO2-RTK](https://github.com/xuankuzcr/FAST-LIVO2-RTK). It preserves a
pure local FAST-LIVO2 estimator while adding two independent global estimators:

1. a real-time error-state Kalman filter (ESIKF) that fuses local LIVO motion
   increments with RTK position, velocity, and optional dual-antenna heading;
2. an asynchronous GTSAM iSAM2 backend that jointly optimizes LIVO relative
   poses, lever-arm-aware RTK factors, and Scan Context loop closures verified
   by GICP.

The separation between local and global estimation is intentional. RTK updates
never modify the local voxel map or the pure FAST-LIVO2 state, which makes it
possible to compare local SLAM against RTK-fused and graph-optimized results
from the same run.

## Contents

- [Features](#features)
- [System architecture](#system-architecture)
- [Repository structure](#repository-structure)
- [Default sensor setup](#default-sensor-setup)
- [Requirements](#requirements)
- [Clone the repository](#clone-the-repository)
- [Build with Docker](#build-with-docker)
- [Native build on Ubuntu 22.04](#native-build-on-ubuntu-2204)
- [Configuration](#configuration)
- [Run with live sensors](#run-with-live-sensors)
- [Record a ROS 2 bag](#record-a-ros-2-bag)
- [Run mapping from a ROS 2 bag](#run-mapping-from-a-ros-2-bag)
- [Verify RTK fusion](#verify-rtk-fusion)
- [Topics, TF, and services](#topics-tf-and-services)
- [Saved outputs](#saved-outputs)
- [Compare local and RTK-fused trajectories](#compare-local-and-rtk-fused-trajectories)
- [Implementation guide](#implementation-guide)
- [Troubleshooting](#troubleshooting)

## Features

- ROS 2 Humble and `ament_cmake` port of the FAST-LIVO2 front end.
- LiDAR, IMU, and monocular camera fusion in the local FAST-LIVO2 ESIKF.
- Continuous pure local odometry that is not contaminated by global RTK
  corrections.
- CHCNAV PVT adapter with fixed/float solution filtering.
- WGS84 latitude/longitude/height conversion to a local ENU map frame.
- Configurable IMU-to-GNSS-antenna lever arm.
- Real-time nine-state global ESIKF for rotation, position, and velocity.
- RTK updates for 3D position, 3D velocity, and valid dual-antenna heading.
- Normalized innovation squared (NIS) gates for inconsistent RTK updates.
- Asynchronous GTSAM iSAM2 pose graph with LIVO, RTK, and loop factors.
- Scan Context loop candidate generation and GICP geometric verification.
- Smoothed `map -> camera_init` correction on SE(3).
- Final map reconstruction from optimized keyframe poses.
- ROS 2 service-based map saving; no stdin or Enter-key dependency.
- ROS 1 bag conversion compatibility for upstream Livox/GNSS message layouts.

## System architecture

```text
Livox point cloud ----+
Livox IMU ------------+--> FAST-LIVO2 local ESIKF
Camera image ---------+          |
                                 | pure local odometry
                                 | /odometry/fast_livo2
                                 v
                         +--------------------+
                         |                    |
                         v                    v
             real-time global ESIKF     keyframe selector
                         ^                    |
                         |                    v
CHCNAV PVT -> adapter -> quality gate   GTSAM iSAM2 worker
                         |              + LIVO relative poses
                         |              + RTK factors
                         |              + Scan Context/GICP loops
                         v                    |
          /odometry/fast_livo2_global         v
                                   /odometry/fast_livo2_graph
                                   /path_global
                                   optimized final map
```

### Local estimator

FAST-LIVO2 fuses LiDAR, IMU, and camera measurements and publishes continuous
local odometry on `/odometry/fast_livo2`. The default live configuration keeps
`gps.frontend_fusion_en` set to `false`; therefore, this trajectory remains a
pure LIVO baseline.

### Real-time global estimator

The global ESIKF propagates at the local odometry rate using pure LIVO relative
motion. At a time-matched RTK epoch, it may update antenna position, antenna
velocity, and dual-antenna heading. The lever arm is included when converting
between the IMU origin and GNSS antenna. Its output is published on
`/odometry/fast_livo2_global` in the ENU `map` frame.

### Global graph optimizer

The backend selects synchronized odometry/cloud keyframes without blocking the
front end. A worker thread incrementally adds factors to GTSAM iSAM2. RTK
position factors use receiver covariance and a robust Huber loss. Loop
candidates are generated with Scan Context and accepted only after GICP
verification. The latest graph correction is smoothed and projected onto the
high-rate local state as `/odometry/fast_livo2_graph`.

### Final map reconstruction

When the save service is called, the backend recalculates the current iSAM2
estimate, transforms every stored keyframe cloud by its optimized pose, merges
the clouds, and applies a final voxel filter.

## Repository structure

Every buildable ROS 2 package is below a single `src/` source root.

```text
FAST-LIVO2-RTK-ROS2/
├── src/
│   ├── fast_livo/                  # local estimator and global backend
│   │   ├── config/                 # estimator parameter files
│   │   ├── include/                # front-end/backend headers
│   │   ├── launch/                 # generic mapping launch file
│   │   ├── rviz_cfg/               # RViz configurations
│   │   ├── scripts/                # utility scripts
│   │   └── src/                    # C++ implementation
│   ├── chcnav/                     # CHCNAV serial/CGI protocol driver
│   ├── chcnav_gnss_adapter/        # CHCNAV PVT -> gnss_comm adapter
│   ├── msg_interfaces/             # CHCNAV ROS 2 custom messages
│   ├── livox_ros_driver2/          # Livox ROS 2 device driver
│   ├── livox_pointcloud_converter/ # PointCloud2/custom conversion utility
│   ├── mvs_ros2_driver/            # industrial camera driver
│   └── vendor/
│       ├── gnss_comm/              # GNSS PVT/time interfaces
│       ├── livox_ros_driver/       # ROS 1 message-layout compatibility
│       ├── vikit_common/           # camera models and vision utilities
│       └── vikit_ros/              # ROS 2 vikit integration
├── docker/
│   ├── Dockerfile.deps             # reproducible third-party dependencies
│   ├── Dockerfile                  # complete runtime image
│   ├── build.sh
│   ├── build_ws.sh
│   ├── run.sh
│   ├── run_demo.sh
│   └── scripts/
│       ├── build_gtsam.sh
│       └── build_sophus.sh
├── .dockerignore
├── .gitignore
├── LICENSE
└── README.md
```

`build/`, `install/`, `log/`, local bags, point clouds, and generated
trajectories are ignored by Git. A local `.artifacts/` directory may exist in a
development checkout to hold migration backups or stale build caches; it is
not part of the repository and is not required at runtime.

## Default sensor setup

The live configuration
`src/chcnav_gnss_adapter/config/mid360_a0gc_rtk.yaml` targets:

| Sensor | Default topic | ROS 2 type |
|---|---|---|
| Livox MID360 point cloud | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` |
| Livox MID360 IMU | `/livox/imu` | `sensor_msgs/msg/Imu` |
| Left industrial camera | `/left_camera/image` | `sensor_msgs/msg/Image` |
| Raw CHCNAV PVT | `/chcnav/devpvt` | `msg_interfaces/msg/Hcinspvatzcb` |
| Adapted RTK PVT | `/ublox_driver/receiver_pvt` | `gnss_comm/msg/GnssPVTSolnMsg` |

The checked-in calibration and lever-arm values are installation-specific.
They must be verified before using the map for measurement, navigation, or
georeferencing.

## Requirements

### Supported platform

- Ubuntu 22.04
- ROS 2 Humble
- C++17 compiler
- CMake 3.8 or newer
- `colcon`

### Core dependencies

- Eigen3, PCL, OpenCV, Boost, and GeographicLib
- Sophus commit `a621ff` (non-templated/double-only API)
- GTSAM 4.2
- ROS 2 dependencies declared in `src/fast_livo/package.xml`

The recommended GTSAM build uses system Eigen, disables `march-native`, and
disables GTSAM TBB integration. This avoids Eigen alignment and TBB ABI
conflicts with PCL, OpenCV, and other ROS 2 libraries.

Do not assume that a successful build with an arbitrary system GTSAM version is
equivalent to the supported configuration. Inspect the selected library with:

```bash
grep GTSAM_DIR build/fast_livo/CMakeCache.txt
ldd install/fast_livo/lib/fast_livo/fastlivo_mapping | grep -i gtsam
```

## Clone the repository

The repository can act as a standalone colcon workspace:

```bash
git clone --branch ros2-humble \
  https://github.com/sb-im/FAST-LIVO2-RTK-ROS2.git
cd FAST-LIVO2-RTK-ROS2
```

Alternatively, clone it into the `src/` directory of a parent workspace:

```bash
mkdir -p ~/fast_livo2_ws/src
cd ~/fast_livo2_ws/src
git clone --branch ros2-humble \
  https://github.com/sb-im/FAST-LIVO2-RTK-ROS2.git
cd ~/fast_livo2_ws
```

Commands below use the repository as the workspace root unless a parent
workspace is explicitly mentioned.

## Build with Docker

Docker is recommended because it builds the exact Sophus and GTSAM variants
expected by this project.

### Build dependencies and workspace

From the repository root:

```bash
./docker/build.sh
```

The script builds `fastlivo-rtk-ros2:deps`, bind-mounts the repository at
`/root/ros2_ws`, and runs the colcon build.

```bash
./docker/build.sh deps    # dependency image only
./docker/build.sh ws      # dependencies and mounted workspace
./docker/build.sh image   # self-contained runtime image
```

If Docker requires root privileges, the helper falls back to `sudo docker`.

### Run a bag in Docker

```bash
./docker/run.sh /absolute/path/to/ros2_bag_directory
RATE=0.5 ./docker/run.sh /absolute/path/to/ros2_bag_directory
```

The headless runner starts the estimator before playback, waits for input
subscriptions, plays the bag, drains pending work, calls the map-save service,
and verifies the final outputs.

## Native build on Ubuntu 22.04

### 1. Install ROS 2 Humble

Install ROS 2 Humble Desktop using the official ROS 2 procedure, then verify:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

### 2. Install system packages

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev libpcl-dev libboost-all-dev \
  libgoogle-glog-dev libgflags-dev \
  libgeographic-dev geographiclib-tools \
  libtbb-dev libomp-dev \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs \
  ros-humble-cv-bridge ros-humble-image-transport
```

Initialize and use `rosdep` if needed:

```bash
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

Sophus and GTSAM do not have suitable rosdep keys for the required versions and
must be installed separately.

### 3. Install Sophus

The provided script installs Sophus `a621ff` under `/usr/local` and applies the
Ubuntu 22.04 compiler compatibility patches:

```bash
sudo bash docker/scripts/build_sophus.sh
```

```bash
test -f /usr/local/include/sophus/se3.h
test -f /usr/local/lib/libSophus.so
test -f /usr/local/lib/cmake/Sophus/SophusConfig.cmake
```

### 4. Install GTSAM 4.2

```bash
sudo bash docker/scripts/build_gtsam.sh
```

```bash
test -f /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake
ls -l /usr/local/lib/libgtsam.so*
```

### 5. Build the workspace

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --base-paths src \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The same command works from a parent workspace root because colcon discovers
packages recursively below `src/`.

### 6. Verify the build

```bash
colcon list --base-paths src --names-only | sort
ros2 pkg prefix fast_livo
ros2 pkg prefix chcnav_gnss_adapter
ros2 pkg prefix livox_ros_driver2
ros2 launch chcnav_gnss_adapter mid360_chcnav_rtk.launch.py --show-args
```

Expected packages include:

```text
chcnav
chcnav_gnss_adapter
fast_livo
gnss_comm
livox_pointcloud_converter
livox_ros_driver
livox_ros_driver2
msg_interfaces
mvs_ros2_driver
vikit_common
vikit_ros
```

If source directories were moved, do not reuse a CMake cache generated at the
old absolute path. Archive or remove `build/`, `install/`, and `log/`, then
perform a clean build.

## Configuration

### Live configuration

The combined launch file uses:

```text
src/chcnav_gnss_adapter/config/mid360_a0gc_rtk.yaml
```

Important sections:

- `laserMapping`: camera model and output directory;
- `common`: LiDAR, IMU, and image topics;
- `extrin_calib`: LiDAR/IMU and camera extrinsics;
- `time_offset`: IMU/image time offsets;
- `preprocess`: LiDAR type and filtering;
- `vio`, `imu`, `lio`: local estimator parameters;
- `gps`: RTK input, lever arm, and quality settings;
- `realtime_esikf`: process noise and NIS gates;
- `backend`: keyframes, RTK association, loops, and final map settings.

### Calibration values that must be checked

1. Camera intrinsics and distortion coefficients.
2. Camera-to-LiDAR rotation `Rcl` and translation `Pcl`.
3. IMU-to-LiDAR extrinsics.
4. `gps.extrinsic_T`: MID360 IMU origin to GNSS positioning antenna, expressed
   in the MID360 IMU frame, in metres.
5. `realtime_esikf.heading_offset_deg`: installed offset between the CHCNAV
   antenna heading axis and FAST-LIVO2 body x-axis.
6. `gps.gps_time_offset` and sensor timestamp synchronization.

Do not substitute the receiver's internal INS-to-antenna lever arm for the
MID360-IMU-to-GNSS-antenna lever arm required here.

### Combined launch arguments

```bash
ros2 launch chcnav_gnss_adapter mid360_chcnav_rtk.launch.py --show-args
```

| Argument | Default | Description |
|---|---|---|
| `params_file` | installed `mid360_a0gc_rtk.yaml` | estimator configuration |
| `output_dir` | `${PWD}/fast_livo_output` | trajectory/map output directory |
| `chcnav_topic` | `/chcnav/devpvt` | raw CHCNAV PVT input |
| `accept_float` | `false` | accept RTK float solutions in the adapter |
| `rviz` | `true` | start RViz |

The default command is sufficient when these values match the installation:

```bash
ros2 launch chcnav_gnss_adapter mid360_chcnav_rtk.launch.py
```

The default output path depends on the shell's working directory. Use a known
directory or pass an absolute `output_dir` for reproducible runs.

## Run with live sensors

Source ROS 2 and the workspace in every terminal:

```bash
cd ~/fast_livo2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Use the repository path instead when it is the standalone workspace root.

### Terminal 1: CHCNAV receiver

```bash
ros2 launch chcnav demo_1.py
```

The checked-in launch uses `/dev/ttyUSB0` at 460800 baud. Update
`src/chcnav/launch/demo_1.py` if required and ensure serial-device permissions.

### Terminal 2: Livox MID360

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Terminal 3: camera

Start the LiDAR before the triggered camera so the camera can obtain its shared
LiDAR timestamp source:

```bash
ros2 launch mvs_ros2_driver single_camera.py
```

The serial number and topic are configured in
`src/mvs_ros2_driver/config/left_camera_trigger.yaml`.

### Terminal 4: adapter, estimator, backend, and RViz

```bash
ros2 launch chcnav_gnss_adapter mid360_chcnav_rtk.launch.py
```

For a reproducible output directory:

```bash
ros2 launch chcnav_gnss_adapter mid360_chcnav_rtk.launch.py \
  output_dir:=$HOME/fast_livo_output/run_001 \
  accept_float:=false \
  rviz:=true
```

### Confirm live inputs

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /left_camera/image
ros2 topic hz /chcnav/devpvt
ros2 topic hz /ublox_driver/receiver_pvt
```

All terminals must use the same domain:

```bash
echo "${ROS_DOMAIN_ID:-0}"
```

## Record a ROS 2 bag

Record the three local-estimator inputs and raw CHCNAV PVT:

```bash
mkdir -p bags
ros2 bag record \
  -o bags/fast_livo2_rtk_run_001 \
  --max-cache-size 1073741824 \
  /livox/lidar \
  /livox/imu \
  /left_camera/image \
  /chcnav/devpvt
```

This is the recommended raw-data bag. During replay, the adapter recreates
`/ublox_driver/receiver_pvt` and applies the fixed/float policy.

Before recording:

```bash
ros2 topic list -t | grep -E 'livox|left_camera|chcnav'
```

Stop with one `Ctrl+C`, then inspect:

```bash
ros2 bag info bags/fast_livo2_rtk_run_001
```

Each required topic must have a non-zero count. A useful sequence begins
stationary, contains translation and rotation, includes a loop when possible,
maintains RTK fixed status, and ends stationary.

Do not replay both `/chcnav/devpvt` and a separately recorded
`/ublox_driver/receiver_pvt` into the combined launch unless one is filtered
with `--topics`; otherwise duplicate GNSS epochs may reach the backend.

## Run mapping from a ROS 2 bag

Do not start hardware drivers during offline playback.

### Terminal 1: start mapping first

```bash
cd ~/fast_livo2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch chcnav_gnss_adapter mid360_chcnav_rtk.launch.py \
  output_dir:=$PWD/fast_livo_output/bag_run_001 \
  accept_float:=false \
  rviz:=true
```

Wait for subscriptions:

```bash
ros2 node info /laserMapping
```

### Terminal 2: play the bag

```bash
cd ~/fast_livo2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag play bags/fast_livo2_rtk_run_001 \
  --rate 0.5 \
  --delay 3
```

Start at `0.5` rate for a new machine or a large image stream. Increase to
`1.0` after confirming that callbacks and the backend keep up.

If a bag uses different names, remap during playback:

```bash
ros2 bag play /path/to/bag \
  --remap \
    /camera/image_raw:=/left_camera/image \
    /recorded/lidar:=/livox/lidar \
    /recorded/imu:=/livox/imu \
    /recorded/chcnav_pvt:=/chcnav/devpvt \
  --rate 0.5 --delay 3
```

The image type must be `sensor_msgs/msg/Image`. Remapping a
`sensor_msgs/msg/CompressedImage` does not change its type.

After playback, wait until keyframe logs stop changing, then save:

```bash
ros2 service call /global_backend/save_map std_srvs/srv/Trigger "{}"
```

A normal single `Ctrl+C` also reconstructs the map during shutdown, but the
service is recommended because completion can be verified before exit. Do not
force-kill the process while saving.

## Verify RTK fusion

RTK availability, acceptance, and graph fusion are different conditions.

### 1. Raw and adapted RTK streams

```bash
ros2 topic hz /chcnav/devpvt
ros2 topic hz /ublox_driver/receiver_pvt
ros2 topic echo /ublox_driver/receiver_pvt --once
```

A fixed adapted solution should contain:

```yaml
valid_fix: true
diff_soln: true
carr_soln: 2
```

The adapter accepts CHCNAV status 4 and 8 as fixed. Status 5 and 9 are float
and are rejected with `accept_float=false`. Status 4 also permits
dual-antenna heading when its value and standard deviation are valid.

### 2. Real-time global output

```bash
ros2 topic hz /odometry/fast_livo2
ros2 topic hz /odometry/fast_livo2_global
```

Local odometry without global output normally means that no valid, time-matched
RTK observation initialized the global ESIKF.

### 3. GTSAM factor insertion

This startup message confirms only that the feature is configured:

```text
Global fusion: real-time ESIKF + online iSAM2/RTK/Scan Context/GICP
```

The definitive per-keyframe evidence is:

```text
Backend keyframe=... factors=... rtk=yes loop=...
```

`rtk=yes` means that the keyframe matched an approved RTK observation within
the synchronization tolerance and RTK factors were inserted. Occasional
`rtk=no` is possible when keyframe/RTK rates differ; continuous `rtk=no`
indicates a data, time, or quality problem.

```bash
ros2 launch chcnav_gnss_adapter mid360_chcnav_rtk.launch.py \
  2>&1 | tee /tmp/fast_livo_rtk.log
grep 'Backend keyframe' /tmp/fast_livo_rtk.log
```

### 4. Georeference output

After saving, `TUM/enu_origin_lla.txt` proves that a valid RTK measurement
established the ENU origin. It does not prove that every keyframe received an
RTK factor; use the `rtk=yes` logs for that.

## Topics, TF, and services

### Inputs

| Topic | Type | Purpose |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | LiDAR scan |
| `/livox/imu` | `sensor_msgs/msg/Imu` | IMU |
| `/left_camera/image` | `sensor_msgs/msg/Image` | monocular image |
| `/chcnav/devpvt` | `msg_interfaces/msg/Hcinspvatzcb` | raw CHCNAV PVT |
| `/ublox_driver/receiver_pvt` | `gnss_comm/msg/GnssPVTSolnMsg` | normalized RTK PVT |

### Principal outputs

| Topic | Type | Description |
|---|---|---|
| `/odometry/fast_livo2` | `nav_msgs/msg/Odometry` | pure local LIVO odometry |
| `/path` | `nav_msgs/msg/Path` | pure local path |
| `/odometry/fast_livo2_global` | `nav_msgs/msg/Odometry` | real-time RTK/LIVO ESIKF |
| `/path_rtk` | `nav_msgs/msg/Path` | quality-gated raw RTK positions in ENU |
| `/path_global_esikf` | `nav_msgs/msg/Path` | real-time RTK/LIVO ESIKF path |
| `/odometry/fast_livo2_graph` | `nav_msgs/msg/Odometry` | current graph-corrected pose |
| `/path_global` | `nav_msgs/msg/Path` | optimized keyframe path |
| `/global_backend/keyposes` | `sensor_msgs/msg/PointCloud2` | optimized keyframe positions |
| `/global_backend/loop_edges` | `visualization_msgs/msg/MarkerArray` | accepted loops |
| `/global_backend/final_map` | `sensor_msgs/msg/PointCloud2` | rebuilt final map |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | registered mapping cloud |
| `/rgb_img` | `sensor_msgs/msg/Image` | processed/annotated image |
| `/synced_cloud` | `sensor_msgs/msg/PointCloud2` | backend keyframe cloud |

`/gps/odometry` is a diagnostic pure-RTK output from the legacy frontend
callback. With default `gps.frontend_fusion_en=false` and
`gps.debug_mode=false`, that callback is not subscribed. Use a separate
parameter file with `gps.debug_mode=true` to enable the diagnostic output while
keeping frontend RTK fusion disabled.

### TF

```text
map -> camera_init
```

This transform aligns the local LIVO frame with the global ENU/map frame.

### Service

| Service | Type | Description |
|---|---|---|
| `/global_backend/save_map` | `std_srvs/srv/Trigger` | rebuild/save optimized trajectory and map |

## Saved outputs

Global outputs are written below `output_dir`:

```text
<output_dir>/
├── TUM/
│   ├── global_optimized.txt
│   └── enu_origin_lla.txt
├── global_pcd/
│   └── final_optimized_map.pcd
└── debug/
```

`global_optimized.txt` is a TUM trajectory:

```text
# timestamp tx ty tz qx qy qz qw
```

`enu_origin_lla.txt` records:

```text
# latitude_deg longitude_deg altitude_ellipsoid_m
```

`final_optimized_map.pcd` is reconstructed from optimized keyframes and then
voxel filtered.

When `evo.pose_output_en=true`, pure local LIVO is written to:

```text
src/fast_livo/Log/result/<evo.seq_name>.txt
```

For the live configuration:

```text
src/fast_livo/Log/result/MID360_A0GC_RTK.txt
```

It is TUM format and is overwritten when a new run begins. Archive it before
the next experiment.

## Compare local and RTK-fused trajectories

| Estimate | Topic/file | RTK used? |
|---|---|---|
| local LIVO | `/odometry/fast_livo2`, `/path`, local TUM | no |
| real-time global ESIKF | `/odometry/fast_livo2_global` | yes |
| GTSAM global graph | `/odometry/fast_livo2_graph`, `/path_global`, global TUM | yes, plus loops |

Record comparison topics for offline analysis:

```bash
ros2 bag record \
  -o comparison_result \
  /odometry/fast_livo2 \
  /odometry/fast_livo2_global \
  /odometry/fast_livo2_graph \
  /path /path_rtk /path_global_esikf /path_global \
  /ublox_driver/receiver_pvt
```

The combined RTK launch loads `mid360_chcnav_rtk.rviz`. Its **Trajectory
Comparison (enable manually)** group contains differently colored displays for
`/path`, `/path_rtk`, `/path_global_esikf`, and `/path_global`. All comparison
trajectories are disabled initially; select the corresponding RViz checkboxes
when they are needed.

For `evo`, the local and global trajectories have different origins and
orientations. Apply SE(3) alignment for shape/drift comparison, but do not
apply scale correction because both trajectories are metric.

```bash
python3 -m pip install --user evo
evo_traj tum \
  src/fast_livo/Log/result/MID360_A0GC_RTK.txt \
  fast_livo_output/TUM/global_optimized.txt \
  --ref=fast_livo_output/TUM/global_optimized.txt \
  --align --plot
```

For quantitative accuracy, use independent ground truth. The RTK stream used
as a fusion input is not statistically independent ground truth.

## Implementation guide

| File | Responsibility |
|---|---|
| `src/fast_livo/src/LIVMapper.cpp` | frontend, callbacks, local odometry/path/TUM |
| `src/fast_livo/src/global_esikf.cpp` | real-time global propagation and RTK updates |
| `src/fast_livo/include/global_esikf.h` | RTK observation/filter state interfaces |
| `src/fast_livo/src/optimization.cpp` | RTK validation, ENU, keyframes, GTSAM, loops, map save |
| `src/fast_livo/include/optimization.h` | asynchronous backend structures/parameters |
| `src/chcnav_gnss_adapter/src/chcnav_to_gnss.cpp` | CHCNAV status/quality conversion |
| `src/chcnav_gnss_adapter/launch/mid360_chcnav_rtk.launch.py` | combined launch |
| `src/chcnav_gnss_adapter/config/mid360_a0gc_rtk.yaml` | live configuration |

The dense numerical frontend remains close to upstream. `ros1_compat.hpp`
provides logging/time shims while publishers, subscriptions, parameters, TF,
image transport, and message filters use native ROS 2 APIs. Custom interfaces
preserve original field layouts so converted bags can play without
`ros1_bridge`.

## Troubleshooting

### CMake points to a deleted `thirdparty` or old source path

An old CMake cache contains absolute paths. Archive generated state and rebuild:

```bash
mv build build.stale
mv install install.stale
mv log log.stale
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### GTSAM is absent from the repository but the project builds

`find_package(GTSAM REQUIRED)` found a system installation. Inspect it:

```bash
grep GTSAM_DIR build/fast_livo/CMakeCache.txt
grep -E 'GTSAM_VERSION_(MAJOR|MINOR|PATCH|STRING)' \
  /usr/local/include/gtsam/config.h
ldd install/fast_livo/lib/fast_livo/fastlivo_mapping | grep -i gtsam
```

The supported build is GTSAM 4.2 from `docker/scripts/build_gtsam.sh`.

### Camera image is absent during bag playback

```bash
ros2 bag info /path/to/bag
ros2 topic list -t | grep -Ei 'image|camera'
```

Remap a raw `sensor_msgs/msg/Image` to `/left_camera/image`. Decompress a
compressed stream first. Confirm all terminals share the same `ROS_DOMAIN_ID`.

If `/left_camera/image` exists but `/rgb_img` does not, check LiDAR/image header
timestamps, QoS, `img_time_offset`, and playback rate.

### RViz does not show mapping topics

Check the active profile. A calibration profile may contain only `/edge_cloud`,
`/plane_cloud`, and similar topics. Add `/cloud_registered`, `/path`,
`/path_global`, `/rgb_img`, and the desired odometry topics, or load a dedicated
mapping profile.

### Backend always reports `rtk=no`

Check, in order:

1. `/chcnav/devpvt` exists;
2. `/ublox_driver/receiver_pvt` exists;
3. `valid_fix=true`, `diff_soln=true`, `carr_soln=2`;
4. horizontal/vertical accuracy passes configured limits;
5. GNSS week/time-of-week is valid;
6. RTK/LIVO time difference is within `backend.rtk_sync_tolerance` (0.10 s by default);
7. `gps.gps_time_offset` is correct;
8. lever arm and heading offset match the installation.

### Map service reports no optimized keyframes

Confirm `/odometry/fast_livo2` and `/synced_cloud` are published and backend
keyframe logs appear. Start the estimator before playback and use `--delay 3`.

### Shutdown does not produce a complete map

Call the save service, wait for `Final map rebuilt:`, and then stop the launch.
Do not press `Ctrl+C` repeatedly or force-kill during reconstruction.

## Known notes

- The compatibility `ros::Time::now()` uses wall time. Some visualization
  headers are therefore cosmetic under simulated time; saved trajectories and
  maps use sensor timestamps.
- Local LIVO and global ENU trajectories use different frames by design.
- A published RTK message is not automatically an accepted factor. Accuracy,
  solution status, timestamp, innovation, and synchronization gates apply.
- Checked-in extrinsics are examples for one installation, not universal
  calibration.

## License and acknowledgements

See [LICENSE](LICENSE) and each package's license declaration. This project
builds on FAST-LIVO2/FAST-LIVO2-RTK, GTSAM, Sophus, Livox ROS drivers, vikit,
CHCNAV interfaces, PCL, OpenCV, GeographicLib, and ROS 2. Vendored packages
retain their upstream licenses; see `src/vendor/README.md`.
