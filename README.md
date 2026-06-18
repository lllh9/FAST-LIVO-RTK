# FAST-LIVO2-RTK — ROS 2 Humble port

A ROS 2 (Humble) port of [FAST-LIVO2-RTK](https://github.com/xuankuzcr/FAST-LIVO2-RTK)
— direct LiDAR-Inertial-Visual odometry (FAST-LIVO2) extended with RTK/GNSS
constrained global optimization. The original is a ROS 1 (catkin) package; this
tree builds and runs the same algorithm on ROS 2 Humble, headless, in Docker,
on the project's RTK example rosbag.

> This port runs the same algorithm as the ROS 1 original and was validated by
> comparing its trajectory / map outputs against the ROS 1 reference run on the
> same example bag (see [Validation](#validation)).

## Layout

`fast_livo` (the actual port) is the only first-party package and lives under
`src/`. The four ROS 2 dependencies have no published release, so they are
vendored under `thirdparty/`; the Docker build compiles `src/` + `thirdparty/`
together into one colcon workspace.

```
FAST-LIVO2-RTK-ROS2/            # (intended as the `ros2-humble` branch of FAST-LIVO2-RTK)
├─ src/
│  └─ fast_livo/                # the ported package (ament_cmake, rclcpp)
│     ├─ include/ros1_compat.hpp   # ROS_* logging + ros::Time/Duration/Rate shims
│     ├─ include/tf_compat.hpp     # tf:: quaternion helpers backed by tf2
│     └─ src/  include/  config/  launch/  rviz_cfg/  scripts/
├─ thirdparty/                  # vendored ROS 2 deps (no upstream release)
│  ├─ livox_ros_driver/         # interface pkg: CustomMsg / CustomPoint
│  ├─ gnss_comm/                # interface pkg: GnssPVTSolnMsg / GnssTimeMsg
│  ├─ vikit_common/             # camera models / vision utils (ROS 2)
│  └─ vikit_ros/                # vikit ROS 2 glue
├─ docker/
│  ├─ Dockerfile.deps           # ROS 2 Humble + Sophus(a621ff) + GTSAM 4.2 + GeographicLib
│  ├─ Dockerfile                # self-contained image (deps + built workspace)
│  ├─ build.sh  run.sh  build_ws.sh  run_demo.sh
│  └─ scripts/build_sophus.sh  build_gtsam.sh
├─ thirdparty/README.md        # provenance + licenses of the vendored deps
├─ README.md  LICENSE  .gitignore
```

To build the workspace by hand (outside the helper scripts):
`colcon build --base-paths src thirdparty`.

## Quick start

```bash
# 1. Build the deps image and the colcon workspace (Docker needs sudo here).
./docker/build.sh

# 2. Get the example bag and convert it to a ROS 2 (rosbag2) bag.
#    Download HH-LVGO-01.bag (upstream README's Google Drive link), e.g.:
pip install gdown rosbags
gdown 1RIRcqjaw3x8l-S-Dc655xHi_bKkI7q66 -O HH-LVGO-01.bag
#    Convert into ./bags/ — version 6 = Humble-readable metadata (~5 min, ~40 GB):
rosbags-convert --src HH-LVGO-01.bag \
  --dst bags/HH-LVGO-01-ros2 --dst-storage sqlite3 --dst-version 6

# 3. Run headless on the converted bag; results land in src/fast_livo/output/.
./docker/run.sh                 # uses ./bags/HH-LVGO-01-ros2
# or:  ./docker/run.sh /path/to/ros2_bag_dir
```

## What the port changes (and what it does not)

The algorithm code (ESIKF LIVO front-end, voxel map, VIO, GTSAM/RTK back-end)
is unchanged. Only the ROS plumbing is migrated:

| Concern | ROS 1 | ROS 2 (this port) |
|---|---|---|
| Build | catkin / `package.xml` v2 | `ament_cmake` / `package.xml` v3 |
| Node | `ros::NodeHandle` (global) | one `rclcpp::Node` shared by front-/back-end |
| Pub/Sub | `ros::Publisher`/`Subscriber` | typed `rclcpp::Publisher/Subscription` |
| Spinning | `ros::spinOnce()` in `run()` | `rclcpp::spin_some(node)` in `run()` |
| Timer | `nh.createTimer` | `create_wall_timer` |
| Params | `nh.param("a/b", …)` | `get_param(node,"a/b",…)` → `a.b`, auto-declared from overrides |
| Logging | `ROS_INFO/WARN/ERROR` | shimmed to `RCLCPP_*` (see `ros1_compat.hpp`) |
| Time | `ros::Time/Duration`, `.toSec()` | shimmed (`ros1_compat.hpp`); `msg.stamp.toSec()` → `toSec(stamp)` |
| tf | `tf::` | `tf2` / `tf2_ros` (`tf_compat.hpp` for the quaternion helpers) |
| Image | `cv_bridge` / `image_transport` | ROS 2 equivalents |
| LiDAR msg | `livox_ros_driver/CustomMsg` (vendored hdr) | `livox_ros_driver` interface pkg (same fields) |
| GNSS msg | `gnss_comm/GnssPVTSolnMsg` | `gnss_comm` interface pkg (same fields) |
| Sync | `message_filters` + `boost::bind` | `message_filters` + `std::bind` |

`include/ros1_compat.hpp` is the key to a small, reviewable diff: it provides
`ROS_*` macros, a double-backed `ros::Time`/`ros::Duration`/`ros::Rate`, and a
`toSec()` helper, so the dense numerical code keeps its original form.

### Bag conversion / message compatibility

The ROS 1 bag is converted with `rosbags-convert`. The custom message
definitions are embedded in the ROS 1 bag, so conversion preserves the type
names `livox_ros_driver/msg/CustomMsg` and `gnss_comm/msg/GnssPVTSolnMsg`. The
interface packages here are named to match (identical field layout), so the
converted bag plays back straight into the node — no `ros1_bridge` needed.
`--dst-version 6` makes the rosbag2 metadata readable by Humble.

## Notes / gotchas

- **GTSAM must be 4.2** (`Pose3::Identity()`), built with system Eigen,
  `march-native OFF`, `TBB OFF` to avoid Eigen ABI/alignment segfaults. `vikit_common`
  also has `-march=native` stripped for the same reason.
- **Sophus** is the `a621ff` non-templated/double-only build; the deps image
  installs a `SophusConfig.cmake` that also exposes a `Sophus::Sophus` target
  (vikit links it).
- The offline back-end blocks on `std::cin.get()`; `docker/run_demo.sh` drives
  stdin via a FIFO, plays the bag, drains the front-end, then triggers the
  GTSAM/RTK batch and checks the outputs.
- Camera intrinsics for HH-LVGO-01 live in `config/HH-LVGO.yaml` under the
  `laserMapping` namespace with the keys the ROS 2 vikit loader expects
  (`model/width/height/scale/fx/fy/cx/cy/d0..d3`).
- **Known difference (cosmetic):** the `ros::Time::now()` shim uses the system
  wall clock, so it ignores `/use_sim_time` and `/clock`. This affects only the
  *header stamps* of published viz/odom/TF topics (path, `/aft_mapped_to_init`,
  mavros pose, `/rgb_img`) for live/RViz consumers. The saved trajectory and map
  use the sensor message timestamps, so the validated outputs are unaffected.

## Validation

The headless run on the converted HH-LVGO-01 bag matches the ROS 1 reference:

| Metric | ROS 1 | ROS 2 port |
|---|---|---|
| Optimized poses | 2737 | 2735 |
| Trajectory length | 297.31 m | 297.48 m |
| Raw RTK track poses | 2714 | 2714 (identical) |
| Global map points | 34,021,218 | 34,004,537 |

Per-pose position difference (all 2735 poses matched within 60 ms): **mean
0.36 cm, median 0.21 cm, p95 1.27 cm**. The small residual comes from
message-delivery timing (ROS 2 `spin_some`/DDS vs ROS 1's global callback queue),
not from the algorithm.

## Outputs

The output directory depends on how you launch (the node creates it if needed):

- **`docker/run.sh` / `run_demo.sh`** → `src/fast_livo/output/` (mounted to the host).
- **`ros2 launch fast_livo mapping.launch.py`** → `./fast_livo_output/` in the
  launch working directory by default (override with `output_dir:=<path>`).
- **`ros2 run` with a params file directly** → the `outputfilepath` value in the
  YAML (default the relative `output/`), unless overridden with
  `-p laserMapping.outputfilepath:=<path>`.

Contents (same set as the ROS 1 project):
`TUM/opt_trajectory_after.txt` (optimized trajectory),
`TUM/opt_trajectory_before.txt` (raw RTK track),
`global_pcd/{before,after}_optimization.pcd`, `vel/`, `run.log`.
