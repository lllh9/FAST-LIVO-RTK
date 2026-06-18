# Vendored ROS 2 dependencies

These packages are bundled because they have no published ROS 2 (Humble)
release. The Docker build compiles them together with `fast_livo`
(`colcon build --base-paths src thirdparty`). Each retains its upstream license;
see the `license` tag in each `package.xml`.

| Package | What it is | Upstream | License |
|---|---|---|---|
| `vikit_common` | Camera models + vision math used by the VIO front-end. ROS 2 (ament) port; patched to the non-templated Sophus a621ff API and with the unused SVO `img_align`/`homography` files dropped. | uzh-rpg `rpg_vikit` / the `xuankuzcr/rpg_vikit` fork FAST-LIVO2 builds against | GPLv3 |
| `vikit_ros` | ROS 2 glue for vikit (camera loader reading node params, marker/TF output helpers). | same as `vikit_common` | GPLv3 |
| `gnss_comm` | Interface-only package providing `GnssPVTSolnMsg` / `GnssTimeMsg`. Field layout matches the ROS 1 messages so converted rosbags play directly. | HKUST-Aerial-Robotics `gnss_comm` (messages only) | GPLv3 |
| `livox_ros_driver` | Interface-only package providing `CustomMsg` / `CustomPoint`. Field layout matches the ROS 1 Livox messages. | Livox-SDK `livox_ros_driver` (messages only) | BSD-3-Clause |

Notes
- `gnss_comm` and `livox_ros_driver` here are **interface-only** — they ship just
  the `.msg` definitions FAST-LIVO2-RTK consumes, not the upstream drivers or C++
  utilities.
- Sophus (commit `a621ff`, non-templated/double-only) and GTSAM 4.2 are **not**
  vendored here; they are built from source in the deps Docker image
  (`docker/scripts/build_sophus.sh`, `build_gtsam.sh`).
- If/when upstream ROS 2 releases of vikit / gnss_comm / livox become standard,
  these can be dropped in favour of `rosdep`-resolved dependencies.
