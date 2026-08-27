# livox_pointcloud_converter

ROS 2 node that converts the `sensor_msgs/msg/PointCloud2` published by
`livox_ros_driver2` from PointXYZRTL to PointXYZIT without modifying the driver.

Input fields:

```text
x(float32), y(float32), z(float32), intensity(float32),
tag(uint8), line(uint8), timestamp(float64, absolute nanoseconds)
```

Output fields:

```text
x(float32), y(float32), z(float32), intensity(float32),
time(float64, seconds relative to header.stamp)
```

Build and run:

```bash
cd ~/llh/fast_livo2_humble_ws
colcon build --packages-select livox_pointcloud_converter --symlink-install
source install/setup.bash
ros2 launch livox_pointcloud_converter livox_pointcloud_converter.launch.py
```

The default input topic is `/livox/lidar`, and the default output topic is
`/livox/points`. Both can be overridden:

```bash
ros2 launch livox_pointcloud_converter livox_pointcloud_converter.launch.py \
  input_topic:=/livox/lidar output_topic:=/livox/points
```

Check the output layout:

```bash
ros2 topic echo /livox/points --once --field fields
```

Record the converted cloud together with the calibration image:

```bash
ros2 bag record /livox/points /left_camera/image /livox/imu
```

Then pass the converted topic to `direct_visual_lidar_calibration`:

```bash
ros2 run direct_visual_lidar_calibration preprocess BAG_PATH RESULT_PATH \
  -adv --image_topic /left_camera/image --points_topic /livox/points \
  --intensity_channel intensity
```

For an existing bag that only contains `/livox/lidar`, start this node, replay
the old bag, and record `/livox/points` plus the image topic into a new bag.
## PointCloud2 to Livox CustomMsg (FAST-LIVO2)

Convert the Livox Driver2 `PointXYZRTL` PointCloud2 format to
`livox_ros_driver2/msg/CustomMsg`:

```bash
ros2 launch livox_pointcloud_converter pointcloud2_to_custom.launch.py
```

Defaults:

- input: `/livox/lidar` (`sensor_msgs/msg/PointCloud2`)
- output: `/livox/lidar_custom` (`livox_ros_driver2/msg/CustomMsg`)

The launch file accepts `input_topic`, `output_topic`, and `lidar_id`. For
example, keep the converted topic name `/livox/lidar` while replaying the raw
cloud under a temporary name:

```bash
ros2 launch livox_pointcloud_converter pointcloud2_to_custom.launch.py \
  input_topic:=/livox/lidar_raw output_topic:=/livox/lidar lidar_id:=0

ros2 bag play BAG_PATH --remap /livox/lidar:=/livox/lidar_raw
```

Do not configure the same topic as both input and output.

Set FAST-LIVO2's `common.lid_topic` to `/livox/lidar_custom` and keep
`preprocess.lidar_type: 1`.
