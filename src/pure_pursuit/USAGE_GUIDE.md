# Usage Guide

## Map-Based LiDAR + IMU Navigation

All new map-based nodes live inside `pure_pursuit`; launch files may start
existing drivers from other packages but do not require editing those packages.

### 1. Build

```bash
cd /home/f1/f1
colcon build --packages-select pure_pursuit
source install/setup.bash
```

### 2. Mapping

Drive the car manually while SLAM records the map:

```bash
ros2 launch pure_pursuit mapping.launch.py
```

Default inputs and transforms:

```text
/scan -> slam_toolbox
/imu/data_raw -> imu_bridge -> /imu
odom -> base_link from wall_follow/lidar_scan_odom
map -> odom from slam_toolbox
```

If the `wall_follow` IMU publisher is not available, use the local ICM-20948
publisher instead:

```bash
ros2 launch pure_pursuit mapping.launch.py \
  with_imu_driver:=false \
  with_imu_bridge:=false \
  with_direct_imu_driver:=true
```

Save the finished SLAM map with the `slam_toolbox` save-map service/plugin and
store it under:

```text
/home/f1/f1/src/pure_pursuit/maps/
```

### 3. Localization

Run localization against the saved map:

```bash
ros2 launch pure_pursuit localization.launch.py \
  map_file:=/home/f1/f1/src/pure_pursuit/maps/map
```

The target localization pose is the TF chain:

```text
map -> odom -> base_link
```

### 4. Teach Path

With localization running and the car driven manually, record the repeat path:

```bash
ros2 run pure_pursuit trajectory_logger --ros-args \
  -p output_path:=/home/f1/f1/src/pure_pursuit/paths/path.csv \
  -p min_distance:=0.2
```

CSV format:

```text
x,y,yaw
0.000000,0.000000,0.000000
0.200000,0.100000,0.020000
```

### 5. Repeat Path

Run localization, path follower, and the existing `/cmd_vel` steering interface:

```bash
ros2 launch pure_pursuit full_autonomous.launch.py \
  map_file:=/home/f1/f1/src/pure_pursuit/maps/map \
  path_file:=/home/f1/f1/src/pure_pursuit/paths/path.csv
```

Press the configured joystick start button before autonomous steering begins.
The local pure pursuit node publishes `geometry_msgs/Twist` on `/cmd_vel`.

To use this node on the physical car, run

```bash
ros2 launch pure_pursuit pure_pursuit.launch.py
```

This will use the parameters from the config file `config/config.yaml`

If you want to test it out in simulation, which uses different topic names, run

```bash
ros2 launch pure_pursuit sim_pure_pursuit_launch.py
```

### Trying different parameters without rebuilding

Setting the gain
```bash
ros2 param set pure_pursuit K_p 0.1
```

Setting the velocity profile

```bash
ros2 param set pure_pursuit velocity_percentage 0.7
```

### Recording waypoints with a joystick

Start the joystick node:

```bash
ros2 run joy joy_node
```

Start odometry from the VESC topics:

```bash
ros2 run wall_follow vesc_imu_odom --ros-args \
  -p imu_topic:=/sensors/imu/raw \
  -p motor_speed_topic:=/motor_speed \
  -p odom_topic:=/odom
```

In another terminal, start the waypoint recorder:

```bash
ros2 run pure_pursuit waypoint_recorder --ros-args -p odom_topic:=/odom
```

Hold joystick button `4` while driving to record waypoints. Press `Ctrl+C` to stop and close the CSV file.
By default, the recorder writes:

```text
src/pure_pursuit/racelines/recorded_waypoints.csv
```

The CSV format is compatible with the pure pursuit node:

```text
x,y,velocity
```

Useful overrides:

```bash
ros2 run pure_pursuit waypoint_recorder --ros-args \
  -p odom_topic:=/odom \
  -p output_path:=/home/f1/f1/src/pure_pursuit/racelines/my_track.csv \
  -p min_distance:=0.15 \
  -p default_velocity:=2.0
```

If you are not sure whether odometry is published on `/odom` or a localization
topic such as `/pf/pose/odom`, keep `/odom` as a fallback:

```bash
ros2 run pure_pursuit waypoint_recorder --ros-args \
  -p odom_topic:=/pf/pose/odom \
  -p fallback_odom_topics:=/odom \
  -p output_path:=/home/f1/f1/src/pure_pursuit/racelines/my_track.csv
```
