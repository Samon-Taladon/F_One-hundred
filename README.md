codex

colcon build --packages-select pure_pursuit
colcon build --packages-select follow_gap_navigation


source install/setup.bash
ros2 launch vesc_driver vesc_driver_node.launch.py

source install/setup.bash
ros2 run joy joy_node

source install/setup.bash
ros2 run teleop_twist_joy teleop_node --ros-args --params-file my_joy_config.yaml

sudo chmod 777 /dev/ttyACM0 

python3 /home/champion/f1/src/bmi160.py

source install/setup.bash
ros2 launch /home/champion/f1/src/imu_visualizer_launch.py

ros2 topic list

ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link imu_link

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom


# Champion ---> RLIDAR

ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200 \
    -p frame_id:=laser_frame \
    -p inverted:=false \
    -p angle_compensate:=true
    
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame
    
    rviz2


laser
sudo ip addr add 192.168.1.100/24 dev enP8p1s0
ip addr show enP8p1s0
ping 192.168.1.10
ros2 run urg_node urg_node_driver --ros-args -p ip_address:=192.168.1.10

// ________________________________________________________________________________

# Auto Driver
    
/usr/bin/python /home/champion/f1/src/wall_follower/src/wall_follow/wall_follow/wall_follower.py
ros2 run wall_follow wall_follower

#  New Auto Driver
source install/setup.bash
ros2 launch follow_gap_navigation follow_gap_navigation.launch.py


ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

ros2 run tf2_tools view_frames

rqt_graph




cd /home/champion/f1/src/wall_follower
source install/setup.bash
ros2 launch wall_follow follow_the_gap_rviz.launch.py

git add .
git commit -m "update code"
git push origin main









ros2 launch wall_follow lidar_scan_odom.launch.py
ros2 run lidar_waypoint_racing waypoint_recorder     --ros-args -p distance_threshold:=0.2



  หลักการทำงาน

  - รอบแรกเริ่มที่ normal_speed
  - ช่วงปลอดภัยเพิ่ม 0.05 m/s
  - ช่วงอันตรายลด 0.10 m/s
  - Emergency stop/reverse ถือเป็นช่วงอันตราย
  - ความเร็วจริงคือค่าต่ำสุดระหว่าง LiDAR safety และ learned speed
  - บันทึกผลลง CSV อัตโนมัติ

  วิธีใช้

   1. Build

  cd ~/f1
  colcon build --packages-select \
    wall_follow \
    lidar_waypoint_racing \
    follow_gap_navigation
  source install/setup.bash

  2. เปิดอุปกรณ์

  Terminal 1:

  source install/setup.bash
  ros2 launch vesc_driver vesc_driver_node.launch.py

  Terminal 2:

  ros2 run urg_node urg_node_driver --ros-args \
    -p ip_address:=192.168.1.10

  3. สร้าง Odometry

  Terminal 3:

  source install/setup.bash
  ros2 launch wall_follow lidar_scan_odom.launch.py

  ตรวจสอบ:

  source install/setup.bash
  ros2 run tf2_ros tf2_echo odom base_link

  ค่าตำแหน่งต้องเปลี่ยนเมื่อรถเคลื่อนที่

  4. บันทึกเส้นทางรอบแรก

  Terminal 4:

  source install/setup.bash
  ros2 run lidar_waypoint_racing waypoint_recorder --ros-args \
    -p odom_topic:=/odom \
    -p output_file:=/home/f1/f1/adaptive_waypoints.csv \
    -p distance_threshold:=0.15

  Terminal 5 เปิดรถวิ่ง:

  source install/setup.bash
  ros2 launch follow_gap_navigation follow_gap_navigation.launch.py \
    adaptive_speed_enabled:=false \
    normal_speed:=0.7 \
    min_speed:=0.4 \
    max_speed:=1.0

  เมื่อรถวิ่งครบรอบ ให้กด Ctrl+C ที่ waypoint_recorder จะได้ไฟล์:

  /home/f1/f1/adaptive_waypoints.csv

  5. เปิดการเรียนรู้ความเร็ว

  หยุด Follow Gap เดิม แล้วรัน:

  source install/setup.bash
  ros2 launch follow_gap_navigation follow_gap_navigation.launch.py \
    adaptive_speed_enabled:=true \
    adaptive_learning_enabled:=true \
    map_frame:=odom \
    base_frame:=base_link \
    waypoint_file:=/home/f1/f1/adaptive_waypoints.csv \
    speed_profile_file:=/home/f1/f1/adaptive_speed_profile.csv \
    normal_speed:=0.7 \
    min_speed:=0.4 \
    max_speed:=1.2 \
    learning_speed_increment:=0.03 \
    learning_speed_penalty:=0.10











# Odom

Terminal 1:
source install/setup.bash
ros2 launch vesc_driver vesc_driver_node.launch.py


Terminal 2:
source install/setup.bash
ros2 run joy joy_node



Terminal 3:
ros2 run urg_node urg_node_driver --ros-args -p ip_address:=192.168.1.10


Terminal 4:
ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link imu_link


Terminal 5: หยุด
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link


Terminal 6: หยุด
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

rviz2


