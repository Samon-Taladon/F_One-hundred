codex

colcon build --packages-select pure_pursuit

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









