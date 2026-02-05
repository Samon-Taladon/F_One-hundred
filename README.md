source install/setup.bash
ros2 launch vesc_driver vesc_driver_node.launch.py

source install/setup.bash
ros2 run joy joy_node

source install/setup.bash
ros2 run teleop_twist_joy teleop_node --ros-args --params-file my_joy_config.yaml

sudo chmod 777 /dev/ttyACM0

# Champion
