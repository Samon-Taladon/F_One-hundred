# Veddar VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

## How to test

1. Clone this repository by `git clone -b humble https://github.com/yoodyui/INC493.git` into `src`.
2. `rosdep install -i --from-path src --rosdistro humble -y`
3. Plug in the VESC with a USB cable.
4. Modify `vesc/vesc_driver/src/vesc_driver.cpp` to reflect any changes on BP's comments.
5. Build the packages `colcon build` or `colcon build --packages-select vesc_driver`
6. Source files `source install/setup.bash`
7. `ros2 launch vesc_driver vesc_driver_node.launch.py`
8. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`
9. `ros2 topic echo motor_speed` to see the command to BLDC
10. `ros2 topic echo servo_angle` to see the command to servo motor
