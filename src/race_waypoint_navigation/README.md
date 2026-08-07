# NEW PACKAGE for race_waypoint_navigation

1. Build

  cd ~/f1
  colcon build
  source install/setup.bash

  2. สร้าง Map

  cd ~/f1
  source install/setup.bash
  ros2 launch pure_pursuit mapping.launch.py

  ดู map ที่ได้ ด้วยคำสั่ง
  rviz2 -d install/wheel_odometry/share/wheel_odometry/rviz/odom_debug.rviz

  ขับรถให้ครบ loop แล้วบันทึก map:

  cd ~/f1
  source install/setup.bash
  mkdir -p /home/f1/f1/src/pure_pursuit/maps
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/f1/f1/src/pure_pursuit/maps/map0608'}"

  3. ทดสอบ Localization

  cd ~/f1
  source install/setup.bash
  ros2 launch wheel_odometry complete_localization.launch.py \
    map_file:=/home/f1/f1/src/pure_pursuit/maps/map0608

  พร้อมกับรันใช้ joy
  ros2 run joy joy_node


  ใน RViz กด 2D Pose Estimate ให้รถตรงกับ map

  4. เก็บ Waypoint ด้วย Joy

  cd ~/f1
  source install/setup.bash
  ros2 launch race_waypoint_navigation record_waypoints.launch.py \
    map_file:=/home/f1/f1/src/pure_pursuit/maps/map \
    output_path:=/home/f1/f1/src/race_waypoint_navigation/waypoints/raw_path0608.csv

  ค่า default:

  - กดค้างปุ่ม ZL เพื่อ record หยุดกดปุ่มเพื่อ stop
  - บันทึกทุก ๆ 0.15 m

  5. สร้าง Raceline จาก Waypoint

  cd ~/f1
  source install/setup.bash
  ros2 run race_waypoint_navigation raceline_processor \
    --input /home/f1/f1/src/race_waypoint_navigation/waypoints/raw_path0608.csv \
    --output /home/f1/f1/src/race_waypoint_navigation/waypoints/raceline0608.csv \
    --spacing 0.15 \
    --min-speed 0.25 \
    --max-speed 1.2

  6. วิ่ง Pure Pursuit + Obstacle Layer

  cd ~/f1
  source install/setup.bash
  ros2 launch race_waypoint_navigation race_pure_pursuit.launch.py \
    map_file:=/home/f1/f1/src/pure_pursuit/maps/map0608 \
    path_file:=/home/f1/f1/src/race_waypoint_navigation/waypoints/raceline0608.csv \
    max_speed:=1.2 \
    auto_start:=false

  terminal2:
  ros2 run joy joy_node


  # AMCL

   source install/setup.bash
  ros2 launch race_waypoint_navigation race_pure_pursuit.launch.py \
    with_rviz:=true \
    localization_backend:=amcl \
    map_yaml_file:=/home/f1/f1/src/pure_pursuit/maps/map0608.yaml \
    raw_path_file:=/home/f1/f1/src/race_waypoint_navigation/waypoints/raw_path0608.csv \
    path_file:=/home/f1/f1/src/race_waypoint_navigation/waypoints/raceline0608.csv \
    max_speed:=1.2 \
    auto_start:=false

  RViz จะแสดง raw_path สีส้ม, raceline สีเขียว และ target waypoint สีฟ้า
  จาก topic /race_path_markers และ /race_target_waypoint


  ค่า default:

  - กดปุ่ม A เพื่อเริ่มวิ่ง
  - กดปุ่ม B เพื่อหยุด
  - publish คำสั่งไป /cmd_vel
  - subscribe /scan เพื่อหลบ obstacle แบบ realtime

  สถานะ controller ดูได้ที่:

  ros2 topic echo /race_state

  state ที่มี:

  FOLLOW_WAYPOINT
  SLOW_DOWN
  AVOID_GAP
  EMERGENCY_STOP

  หลักการคือใช้ pure pursuit เป็น controller หลัก ถ้า LiDAR เจอสิ่งกีดขวางด้านหน้า จะลดความเร็ว/หลบตาม gap และเมื่อทางโล่งกับเข้าใกล้ path แล้วจะกลับไป
  FOLLOW_WAYPOINT.




## Flow การรันระบบแบบ auto generate

  1. Build

  cd ~/f1
  colcon build
  source install/setup.bash

  2. สร้าง map ตาม README แล้ว save occupancy map เพิ่ม

  ros2 launch pure_pursuit mapping.launch.py

  หลังขับครบ loop:

  ros2 run nav2_map_server map_saver_cli -f /home/f1/f1/src/pure_pursuit/maps/map0608

  3. Generate raw waypoint จาก map

  ros2 run race_waypoint_navigation map_centerline_generator \
    --map /home/f1/f1/src/pure_pursuit/maps/map0608.yaml \
    --reference-path /home/f1/f1/src/race_waypoint_navigation/waypoints/raw_path0608.csv \
    --output /home/f1/f1/src/race_waypoint_navigation/waypoints/raw_path_auto0608.csv \
    --spacing 0.15 \
    --robot-radius 0.18 \
    --safety-margin 0.10


  คำสั่งนี้ใช้ raw_path0608.csv ที่เก็บเองเป็น reference เพื่อรักษาลำดับเส้นทาง
  แล้วใช้ map06082.yaml หา midpoint ของพื้นที่ว่างตามแนวตั้งฉากกับทางที่เคยขับ
  ผลลัพธ์ raw_path_auto06082.csv จึงเป็น waypoint ที่อิงทางที่เก็บเอง แต่ถูกจัดให้อยู่กลาง corridor ของ map มากขึ้น

  4. สร้าง raceline ด้วย processor เดิม

  ros2 run race_waypoint_navigation raceline_processor \
    --input /home/f1/f1/src/race_waypoint_navigation/waypoints/raw_path_auto0608.csv \
    --output /home/f1/f1/src/race_waypoint_navigation/waypoints/raceline_auto0608.csv \
    --spacing 0.15 \
    --min-speed 2.0 \
    --max-speed 4.5


  5. รัน pure pursuit เหมือนเดิม แต่เปลี่ยน path_file

  ros2 launch race_waypoint_navigation race_pure_pursuit.launch.py \
    map_file:=/home/f1/f1/src/pure_pursuit/maps/map0608 \
    path_file:=/home/f1/f1/src/race_waypoint_navigation/waypoints/raceline_auto06082.csv \
    max_speed:=1.2 \
    auto_start:=false





  ros2 launch race_waypoint_navigation race_pure_pursuit.launch.py \
    with_rviz:=true \
    localization_backend:=amcl \
    map_yaml_file:=/home/f1/f1/src/pure_pursuit/maps/map0608.yaml \
    raw_path_file:=/home/f1/f1/src/race_waypoint_navigation/waypoints/raw_path_auto0608.csv \
    path_file:=/home/f1/f1/src/race_waypoint_navigation/waypoints/raceline_auto0608.csv \
    max_speed:=4.5 \
    auto_start:=false