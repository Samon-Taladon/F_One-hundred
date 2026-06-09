#include "pure_pursuit.hpp"

#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit_node") {
    // initialise parameters
    this->declare_parameter("waypoints_path", "/sim_ws/src/pure_pursuit/racelines/e7_floor5.csv");
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("car_refFrame", "ego_racecar/base_link");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("drive_msg_type", "ackermann");
    this->declare_parameter("rviz_current_waypoint_topic", "/current_waypoint");
    this->declare_parameter("rviz_lookahead_waypoint_topic", "/lookahead_waypoint");
    this->declare_parameter("joy_topic", "/joy");
    this->declare_parameter("start_button_index", 0);
    this->declare_parameter("stop_button_index", 1);
    this->declare_parameter("global_refFrame", "map");
    this->declare_parameter("min_lookahead", 0.5);
    this->declare_parameter("max_lookahead", 1.0);
    this->declare_parameter("lookahead_ratio", 8.0);
    this->declare_parameter("K_p", 0.5);
    this->declare_parameter("steering_limit", 25.0);
    this->declare_parameter("velocity_percentage", 0.6);
    this->declare_parameter("fixed_motor_speed_erpm", 0.0);
    this->declare_parameter("speed_to_erpm_gain", 4614.0);
    this->declare_parameter("speed_to_erpm_offset", 0.0);
    this->declare_parameter("invert_steering", false);
    this->declare_parameter("use_relative_origin", true);
    this->declare_parameter("use_tf_transform", false);

    // Default Values
    waypoints_path = this->get_parameter("waypoints_path").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    car_refFrame = this->get_parameter("car_refFrame").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    drive_msg_type = this->get_parameter("drive_msg_type").as_string();
    rviz_current_waypoint_topic = this->get_parameter("rviz_current_waypoint_topic").as_string();
    rviz_lookahead_waypoint_topic = this->get_parameter("rviz_lookahead_waypoint_topic").as_string();
    joy_topic = this->get_parameter("joy_topic").as_string();
    start_button_index = this->get_parameter("start_button_index").as_int();
    stop_button_index = this->get_parameter("stop_button_index").as_int();
    global_refFrame = this->get_parameter("global_refFrame").as_string();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    K_p = this->get_parameter("K_p").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    fixed_motor_speed_erpm = this->get_parameter("fixed_motor_speed_erpm").as_double();
    speed_to_erpm_gain = this->get_parameter("speed_to_erpm_gain").as_double();
    speed_to_erpm_offset = this->get_parameter("speed_to_erpm_offset").as_double();
    invert_steering = this->get_parameter("invert_steering").as_bool();
    use_relative_origin = this->get_parameter("use_relative_origin").as_bool();
    use_tf_transform = this->get_parameter("use_tf_transform").as_bool();

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, _1));
    subscription_joy = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic, 10, std::bind(&PurePursuit::joy_callback, this, _1));
    timer_ = this->create_wall_timer(2000ms, std::bind(&PurePursuit::timer_callback, this));

    if (drive_msg_type == "twist") {
        publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(drive_topic, 25);
        RCLCPP_INFO(this->get_logger(), "Publishing drive commands as geometry_msgs/Twist on %s", drive_topic.c_str());
    } else {
        publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
        RCLCPP_INFO(this->get_logger(), "Publishing drive commands as ackermann_msgs/AckermannDriveStamped on %s", drive_topic.c_str());
    }
    vis_current_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_current_waypoint_topic, 10);
    vis_lookahead_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_lookahead_waypoint_topic, 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Pure pursuit node has been launched");
    RCLCPP_INFO(this->get_logger(), "Controller origin mode: %s, transform mode: %s",
                use_relative_origin ? "relative to first odom" : "absolute odom",
                use_tf_transform ? "tf2" : "odom pose");
    RCLCPP_INFO(this->get_logger(), "Waiting for joy button A (%d) to start. Press B (%d) to stop.", start_button_index, stop_button_index);

    load_waypoints();
}

double PurePursuit::to_radians(double degrees) {
    double radians;
    return radians = degrees * M_PI / 180.0;
}

double PurePursuit::to_degrees(double radians) {
    double degrees;
    return degrees = radians * 180.0 / M_PI;
}

double PurePursuit::p2pdist(double &x1, double &x2, double &y1, double &y2) {
    double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    return dist;
}

void PurePursuit::load_waypoints() {
    csvFile_waypoints.open(waypoints_path, std::ios::in);

    if (!csvFile_waypoints.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path.c_str());
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "CSV File Opened");
    }

    // std::vector<std::string> row;
    std::string line, word, temp;

    while (!csvFile_waypoints.eof()) {
        std::getline(csvFile_waypoints, line, '\n');
        std::stringstream s(line);

        int j = 0;
        while (getline(s, word, ',')) {
            if (!word.empty()) {
                if (j == 0) {
                    waypoints.X.push_back(std::stod(word));
                } else if (j == 1) {
                    waypoints.Y.push_back(std::stod(word));
                } else if (j == 2) {
                    waypoints.V.push_back(std::stod(word));
                }
            }
            j++;
        }
    }

    csvFile_waypoints.close();
    num_waypoints = waypoints.X.size();
    if (num_waypoints == 0) {
        RCLCPP_ERROR(this->get_logger(), "No waypoints loaded from %s", waypoints_path.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %s", num_waypoints, waypoints_path.c_str());

    double average_dist_between_waypoints = 0.0;
    for (int i = 0; i < num_waypoints - 1; i++) {
        average_dist_between_waypoints += p2pdist(waypoints.X[i], waypoints.X[i + 1], waypoints.Y[i], waypoints.Y[i + 1]);
    }
    average_dist_between_waypoints /= num_waypoints;
    RCLCPP_INFO(this->get_logger(), "Average distance between waypoints: %f", average_dist_between_waypoints);
}

void PurePursuit::visualize_lookahead_point(Eigen::Vector3d &point) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = global_refFrame;
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 1.0;

    marker.pose.position.x = point(0);
    marker.pose.position.y = point(1);
    marker.id = 1;
    vis_lookahead_point_pub->publish(marker);
}

void PurePursuit::visualize_current_point(Eigen::Vector3d &point) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = global_refFrame;
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.b = 1.0;

    marker.pose.position.x = point(0);
    marker.pose.position.y = point(1);
    marker.id = 1;
    vis_current_point_pub->publish(marker);
}

void PurePursuit::get_waypoint() {
    // Main logic: Search within the next 500 points
    double longest_distance = 0;
    int final_i = -1;
    int start = waypoints.index;
    int end = (waypoints.index + 500) % num_waypoints;

    // Lookahead needs to be between the min_lookhead and the max_lookahead
    double lookahead = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead);

    if (end < start) {  // If we need to loop around
        for (int i = start; i < num_waypoints; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
        for (int i = 0; i < end; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    } else {
        for (int i = start; i < end; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    }

    if (final_i == -1) {  // if we haven't found anything, search from the beginning
        final_i = 0;
        for (int i = 0; i < num_waypoints; i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    }

    // Find the closest point to the car, and use the velocity index for that
    double shortest_distance = p2pdist(waypoints.X[0], x_car_world, waypoints.Y[0], y_car_world);
    int velocity_i = 0;
    for (int i = 0; i < num_waypoints; i++) {
        if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= shortest_distance) {
            shortest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
            velocity_i = i;
        }
    }

    // If a waypoint is not found within our radius, then waypoints.index = 0
    waypoints.index = final_i;
    waypoints.velocity_index = velocity_i;
}

void PurePursuit::quat_to_rot(double q0, double q1, double q2, double q3) {
    double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);
    double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
    double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));

    double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
    double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
    double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));

    double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
    double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
    double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

    rotation_m << r00, r01, r02, r10, r11, r12, r20, r21, r22;
}

double PurePursuit::quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

void PurePursuit::transformandinterp_waypoint() {  // pass old waypoint here
    // initialise vectors
    waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;
    waypoints.current_point_world << waypoints.X[waypoints.velocity_index], waypoints.Y[waypoints.velocity_index], 0.0;

    visualize_lookahead_point(waypoints.lookahead_point_world);
    visualize_current_point(waypoints.current_point_world);

    if (!use_tf_transform) {
        transform_waypoint_from_odom();
        return;
    }

    // look up transformation at that instant from tf_buffer_
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        // Get the transform from the base_link reference to world reference frame
        transformStamped = tf_buffer_->lookupTransform(car_refFrame, global_refFrame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not transform. Error: %s", ex.what());
        transform_waypoint_from_odom();
        return;
    }

    // transform points (rotate first and then translate)
    Eigen::Vector3d translation_v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    waypoints.lookahead_point_car = (rotation_m * waypoints.lookahead_point_world) + translation_v;
}

void PurePursuit::transform_waypoint_from_odom() {
    const double dx = waypoints.lookahead_point_world(0) - x_car_world;
    const double dy = waypoints.lookahead_point_world(1) - y_car_world;
    const double cos_yaw = std::cos(yaw_car_world);
    const double sin_yaw = std::sin(yaw_car_world);

    waypoints.lookahead_point_car << (cos_yaw * dx) + (sin_yaw * dy),
        (-sin_yaw * dx) + (cos_yaw * dy),
        0.0;
}

double PurePursuit::p_controller() {
    double r = waypoints.lookahead_point_car.norm();  // r = sqrt(x^2 + y^2)
    double y = waypoints.lookahead_point_car(1);
    double angle = K_p * 2 * y / pow(r, 2);  // Calculated from https://docs.google.com/presentation/d/1jpnlQ7ysygTPCi8dmyZjooqzxNXWqMgO31ZhcOlKVOE/edit#slide=id.g63d5f5680f_0_33

    return angle;
}

double PurePursuit::get_velocity(double steering_angle) {
    double velocity = 0;

    if (fixed_motor_speed_erpm > 0.0 && speed_to_erpm_gain != 0.0) {
        return (fixed_motor_speed_erpm - speed_to_erpm_offset) / speed_to_erpm_gain;
    }

    if (waypoints.V[waypoints.velocity_index]) {
        velocity = waypoints.V[waypoints.velocity_index] * velocity_percentage;
    } else {  // For waypoints loaded without velocity profiles
        if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
            velocity = 6.0 * velocity_percentage;
        } else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
            velocity = 2.5 * velocity_percentage;
        } else {
            velocity = 2.0 * velocity_percentage;
        }
    }

    return velocity;
}

void PurePursuit::publish_message(double steering_angle) {
    double clipped_steering = 0.0;
    if (steering_angle < 0.0) {
        clipped_steering = std::max(steering_angle, -to_radians(steering_limit));  // ensure steering angle is dynamically capable
    } else {
        clipped_steering = std::min(steering_angle, to_radians(steering_limit));  // ensure steering angle is dynamically capable
    }

    curr_velocity = get_velocity(clipped_steering);

    RCLCPP_INFO(this->get_logger(), "index: %d ... distance: %.2fm ... Speed: %.2fm/s ... Motor Speed Target: %.2f eRPM ... Steering Angle: %.2f ... K_p: %.2f ... velocity_percentage: %.2f", waypoints.index, p2pdist(waypoints.X[waypoints.index], x_car_world, waypoints.Y[waypoints.index], y_car_world), curr_velocity, fixed_motor_speed_erpm, to_degrees(clipped_steering), K_p, velocity_percentage);

    if (drive_msg_type == "twist") {
        auto cmd_msgObj = geometry_msgs::msg::Twist();
        cmd_msgObj.linear.x = curr_velocity;
        cmd_msgObj.angular.z = clipped_steering;
        publisher_cmd_vel->publish(cmd_msgObj);
    } else {
        auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msgObj.drive.steering_angle = clipped_steering;
        drive_msgObj.drive.speed = curr_velocity;
        publisher_drive->publish(drive_msgObj);
    }
}

void PurePursuit::publish_stop_message() {
    curr_velocity = 0.0;

    if (drive_msg_type == "twist") {
        auto cmd_msgObj = geometry_msgs::msg::Twist();
        cmd_msgObj.linear.x = 0.0;
        cmd_msgObj.angular.z = 0.0;
        publisher_cmd_vel->publish(cmd_msgObj);
    } else {
        auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msgObj.drive.steering_angle = 0.0;
        drive_msgObj.drive.speed = 0.0;
        publisher_drive->publish(drive_msgObj);
    }
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
    if (num_waypoints == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for a valid waypoint CSV");
        return;
    }

    const double raw_x = odom_submsgObj->pose.pose.position.x;
    const double raw_y = odom_submsgObj->pose.pose.position.y;

    if (use_relative_origin && !odom_origin_initialized) {
        odom_origin_x = raw_x;
        odom_origin_y = raw_y;
        odom_origin_initialized = true;
        RCLCPP_INFO(this->get_logger(), "Controller odom origin set to x=%.3f y=%.3f",
                    odom_origin_x, odom_origin_y);
    }

    x_car_world = use_relative_origin ? raw_x - odom_origin_x : raw_x;
    y_car_world = use_relative_origin ? raw_y - odom_origin_y : raw_y;
    yaw_car_world = quaternion_to_yaw(odom_submsgObj->pose.pose.orientation);

    if (!drive_enabled) {
        publish_stop_message();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Pure pursuit is stopped. Press joy button A (%d) to start.", start_button_index);
        return;
    }

    // interpolate between different way-points
    get_waypoint();

    // use tf2 transform the goal point
    transformandinterp_waypoint();

    // Calculate curvature/steering angle
    double steering_angle = p_controller();
    if (invert_steering) {
        steering_angle *= -1.0;
    }

    // publish object and message: AckermannDriveStamped on drive topic
    publish_message(steering_angle);
}

void PurePursuit::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msgObj) {
    const bool stop_pressed = stop_button_index >= 0 &&
                              static_cast<size_t>(stop_button_index) < joy_msgObj->buttons.size() &&
                              joy_msgObj->buttons[stop_button_index] == 1;
    const bool start_pressed = start_button_index >= 0 &&
                               static_cast<size_t>(start_button_index) < joy_msgObj->buttons.size() &&
                               joy_msgObj->buttons[start_button_index] == 1;

    if (stop_pressed) {
        if (drive_enabled) {
            RCLCPP_INFO(this->get_logger(), "Joy button B pressed. Stopping pure pursuit.");
        }
        drive_enabled = false;
        publish_stop_message();
        return;
    }

    if (start_pressed && !drive_enabled) {
        drive_enabled = true;
        RCLCPP_INFO(this->get_logger(), "Joy button A pressed. Starting pure pursuit.");
    }
}

void PurePursuit::timer_callback() {
    // Periodically check parameters and update
    K_p = this->get_parameter("K_p").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    fixed_motor_speed_erpm = this->get_parameter("fixed_motor_speed_erpm").as_double();
    speed_to_erpm_gain = this->get_parameter("speed_to_erpm_gain").as_double();
    speed_to_erpm_offset = this->get_parameter("speed_to_erpm_offset").as_double();
    invert_steering = this->get_parameter("invert_steering").as_bool();
    use_tf_transform = this->get_parameter("use_tf_transform").as_bool();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>();  // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
