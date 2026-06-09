/*
Pure Pursuit Implementation in C++. Includes features such as dynamic lookahead. Does not have waypoint
interpolation yet.
*/
#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/quaternion.hpp>
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

#define _USE_MATH_DEFINES
using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node {
   public:
    PurePursuit();

   private:
    // global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    struct csvFileData {
        std::vector<double> X;
        std::vector<double> Y;
        std::vector<double> V;

        int index = 0;
        int velocity_index = 0;

        Eigen::Vector3d lookahead_point_world;  // from world reference frame (usually `map`)
        Eigen::Vector3d lookahead_point_car;    // from car reference frame
        Eigen::Vector3d current_point_world;    // Locks on to the closest waypoint, which gives a velocity profile
    };

    Eigen::Matrix3d rotation_m;

    double x_car_world;
    double y_car_world;
    double yaw_car_world = 0.0;
    double odom_origin_x = 0.0;
    double odom_origin_y = 0.0;

    std::string odom_topic;
    std::string car_refFrame;
    std::string drive_topic;
    std::string drive_msg_type;
    std::string global_refFrame;
    std::string rviz_current_waypoint_topic;
    std::string rviz_lookahead_waypoint_topic;
    std::string waypoints_path;
    std::string joy_topic;
    double K_p;
    double min_lookahead;
    double max_lookahead;
    double lookahead_ratio;
    double steering_limit;
    double velocity_percentage;
    double fixed_motor_speed_erpm;
    double speed_to_erpm_gain;
    double speed_to_erpm_offset;
    double curr_velocity = 0.0;
    int start_button_index = 0;
    int stop_button_index = 1;

    bool emergency_breaking = false;
    bool drive_enabled = false;
    bool invert_steering = false;
    bool use_relative_origin = true;
    bool use_tf_transform = false;
    bool odom_origin_initialized = false;
    std::string lane_number = "left";  // left or right lane

    // file object
    std::fstream csvFile_waypoints;

    // struct initialisation
    csvFileData waypoints;
    int num_waypoints = 0;

    // Timer initialisation
    rclcpp::TimerBase::SharedPtr timer_;

    // declare subscriber sharedpointer obj
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joy;

    // declare publisher sharedpointer obj
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_current_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_lookahead_point_pub;

    // declare tf shared pointers
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // private functions
    double to_radians(double degrees);
    double to_degrees(double radians);
    double p2pdist(double &x1, double &x2, double &y1, double &y2);

    void load_waypoints();

    void visualize_lookahead_point(Eigen::Vector3d &point);
    void visualize_current_point(Eigen::Vector3d &point);

    void get_waypoint();

    void quat_to_rot(double q0, double q1, double q2, double q3);
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q);

    void transformandinterp_waypoint();
    void transform_waypoint_from_odom();

    double p_controller();

    double get_velocity(double steering_angle);

    void publish_message(double steering_angle);
    void publish_stop_message();

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj);
    void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msgObj);

    void timer_callback();
};
