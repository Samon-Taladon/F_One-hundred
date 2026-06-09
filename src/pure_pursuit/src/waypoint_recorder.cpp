#include <cmath>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#ifndef PURE_PURSUIT_SOURCE_DIR
#define PURE_PURSUIT_SOURCE_DIR "."
#endif

using std::placeholders::_1;
using namespace std::chrono_literals;

class WaypointRecorder : public rclcpp::Node {
   public:
    WaypointRecorder() : Node("waypoint_recorder") {
        const std::string default_output_path =
            std::string(PURE_PURSUIT_SOURCE_DIR) + "/racelines/recorded_waypoints.csv";

        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("fallback_odom_topics", "/odom");
        this->declare_parameter("joy_topic", "/joy");
        this->declare_parameter("output_path", default_output_path);
        this->declare_parameter("record_button", 4);
        this->declare_parameter("stop_button", -1);
        this->declare_parameter("min_distance", 0.10);
        this->declare_parameter("default_velocity", 2.0);
        this->declare_parameter("use_odom_velocity", true);
        this->declare_parameter("append", false);
        this->declare_parameter("relative_coordinates", true);

        odom_topic_ = this->get_parameter("odom_topic").as_string();
        fallback_odom_topics_ = this->get_parameter("fallback_odom_topics").as_string();
        joy_topic_ = this->get_parameter("joy_topic").as_string();
        output_path_ = this->get_parameter("output_path").as_string();
        record_button_ = this->get_parameter("record_button").as_int();
        stop_button_ = this->get_parameter("stop_button").as_int();
        min_distance_ = this->get_parameter("min_distance").as_double();
        default_velocity_ = this->get_parameter("default_velocity").as_double();
        use_odom_velocity_ = this->get_parameter("use_odom_velocity").as_bool();
        append_ = this->get_parameter("append").as_bool();
        relative_coordinates_ = this->get_parameter("relative_coordinates").as_bool();

        const auto open_mode = append_ ? (std::ios::out | std::ios::app) : (std::ios::out | std::ios::trunc);
        csv_file_.open(output_path_, open_mode);
        if (!csv_file_.is_open()) {
            RCLCPP_FATAL(this->get_logger(), "Cannot open waypoint output CSV: %s", output_path_.c_str());
            throw std::runtime_error("Cannot open waypoint output CSV");
        }

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_, 10, std::bind(&WaypointRecorder::joy_callback, this, _1));

        odom_topics_ = get_odom_topics();
        for (const auto &topic : odom_topics_) {
            odom_subs_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
                topic, 25, [this, topic](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    odom_callback(msg, topic);
                }));
        }
        status_timer_ = this->create_wall_timer(2s, std::bind(&WaypointRecorder::status_callback, this));

        RCLCPP_INFO(this->get_logger(), "Waypoint recorder is writing to %s", output_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Listening for odometry on: %s", join_odom_topics().c_str());
        if (relative_coordinates_) {
            RCLCPP_INFO(this->get_logger(), "Recording relative waypoints. First saved waypoint will be x=0.000 y=0.000.");
        }
        RCLCPP_INFO(this->get_logger(), "Hold joy button %d to record.", record_button_);
        if (stop_button_ >= 0) {
            RCLCPP_INFO(this->get_logger(), "Press joy button %d to stop.", stop_button_);
        }
    }

    ~WaypointRecorder() override {
        if (csv_file_.is_open()) {
            csv_file_.flush();
            csv_file_.close();
        }
        RCLCPP_INFO(this->get_logger(), "Saved %zu waypoints to %s", waypoint_count_, output_path_.c_str());
    }

   private:
    void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        if (stop_button_ >= 0 && static_cast<size_t>(stop_button_) < msg->buttons.size() &&
            msg->buttons[stop_button_] == 1) {
            RCLCPP_INFO(this->get_logger(), "Stop button pressed. Shutting down waypoint recorder.");
            rclcpp::shutdown();
            return;
        }

        bool should_record = true;
        if (record_button_ >= 0) {
            should_record = static_cast<size_t>(record_button_) < msg->buttons.size() &&
                            msg->buttons[record_button_] == 1;
        }

        if (should_record != recording_enabled_) {
            RCLCPP_INFO(this->get_logger(), "%s waypoint recording",
                        should_record ? "Started" : "Paused");
        }
        recording_enabled_ = should_record;
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, const std::string &topic) {
        if (active_odom_topic_.empty()) {
            active_odom_topic_ = topic;
            RCLCPP_INFO(this->get_logger(), "Using odometry from %s", active_odom_topic_.c_str());
        } else if (topic != active_odom_topic_) {
            if (topic == odom_topic_ && active_odom_topic_ != odom_topic_) {
                active_odom_topic_ = topic;
                RCLCPP_INFO(this->get_logger(), "Switched to primary odometry topic %s",
                            active_odom_topic_.c_str());
            } else {
                return;
            }
        }

        received_odom_ = true;
        last_odom_x_ = msg->pose.pose.position.x;
        last_odom_y_ = msg->pose.pose.position.y;

        if (!recording_enabled_ || !csv_file_.is_open()) {
            return;
        }

        const double raw_x = msg->pose.pose.position.x;
        const double raw_y = msg->pose.pose.position.y;

        if (relative_coordinates_ && !origin_initialized_) {
            origin_x_ = raw_x;
            origin_y_ = raw_y;
            origin_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Waypoint origin set to odom x=%.3f y=%.3f", origin_x_, origin_y_);
        }

        const double x = relative_coordinates_ ? raw_x - origin_x_ : raw_x;
        const double y = relative_coordinates_ ? raw_y - origin_y_ : raw_y;

        if (has_last_point_) {
            const double distance = std::hypot(x - last_x_, y - last_y_);
            if (distance < min_distance_) {
                return;
            }
        }

        double velocity = default_velocity_;
        if (use_odom_velocity_) {
            const double vx = msg->twist.twist.linear.x;
            const double vy = msg->twist.twist.linear.y;
            velocity = std::hypot(vx, vy);
            if (velocity <= std::numeric_limits<double>::epsilon()) {
                velocity = default_velocity_;
            }
        }

        csv_file_ << x << "," << y << "," << velocity << "\n";
        csv_file_.flush();

        last_x_ = x;
        last_y_ = y;
        has_last_point_ = true;
        ++waypoint_count_;

        RCLCPP_INFO(this->get_logger(), "Saved waypoint %zu: %.3f, %.3f, %.3f",
                    waypoint_count_, x, y, velocity);
    }

    void status_callback() {
        if (!recording_enabled_) {
            return;
        }

        if (!received_odom_) {
            RCLCPP_WARN(this->get_logger(),
                        "Recording is enabled, but no odometry has arrived on any configured topic: %s",
                        join_odom_topics().c_str());
            return;
        }

        if (waypoint_count_ == 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Odometry is arriving on %s at x=%.3f y=%.3f, but no waypoint has been saved yet.",
                        active_odom_topic_.c_str(), last_odom_x_, last_odom_y_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Recording: %zu waypoints saved. Last odom x=%.3f y=%.3f",
                    waypoint_count_, last_odom_x_, last_odom_y_);
    }

    std::vector<std::string> get_odom_topics() const {
        std::vector<std::string> topics;
        add_unique_topic(topics, odom_topic_);

        std::stringstream stream(fallback_odom_topics_);
        std::string topic;
        while (std::getline(stream, topic, ',')) {
            add_unique_topic(topics, topic);
        }

        return topics;
    }

    static void add_unique_topic(std::vector<std::string> &topics, std::string topic) {
        topic.erase(std::remove_if(topic.begin(), topic.end(), [](unsigned char c) {
            return std::isspace(c);
        }), topic.end());
        if (topic.empty()) {
            return;
        }
        if (std::find(topics.begin(), topics.end(), topic) == topics.end()) {
            topics.push_back(topic);
        }
    }

    std::string join_odom_topics() const {
        std::string joined;
        for (size_t i = 0; i < odom_topics_.size(); ++i) {
            if (i > 0) {
                joined += ", ";
            }
            joined += odom_topics_[i];
        }
        return joined;
    }

    std::string odom_topic_;
    std::string fallback_odom_topics_;
    std::string joy_topic_;
    std::string output_path_;
    int record_button_;
    int stop_button_;
    double min_distance_;
    double default_velocity_;
    bool use_odom_velocity_;
    bool append_;
    bool relative_coordinates_;
    bool recording_enabled_ = false;
    bool received_odom_ = false;
    bool has_last_point_ = false;
    bool origin_initialized_ = false;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_odom_x_ = 0.0;
    double last_odom_y_ = 0.0;
    size_t waypoint_count_ = 0;
    std::vector<std::string> odom_topics_;
    std::string active_odom_topic_;

    std::ofstream csv_file_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
