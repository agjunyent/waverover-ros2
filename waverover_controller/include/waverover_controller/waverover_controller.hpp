#pragma once

#include <math.h>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "waverover_helpers/helpers.hpp"

#include "waverover_controller/json.hpp"
#include "waverover_controller/rover_commands.hpp"
#include "waverover_controller/uart_serial_port.hpp"

class WaveRoverController
{
public:
    WaveRoverController(const rclcpp::Node::SharedPtr &node);
    ~WaveRoverController();

private:
    const rclcpp::Node::SharedPtr& node;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher;

    std::unique_ptr<UARTSerialPort> serial_port;
    std::string port_name;
    int baud_rate;

    int update_data_period_ms;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    std::string getImuData();
    std::string getBatteryData();
    
    int get_wheels_speed_period_ms = 10;
    int publish_odometry_period_ms = 10;
    float left_wheel_speed_cm_s = 0;
    float right_wheel_speed_cm_s = 0;
    rclcpp::TimerBase::SharedPtr get_wheels_speed_timer;
    void getWheelsSpeed();

    double x_int, y_int, theta_int;
    rclcpp::Time last_time_odom = node->now();

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr publish_odom_timer;

    void publishOdometryData();

    rclcpp::TimerBase::SharedPtr update_data_timer;
    void updateData();

    std::map<std::string, int> pid_map = {
        {"P", 0},
        {"I", 0},
        {"D", 0},
        {"L", 0}
    };
    void updatePID();

    static std::string twistToCommand(const geometry_msgs::msg::Twist::SharedPtr msg);
};
