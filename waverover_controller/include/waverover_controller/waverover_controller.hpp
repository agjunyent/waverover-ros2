#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

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
    float left_wheel_speed = 0;
    float right_wheel_speed = 0;
    rclcpp::TimerBase::SharedPtr get_wheels_speed_timer;
    void getWheelsSpeed();

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
