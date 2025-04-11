#include "rclcpp/rclcpp.hpp"

#include "waverover_controller/waverover_controller.hpp"

WaveRoverController::WaveRoverController(const rclcpp::Node::SharedPtr& node) :
    node(node)
{
    port_name = WaveRoverHelpers::setAndGetParameter<std::string>(node, "port_name");
    baud_rate = WaveRoverHelpers::setAndGetParameter<int>(node, "baud_rate");
    update_data_period_ms = WaveRoverHelpers::setAndGetParameter<int>(node, "update_data_period_ms");
    get_wheels_speed_period_ms = WaveRoverHelpers::setAndGetParameter<int>(node, "get_wheels_speed_period_ms");

    pid_map = WaveRoverHelpers::setAndGetParameters<int>(node, "pid", pid_map);

    serial_port = std::make_unique<UARTSerialPort>(port_name, baud_rate);
    if (!serial_port->isOpen())
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to open serial port: " << port_name);
        return;
    }

    updatePID();

    cmd_vel_subscriber = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&WaveRoverController::cmdVelCallback, this, std::placeholders::_1));

    imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    battery_publisher = node->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);

    update_data_timer = node->create_wall_timer(std::chrono::milliseconds(update_data_period_ms), std::bind(&WaveRoverController::updateData, this));
    get_wheels_speed_timer = node->create_wall_timer(std::chrono::milliseconds(get_wheels_speed_period_ms), std::bind(&WaveRoverController::getWheelsSpeed, this));
}

WaveRoverController::~WaveRoverController()
{
    update_data_timer->cancel();
}

void WaveRoverController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Convert the Twist message to a string command
    std::string command = WaveRoverController::twistToCommand(msg);

    // Send the command to the serial port
    serial_port->sendRequestSync(command);
}

std::string WaveRoverController::getImuData()
{
    std::string imu_data;
    nlohmann::json message_json = {};
    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::IMU_INFO;
    serial_port->getResponseSync(message_json.dump(), imu_data);
    return imu_data;
}

std::string WaveRoverController::getBatteryData()
{
    std::string battery_data;
    nlohmann::json message_json = {};
    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::BATTERY_INFO;
    serial_port->getResponseSync(message_json.dump(), battery_data);
    return battery_data;
}

void WaveRoverController::getWheelsSpeed()
{
    std::string wheels_speed_data;
    nlohmann::json message_json = {};
    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::WHEELS_SPEED;
    serial_port->getResponseSync(message_json.dump(), wheels_speed_data);
    nlohmann::json wheels_speed_json = nlohmann::json::parse(wheels_speed_data);
    left_wheel_speed = wheels_speed_json["L"];
    right_wheel_speed = wheels_speed_json["R"];
    std::cout << "Left wheel speed: " << left_wheel_speed << ", Right wheel speed: " << right_wheel_speed << std::endl;
}

void WaveRoverController::updateData()
{
    std::string imu_data = getImuData();
    std::string battery_data = getBatteryData();
    nlohmann::json imu_json = nlohmann::json::parse(imu_data);
    nlohmann::json battery_json = nlohmann::json::parse(battery_data);

    // Publish the IMU data
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = node->now();
    imu_msg.linear_acceleration.x = imu_json["ax"];
    imu_msg.linear_acceleration.y = imu_json["ay"];
    imu_msg.linear_acceleration.z = imu_json["az"];
    imu_msg.angular_velocity.x = imu_json["gx"];
    imu_msg.angular_velocity.y = imu_json["gy"];
    imu_msg.angular_velocity.z = imu_json["gz"];
    imu_msg.orientation.x = imu_json["r"]; // ORIENTATION HAS ROLL PITCH YAW INSTEAD OF QUATERNION
    imu_msg.orientation.y = imu_json["p"];
    imu_msg.orientation.z = imu_json["y"];
    imu_publisher->publish(imu_msg);

    // Publish the battery data
    auto battery_msg = sensor_msgs::msg::BatteryState();
    battery_msg.header.stamp = node->now();
    battery_msg.voltage = battery_json["v"];
    battery_msg.current = static_cast<float>(battery_json["mA"]) / 1000.0; // Convert mA to A
    battery_publisher->publish(battery_msg);
}

void WaveRoverController::updatePID()
{
    nlohmann::json message_json = {};
    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::PID_SET;
    message_json["P"] = pid_map["P"];
    message_json["I"] = pid_map["I"];
    message_json["D"] = pid_map["D"];
    message_json["L"] = pid_map["L"];
    serial_port->sendRequestSync(message_json.dump());
}

std::string WaveRoverController::twistToCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    nlohmann::json message_json = {};

    // Cap values at [-1 .. 1]
    float x = std::max(std::min(msg->linear.x, 1.0), -1.0);
    float z = std::max(std::min(msg->angular.z, 1.0), -1.0);

    message_json["T"] = WAVE_ROVER_COMMAND_TYPE::ROS_CTRL;
    message_json["X"] = x;
    message_json["Z"] = z;
    return message_json.dump();
}
