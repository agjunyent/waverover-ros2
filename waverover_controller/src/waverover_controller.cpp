#include "rclcpp/rclcpp.hpp"

#include "waverover_controller/waverover_controller.hpp"

WaveRoverController::WaveRoverController(const rclcpp::Node::SharedPtr& node) :
    node(node), x_int(0.0), y_int(0.0), theta_int(0.0)
{
    port_name = waverover_helpers::setAndGetParameter<std::string>(node, "port_name");
    baud_rate = waverover_helpers::setAndGetParameter<int>(node, "baud_rate");
    update_imu_period_ms = waverover_helpers::setAndGetParameter<int>(node, "update_imu_period_ms");
    update_battery_period_ms = waverover_helpers::setAndGetParameter<int>(node, "update_battery_period_ms");
    publish_odometry_period_ms = waverover_helpers::setAndGetParameter<int>(node, "publish_odometry_period_ms");

    pid_map = waverover_helpers::setAndGetParameters<int>(node, "pid", pid_map);
    imu_offsets = waverover_helpers::setAndGetParameters<double>(node, "imu_offsets", imu_offsets);

    serial_port = std::make_unique<UARTSerialPort>(port_name, baud_rate);
    if (!serial_port->isOpen())
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to open serial port: " << port_name);
        return;
    }

    updatePID();

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    cmd_vel_subscriber = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&WaveRoverController::cmdVelCallback, this, std::placeholders::_1));

    odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    battery_publisher = node->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);

    publish_odom_timer = node->create_wall_timer(std::chrono::milliseconds(publish_odometry_period_ms), std::bind(&WaveRoverController::publishOdometryData, this));
    update_battery_timer = node->create_wall_timer(std::chrono::milliseconds(update_battery_period_ms), std::bind(&WaveRoverController::updateBatteryData, this));
    update_imu_timer = node->create_wall_timer(std::chrono::milliseconds(update_imu_period_ms), std::bind(&WaveRoverController::updateImuData, this));
}

WaveRoverController::~WaveRoverController()
{
    update_battery_timer->cancel();
    update_imu_timer->cancel();
    publish_odom_timer->cancel();
}

void WaveRoverController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Convert the Twist message to a string command
    std::string command = WaveRoverController::twistToCommand(msg);

    std::cout << "Command: " << command << std::endl;

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
}

void WaveRoverController::publishOdometryData()
{
    static auto odom_msg = nav_msgs::msg::Odometry();
    static geometry_msgs::msg::TransformStamped tf_msg;
    static tf2::Quaternion q;

    getWheelsSpeed();

    double v_l = left_wheel_speed;
    double v_r = right_wheel_speed;

    double v = (v_r + v_l) / 2.0;
    double omega = (v_r - v_l) / 0.125;  // rad/s

    rclcpp::Time now = node->now();
    double dt = (now - last_time_odom).seconds();
    last_time_odom = now;

    // Integrate to update pose
    double delta_x = v * cos(theta_int) * dt;
    double delta_y = v * sin(theta_int) * dt;
    double delta_theta = omega * dt;

    x_int += delta_x;
    y_int += delta_y;
    theta_int += delta_theta;

    // Publish odometry message
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = x_int;
    odom_msg.pose.pose.position.y = y_int;
    odom_msg.pose.pose.position.z = 0.0;

    q.setRPY(0, 0, theta_int);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.angular.z = omega;

    odom_publisher->publish(odom_msg);

    // Broadcast the transform
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = x_int;
    tf_msg.transform.translation.y = y_int;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = tf2::toMsg(q);

    tf_broadcaster->sendTransform(tf_msg);
}

void WaveRoverController::updateImuData()
{
    std::string imu_data = getImuData();
    nlohmann::json imu_json = nlohmann::json::parse(imu_data);

    // Publish the IMU data
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = node->now();
    imu_msg.linear_acceleration.x = static_cast<double>(imu_json["ax"]) - imu_offsets["ax"];
    imu_msg.linear_acceleration.y = static_cast<double>(imu_json["ay"]) - imu_offsets["ay"];
    imu_msg.linear_acceleration.z = static_cast<double>(imu_json["az"]) - imu_offsets["az"];
    imu_msg.angular_velocity.x = static_cast<double>(imu_json["gx"]) - imu_offsets["gx"];
    imu_msg.angular_velocity.y = static_cast<double>(imu_json["gy"]) - imu_offsets["gy"];
    imu_msg.angular_velocity.z = static_cast<double>(imu_json["gz"]) - imu_offsets["gz"];
    tf2::Quaternion q;
    q.setRPY(0, 0, imu_json["y"]);
    imu_msg.orientation = tf2::toMsg(q);
    imu_publisher->publish(imu_msg);
}

void WaveRoverController::updateBatteryData()
{
    std::string battery_data = getBatteryData();
    nlohmann::json battery_json = nlohmann::json::parse(battery_data);

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
