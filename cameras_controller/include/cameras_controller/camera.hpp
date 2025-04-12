#pragma once

#include <string>
#include <memory>
#include <iostream>
#include <thread>
#include <atomic>
#include <fstream>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Camera
{
public:
    Camera(const rclcpp::Node::SharedPtr& node, const std::string& port_or_ip, const int fps, const std::array<int, 3> resolution);
    ~Camera();

    virtual void shutdownCamera() = 0;
    virtual void restartCamera() = 0;

    float secondsFromLastFrame() const;

protected:
    const rclcpp::Node::SharedPtr& node;

    const std::string port_or_ip;
    const int fps;
    const std::array<int, 3> resolution;

    std::atomic<bool> is_reading;
    std::atomic<bool> frame_ready;
    virtual bool frameReady() = 0;
    sensor_msgs::msg::Image frame;

    virtual sensor_msgs::msg::Image::SharedPtr getFrame() = 0;

    void readNewFrame();

private:
    rclcpp::Time last_frame_time;

    rclcpp::TimerBase::SharedPtr timer_publish_camera_frame;

    image_transport::ImageTransport it;
    image_transport::Publisher pub_frame;
    void publishCameraFrameTimer();
};
