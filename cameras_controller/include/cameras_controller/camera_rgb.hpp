#pragma once

#include <atomic>
#include <string>
#include <memory>
#include <iostream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cameras_controller/camera.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class CameraRGB final : public Camera
{
public:
    CameraRGB(const rclcpp::Node::SharedPtr& node, const std::string& port_or_ip, const int fps, const std::array<int, 3> resolution);
    ~CameraRGB();

    void shutdownCamera() override;
    void restartCamera() override;

private:
    sensor_msgs::msg::Image img_msg;
    void readFramesThread();

    std::string gstreamer_input;
    cv::VideoCapture cap;

    std::thread thread_read_frames;
    size_t memcpy_size;

    cv::Mat frame;
    rclcpp::Time timestamp;

    bool frameReady() override;
    sensor_msgs::msg::Image::SharedPtr getFrame() override;

    int getPortNumber() const;
};
