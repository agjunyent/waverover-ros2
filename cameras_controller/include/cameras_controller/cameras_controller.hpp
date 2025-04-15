#pragma once

#include <string>
#include <memory>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "waverover_helpers/helpers.hpp"
#include "cameras_controller/camera_rgb.hpp"
#include "cameras_controller/camera_thm.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CamerasController
{
public:

    CamerasController(const rclcpp::Node::SharedPtr& node);
    ~CamerasController();

    void attachCamera(const std::string& camera_type, const std::string& port_or_ip, const int fps, const std::array<int, 3> resolution);

private:
    const rclcpp::Node::SharedPtr& node;

    std::vector<std::unique_ptr<Camera>> cameras;

    std::string port_or_ip;
    int params_fps;
    std::vector<int64_t> params_resolution;

    float restart_seconds_no_frames;
    rclcpp::TimerBase::SharedPtr watchdog_cameras;
    void watchdogCamerasCallback();
};
