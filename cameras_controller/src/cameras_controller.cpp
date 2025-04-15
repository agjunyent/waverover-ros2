#include "cameras_controller/cameras_controller.hpp"

//==============================================================================
//				  Constructors
//==============================================================================

CamerasController::CamerasController(const rclcpp::Node::SharedPtr& node) :
    node(node)
{
    std::vector<std::string> camera_types = waverover_helpers::setAndGetParameter<std::vector<std::string>>(node, "camera_types");

    for (const auto& camera_type : camera_types)
    {
        port_or_ip = waverover_helpers::setAndGetParameter<std::string>(node, camera_type + ".port_or_ip");
        params_fps = waverover_helpers::setAndGetParameter<int>(node, camera_type + ".fps");
        params_resolution = waverover_helpers::setAndGetParameter<std::vector<int64_t>>(node, camera_type + ".resolution");
        restart_seconds_no_frames = waverover_helpers::setAndGetParameter<float>(node, camera_type + ".restart_seconds_no_frames");

        std::array<int, 3> camera_resolution;
        std::copy_n(params_resolution.begin(), 3, camera_resolution.begin());
        attachCamera(camera_type, port_or_ip, params_fps, camera_resolution);
    }

    watchdog_cameras = node->create_wall_timer(
        std::chrono::seconds(30),
        std::bind(&CamerasController::watchdogCamerasCallback, this)
    );
}

//==============================================================================
//				  Destructors
//==============================================================================

CamerasController::~CamerasController()
{
    for (const auto& camera : cameras)
    {
        camera->shutdownCamera();
    }
}

//==============================================================================
//			   Private methods
//==============================================================================

void CamerasController::attachCamera(const std::string& camera_type, const std::string& port_or_ip, const int fps, const std::array<int, 3> resolution)
{
    RCLCPP_INFO_STREAM(node->get_logger(), "Attaching camera: " << port_or_ip);
    if (camera_type == "rgb")
    {
        cameras.push_back(std::make_unique<CameraRGB>(node, port_or_ip, fps, resolution));
    }
    else if (camera_type == "thm")
    {
        cameras.push_back(std::make_unique<CameraTHM>(node, port_or_ip, fps, resolution));
    }
    else
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Camera type not recognized: " << camera_type);
    }
}

void CamerasController::watchdogCamerasCallback()
{
    for (const auto& camera : cameras)
    {
        if (camera->secondsFromLastFrame() > restart_seconds_no_frames)
        {
            RCLCPP_INFO_STREAM(node->get_logger(), "Watchdog: Restarting Camera!");
            camera->restartCamera();
        }
    }
}
