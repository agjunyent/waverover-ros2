#include "cameras_controller/cameras_controller.hpp"

//==============================================================================
//				  Constructors
//==============================================================================

CamerasController::CamerasController(const rclcpp::Node::SharedPtr& node) :
    node(node)
{
    port_or_ip = waverover_helpers::setAndGetParameter<std::string>(node, "port_or_ip");
    params_fps = waverover_helpers::setAndGetParameter<int>(node, "fps");
    params_resolution = waverover_helpers::setAndGetParameter<std::vector<int64_t>>(node, "resolution");
    restart_seconds_no_frames = waverover_helpers::setAndGetParameter<float>(node, "restart_seconds_no_frames");

    std::array<int, 3> camera_resolution;
    std::copy_n(params_resolution.begin(), 3, camera_resolution.begin());

    attachCamera(port_or_ip, params_fps, camera_resolution);

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

void CamerasController::attachCamera(const std::string& port_or_ip, const int fps, const std::array<int, 3> resolution)
{
    RCLCPP_INFO_STREAM(node->get_logger(), "Attaching camera: " << port_or_ip);
    cameras.push_back(std::make_unique<CameraRGB>(node, port_or_ip, fps, resolution));
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
