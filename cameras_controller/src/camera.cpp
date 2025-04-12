#include "cameras_controller/camera.hpp"

//==============================================================================
//				  Constructors
//==============================================================================

Camera::Camera(const rclcpp::Node::SharedPtr& node, const std::string& port_or_ip, const int fps, const std::array<int, 3> resolution) :
    node(node), port_or_ip(port_or_ip), fps(fps), resolution(resolution), it(node)
{
    last_frame_time = node->now();

    pub_frame = it.advertise("camera_frame", 1);

    timer_publish_camera_frame = node->create_wall_timer(std::chrono::duration<float>(0.01), std::bind(&Camera::publishCameraFrameTimer, this));
}

//==============================================================================
//				  Destructors
//==============================================================================

Camera::~Camera()
{

}

void Camera::publishCameraFrameTimer()
{
    if (frameReady())
    {
        auto frame_msg = getFrame();
        pub_frame.publish(frame_msg);
    }
}

void Camera::readNewFrame()
{
    last_frame_time = node->now();
}

float Camera::secondsFromLastFrame() const
{
    return (node->now() - last_frame_time).seconds();
}
