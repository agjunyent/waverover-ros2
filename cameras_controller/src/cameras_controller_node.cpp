#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "cameras_controller/cameras_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    // options.allow_undeclared_parameters(true);
    // options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cameras_controller", options);

    CamerasController cameras_controller(node);

    rclcpp::executors::MultiThreadedExecutor exec;

    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
