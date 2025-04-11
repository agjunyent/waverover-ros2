#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "waverover_controller/waverover_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("waverover_controller");

    WaveRoverController waverover_controller(node);

    rclcpp::executors::MultiThreadedExecutor exec;

    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
