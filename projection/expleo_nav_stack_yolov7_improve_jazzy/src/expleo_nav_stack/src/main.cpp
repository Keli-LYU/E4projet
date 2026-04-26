#include <memory>

#include "expleo_nav_stack/expleo_planner_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<expleo_nav_stack::ExpleoPlannerServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
