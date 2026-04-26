// =====================================================================================
//
//       Filename:  twist_to_regulated_control_node.cpp
//
//    Description:  Node entry point to control the vehicle in carla from twist
//    messages
//
//        Version:  1.0
//        Created:  11/10/2022 15:55:00
//       Revision:  none
//       Compiler:  g++
//
//         Author:  Thibaud Dubautbout (TD), thibaud.duhautbout@expleogroup.com
//   Organization:  Expleo Group
//
// =====================================================================================
#include "carla_twist_to_regulated_control/TwistToRegulatedControl.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistToRegulatedControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
