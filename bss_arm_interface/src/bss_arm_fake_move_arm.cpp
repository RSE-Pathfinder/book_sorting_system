/**
 * @file bss_move_arm.cpp
 * @author Paul Yong Shao En (2102088@sit.singaporetech.edu.sg)
 * @brief Interface to bss_controller package that wraps around the ros2_RobotSimulation package
 * @version 0.1
 * @date 2023-01-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "bss_arm_interface/bss_arm.hpp"

void service_callback(
  const std::shared_ptr<bss_interface::srv::MoveArm::Request> request, 
  std::shared_ptr<bss_interface::srv::MoveArm::Response> response)
{
  response->error_code = request->row + request->col;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nRow: %ld" " Col: %ld", request->row, request->col);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Responding with Error Code: [%ld]", response->error_code);
}

int main (int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Initialise Node
  std::shared_ptr<rclcpp::Node> bss_arm_fake_move_arm_node = rclcpp::Node::make_shared("bss_arm_fake_move_arm_node");

  // Initialise Action Server
  rclcpp::Service<bss_interface::srv::MoveArm>::SharedPtr bss_arm_service_ = bss_arm_fake_move_arm_node->create_service<bss_interface::srv::MoveArm>(
    "move_arm_client", 
    &service_callback
    );

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BSS Arm Fake Move Arm Node Ready.");

  rclcpp::spin(bss_arm_fake_move_arm_node);

  rclcpp::shutdown();
  return 0;
}