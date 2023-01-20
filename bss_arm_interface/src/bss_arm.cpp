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

// BSSArm::BSSArm() : Node("bss_arm_node")
// {
//   // Initialise Action Server
//   bss_arm_service = this->create_service<bss_interface::srv::MoveArm>(
//     "bss_arm_service", 
//     &service_callback
//     );

//   // Initialise Action Client
//   robot_simulation_client = this->create_client<bss_interface::srv::MoveArm>(
//     "move_arm_client"
//     );

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BSS Arm Node Ready.");
// }

// BSSArm::~BSSArm()
// {
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BSS Arm Node Terminated.");
// }

// void BSSArm()
// {
//   // Initialise Action Server
//   rclcpp::Service<bss_interface::srv::MoveArm>::SharedPtr bss_arm_service_ = this->create_service<bss_interface::srv::MoveArm>(
//     "bss_arm_service", 
//     &service_callback
//     );

//   // // Initialise Action Client
//   // rclcpp::Client<bss_interface::srv::MoveArm>::SharedPtr robot_simulation_client_ = rclcpp::create_client<bss_interface::srv::MoveArm>(
//   //   "move_arm_client"
//   //   );

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BSS Arm Node Ready.");
// }

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

  std::shared_ptr<rclcpp::Node> bss_arm_node = rclcpp::Node::make_shared("bss_arm_node");

  rclcpp::Service<bss_interface::srv::MoveArm>::SharedPtr bss_arm_service_ = bss_arm_node->create_service<bss_interface::srv::MoveArm>(
    "bss_arm_service", 
    &service_callback
    );

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BSS Arm Node Ready.");

  rclcpp::spin(bss_arm_node);
  rclcpp::shutdown();

  return 0;
}