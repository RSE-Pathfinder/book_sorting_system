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

  // Initialise Node
  std::shared_ptr<rclcpp::Node> bss_arm_node = rclcpp::Node::make_shared("bss_arm_node");

  // Initialise Action Server
  rclcpp::Service<bss_interface::srv::MoveArm>::SharedPtr bss_arm_service_ = bss_arm_node->create_service<bss_interface::srv::MoveArm>(
    "bss_arm_service", 
    &service_callback
    );

  // Initialise Action Client
  rclcpp::Client<bss_interface::srv::MoveArm>::SharedPtr robot_simulation_client_ = bss_arm_node->create_client<bss_interface::srv::MoveArm>(
    "move_arm_client"
    );

  // Wait for Action Client to be Active
  while (!robot_simulation_client_->wait_for_service(std::literals::chrono_literals::operator""s(1))) 
  {
    if (!rclcpp::ok()) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BSS Arm Node Ready.");

  // Initialise Action Client Requst Message
  auto request = std::make_shared<bss_interface::srv::MoveArm::Request>();
  request->row = 0;
  request->col = 1;

  // Initialise Action Client Result Message
  auto result = robot_simulation_client_->async_send_request(request);

  // Wait for the Action Client Result
  if (rclcpp::spin_until_future_complete(bss_arm_node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error Code: %ld", result.get()->error_code);
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::spin(bss_arm_node);

  rclcpp::shutdown();
  return 0;
}