/**
 * @file bss_arm.hpp
 * @author Paul Yong Shao En (2102088@sit.singaporetech.edu.sg)
 * @brief Interface to bss_controller package that wraps around the ros2_RobotSimulation package
 * @version 0.1
 * @date 2023-01-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef BSS_ARM_INTERFACE__BSS_ARM_HPP_
#define BSS_ARM_INTERFACE__BSS_ARM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "bss_interface/srv/move_arm.hpp"
/* Header file for Robot Simulation */

#include <chrono> // 1s

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// class BSSArm : public rclcpp::Node
// {
// public:
//   BSSArm();
//   ~BSSArm();

// private:
//   // ROS Action Servers
//   rclcpp::Service<bss_interface::srv::MoveArm>::SharedPtr bss_arm_service_;

//   // // ROS Action Clients
//   // rclcpp::Client<bss_interface::srv::MoveArm>::SharedPtr robot_simulation_client_;

//   // ROS Messages
//   /* Service Type for Robot Simulation */

//   // Function Prototypes
//   void service_callback(
//     const std::shared_ptr<bss_interface::srv::MoveArm::Request> move_arm_rqt,
//     std::shared_ptr<bss_interface::srv::MoveArm::Response> move_arm_rsp);
// };
#endif  // BSS_ARM_INTERFACE__BSS_ARM_HPP_
