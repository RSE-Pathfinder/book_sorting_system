/**
 * @file scout_virtual_node.cpp
 * @author Muhammad Syamim (Syazam33@gmail.com)
 * @brief Virtual Node for Weston Robot SCOUT in Gazebo
 * @version 0.1
 * @date 2023-02-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "wrp_ros2/mobile_base/mobile_base_node.hpp"

#include "wrp_sdk/mobile_base/westonrobot/mobile_base.hpp"
#include "wrp_sdk/mobile_base/agilex/agilex_base_v2_adapter.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#ifdef TF2_CPP_HEADERS
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
using namespace std::placeholders;

namespace westonrobot {

MobileBaseNode::MobileBaseNode(const rclcpp::NodeOptions& options)
    : Node("mobile_base_node", options) {
  if (!MobileBaseNode::ReadParameters()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load parameters");
    rclcpp::shutdown();
  }

  if (!MobileBaseNode::SetupRobot()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to setup robot");
    rclcpp::shutdown();
  }

  if (!MobileBaseNode::SetupInterfaces()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to setup interfaces");
    rclcpp::shutdown();
  }
}

bool MobileBaseNode::ReadParameters() {
  // Declare default parameters
  this->declare_parameter<std::string>("can_device", "can0");
  this->declare_parameter<std::string>("robot_type", "weston");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<bool>("auto_reconnect", true);
  this->declare_parameter<bool>("publish_odom", true);
  this->declare_parameter<std::string>("motion_type", "skid_steer");

  // Get parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "--- Parameters loaded are ---");

  this->get_parameter("robot_type", robot_type_);
  RCLCPP_INFO_STREAM(this->get_logger(), "robot_type: " << robot_type_);

  this->get_parameter("can_device", can_device_);
  RCLCPP_INFO_STREAM(this->get_logger(), "can_device: " << can_device_);

  this->get_parameter("base_frame", base_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "base_frame: " << base_frame_);

  this->get_parameter("odom_frame", odom_frame_);
  RCLCPP_INFO_STREAM(this->get_logger(), "odom_frame: " << odom_frame_);

  this->get_parameter("auto_reconnect", auto_reconnect_);
  RCLCPP_INFO_STREAM(this->get_logger(), "auto_reconnect: " << auto_reconnect_);

  this->get_parameter("publish_odom", publish_odom_);
  RCLCPP_INFO_STREAM(this->get_logger(), "publish_odom: " << publish_odom_);

  this->get_parameter("motion_type", motion_type_);
  RCLCPP_INFO_STREAM(this->get_logger(), "motion_type: " << motion_type_);

  RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------");

  return true;
}

bool MobileBaseNode::SetupRobot() {
  // Create appropriate adaptor
  if (robot_type_ == "weston") {
    robot_ = std::make_shared<MobileBase>();
  } else if (robot_type_ == "agilex") {
    robot_ = std::make_shared<AgilexBaseV2Adapter>();
  } else if (robot_type_ == "vbot") {
    robot_ = std::make_shared<MobileBase>(true);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Unknown robot base type\n Supported are \"weston\", "
                        "\"agilex\" & \"vbot\"");
    return false;
  }

    RCLCPP_INFO_STREAM(this->get_logger(), "Skipping Port Connection for virtual node\n");

  // Connect to robot through can device
//   if (!robot_->Connect(can_device_)) {
//     RCLCPP_ERROR_STREAM(
//         this->get_logger(),
//         "Failed to connect to robot through port: " << can_device_);
//     return false;
//   }

  last_time_ = this->now();
  return true;
}

bool MobileBaseNode::SetupInterfaces() {
  // setup subscribers
  motion_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 5, std::bind(&MobileBaseNode::MotionCmdCallback, this, _1));

  // setup publishers
  system_state_publisher_ =
      this->create_publisher<wrp_ros2::msg::SystemState>("~/system_state", 10);
  motion_state_publisher_ =
      this->create_publisher<wrp_ros2::msg::MotionState>("~/motion_state", 10);
  actuator_state_publisher_ =
      this->create_publisher<wrp_ros2::msg::ActuatorStateArray>(
          "~/actuator_state", 10);
  battery_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state",
                                                             10);

  if (publish_odom_) {
    // setup tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("~/odom", 50);
  }
  ultrasonic_data_publisher_ =
      this->create_publisher<wrp_ros2::msg::RangeDataArray>("~/ultrasonic_data",
                                                            10);
  tof_data_publisher_ =
      this->create_publisher<wrp_ros2::msg::RangeDataArray>("~/tof_data", 10);

  // setup services
  access_control_service_ = this->create_service<wrp_ros2::srv::AccessControl>(
      "~/access_control",
      std::bind(&MobileBaseNode::AccessControlCallback, this, _1, _2));
  assisted_mode_control_service_ =
      this->create_service<wrp_ros2::srv::AssistedModeControl>(
          "~/assisted_mode_control",
          std::bind(&MobileBaseNode::AssistedModeControlCallback, this, _1,
                    _2));
  light_control_service_ = this->create_service<wrp_ros2::srv::LightControl>(
      "~/light_control",
      std::bind(&MobileBaseNode::LightControlCallback, this, _1, _2));
  motion_reset_service_ = this->create_service<wrp_ros2::srv::MotionReset>(
      "~/motion_reset",
      std::bind(&MobileBaseNode::MotionResetCallback, this, _1, _2));

  // setup timers
  publish_timer_ = this->create_wall_timer(  // 50hz loop rate
      std::chrono::milliseconds(20),
      std::bind(&MobileBaseNode::PublishLoopCallback, this));

  return true;
}

void MobileBaseNode::MotionCmdCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  MotionCommand cmd;

  cmd.linear.x = msg->linear.x;
  cmd.linear.y = msg->linear.y;
  cmd.linear.z = msg->linear.z;
  cmd.angular.x = msg->angular.x;
  cmd.angular.y = msg->angular.y;
  cmd.angular.z = msg->angular.z;

  if (robot_->SdkHasControlToken()) {
    robot_->SetMotionCommand(cmd);
  }
}

void MobileBaseNode::AccessControlCallback(
    const wrp_ros2::srv::AccessControl::Request::SharedPtr request,
    wrp_ros2::srv::AccessControl::Response::SharedPtr response) {
  HandshakeReturnCode result;
  switch (request->action_type) {
    case wrp_ros2::srv::AccessControl::Request::ACTION_TYPE_REQUEST_CONTROL: {
      result = robot_->RequestControl();
      response->result_code = static_cast<uint32_t>(result);
      break;
    }
    case wrp_ros2::srv::AccessControl::Request::ACTION_TYPE_RENOUNCE_CONTROL: {
      result = robot_->RenounceControl();
      response->result_code = static_cast<uint32_t>(result);
      break;
    }

    default: {
      response->result_code = 11;
      break;
    }
  }
  return;
}

void MobileBaseNode::AssistedModeControlCallback(
    const wrp_ros2::srv::AssistedModeControl::Request::SharedPtr request,
    wrp_ros2::srv::AssistedModeControl::Response::SharedPtr response) {
  AssistedModeSetCommand cmd;
  cmd.enable = request->enable;
  /**ATTN: Should we check for token or change the interface here?
   * If multiple user attempts to change, each one might have a different
   * thinking if assisted mode is on or off.
   */
  robot_->SetAssistedMode(cmd);
  response->state = request->enable;
  return;
}

void MobileBaseNode::LightControlCallback(
    const wrp_ros2::srv::LightControl::Request::SharedPtr request,
    wrp_ros2::srv::LightControl::Response::SharedPtr response) {
  LightCommand cmd;

  cmd.id = request->id;
  cmd.command.mode = static_cast<LightMode>(request->command.mode);
  cmd.command.intensity = request->command.intensity;

  if (robot_->SdkHasControlToken()) {
    robot_->SetLightCommand(cmd);
  }
  auto light_state = robot_->GetLightState();
  response->state.mode = static_cast<uint32_t>(light_state.state.mode);
  response->state.intensity = light_state.state.intensity;
  return;
}

void MobileBaseNode::MotionResetCallback(
    const wrp_ros2::srv::MotionReset::Request::SharedPtr request,
    wrp_ros2::srv::MotionReset::Response::SharedPtr response) {
  MotionResetCommand cmd;

  cmd.type = static_cast<MotionResetCommandType>(request->type);

  if (robot_->SdkHasControlToken()) {
    robot_->SetMotionResetCommand(cmd);
    response->result_code =
        wrp_ros2::srv::MotionReset::Response::MOTION_RESET_SUCCCESS;
  } else {
    response->result_code =
        wrp_ros2::srv::MotionReset::Response::MOTION_RESET_FAILURE;
  }

  return;
}

void MobileBaseNode::PublishLoopCallback() {
  if (auto_reconnect_ && !robot_->SdkHasControlToken()) {
    auto return_code = robot_->RequestControl();
    if (return_code != HandshakeReturnCode::kControlAcquired) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Failed to gain control token, error code: "
                             << static_cast<int>(return_code));
    }
  }
  PublishRobotState();
  PublishSensorData();
  if (publish_odom_) {
    PublishWheelOdometry();
  }
}

void MobileBaseNode::PublishRobotState() {
  auto system_state = robot_->GetSystemState();
  auto motion_state = robot_->GetMotionState();
  auto actuator_state = robot_->GetActuatorState();
  auto battery_state = robot_->GetBatteryState();

  // system state
  wrp_ros2::msg::SystemState system_state_msg;
  system_state_msg.rc_connected = system_state.rc_connected;
  system_state_msg.error_code = static_cast<uint32_t>(system_state.error_code);
  system_state_msg.operational_state =
      static_cast<uint32_t>(system_state.operational_state);
  system_state_msg.control_state =
      static_cast<uint32_t>(system_state.control_state);

  system_state_publisher_->publish(system_state_msg);

  // motion state
  wrp_ros2::msg::MotionState motion_state_msg;
  motion_state_msg.desired_linear.x = motion_state.desired_linear.x;
  motion_state_msg.desired_linear.y = motion_state.desired_linear.y;
  motion_state_msg.desired_linear.z = motion_state.desired_linear.z;
  motion_state_msg.desired_angular.x = motion_state.desired_angular.x;
  motion_state_msg.desired_angular.y = motion_state.desired_angular.y;
  motion_state_msg.desired_angular.z = motion_state.desired_angular.z;

  motion_state_msg.source = static_cast<uint32_t>(motion_state.source);
  motion_state_msg.collision_detected = motion_state.collision_detected;
  motion_state_msg.assisted_mode_enabled = motion_state.assisted_mode_enabled;

  motion_state_publisher_->publish(motion_state_msg);

  // actuator state
  wrp_ros2::msg::ActuatorStateArray actuator_state_msg;
  for (size_t i = 0; i < actuator_state.size(); ++i) {
    wrp_ros2::msg::ActuatorState actuator_msg;
    actuator_msg.id = actuator_state[i].id;
    actuator_msg.motor.rpm = actuator_state[i].motor.rpm;
    actuator_msg.motor.current = actuator_state[i].motor.current;
    actuator_msg.motor.pulse_count = actuator_state[i].motor.pulse_count;
    actuator_msg.driver.driver_voltage =
        actuator_state[i].driver.driver_voltage;
    actuator_msg.driver.driver_temperature =
        actuator_state[i].driver.driver_temperature;
    actuator_msg.driver.motor_temperature =
        actuator_state[i].driver.motor_temperature;
    actuator_msg.driver.driver_state = actuator_state[i].driver.driver_state;
    actuator_state_msg.states.push_back(actuator_msg);
  }
  actuator_state_publisher_->publish(actuator_state_msg);

  // battery state
  sensor_msgs::msg::BatteryState battery_state_msg;
  battery_state_msg.header.stamp = this->now();
  battery_state_msg.voltage = battery_state.voltage;
  battery_state_msg.current = std::numeric_limits<float>::quiet_NaN();
  battery_state_msg.charge = std::numeric_limits<float>::quiet_NaN();
  battery_state_msg.capacity = std::numeric_limits<float>::quiet_NaN();
  battery_state_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
  battery_state_msg.percentage = battery_state.percentage / 100.0f;
  battery_state_msg.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_state_msg.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_state_msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_state_msg.present = battery_state.present;
  battery_state_publisher_->publish(battery_state_msg);
}

void MobileBaseNode::PublishSensorData() {
  // TODO: bumper publishing?
  auto ultrasonic_data = robot_->GetUltrasonicData();
  auto tof_data = robot_->GetTofData();

  // ultrasonic data
  wrp_ros2::msg::RangeDataArray ultrasonic_data_msg;
  ultrasonic_data_msg.type =
      wrp_ros2::msg::RangeDataArray::RANGE_SENSOR_TYPE_ULTRASONIC;
  for (size_t i = 0; i < ultrasonic_data.size(); i++) {
    wrp_ros2::msg::RangeData range_data;
    range_data.id = i;
    range_data.field_of_view = ultrasonic_data[i].field_of_view;
    range_data.min_range = ultrasonic_data[i].min_range;
    range_data.max_range = ultrasonic_data[i].max_range;
    range_data.range = ultrasonic_data[i].range;
    ultrasonic_data_msg.data.push_back(range_data);
  }
  ultrasonic_data_publisher_->publish(ultrasonic_data_msg);

  // tof data
  wrp_ros2::msg::RangeDataArray tof_data_msg;
  tof_data_msg.type = wrp_ros2::msg::RangeDataArray::RANGE_SENSOR_TYPE_TOF;
  for (size_t i = 0; i < tof_data.size(); i++) {
    wrp_ros2::msg::RangeData range_data;
    range_data.id = i;
    range_data.field_of_view = tof_data[i].field_of_view;
    range_data.min_range = tof_data[i].min_range;
    range_data.max_range = tof_data[i].max_range;
    range_data.range = tof_data[i].range;
    tof_data_msg.data.push_back(range_data);
  }
  tof_data_publisher_->publish(tof_data_msg);
}

void MobileBaseNode::PublishWheelOdometry() {
  // TODO calculate odometry according to robot type
  auto robot_odom = robot_->GetOdometry();

  // ATTN: odometry directly from wrp_sdk still in progress
  geometry_msgs::msg::Twist robot_twist;
  robot_twist.linear.x = robot_odom.linear.x;
  robot_twist.linear.y = robot_odom.linear.y;
  robot_twist.linear.z = robot_odom.linear.z;
  robot_twist.angular.x = robot_odom.angular.x;
  robot_twist.angular.y = robot_odom.angular.y;
  robot_twist.angular.z = robot_odom.angular.z;

  nav_msgs::msg::Odometry odom_msg =
      MobileBaseNode::CalculateOdometry(robot_twist);

  // publish tf transformation
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = odom_msg.header.stamp;
  tf_msg.header.frame_id = odom_msg.header.frame_id;
  tf_msg.child_frame_id = odom_msg.child_frame_id;

  tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
  tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(tf_msg);
  odom_publisher_->publish(odom_msg);
}

nav_msgs::msg::Odometry MobileBaseNode::CalculateOdometry(
    geometry_msgs::msg::Twist robot_twist) {
  auto current_time = this->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // TODO: perform calculation based on robot type & wheel base other than scout
  // & scout mini
  double linear_speed = robot_twist.linear.x;
  double angular_speed = robot_twist.angular.z;
  double lateral_speed = robot_twist.linear.y;

  double d_x =
      (linear_speed * std::cos(theta_) - lateral_speed * std::sin(theta_)) * dt;
  double d_y =
      (linear_speed * std::sin(theta_) + lateral_speed * std::cos(theta_)) * dt;
  double d_theta = angular_speed * dt;

  position_x_ += d_x;
  position_y_ += d_y;
  theta_ += d_theta;

  geometry_msgs::msg::Quaternion odom_quat =
      MobileBaseNode::CreateQuaternionMsgFromYaw(theta_);

  // publish odometry and tf messages
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = linear_speed;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_speed;

  return odom_msg;
}

geometry_msgs::msg::Quaternion MobileBaseNode::CreateQuaternionMsgFromYaw(
    double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

}  // namespace westonrobot

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<westonrobot::MobileBaseNode>());
  rclcpp::shutdown();
  return 0;
}