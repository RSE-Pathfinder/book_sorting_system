/**
 * @file scout_base.cpp
 * @author Muhammad Syamim (Syazam33@gmail.com)
 * @brief Class object for Weston robot SCOUT V2 using wrp_sdk ver.0.8.7
 * @version 0.1
 * @date 2023-02-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//Standard Library
#include <memory>
#include <atomic>
#include <string>
#include <chrono>
#include <vector>
#include <utility>

//ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <scout_demo_node/msg/system_state.hpp>

//Found in folder "/opt/weston_robot/include"
#include "wrp_sdk/mobile_base/westonrobot/mobile_base.hpp"

//Defaults
using namespace std::placeholders;
using namespace std::chrono_literals;
namespace DEFAULTS{
    static const char* CMD_VEL_TOPIC = "/scout/cmd_vel";
    static const float ODOM_SCALER = 0.6;
};

//Function Prototypes
void ControlLostCallback(void);

//Global Variables
std::atomic<bool> has_control_token;

//Scout wrapper object
class scout_node : public rclcpp::Node, westonrobot::MobileBase{
    public:
    //Default Constructor
    scout_node(std::string device_name) :
        Node("Scout_Base_Node"), device_name(device_name), linear_goal({0,0,0}), angular_goal({0,0,0}){
        //Initialise Node variables
        has_control_token = false;
        last_time = now();
        RegisterLoseControlCallback(ControlLostCallback);

        //Connect to CAN bus
        Connect(device_name);
        RCLCPP_INFO(get_logger(), "Connected to %s", device_name.c_str());

        //Setup Status Streams
        //Verify update rate
        system_state_pub = create_publisher<scout_demo_node::msg::SystemState>("scout/SystemState", 10);
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("scout/odom", 10);
        system_state_timer = create_wall_timer(100ms, std::bind(&scout_node::system_state_callback, this));
        motion_state_timer = create_wall_timer(10ms, std::bind(&scout_node::motion_state_callback, this));

        //Setup Command Stream
        cmd_vel_topic = create_subscription<geometry_msgs::msg::Twist>(
            DEFAULTS::CMD_VEL_TOPIC, 10, std::bind(&scout_node::cmd_vel_callback, this, _1));
        RCLCPP_INFO(get_logger(), "Listening to twist commands from '%s'", DEFAULTS::CMD_VEL_TOPIC);

        //Request Base Control
        RequestControlToken();
        RCLCPP_INFO(get_logger(), "Gained Control Token!");
    }

    //Private Class Functions
    private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_msg) {
        RCLCPP_INFO(get_logger(), "Received linear X: %.2f, angular Z: %.2f", cmd_msg->linear.x, cmd_msg->angular.z);
        linear_goal.x = cmd_msg->linear.x;
        angular_goal.z = cmd_msg->angular.z;
        cmd_loop();
    }

    /**
     * @brief Callback function for system status information
     * 
     */
    void system_state_callback(){
        scout_demo_node::msg::SystemState system_state_msg;
        system_state = GetSystemState();
        system_state_msg.rc_connected = system_state.rc_connected;
        system_state_msg.control_state = system_state.ctrl_state;
        system_state_msg.operational_state = system_state.operational_state;
        system_state_msg.error_code = system_state.error_code;
        system_state_pub->publish(system_state_msg);
    }

    /**
     * @brief Callback function to calculate and publish odometry
     * 
     * Odometry from SDK not available. Only velocity feedback available from SDK
     * 
     */
    void motion_state_callback(){
        motion_state = GetMotionState();

        //Calculate change in time
        auto current_time = this->now();
        double elapsed_time = (current_time - last_time).seconds();
        last_time = current_time;

        double resultant = motion_state.actual_motion.linear.x * elapsed_time;
        double angle = motion_state.actual_motion.angular.z * elapsed_time;

        //Update Relative Position
        odom.pose.pose.position.x += (std::acos(angle) * resultant) * DEFAULTS::ODOM_SCALER;
        odom.pose.pose.position.y += (std::asin(angle) * resultant) * DEFAULTS::ODOM_SCALER;
        odom.pose.pose.orientation.z += angle;
        //Update Velocities
        odom.twist.twist.linear.x = motion_state.actual_motion.linear.x;
        odom.twist.twist.angular.z = motion_state.actual_motion.angular.z;

        //Publish Odometry
        odom_pub->publish(odom);
    }

    /**
     * @brief Callback function for light control mode
     * 
     */
    void light_state_callback(){
        //TODO
    }

    /**
     * @brief Callback function for sensor data
     * 
     */
    void sensor_state_callback(){
        //TODO
    }

    HandshakeResultType RequestControlToken() {
        // You need to gain the control token in order to control the robot to move
        auto feedback = RequestControl();

        switch (feedback.code) {
            case HANDSHAKE_RESULT_ROBOT_BASE_NOT_ALIVE:
            RCLCPP_ERROR(get_logger(), "RobotBaseNotAlive");
            break;
            case HANDSHAKE_RESULT_CONTROL_ACQUIRED:
            RCLCPP_INFO(get_logger(), "ControlAcquired");
            break;
            case HANDSHAKE_RESULT_CONTROL_REJECTED_ROBOT_BASE_FAULT:
            RCLCPP_ERROR(get_logger(), "ControlRejected_RobotBaseFault");
            break;
            case HANDSHAKE_RESULT_CONTROL_REJECTED_RC_HALT_TRIGGERED:
            RCLCPP_ERROR(get_logger(), "ControlRejected_RcHaltTriggered");
            break;
            case HANDSHAKE_RESULT_CONTROL_REJECTED_RC_CONTROL_ACTIVE:
            RCLCPP_WARN(get_logger(), "ControlRejected_RcControlActive");
            break;
            case HANDSHAKE_RESULT_CONTROL_REJECTED_TOKEN_TRANSFER_INTERRUPTED:
            RCLCPP_ERROR(get_logger(), "ControlRejected_TokenTransferInterrupted");
            break;
            case HANDSHAKE_RESULT_CONTROL_REQUEST_TIMEOUT:
            RCLCPP_WARN(get_logger(), "ControlRequestTimeout");
            break;
        }

        if (feedback.code == HANDSHAKE_RESULT_CONTROL_ACQUIRED) {
            has_control_token = true;
            RCLCPP_INFO(get_logger(), "Has Token ");
        } else {
            RCLCPP_INFO(get_logger(), "Failed to gain control token, robot will not be controlled by the SDK");
        }

        return feedback;
    } 

    void cmd_loop(void){
        SetMotionCommand(linear_goal, angular_goal);
        RCLCPP_INFO(get_logger(), "Sent X: %.2f, Z: %.2f", linear_goal.x, angular_goal.z);
    }

    //Private Class Variables
    private:
    std::string device_name;
    rclcpp::Time last_time;
    nav_msgs::msg::Odometry odom;

    //Publishers & Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_topic;
    rclcpp::Publisher<scout_demo_node::msg::SystemState>::SharedPtr system_state_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    //Timers
    rclcpp::TimerBase::SharedPtr system_state_timer;
    rclcpp::TimerBase::SharedPtr motion_state_timer;
    rclcpp::TimerBase::SharedPtr light_state_timer;
    rclcpp::TimerBase::SharedPtr sensor_state_timer;

    //Movement goals
    ZVector3 linear_goal;
    ZVector3 angular_goal;

    //Actual states
    SystemStateMsg system_state;
    MotionStateMsg motion_state;
    LightStateMsg light_state;
    UltrasonicDataMsg ultra_data;
    std::vector<TofDataMsg> tof_data;
};

/**
 * @brief Callback Function for connection loss
 */
void ControlLostCallback(void) {
    // This function should be non-blocking and short
    std::cout << "SDK has lost control!" << std::endl;
    has_control_token = false;
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<scout_node>("can0"));
    rclcpp::shutdown();
    return 0;
}