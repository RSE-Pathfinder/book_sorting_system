/**
 * @file scout_base.cpp
 * @author Muhammad Syamim (Syazam33@gmail.com)
 * @brief ROS2 driver object for Weston robot SCOUT V2 using wrp_sdk ver.0.8.7
 * @version 0.1
 * @date 2023-02-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//C++ Standard Library
#include <memory>
#include <atomic>
#include <string>
#include <chrono>
#include <vector>
#include <utility>

//ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <scout_demo_node/msg/system_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//Found in folder "/opt/weston_robot/include"
#include "wrp_sdk/mobile_base/westonrobot/mobile_base.hpp"

//Defaults
using namespace std::placeholders;
using namespace std::chrono_literals;
namespace DEFAULTS{
    namespace NAMES{
        static const char* NODE         = "Scout_Base_Node";
        static const char* DEVICE       = "can0";
    }
    namespace TOPICS{
        static const char* CMD_VEL      = "~/cmd_vel";
        static const char* ODOM         = "~/odom";
        static const char* SYSTEM_STATE = "~/SystemState";
    }
    namespace FRAMES{
        static const char* ODOM         = "odom";
        static const char* BASE         = "base_footprint";
    }
    namespace SCALERS{
        static const float ODOM         = 0.6;
    }

}

//Function Prototypes
void ControlLostCallback(void);

//Global Variables
std::atomic<bool> has_control_token;

//Scout wrapper object
class scout_node : public rclcpp::Node, westonrobot::MobileBase{
    public:
    //Default Constructor
    scout_node(std::string device_name) :
        Node(DEFAULTS::NAMES::NODE), device_name(device_name), robot_frame(DEFAULTS::FRAMES::BASE), linear_goal({0,0,0}), angular_goal({0,0,0}){
        //Initialise Node variables
        has_control_token = false;
        last_time = now();
        RegisterLoseControlCallback(ControlLostCallback);
        tf_pub = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // static_tf_pub = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // make_base_tf();

        //Connect to CAN bus
        Connect(device_name);
        RCLCPP_INFO(get_logger(), "Connected to %s", device_name.c_str());

        //Setup Status Streams
        system_state_pub    = create_publisher<scout_demo_node::msg::SystemState>(DEFAULTS::TOPICS::SYSTEM_STATE, 10);
        odom_pub            = create_publisher<nav_msgs::msg::Odometry>(DEFAULTS::TOPICS::ODOM, 10);

        //Verify update rate
        system_state_timer  = create_wall_timer(100ms, std::bind(&scout_node::system_state_callback, this));
        motion_state_timer  = create_wall_timer(10ms, std::bind(&scout_node::motion_state_callback, this));

        //Setup Command Stream
        cmd_vel_topic = create_subscription<geometry_msgs::msg::Twist>(
            DEFAULTS::TOPICS::CMD_VEL, 10, std::bind(&scout_node::cmd_vel_callback, this, _1));
        RCLCPP_INFO(get_logger(), "Listening to twist commands from '%s'", DEFAULTS::TOPICS::CMD_VEL);

        //Request Base Control
        RequestControlToken();
        // RCLCPP_INFO(get_logger(), "Gained Control Token!");
    }

    //Private Class Functions
    private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_msg) {
        RCLCPP_INFO(get_logger(), "Received linear X: %.2f, angular Z: %.2f", cmd_msg->linear.x, cmd_msg->angular.z);
        //Set goal
        linear_goal.x = cmd_msg->linear.x;
        angular_goal.z = cmd_msg->angular.z;
        
        //Send goal
        if(has_control_token){
            SetMotionCommand(linear_goal, angular_goal);
            RCLCPP_INFO(get_logger(), "Sent X: %.2f, Z: %.2f", linear_goal.x, angular_goal.z);
        }else{
            RCLCPP_WARN(get_logger(), "SDK No Control, Send again");
            RequestControlToken();
        }
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
        auto current_time = now();
        double elapsed_time = (current_time - last_time).seconds();
        last_time = current_time;

        //Convert polar vector into 2D pose
        double resultant = motion_state.actual_motion.linear.x * elapsed_time;
        double angle = motion_state.actual_motion.angular.z * elapsed_time;
        current_heading += angle;
        //Convert to quaternion
        tf2::Quaternion angle_quaternion;
        angle_quaternion.setRPY(0, 0, current_heading);

        //Odometry Header
        odom.header.stamp = current_time;
        odom.header.frame_id = DEFAULTS::FRAMES::ODOM;
        odom.child_frame_id = robot_frame;

        //Update Relative Position
        odom.pose.pose.position.x += (std::acos(angle) * resultant) * DEFAULTS::SCALERS::ODOM;
        odom.pose.pose.position.y += (std::asin(angle) * resultant) * DEFAULTS::SCALERS::ODOM;
        odom.pose.pose.orientation.x = angle_quaternion.x();
        odom.pose.pose.orientation.y = angle_quaternion.y();
        odom.pose.pose.orientation.z = angle_quaternion.z();
        odom.pose.pose.orientation.w = angle_quaternion.w();
        
        //Update Velocities
        odom.twist.twist.linear.x = motion_state.actual_motion.linear.x;
        odom.twist.twist.angular.z = motion_state.actual_motion.angular.z;

        //Publish Odometry
        odom_pub->publish(odom);

        //Update TF
        update_odom_tf();
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

    /**.     * @brief Updates and publishes odom tf frame
     * 
     */
    void update_odom_tf(){
        //Update transform frame
        tf.header.stamp = odom.header.stamp;
        tf.header.frame_id = odom.header.frame_id;
        tf.child_frame_id = odom.child_frame_id;
        tf.transform.translation.x = odom.pose.pose.position.x;
        tf.transform.translation.y = odom.pose.pose.position.y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom.pose.pose.orientation;

        //Publish Transform
        tf_pub->sendTransform(tf);
    }

    void make_base_tf(){
        geometry_msgs::msg::TransformStamped static_tf_msg;
        //TF Header
        static_tf_msg.header.stamp = now();
        static_tf_msg.header.frame_id = DEFAULTS::FRAMES::BASE;
        static_tf_msg.child_frame_id = "map";
        
        //TF Data
        static_tf_msg.transform.translation.x = 0;
        static_tf_msg.transform.translation.y = 0;
        static_tf_msg.transform.translation.z = 0;
        static_tf_msg.transform.rotation.x = 0.0;
        static_tf_msg.transform.rotation.y = 0.0;
        static_tf_msg.transform.rotation.z = 0.0;
        static_tf_msg.transform.rotation.w = 1.0;

        //Publish TF data in /static_tf
        static_tf_pub->sendTransform(static_tf_msg);
    }

    /**
     * @brief Gains access to the Scout base
     * 
     * @return HandshakeResultType
     */
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
            case HANDSHAKE_RESULT_ALREADY_GAINED_CONTROL:
            RCLCPP_INFO(get_logger(), "ControlAlreadyAcquired");
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
            default:
            RCLCPP_INFO(get_logger(), "Handshake Result: %d", feedback.code);
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

    //Private Class Variables
    private:
    std::string device_name;
    std::string robot_frame;
    rclcpp::Time last_time;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::TransformStamped tf;
    double current_heading;

    //Publishers & Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_topic;
    rclcpp::Publisher<scout_demo_node::msg::SystemState>::SharedPtr system_state_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub;

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
    rclcpp::spin(std::make_shared<scout_node>(DEFAULTS::NAMES::DEVICE));
    rclcpp::shutdown();
    return 0;
}