/**
 * @file ydlidar_driver.cpp
 * @author Muhammad Syamim (Syazam33@gmail.com)
 * @brief ROS2 driver object for ydlidar
 * @version 0.1
 * @date 2023-03-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//C++ Standard Library
#include <limits>       // std::numeric_limits
#include <chrono>
#include <atomic>

//ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

//Lidar SDK
#include "src/CYdLidar.h"
#include "ydlidar_config.h"

#define SDKROS2Version "1.0.1"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace DEFAULTS{
  namespace NAME{
    static const char* NODE 			= "lidar_driver_node";
	static const char* LIDAR_FRAME		= "base_laser";
	static const char* BASE_FRAME		= "base_footprint";
  }
  namespace TOPIC{
    static const char* SCAN 			= "~/scan";
    static const char* POINTCLOUD 		= "~/point_cloud";
  }
  namespace SERVICE{
	static const char* START			= "start_scan";
	static const char* STOP				= "stop_scan";
  }
  namespace LIDAR{
	namespace INT{
		static const int BAUDRATE 				= 128000;
		static const int LIDAR_TYPE 			= TYPE_TRIANGLE;
		static const int DEVICE_TYPE 			= YDLIDAR_TYPE_SERIAL;
		static const int SAMPLE_RATE 			= 5;
		static const int ABNORMAL_CHECK_COUNT	= 4;

	}
	namespace BOOL{
		static const bool FIXED_RESOLUTION 			= true;
		static const bool REVERSION 				= true;
		static const bool INVERTED 					= true;
		static const bool AUTO_RECONNECT 			= true;
		static const bool ISSINGLE_CHANNEL 			= false;
		static const bool INTENSITY 				= false;
		static const bool SUPPORT_MOTOR_DTR 		= false;
		static const bool INVALID_RANGE_IS_INF		= false;
		static const bool POINT_CLOUD_PRESERVATIVE	= false;
	}
	namespace FLOAT{
		static const float ANGLE_MAX	= 180.0f;
		static const float ANGLE_MIN	= -180.0f;
		static const float RANGE_MAX	= 10.0;
		static const float RANGE_MIN	= 0.12;
		static const float FREQUENCY	= 10;
	}
	namespace STRING{
		static const char* PORT			= "dev/ydlidar";
		static const char* IGNORE_ARRAY	= "";
	}
  }
  namespace SIZE{
	static const size_t INT 	= sizeof(int);
	static const size_t BOOL 	= sizeof(bool);
	static const size_t FLOAT 	= sizeof(float);
  }
  namespace TF{
	namespace LINEAR{
		static const int X	= 0.0;
		static const int Y	= 0.0;
		static const int Z	= 0.0;
	}
	namespace ANGULAR{
		static const int X	= 0.0;
		static const int Y	= 0.0;
		static const int Z	= 0.0;
		static const int W	= 1.0;
	}
  }
}

//Lidar wrapper object
class CYdLidarNode : public rclcpp::Node, CYdLidar{
//Public Function Interface
public:
//Default Constructor
CYdLidarNode(): Node(DEFAULTS::NAME::NODE){
  	
	//Lidar setup
	RCLCPP_INFO(get_logger(), "YDLIDAR SDK Version: %s", SDKROS2Version);
	init();

	//Setup Scan Streams
	scan_pub  = create_publisher<sensor_msgs::msg::LaserScan>(DEFAULTS::TOPIC::SCAN, 10);
	pc_pub    = create_publisher<sensor_msgs::msg::PointCloud>(DEFAULTS::TOPIC::POINTCLOUD, 10);

	//Setup static TF publisher
	tf_pub = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

	//Setup Timers
	scan_timer = create_wall_timer(100ms, std::bind(&CYdLidarNode::scan_callback, this));

	//Setup Services
	start_service 	= create_service<std_srvs::srv::Empty>(DEFAULTS::SERVICE::START, std::bind(&CYdLidarNode::start_scan, this, _1, _2));
	stop_service 	= create_service<std_srvs::srv::Empty>(DEFAULTS::SERVICE::STOP, std::bind(&CYdLidarNode::stop_scan, this, _1, _2));

	//Publish static tf once
	make_transform();
}

//Default Destructor
~CYdLidarNode(){
	turnOff();
	RCLCPP_INFO(get_logger(), "Now YDLIDAR is stopping .......");
	disconnecting();
}

/**
 * @brief Callback function to read and publish scan data
 * 
 */
void scan_callback(){
	if(!isOn) return;
	if(!doProcessSimple(scan)){
		RCLCPP_ERROR(get_logger(), "Failed to get scan data!!!");
		RCLCPP_INFO(get_logger(), "%s", DescribeError());
		return;
	}

	//Fill header
	scan_msg.header.stamp = now();
	scan_msg.header.frame_id = DEFAULTS::NAME::LIDAR_FRAME;
	pc_msg.header = scan_msg.header;

	//Fill scan config
	scan_msg.angle_min 			= scan.config.min_angle;
	scan_msg.angle_max 			= scan.config.max_angle;
	scan_msg.angle_increment 	= scan.config.angle_increment;
	scan_msg.scan_time 			= scan.config.scan_time;
	scan_msg.time_increment 	= scan.config.time_increment;
	scan_msg.range_min 			= scan.config.min_range;
	scan_msg.range_max 			= scan.config.max_range;

	bool invalid_range_is_inf(get_parameter("invalid_range_is_inf").get_parameter_value().get<bool>());
	bool point_cloud_preservative(get_parameter("point_cloud_preservative").get_parameter_value().get<bool>());

	// float max_range = 0, min_range = 10.0;

	int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
	scan_msg.ranges.resize(size);
	scan_msg.intensities.resize(size);
	for(size_t i=0; i < scan.points.size(); i++) {
		int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
		if(index >=0 && index < size) {
			scan_msg.ranges[index] = scan.points[i].range;
			scan_msg.intensities[index] = scan.points[i].intensity;
			// max_range = scan.points[i].range > max_range ? scan.points[i].range : max_range;
			// min_range = scan.points[i].range < min_range ? scan.points[i].range : min_range;
		}
	}

	// RCLCPP_INFO(get_logger(), "Max: %.4f | Min: %.4f | Increments: %.4f, | Size: %d", max_range, min_range, scan.config.angle_increment, scan.points.size);

	scan_pub->publish(scan_msg);
	pc_pub->publish(pc_msg);
}

/**
 * @brief Callback function to stop scanning
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
void stop_scan(const std::shared_ptr<std_srvs::srv::Empty_Request> req, const std::shared_ptr<std_srvs::srv::Empty_Response> res) {
  RCLCPP_INFO(get_logger(), "Stop scan");
  isOn = false;
  turnOff();
}

/**
 * @brief Callback function to start scanning
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
void start_scan(const std::shared_ptr<std_srvs::srv::Empty_Request> req, const std::shared_ptr<std_srvs::srv::Empty_Response> res) {
  RCLCPP_INFO(get_logger(), "Start scan");
  isOn = true;
  turnOn();
}

//Private Class Functions
private:
/**
 * @brief Initialises all variables to default values
 * 
 */
void init(){
	// Declare Lidar parameters
	declare_string_params();
	declare_int_params();
	declare_bool_params();
	declare_float_params();

	//Initialise LiDAR
	if(!(isOn = initialize())) RCLCPP_ERROR(get_logger(), "%s", DescribeError());
	if(isOn) turnOn();
}

/**
 * @brief Helper function to declare and set string lidar parameters
 * 
 */
void declare_string_params(){
	std::string tmp_string;
	declare_parameter("port", DEFAULTS::LIDAR::STRING::PORT);
	get_parameter("port", tmp_string);
	setlidaropt(LidarPropSerialPort, tmp_string.c_str(), tmp_string.size());
	
	declare_parameter("ignore_array", DEFAULTS::LIDAR::STRING::IGNORE_ARRAY);
	get_parameter("ignore_array", tmp_string);
	setlidaropt(LidarPropIgnoreArray, tmp_string.c_str(), tmp_string.size());
}

/**
 * @brief Helper function to declare and set int lidar parameters
 * 
 */
void declare_int_params(){
	int tmp_int;
	declare_parameter("baudrate", DEFAULTS::LIDAR::INT::BAUDRATE);
	get_parameter("baudrate", tmp_int);
	setlidaropt(LidarPropSerialBaudrate, &tmp_int, DEFAULTS::SIZE::INT);

	declare_parameter("lidar_type", DEFAULTS::LIDAR::INT::LIDAR_TYPE);
	get_parameter("lidar_type", tmp_int);
	setlidaropt(LidarPropLidarType, &tmp_int, DEFAULTS::SIZE::INT);
	
	bool tmp_bool;
	declare_parameter("isSingle_channel", DEFAULTS::LIDAR::BOOL::ISSINGLE_CHANNEL);
	get_parameter("isSingle_channel", tmp_bool);
	tmp_int = tmp_bool ? 3 : 4;
	setlidaropt(LidarPropSingleChannel, &tmp_int, DEFAULTS::SIZE::INT);

	declare_parameter("device_type", DEFAULTS::LIDAR::INT::DEVICE_TYPE);
	get_parameter("device_type", tmp_int);
	setlidaropt(LidarPropDeviceType, &tmp_int, DEFAULTS::SIZE::INT);
	
	declare_parameter("sample_rate", DEFAULTS::LIDAR::INT::SAMPLE_RATE);
	get_parameter("sample_rate", tmp_int);
	setlidaropt(LidarPropSampleRate, &tmp_int, DEFAULTS::SIZE::INT);

	declare_parameter("abnormal_check_count", DEFAULTS::LIDAR::INT::ABNORMAL_CHECK_COUNT);
	get_parameter("abnormal_check_count", tmp_int);
	setlidaropt(LidarPropAbnormalCheckCount, &tmp_int, DEFAULTS::SIZE::INT);
}

/**
 * @brief Helper function to declare and set bool lidar parameters
 * 
 */
void declare_bool_params(){
	bool tmp_bool;
	declare_parameter("fixed_resolution", DEFAULTS::LIDAR::BOOL::FIXED_RESOLUTION);
	get_parameter("fixed_resolution", tmp_bool);
	setlidaropt(LidarPropAbnormalCheckCount, &tmp_bool, DEFAULTS::SIZE::BOOL);

	declare_parameter("reversion", DEFAULTS::LIDAR::BOOL::REVERSION);
	get_parameter("reversion", tmp_bool);
	setlidaropt(LidarPropReversion, &tmp_bool, DEFAULTS::SIZE::BOOL);

	declare_parameter("inverted", DEFAULTS::LIDAR::BOOL::INVERTED);
	get_parameter("inverted", tmp_bool);
	setlidaropt(LidarPropInverted, &tmp_bool, DEFAULTS::SIZE::BOOL);

	declare_parameter("auto_reconnect", DEFAULTS::LIDAR::BOOL::AUTO_RECONNECT);
	get_parameter("auto_reconnect", tmp_bool);
	setlidaropt(LidarPropAutoReconnect, &tmp_bool, DEFAULTS::SIZE::BOOL);

	declare_parameter("intensity", DEFAULTS::LIDAR::BOOL::INTENSITY);
	get_parameter("intensity", tmp_bool);
	setlidaropt(LidarPropIntenstiy, &tmp_bool, DEFAULTS::SIZE::BOOL);

	declare_parameter("support_motor_dtr", DEFAULTS::LIDAR::BOOL::SUPPORT_MOTOR_DTR);
	get_parameter("support_motor_dtr", tmp_bool);
	setlidaropt(LidarPropSupportMotorDtrCtrl, &tmp_bool, DEFAULTS::SIZE::BOOL);

	declare_parameter("invalid_range_is_inf", DEFAULTS::LIDAR::BOOL::INVALID_RANGE_IS_INF);
	get_parameter("invalid_range_is_inf", tmp_bool);

	declare_parameter("point_cloud_preservative", DEFAULTS::LIDAR::BOOL::POINT_CLOUD_PRESERVATIVE);
	get_parameter("point_cloud_preservative", tmp_bool);
}

/**
 * @brief Helper funciton to declare and set float parameters
 * 
 */
void declare_float_params(){
	float tmp_float;
	declare_parameter("angle_max", DEFAULTS::LIDAR::FLOAT::ANGLE_MAX);
	get_parameter("angle_max", tmp_float);
	setlidaropt(LidarPropMaxAngle, &tmp_float, DEFAULTS::SIZE::FLOAT);

	declare_parameter("angle_min", DEFAULTS::LIDAR::FLOAT::ANGLE_MIN);
	get_parameter("angle_min", tmp_float);
	setlidaropt(LidarPropMinAngle, &tmp_float, DEFAULTS::SIZE::FLOAT);

	declare_parameter("range_max", DEFAULTS::LIDAR::FLOAT::RANGE_MAX);
	get_parameter("range_max", tmp_float);
	setlidaropt(LidarPropMaxRange, &tmp_float, DEFAULTS::SIZE::FLOAT);

	declare_parameter("range_min", DEFAULTS::LIDAR::FLOAT::RANGE_MIN);
	get_parameter("range_min", tmp_float);
	setlidaropt(LidarPropMinRange, &tmp_float, DEFAULTS::SIZE::FLOAT);
	
	declare_parameter("frequency", DEFAULTS::LIDAR::FLOAT::FREQUENCY);
	get_parameter("frequency", tmp_float);
	setlidaropt(LidarPropScanFrequency, &tmp_float, DEFAULTS::SIZE::FLOAT);
}

/**
 * @brief Creates and publishes the static transform once
 * 
 */
void make_transform(){
	geometry_msgs::msg::TransformStamped tf_msg;
	//TF Header
	tf_msg.header.stamp = now();
	tf_msg.header.frame_id = DEFAULTS::NAME::BASE_FRAME;
	tf_msg.child_frame_id = DEFAULTS::NAME::LIDAR_FRAME;
	
	//TF Data
	tf_msg.transform.translation.x = DEFAULTS::TF::LINEAR::X;
	tf_msg.transform.translation.y = DEFAULTS::TF::LINEAR::Y;
	tf_msg.transform.translation.z = DEFAULTS::TF::LINEAR::Z;
	tf_msg.transform.rotation.x = DEFAULTS::TF::ANGULAR::X;
	tf_msg.transform.rotation.y = DEFAULTS::TF::ANGULAR::Y;
	tf_msg.transform.rotation.z = DEFAULTS::TF::ANGULAR::Z;
	tf_msg.transform.rotation.w = DEFAULTS::TF::ANGULAR::W;

	//Publish TF data in /static_tf
	tf_pub->sendTransform(tf_msg);
}

//Private Class Variables
private:
std::atomic<bool> isOn;

//Scan Data
LaserScan scan;
sensor_msgs::msg::LaserScan scan_msg;
sensor_msgs::msg::PointCloud pc_msg;

//Publishers
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pc_pub;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub;

//Timers
rclcpp::TimerBase::SharedPtr scan_timer;
rclcpp::TimerBase::SharedPtr pc_timer;

//Services
rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service;
rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CYdLidarNode>());
  rclcpp::shutdown();
  return 0;
}


