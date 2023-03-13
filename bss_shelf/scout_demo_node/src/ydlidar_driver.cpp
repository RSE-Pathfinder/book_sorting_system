/**
 * @file ydlidar_driver.cpp
 * @author Muhammad Syamim (Syazam33@gmail.com)
 * @brief 
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
//#include "ydlidar_ros_driver/LaserFan.h"
#include <std_srvs/srv/empty.hpp>

//Lidar SDK
#include "src/CYdLidar.h"
#include "ydlidar_config.h"

#define SDKROSVerision "1.0.1"

using namespace std::chrono_literals;

namespace DEFAULTS{
  namespace NAME{
    static const char* NODE 			= "lidar_driver_node";
	static const char* FRAME			= "base_laser";
  }
  namespace TOPIC{
    static const char* SCAN 			= "scan";
    static const char* POINTCLOUD 		= "point_cloud";
  }
  namespace SERVICE{
	static const char* START			= "start_scan";
	static const char* STOP				= "stop_scan";
  }
  namespace LIDAR{
	namespace INT{
		static int BAUDRATE 			= 128000;
		static int LIDAR_TYPE 			= TYPE_TRIANGLE;
		static int DEVICE_TYPE 			= YDLIDAR_TYPE_SERIAL;
		static int SAMPLE_RATE 			= 5;
		static int ABNORMAL_CHECK_COUNT	= 4;

	}
	namespace BOOL{
		static bool FIXED_RESOLUTION 			= true;
		static bool REVERSION 					= true;
		static bool INVERTED 					= true;
		static bool AUTO_RECONNECT 				= true;
		static bool ISSINGLE_CHANNEL 			= false;
		static bool INTENSITY 					= false;
		static bool SUPPORT_MOTOR_DTR 			= false;
		static bool INVALID_RANGE_IS_INF		= false;
		static bool POINT_CLOUD_PRESERVATIVE	= false;
	}
	namespace FLOAT{
		static float ANGLE_MAX	= 180.0f;
		static float ANGLE_MIN	= -180.0f;
		static float RANGE_MAX	= 10.0;
		static float RANGE_MIN	= 0.12;
		static float FREQUENCY	= 10;
	}
  }
  namespace SIZE{
	static const size_t INT 	= sizeof(int);
	static const size_t BOOL 	= sizeof(bool);
	static const size_t FLOAT 	= sizeof(float);
  }
}

//Lidar wrapper object
class CYdLidarNode : public rclcpp::Node, CYdLidar{
//Public Function Interface
public:
//Default Constructor
CYdLidarNode(): Node(DEFAULTS::NAME::NODE), port("dev/ydlidar"){
  	
	RCLCPP_INFO(get_logger(), "YDLIDAR ROS Driver Version: %s", SDKROSVerision);

	//Setup Scan Streams
	scan_pub  = create_publisher<sensor_msgs::msg::LaserScan>(DEFAULTS::TOPIC::SCAN, 10);
	pc_pub    = create_publisher<sensor_msgs::msg::PointCloud>(DEFAULTS::TOPIC::POINTCLOUD, 10);

	//Setup Timers
	scan_timer = create_wall_timer(33ms, std::bind(&CYdLidarNode::scan_callback, this));

	//Setup Services
	start_service = create_service<std_srvs::srv::Empty>(DEFAULTS::SERVICE::START, std::bind(&CYdLidarNode::start_scan, this));
	stop_service = create_service<std_srvs::srv::Empty>(DEFAULTS::SERVICE::STOP, std::bind(&CYdLidarNode::stop_scan, this));

	//Lidar setup
	init();
}

void scan_callback(){
	LaserScan scan;
	sensor_msgs::msg::LaserScan scan_msg;
	sensor_msgs::msg::PointCloud pc_msg;

	scan_msg.header.stamp = now();
	scan_msg.header.frame_id = DEFAULTS::NAME::FRAME;
	pc_msg.header = scan_msg.header;

}

/**
 * @brief Callback function to stop scanning
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool stop_scan(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &res) {
  RCLCPP_INFO(get_logger(), "Stop scan");
  return laser.turnOff();
}

/**
 * @brief Callback function to start scanning
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool start_scan(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &res) {
  RCLCPP_INFO(get_logger(), "Start scan");
  return laser.turnOn();
}

void init(){
	//INTS
	baudrate 				= DEFAULTS::LIDAR::INT::BAUDRATE;
	lidar_type 				= DEFAULTS::LIDAR::INT::LIDAR_TYPE;
	device_type 			= DEFAULTS::LIDAR::INT::DEVICE_TYPE;
	sample_rate 			= DEFAULTS::LIDAR::INT::SAMPLE_RATE;
	abnormal_check_count 	= DEFAULTS::LIDAR::INT::ABNORMAL_CHECK_COUNT;
	//BOOLS
	isOn					= false;
	fixed_resolution 		= DEFAULTS::LIDAR::BOOL::FIXED_RESOLUTION;
	reversion 				= DEFAULTS::LIDAR::BOOL::REVERSION;
	inverted 				= DEFAULTS::LIDAR::BOOL::INVERTED;
	auto_reconnect 			= DEFAULTS::LIDAR::BOOL::AUTO_RECONNECT;
	isSingle_channel 		= DEFAULTS::LIDAR::BOOL::ISSINGLE_CHANNEL;
	intensity 				= DEFAULTS::LIDAR::BOOL::INTENSITY;
	support_motor_dtr 		= DEFAULTS::LIDAR::BOOL::SUPPORT_MOTOR_DTR;
	invalid_range_is_inf	= DEFAULTS::LIDAR::BOOL::INVALID_RANGE_IS_INF;
	point_cloud_preservative= DEFAULTS::LIDAR::BOOL::POINT_CLOUD_PRESERVATIVE;
	//FLOATS
	angle_max				= DEFAULTS::LIDAR::FLOAT::ANGLE_MAX;
	angle_min				= DEFAULTS::LIDAR::FLOAT::ANGLE_MIN;
	range_max				= DEFAULTS::LIDAR::FLOAT::RANGE_MAX;
	range_min				= DEFAULTS::LIDAR::FLOAT::RANGE_MIN;
	frequency				= DEFAULTS::LIDAR::FLOAT::FREQUENCY;

	//Update SDK
	setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
	setlidaropt(LidarPropIgnoreArray, "", 0);
	setlidaropt(LidarPropSerialBaudrate, 		&baudrate, 				DEFAULTS::SIZE::INT);
	setlidaropt(LidarPropLidarType, 			&lidar_type, 			DEFAULTS::SIZE::INT);
	setlidaropt(LidarPropDeviceType, 			&device_type, 			DEFAULTS::SIZE::INT);
	setlidaropt(LidarPropSampleRate, 			&sample_rate, 			DEFAULTS::SIZE::INT);
	setlidaropt(LidarPropAbnormalCheckCount, 	&abnormal_check_count, 	DEFAULTS::SIZE::INT);
	setlidaropt(LidarPropFixedResolution, 		&fixed_resolution, 		DEFAULTS::SIZE::BOOL);
	setlidaropt(LidarPropReversion, 			&reversion, 			DEFAULTS::SIZE::BOOL);
	setlidaropt(LidarPropInverted, 				&inverted, 				DEFAULTS::SIZE::BOOL);
	setlidaropt(LidarPropAutoReconnect, 		&auto_reconnect, 		DEFAULTS::SIZE::BOOL);
	setlidaropt(LidarPropSingleChannel, 		&isSingle_channel, 		DEFAULTS::SIZE::BOOL);
	setlidaropt(LidarPropIntenstiy, 			&intensity, 			DEFAULTS::SIZE::BOOL);
	setlidaropt(LidarPropSupportMotorDtrCtrl, 	&b_optvalue, 			DEFAULTS::SIZE::BOOL);
	setlidaropt(LidarPropMaxAngle, 				&angle_max, 			DEFAULTS::SIZE::FLOAT);
	setlidaropt(LidarPropMinAngle, 				&angle_min, 			DEFAULTS::SIZE::FLOAT);
	setlidaropt(LidarPropMaxRange, 				&range_max, 			DEFAULTS::SIZE::FLOAT);
	setlidaropt(LidarPropMinRange, 				&range_min, 			DEFAULTS::SIZE::FLOAT);
	setlidaropt(LidarPropScanFrequency, 		&frequency, 			DEFAULTS::SIZE::FLOAT);

	//Initialise LiDAR
	if(initialize()){
		isOn = true;
	}else{
		isOn = false;
		RCLCPP_ERROR(get_logger(), "%s", DescribeError());
	}
}

//Private Class Variables
private:
std::string port;
std::atomic<int> baudrate;
std::atomic<int> lidar_type;
std::atomic<int> device_type;
std::atomic<int> sample_rate;
std::atomic<int> abnormal_check_count;
std::atomic<bool> isOn;
std::atomic<bool> fixed_resolution;
std::atomic<bool> reversion;
std::atomic<bool> inverted;
std::atomic<bool> auto_reconnect;
std::atomic<bool> isSingle_channel;
std::atomic<bool> intensity;
std::atomic<bool> support_motor_dtr;
std::atomic<float> angle_max;
std::atomic<float> angle_min;
std::atomic<float> range_max;
std::atomic<float> range_min;
std::atomic<float> frequency;
std::atomic<float> invalid_range_is_inf;
std::atomic<float> point_cloud_preservative;

//Publishers
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pc_pub;

//Timers
rclcpp::TimerBase::SharedPtr scan_timer;
rclcpp::TimerBase::SharedPtr pc_timer;

//Services
rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service;
rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service;

};

CYdLidar laser;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CYdLidarNode>());
  rclcpp::shutdown();
  return 0;
  
// RESUME FROM HERE

//  ros::Publisher laser_fan_pub =
//    nh.advertise<ydlidar_ros_driver::LaserFan>("laser_fan", 1);

  while (ret && ros::ok()) {
    LaserScan scan;

    if (laser.doProcessSimple(scan)) {
      sensor_msgs::LaserScan scan_msg;
      sensor_msgs::PointCloud pc_msg;
//      ydlidar_ros_driver::LaserFan fan;
      ros::Time start_scan_time;
      start_scan_time.sec = scan.stamp / 1000000000ul;
      start_scan_time.nsec = scan.stamp % 1000000000ul;
      scan_msg.header.stamp = start_scan_time;
      scan_msg.header.frame_id = frame_id;
      pc_msg.header = scan_msg.header;
//      fan.header = scan_msg.header;
      scan_msg.angle_min = (scan.config.min_angle);
      scan_msg.angle_max = (scan.config.max_angle);
      scan_msg.angle_increment = (scan.config.angle_increment);
      scan_msg.scan_time = scan.config.scan_time;
      scan_msg.time_increment = scan.config.time_increment;
      scan_msg.range_min = (scan.config.min_range);
      scan_msg.range_max = (scan.config.max_range);
//      fan.angle_min = (scan.config.min_angle);
//      fan.angle_max = (scan.config.max_angle);
//      fan.scan_time = scan.config.scan_time;
//      fan.time_increment = scan.config.time_increment;
//      fan.range_min = (scan.config.min_range);
//      fan.range_max = (scan.config.max_range);

      int size = (scan.config.max_angle - scan.config.min_angle) /
                 scan.config.angle_increment + 1;
      scan_msg.ranges.resize(size,
                             invalid_range_is_inf ? std::numeric_limits<float>::infinity() : 0.0);
      scan_msg.intensities.resize(size);
      pc_msg.channels.resize(2);
      int idx_intensity = 0;
      pc_msg.channels[idx_intensity].name = "intensities";
      int idx_timestamp = 1;
      pc_msg.channels[idx_timestamp].name = "stamps";

      for (size_t i = 0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle) /
                              scan.config.angle_increment);

        if (index >= 0 && index < size) {
          if (scan.points[i].range >= scan.config.min_range) {
            scan_msg.ranges[index] = scan.points[i].range;
            scan_msg.intensities[index] = scan.points[i].intensity;
          }
        }

        if (point_cloud_preservative ||
            (scan.points[i].range >= scan.config.min_range &&
             scan.points[i].range <= scan.config.max_range)) {
          geometry_msgs::Point32 point;
          point.x = scan.points[i].range * cos(scan.points[i].angle);
          point.y = scan.points[i].range * sin(scan.points[i].angle);
          point.z = 0.0;
          pc_msg.points.push_back(point);
          pc_msg.channels[idx_intensity].values.push_back(scan.points[i].intensity);
          pc_msg.channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
        }

//        fan.angles.push_back(scan.points[i].angle);
//        fan.ranges.push_back(scan.points[i].range);
//        fan.intensities.push_back(scan.points[i].intensity);
      }

      scan_pub.publish(scan_msg);
      pc_pub.publish(pc_msg);
//      laser_fan_pub.publish(fan);

    } else {
      ROS_ERROR("Failed to get Lidar Data");
    }

    r.sleep();
    ros::spinOnce();
  }

  laser.turnOff();
  ROS_INFO("[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.disconnecting();
  return 0;
}


