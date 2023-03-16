cmake_minimum_required(VERSION 3.5)
project(scout_demo_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add additional path for CMake to find wrp_sdk (IMPORTANT)
list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")
include_directories(include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# Find wrp_sdk and its dependent package wrp_zbus
find_package(wrp_sdk REQUIRED)
find_package(wrp_zbus REQUIRED)

set(message_files
  "msg/SystemState.msg"
)

# Generate custom interface
rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  #DEPENDENCIES # Add packages that above messages depend on
)

# Add target
add_executable(scout_node src/scout_base.cpp)
add_executable(ydlidar_driver_node src/ydlidar_driver.cpp)

# Register interface
rosidl_target_interfaces(scout_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Link Weston robot SDK
target_link_libraries(scout_node westonrobot::wrp_sdk)
# Link ROS2 Dependencies
ament_target_dependencies(scout_node
rclcpp
geometry_msgs
nav_msgs
tf2
tf2_ros)
ament_target_dependencies(ydlidar_driver_node
rclcpp
sensor_msgs)

# Install Executables
install(TARGETS
  scout_node
  ydlidar_driver_node
  DESTINATION lib/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME})

ament_package()
