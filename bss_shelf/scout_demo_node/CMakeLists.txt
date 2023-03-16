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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# Find ydlidar_sdk
find_package(ydlidar_sdk REQUIRED)
# Find wrp_sdk and its dependent package wrp_zbus
find_package(wrp_sdk REQUIRED)
find_package(wrp_zbus REQUIRED)

# Include directories 
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)

# Link directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

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

# Link Weston robot SDK to scout_node
target_link_libraries(scout_node westonrobot::wrp_sdk)

# Link YDLidar SDK to ydlidar_driver_node
target_link_libraries(ydlidar_driver_node ${YDLIDAR_SDK_LIBRARIES})

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

ament_target_dependencies(ydlidar_driver_node
rclcpp
geometry_msgs
sensor_msgs
ydlidar_sdk
std_srvs
tf2
tf2_ros)

# Install Executables
install(TARGETS
  scout_node
  ydlidar_driver_node
  DESTINATION lib/${PROJECT_NAME})

# Install Directory files
install(DIRECTORY
  launch
  param
  map
  DESTINATION share/${PROJECT_NAME})

ament_package()
