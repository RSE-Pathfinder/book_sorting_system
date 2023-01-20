cmake_minimum_required(VERSION 3.5)
project(bss_arm_interface)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bss_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(bss_arm 
  src/bss_arm.cpp
)

ament_target_dependencies(bss_arm
  bss_interface
  rclcpp
  std_msgs
  std_srvs
)

add_executable(bss_arm_fake_move_arm 
  src/bss_arm_fake_move_arm.cpp
)

ament_target_dependencies(bss_arm_fake_move_arm
  bss_interface
  rclcpp
  std_msgs
  std_srvs
)

# Install targets
install(
  TARGETS 
  bss_arm
  bss_arm_fake_move_arm
  DESTINATION 
  lib/${PROJECT_NAME}
)

ament_package()
