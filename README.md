# book_sorting_system
ROS 2 Foxy Packages for book sorting system

How to use:
```bash
git clone https://github.com/RSE2107A-AY2122T3-Team-4/book_sorting_system.git
cd book_sorting_system/
git submodule init && git submodule update
sudo apt install cmake pkg-config
cd bss_shelf/YDLidar-SDK && mkdir build && cd "$_"
cmake ..
make
sudo make install
cd ../../..
```

Launch UR10 in Gazebo:
```bash
ros2 launch ur10_ros2_gazebo ur10_simulation.launch.py
```

Launch UR10 in RVIZ with moveit2 (will also launches Gazebo):
```bash
ros2 launch ur10_ros2_moveit2 ur10_interface.launch.py
```

Working Tree:
```bash
├── bss_arm
│   ├── README.md
│   └── ros2_RobotSimulation
│       ├── include
│       │   ├── MGIdiagram.jpg
│       │   ├── move_group_interface_improved.h
│       │   └── README.md
│       ├── LICENSE.md
│       ├── media
│       │   ├── header.jpg
│       │   ├── irb120egp64.mp4
│       │   └── logo.jpg
│       ├── README.md
│       ├── ros2_actions
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   └── scripts
│       │       ├── moveG_action.cpp
│       │       ├── moveJ_action.cpp
│       │       ├── moveJs_action.cpp
│       │       ├── moveL_action.cpp
│       │       ├── moveR_action.cpp
│       │       ├── moveROT_action.cpp
│       │       ├── moveRP_action.cpp
│       │       ├── moveRs_action.cpp
│       │       ├── moveXYZ_action.cpp
│       │       ├── moveXYZW_action.cpp
│       │       └── moveYPR_action.cpp
│       ├── ros2_data
│       │   ├── action
│       │   │   ├── MoveG.action
│       │   │   ├── MoveJ.action
│       │   │   ├── MoveJs.action
│       │   │   ├── MoveL.action
│       │   │   ├── MoveR.action
│       │   │   ├── MoveROT.action
│       │   │   ├── MoveRP.action
│       │   │   ├── MoveXYZ.action
│       │   │   ├── MoveXYZW.action
│       │   │   └── MoveYPR.action
│       │   ├── CMakeLists.txt
│       │   ├── launch
│       │   │   └── ros2_data.launch.py
│       │   ├── msg
│       │   │   ├── JointPose.msg
│       │   │   └── JointPoseS.msg
│       │   ├── package.xml
│       │   └── src
│       │       └── ros2-data.cpp
│       └── UniversalRobots
│           └── UR10
│               ├── ur10_ros2_gazebo
│               │   ├── CMakeLists.txt
│               │   ├── config
│               │   │   ├── default_kinematics.yaml
│               │   │   ├── initial_positions.yaml
│               │   │   ├── joint_limits.yaml
│               │   │   ├── physical_parameters.yaml
│               │   │   ├── ur10_controller.yaml
│               │   │   └── visual_parameters.yaml
│               │   ├── launch
│               │   │   └── ur10_simulation.launch.py
│               │   ├── meshes
│               │   │   ├── collision
│               │   │   │   ├── base.stl
│               │   │   │   ├── forearm.stl
│               │   │   │   ├── shoulder.stl
│               │   │   │   ├── upperarm.stl
│               │   │   │   ├── wrist1.stl
│               │   │   │   ├── wrist2.stl
│               │   │   │   └── wrist3.stl
│               │   │   └── visual
│               │   │       ├── base.dae
│               │   │       ├── forearm.dae
│               │   │       ├── shoulder.dae
│               │   │       ├── upperarm.dae
│               │   │       ├── wrist1.dae
│               │   │       ├── wrist2.dae
│               │   │       └── wrist3.dae
│               │   ├── package.xml
│               │   ├── urdf
│               │   │   ├── ur10_common.xacro
│               │   │   ├── ur10_gazebo.xacro
│               │   │   ├── ur10_macro.urdf.xacro
│               │   │   ├── ur10_transmission.xacro
│               │   │   └── ur10.urdf.xacro
│               │   └── worlds
│               │       └── ur10.world
│               └── ur10_ros2_moveit2
│                   ├── CMakeLists.txt
│                   ├── config
│                   │   ├── chomp_planning.yaml
│                   │   ├── kinematics.yaml
│                   │   ├── ompl_planning.yaml
│                   │   ├── ur10_controllers.yaml
│                   │   └── ur10.srdf
│                   ├── launch
│                   │   ├── ur10_interface.launch.py
│                   │   ├── ur10.launch.py
│                   │   ├── ur10_moveit2_empty.rviz
│                   │   └── ur10_moveit2.rviz
│                   └── package.xml
├── bss_controller
│   ├── bss_controller_bringup
│   │   ├── bss_controller_bringup
│   │   │   ├── arm_action_client.py
│   │   │   ├── arm_action_server.py
│   │   │   ├── __init__.py
│   │   │   └── __pycache__
│   │   │       ├── arm_action_client.cpython-38.pyc
│   │   │       ├── arm_action_server.cpython-38.pyc
│   │   │       └── __init__.cpython-38.pyc
│   │   ├── package.xml
│   │   ├── resource
│   │   │   └── bss_controller_bringup
│   │   ├── setup.cfg
│   │   ├── setup.py
│   │   └── test
│   │       ├── test_copyright.py
│   │       ├── test_flake8.py
│   │       └── test_pep257.py
│   ├── bss_controller_interface
│   │   ├── action
│   │   │   └── MoveArm.action
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── srv
│   │       └── MoveArm.srv
│   └── README.md
├── bss_shelf
│   ├── bss_shelf_bringup
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   ├── mobile_shelf_gazebo.launch.py
│   │   │   ├── mobile_shelf.launch.py
│   │   │   └── mobile_shelf_spawner.launch.py
│   │   ├── log
│   │   │   ├── COLCON_IGNORE
│   │   │   ├── latest -> latest_list
│   │   │   ├── latest_list -> list_2023-01-11_08-45-13
│   │   │   └── list_2023-01-11_08-45-13
│   │   │       └── logger_all.log
│   │   └── package.xml
│   ├── README.md
│   └── wrp_ros2
│       ├── CMakeLists.txt
│       ├── config
│       │   ├── mobile_base
│       │   │   └── mobile_base_node_config.yaml
│       │   ├── peripheral
│       │   │   ├── gps_receiver_node_config.yaml
│       │   │   └── imu_sensor_node_config.yaml
│       │   └── README.md
│       ├── include
│       │   └── wrp_ros2
│       │       ├── mobile_base
│       │       │   └── mobile_base_node.hpp
│       │       └── peripheral
│       │           ├── gps_receiver_node.hpp
│       │           ├── imu_sensor_node.hpp
│       │           ├── power_regulator_node.hpp
│       │           └── ultrasonic_sensor_node.hpp
│       ├── launch
│       │   ├── description
│       │   │   ├── scout_mini_description.launch.py
│       │   │   ├── scout_v2_description.launch.py
│       │   │   └── scout_v2_nav_description.launch.py
│       │   ├── mobile_base
│       │   │   ├── mobile_base.launch.py
│       │   │   ├── mobile_base_scout_sensors.launch.py
│       │   │   ├── scout.launch.py
│       │   │   ├── scout_mini.launch.py
│       │   │   └── tracer.launch.py
│       │   └── peripheral
│       │       ├── gps_receiver.launch.py
│       │       ├── imu_sensor.launch.py
│       │       ├── power_regulator.launch.py
│       │       └── ultrasonic_sensor.launch.py
│       ├── meshes
│       │   ├── scout_mini
│       │   │   ├── mini_base_link.dae
│       │   │   └── mini_wheel.dae
│       │   └── scout_v2
│       │       ├── base_link.dae
│       │       ├── base_link_full.dae
│       │       ├── base_link.stl
│       │       ├── hokuyo.dae
│       │       ├── l1_shelf_back_1.stl
│       │       ├── l1_shelf_front_1.stl
│       │       ├── wheel_type1.dae
│       │       └── wheel_type2.dae
│       ├── msg
│       │   ├── mobile_base
│       │   │   ├── ActuatorStateArray.msg
│       │   │   ├── ActuatorState.msg
│       │   │   ├── DriverState.msg
│       │   │   ├── LightControlType.msg
│       │   │   ├── MotionCommand.msg
│       │   │   ├── MotionState.msg
│       │   │   ├── MotorState.msg
│       │   │   ├── RangeDataArray.msg
│       │   │   ├── RangeData.msg
│       │   │   ├── RcState.msg
│       │   │   └── SystemState.msg
│       │   └── peripheral
│       │       ├── PowerRegulatorChannelState.msg
│       │       └── PowerRegulatorDeviceState.msg
│       ├── package.xml
│       ├── README.md
│       ├── scripts
│       │   ├── bringup_can2usb_1m.bash
│       │   ├── bringup_can2usb_250k.bash
│       │   ├── bringup_can2usb_500k.bash
│       │   └── setup_can2usb.bash
│       ├── src
│       │   ├── mobile_base
│       │   │   ├── mobile_base_node.cpp
│       │   │   └── README.md
│       │   └── peripheral
│       │       ├── gps_receiver_node.cpp
│       │       ├── imu_sensor_node.cpp
│       │       ├── power_regulator_node.cpp
│       │       ├── README.md
│       │       └── ultrasonic_sensor_node.cpp
│       ├── srv
│       │   ├── mobile_base
│       │   │   ├── AccessControl.srv
│       │   │   ├── AssistedModeControl.srv
│       │   │   ├── LightControl.srv
│       │   │   └── MotionReset.srv
│       │   └── peripheral
│       │       └── PowerRegulatorControl.srv
│       └── urdf
│           ├── generate_urdf.sh
│           ├── scout_mini
│           │   ├── scout_mini.urdf
│           │   ├── scout_mini_wheel.xacro
│           │   └── scout_mini.xacro
│           └── scout_v2
│               ├── scout_v2_fixedwheel_w_shelf.xacro
│               ├── scout_v2_nav.xacro
│               ├── scout_v2.urdf
│               ├── scout_v2.xacro
│               ├── scout_wheel_type1.xacro
│               └── scout_wheel_type2.xacro
├── bss_user_interface
│   └── README.md
├── LICENSE
└── README.md
```
