# book_sorting_system
ROS 2 Foxy Packages for book sorting system

How to use:
```bash
git clone https://github.com/RSE2107A-AY2122T3-Team-4/book_sorting_system.git
cd book_sorting_system/
git submodule init && git submodule update
sudo apt install cmake pkg-config
<<<<<<< HEAD
cd bss_shelf/bss_shelf_bringup/YDLidar-SDK && mkdir build && cd "$_"
=======
cd mobile_shelf/YDLidar-SDK && mkdir build && cd "$_"
>>>>>>> 6b80ac0bb03b5d69faed0628a7d04e23b8c9c9b3
cmake ..
make
sudo make install
cd ../../..
<<<<<<< HEAD
=======
```

Launch UR10 in Gazebo:
```bash
ros2 launch ur10_ros2_gazebo ur10_simulation.launch.py
```

Launch UR10 in RVIZ with moveit2 (will also launches Gazebo):
```bash
ros2 launch ur10_ros2_moveit2 ur10_interface.launch.py
>>>>>>> 6b80ac0bb03b5d69faed0628a7d04e23b8c9c9b3
```

Working Tree:
```bash
BOOK_SORTING_SYSTEM
<<<<<<< HEAD
├── bss_arm
|   ├── ros2_RobotSimulation
│   └── README.md
├── bss_controller
│   ├── bss_controller
│   |   ├── CMakeLists.txt
│   |   └── include
│   ├── bss_controller_bringup
│   |   ├── CMakeLists.txt
│   |   └── include
│   ├── bss_controller_commander
│   |   ├── CMakeLists.txt
│   |   └── include
│   ├── bss_controller_interface
│   |   ├── CMakeLists.txt
│   |   └── include
│   └── README.md
├── bss_shelf
│   ├── bss_shelf_bringup
│   |   ├── CMakeLists.txt
│   |   └── include
│   └── README.md
├── bss_user_interface
│   └── README.md
=======
├── bss_description
│   ├── CMakeLists.txt
│   └── include
├── bss_gazebo
│   ├── CMakeLists.txt
│   └── include
├── bss_hardware
│   ├── CMakeLists.txt
│   └── include
├── bss_interface
│   ├── CMakeLists.txt
│   └── include
├── mobile_shelf
│   ├── lib
│   └── src
├── bss_arm_interface
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include
│   └── src
>>>>>>> 6b80ac0bb03b5d69faed0628a7d04e23b8c9c9b3
├── LICENSE
└── README.md
```
