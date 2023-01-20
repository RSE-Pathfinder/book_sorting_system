# book_sorting_system
ROS 2 Foxy Packages for book sorting system

How to use:
```bash
git clone https://github.com/RSE2107A-AY2122T3-Team-4/book_sorting_system.git
cd book_sorting_system/
git submodule init && git submodule update
sudo apt install cmake pkg-config
cd mobile_shelf/YDLidar-SDK && mkdir build && cd "$_"
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
BOOK_SORTING_SYSTEM
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
├── LICENSE
└── README.md
```
