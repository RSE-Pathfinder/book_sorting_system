# book_sorting_system
ROS 2 Foxy Packages for book sorting system

How to use:
```bash
git clone https://github.com/RSE2107A-AY2122T3-Team-4/book_sorting_system.git
cd book_sorting_system/
git submodule init && git submodule update
sudo apt install cmake pkg-config
cd bss_shelf/bss_shelf_bringup/YDLidar-SDK && mkdir build && cd "$_"
cmake ..
make
sudo make install
cd ../../..
```

Working Tree:
```bash
BOOK_SORTING_SYSTEM
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
├── LICENSE
└── README.md
```
