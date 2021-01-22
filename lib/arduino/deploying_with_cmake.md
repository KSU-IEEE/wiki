# Deploying to arduinos with Cmake 
This document has two main parts: 
- deploying to arduinos using Arduino-CMake-Toolchain  
- deploying to arduinos using rosserial  

# Rosserial Deployment  
Rosserial leverages two different libraries for deployment: `catkin` and `Arduino-Cmake-Toolchain`. To install both of these, run 
```BASH 
$ sudo apt-get udpate
$ sudo apt-get install ros-melodic-rosserial-arduino
```
Likewise, you could run the dependencies script (which I always recomend) with: 
```BASH
cd /path/to/accessibiliy
python3 python/dependencies.py -d
```

## Directory Layout 
A rosserial deployment library should look like the following: 
```
my_directory
- foo
    - foo.cpp 
    - CMakeLists.txt 
CMakeLists.txt  
``` 
Let's look at a few things in this directory. 

First, there's two CMakeLists. The first one is the project CMakeLists. You can have as many arduino projects in `my_directory`, each one **MUST** have it's own directoy with a CMakeLists in it. 

CMakeLists at my_directory/CMakeLists is used for linking, and defining executables. The lirary_name/CMakeLists defines deployment. 

Secondly, the file.cpp should be similar to any .ino files you make. There should be an `init()` and `loop` function  

### library_name/CMakeLists 
Start your project off by defining the project CMakeLists.txt file. It should look like the following for the directory layout above:
```CMAKE
cmake_minimum_required(VERSION 2.8.3)
project(arduino-ros)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY foo
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(foo hello ALL)
rosserial_add_client_target(foo hello-upload)
```
You can literally copy and paste this into your CMakeLists.txt, you just have to replace all mentions of foo with your directory name.

### library_name/foo/CMakeLists 
This file should like the following: 
```CMAKE
cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(hello
  SRCS foo.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyS4
  SERIAL picocom /dev/ttyS4
)
```
Looking at this, there are few things going on. 