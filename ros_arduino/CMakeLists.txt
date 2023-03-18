cmake_minimum_required(VERSION 3.8)
project(ros_arduino)

## Compile as C++11, supported in ROS 2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)


## Declare the package dependencies
set(ROS_DEP_PACKAGES
  rclcpp
  std_msgs
)

## Set up the include directories
include_directories(include)

## Create the executable
add_executable(test_protocol src/test_protocol.cpp src/ROS2Message.cpp)

## Link the dependencies
ament_target_dependencies(test_protocol ${ROS_DEP_PACKAGES})

## Install the executable
install(TARGETS test_protocol
  DESTINATION lib/${PROJECT_NAME}
)

## Install the header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

## Set up the package information
ament_package()
