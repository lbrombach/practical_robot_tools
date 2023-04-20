cmake_minimum_required(VERSION 3.8)
project(ros_arduino)

## Compile as C++11, supported in ROS 2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Set the package dependencies
#set(serialport_DIR /usr/lib/x86_64-linux-gnu/pkgconfig)
#find_package(serialport REQUIRED CONFIG)

## Find libserialport library and header files
#find_library(SERIALPORT_LIB NAMES serialport)
#find_path(SERIALPORT_INCLUDE_DIR NAMES serialport.h)

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

## Declare the package dependencies
set(ROS_DEP_PACKAGES
  rclcpp
  std_msgs
)

## Set up the package information
ament_package()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
 # ${serialport_INCLUDE_DIRS}
 # ${SERIALPORT_INCLUDE_DIR}
 ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_arduino
)


## Create the executable
add_executable(test_protocol src/test_protocol.cpp src/SerialMessage.cpp src/serialization.cpp)
## Link the dependencies
ament_target_dependencies(test_protocol ${ROS_DEP_PACKAGES})
target_link_libraries(test_protocol)


add_executable(test_serial_sending src/test_serial_sending.cpp src/SerialMessage.cpp src/serialization.cpp)
## Link the dependencies
ament_target_dependencies(test_serial_sending ${ROS_DEP_PACKAGES})
target_link_libraries(test_serial_sending)


add_executable(ros2_arduino_pin_ctrl_node src/ros2_arduino_pin_ctrl_node.cpp src/SerialMessage.cpp src/serialization.cpp src/PinDevice.cpp)
## Link the dependencies
ament_target_dependencies(ros2_arduino_pin_ctrl_node ${ROS_DEP_PACKAGES})
target_link_libraries(ros2_arduino_pin_ctrl_node)

add_executable(arduino_pin_ctrl_test_node src/arduino_pin_ctrl_test_node.cpp src/SerialMessage.cpp src/serialization.cpp)
## Link the dependencies
ament_target_dependencies(arduino_pin_ctrl_test_node ${ROS_DEP_PACKAGES})
target_link_libraries(arduino_pin_ctrl_test_node)

## Install the executable
install(TARGETS test_protocol test_serial_sending ros2_arduino_pin_ctrl_node arduino_pin_ctrl_test_node
  DESTINATION lib/${PROJECT_NAME}
)

## Install the header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}pgk_test test/serial_message_test.cpp src/SerialMessage.cpp src/serialization.cpp)
  target_include_directories(${PROJECT_NAME}pgk_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}pgk_test
    std_msgs
  )
 # target_link_libraries(${PROJECT_NAME}pgk_test name_of_local_library)
endif()