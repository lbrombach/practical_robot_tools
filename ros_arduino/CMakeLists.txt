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

## Set up the package information
ament_package()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

## Create the executable
#add_executable(test_protocol src/test_protocol.cpp src/ROS2Bool.cpp src/ROS2Msg.cpp)
## Link the dependencies
#ament_target_dependencies(test_protocol ${ROS_DEP_PACKAGES})

## Create the executable
add_executable(test_protocol src/test_protocol.cpp src/SerialMessage.cpp )
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




if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}pgk_test test/serial_message_test.cpp src/SerialMessage.cpp )
  target_include_directories(${PROJECT_NAME}pgk_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}pgk_test
    std_msgs
  )
 # target_link_libraries(${PROJECT_NAME}pgk_test name_of_local_library)
endif()