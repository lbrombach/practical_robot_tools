cmake_minimum_required(VERSION 3.8)
project(rplidar_motor_control)


#find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  # other package include directories
)



add_executable(rplidar_motor_control src/rplidar_motor_control.cpp)
ament_target_dependencies(rplidar_motor_control rclcpp std_srvs sensor_msgs)

# Link against ROS and other required libraries
target_link_libraries(rplidar_motor_control 
  ${rclcpp_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${sensor_msgs_LIBRARIES})



# Install the executable
install(TARGETS rplidar_motor_control
DESTINATION lib/${PROJECT_NAME})