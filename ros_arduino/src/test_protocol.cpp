#include "ros_arduino/ROS2Message.h"
#include "ros_arduino/ros2MsgTypes.h"
//#include "ros_arduino/ros2MsgUnpack.h"
#include <rclcpp/rclcpp.hpp>

using namespace std;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  cout<<"hello"<<endl;

  // Create a ROS2Message of type MSG_TYPE_INT8 with a value of 213
  const uint8_t int8_data[1] = {213};
  ROS2Message testInt8(MSG_TYPE_INT8, "test_topic", int8_data, sizeof(int8_data));

  // Print the message type, topic name, and data value to the console
  //printf("Message type: %d\n", testInt8.getType());
  //printf("Topic name: %s\n", testInt8.getTopicName().c_str());

  int8_t int8_value;
  //if (testInt8.unpack(&int8_value)) {
  //  printf("Data value: %d\n", int8_value);
  //}

  rclcpp::shutdown();
  return 0;
}
