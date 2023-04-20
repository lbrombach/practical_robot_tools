// A ros2 node that publishes and subscribes to a serial message that contains
// a SerialMessage object with a vector of bytes that contain the desired or
// actual state of the pins on the arduino. The SerialMessage object is
// serialized and published as a message on the topic "arduino_cmd_msg". This
// message is subscribed to by the Serial Driver node which sends the message to
// the arduino. The arduino then sends a message back to the Serial Driver node
// with the actual state of the pins. This message is published on the topic
// "arduino_response_msg".

#include "ros_arduino/SerialMessage.h"
#include "ros_arduino/SerialMessageTypes.h"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>

using namespace std;

std::vector<uint8_t> MsgBuf;

void serial_read_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
  std::cout << "In serial_read_callback" << std::endl;
  const int MAX_MSG_SIZE = 50;

  // Append the new data from the incoming message to MsgBuf
  MsgBuf.insert(MsgBuf.end(), msg->data.begin(), msg->data.end());

  // Look for the start sequence in the MsgBuf
  const uint8_t start_sequence[] = {0xAA, 0xAA, 0xAA};
  auto it = std::search(MsgBuf.begin(), MsgBuf.end(), start_sequence,
                        start_sequence + 3);

  // Check if the start sequence is found and if there is enough data for the
  // message length
  while (it != MsgBuf.end() && MsgBuf.end() - it >= 5) {
    // Extract the message length (big endian)
    uint16_t msg_length = (static_cast<uint16_t>(*(it + 3)) << 8) | *(it + 4);

    // Check if the entire message has been received
    if (MsgBuf.end() - it >= msg_length) {
      // Process the message
      std::vector<uint8_t> packet(it, it + msg_length);
      SerialMessage<uint8_t> receivedMsg;
      receivedMsg = receivedMsg.unpack(packet);
      std::cout << "#####  Received message: #####" << std::endl;
      receivedMsg.show();


      // Remove the processed message from MsgBuf and look for the next start
      // sequence
      MsgBuf.erase(MsgBuf.begin(), it + msg_length);
    } else {
      // Check for a subsequent start sequence and erase everything before it
      auto next_start =
          std::search(it + 1, MsgBuf.end(), start_sequence, start_sequence + 3);
      if (next_start != MsgBuf.end()) {
        std::cout << "Found a subsequent start sequence" << std::endl;
        MsgBuf.erase(MsgBuf.begin(), next_start);
      } else {
        // Wait for more data to be received
        std::cout << "Waiting for more data" << std::endl;
        // Limit the received message length to 50
        if (msg_length > MAX_MSG_SIZE) {
          MsgBuf.clear();
          std::cout << "Message length too long" << std::endl;
          break;
        }
        break;
      }
    }
    it = std::search(MsgBuf.begin(), MsgBuf.end(), start_sequence,
                     start_sequence + 3);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("arduino_pin_ctrl_test_node");

  // Create a publisher for the UInt8MultiArray topic
  auto publisher = node->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "arduino_cmd_msg", 1);

  auto subscriber = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "serial_read", 10,
      std::bind(serial_read_callback, std::placeholders::_1));

  // Create a timer to publish a message every 1 second
  auto timer_callback = [publisher]() -> void {
    std_msgs::msg::UInt8MultiArray message;
    vector<uint8_t> msgPacket;

    std::cout << "Enter a msg type (1 or 2) to publish the message "
                 "(command/pollInputs) "
              << std::endl;

    int i = 1;
    cin >> i;

    if (i == 1) {
      // Create a SerialMessage object
      SerialMessage<uint8_t> msg(MSG_TYPE_UINT8, "outputPins");

      msg.data.push_back(0);
      msg.data.push_back(1);
      msg.data.push_back(1);
      msg.data.push_back(1);
      msg.data.push_back(0);
      msg.data.push_back(0);
      msg.data.push_back(1);
      msg.data.push_back(0);
      msg.show();

      // Pack the SerialMessage object into a vector
      msgPacket = msg.pack();
      message.data.resize(
          msgPacket.size()); // Resize the data vector to match the array size
      message.data = msgPacket;

    } else if (i == 2) {
      // Create a SerialMessage object
      SerialMessage<uint8_t> msg(MSG_TYPE_UINT8, "inputPins");
      // the topic alone is enough to identify that this is a request to poll
      // the arduino pins
      msgPacket = msg.pack();
      message.data.resize(
          msgPacket.size()); // Resize the data vector to match the array size
      message.data = msgPacket;
    }

    // Publish the message
    publisher->publish(message);
  };
  timer_callback();
  // timer_callback();
  auto timer =
      node->create_wall_timer(std::chrono::seconds(10), timer_callback);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
