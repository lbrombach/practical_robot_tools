#include "ros_arduino/PinDevice.h"
#include "ros_arduino/SerialMessage.h"
#include "ros_arduino/SerialMessageTypes.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

using namespace std;

std::vector<PinDevice> pin_devices;
std::vector<uint8_t> input_pins;
std::vector<uint8_t> MsgBuf;

void serial_read_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
  //// std::cout << "In serial_read_callback" << std::endl;
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
      if (receivedMsg.topic == "outputPins") { // this is a message to set the
                                               // state of required devices
        for (int i = 0; i < pin_devices.size() && i < receivedMsg.data.size();
             i++) {
          if (pin_devices[i].getControlType() ==
                  PinDevice::DeviceControlType::ACTIVE_LOW ||
              pin_devices[i].getControlType() ==
                  PinDevice::DeviceControlType::ACTIVE_HIGH) {
            bool newRequiredState = (receivedMsg.data[i] >= 1) ? true : false;
            pin_devices[i].setDeviceState(newRequiredState);
          }
        }
        receivedMsg.show();

        // Remove the processed message from MsgBuf and look for the next start
        // sequence
        MsgBuf.erase(MsgBuf.begin(), it + msg_length);
      } else if (receivedMsg.topic == "inputPins") { // this is a messaage containg
                                                     // the state of the arduino input
                                                     // pins
        // TODO : add code to update the state of the input pins
        receivedMsg.show();
        // Remove the processed message from MsgBuf and look for the next start
        // sequence
        MsgBuf.erase(MsgBuf.begin(), it + msg_length);
      } else {
        // Remove the processed message from MsgBuf and look for the next start
        // sequence
        MsgBuf.erase(MsgBuf.begin(), it + msg_length);
      }

      }else {
        // Check for a subsequent start sequence and erase everything before it
        auto next_start = std::search(it + 1, MsgBuf.end(), start_sequence,
                                      start_sequence + 3);
        if (next_start != MsgBuf.end()) {
          ////     std::cout << "Found a subsequent start sequence" <<
          /// std::endl;
          MsgBuf.erase(MsgBuf.begin(), next_start);
        } else {
          // Wait for more data to be received
          ////     std::cout << "Waiting for more data" << std::endl;
          // Limit the received message length to 50
          if (msg_length > MAX_MSG_SIZE) {
            MsgBuf.clear();
            ////        std::cout << "Message length too long" << std::endl;
            break;
          }
          break;
        }
      }
      it = std::search(MsgBuf.begin(), MsgBuf.end(), start_sequence,
                       start_sequence + 3);
    }
  }
}

void device_required_callback(const std_msgs::msg::Bool::SharedPtr msg,
                              int index) {
  if (index >= 0 && index < pin_devices.size()) {
    pin_devices[index].setDeviceNeeded(msg->data);
    std::cout << "Device " << pin_devices[index].getName()
              << " required: " << msg->data << std::endl;
  } else {
    std::cerr << "Invalid device index: " << index << std::endl;
  }
}

void update_pin_states(
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher) {
  static std::vector<uint8_t> prev_pin_states(pin_devices.size());
  static bool first_run = true;
  std::cout << "here0" << std::endl; ////////////
  SerialMessage<uint8_t> msg(MSG_TYPE_UINT8, "outputPins");
  bool states_changed = false;

  for (int i = 0; i < pin_devices.size(); i++) {

    std::cout << "here1" << std::endl; ////////////

    msg.data.push_back(pin_devices[i].getPinState());
    // if device name == "Hoverboard PWR BTN" then output the msg.data[i] and
    // the prev_pin_states[i]
    if (pin_devices[i].getName() == "Hoverboard PWR BTN") {
      std::cout << "pinstate/prevState/devState : " << (int)msg.data[i] << " "
                << (int)prev_pin_states[i] << " "
                << (int)pin_devices[i].getDeviceState() << std::endl;
    }
    std::cout << "here2" << std::endl; ////////////

    if (msg.data[i] != prev_pin_states[i]) {
      states_changed = true;
    }
  }
  std::cout << "here3" << std::endl; ////////////

  if (states_changed || first_run) {
    first_run = false;
    states_changed = false;
    // Pack the SerialMessage object into a vector
    std::vector<uint8_t> msgPacket = msg.pack();
    std_msgs::msg::UInt8MultiArray message;
    message.data.resize(
        msgPacket.size()); // Resize the data vector to match the array size
    message.data = msgPacket;

    // Publish the message
    publisher->publish(message);

    // Update the previous pin states
    prev_pin_states = msg.data;
    std::cout << "Pin states updated" << std::endl;
  } else {
    std::cout << "No pin states changed" << std::endl;
  }
}

// Initialize output devices with appropriate IDs and control types
void initializeOutputDevices() {
  pin_devices.emplace_back("Hoverboard PWR BTN",
                           PinDevice::OutputDeviceID::HOVERBOARD_PWR_BTN,
                           PinDevice::DeviceControlType::PULSE_ACTIVE_LOW);
  pin_devices.emplace_back("E-Stop Relay",
                           PinDevice::OutputDeviceID::E_STOP_RELAY,
                           PinDevice::DeviceControlType::ACTIVE_LOW);
  pin_devices.emplace_back("Charge Connect Relay",
                           PinDevice::OutputDeviceID::CHARGE_CONNECT_RELAY,
                           PinDevice::DeviceControlType::ACTIVE_LOW);
  pin_devices.emplace_back(
      "Charge Probe Isolate Relay",
      PinDevice::OutputDeviceID::CHARGE_PROBE_ISOLATE_RELAY,
      PinDevice::DeviceControlType::ACTIVE_LOW);
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ros2_arduino_pin_ctrl_node");

  initializeOutputDevices();

  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
      device_subscribers;

  std::vector<std::string> device_names = {
      "device_hoverboard_required",     "device_estop_required",
      "device_charge_connect_required", "device_charge_probe_isolate_required",
      "device_spare_1_required",        "device_spare_2_required",
      "device_spare_3_required",        "device_spare_4_required"};

  device_subscribers.reserve(device_names.size());

  for (const auto &device_name : device_names) {
    int index = &device_name - &device_names[0];
    device_subscribers.push_back(node->create_subscription<std_msgs::msg::Bool>(
        device_name, 10, [index](const std_msgs::msg::Bool::SharedPtr msg) {
          device_required_callback(msg, index);
        }));
  }

  // TODO initialize the input pins vector

  auto subscriber = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "serial_read", 10,
      std::bind(serial_read_callback, std::placeholders::_1));

  // TODO: Create the publisher for the "arduino_cmd_msg" topic
  auto arduino_cmd_msg_publisher =
      node->create_publisher<std_msgs::msg::UInt8MultiArray>("/arduino_cmd_msg",
                                                             1);

  // Create the subscriber for the "poll_pins" topic
  auto poll_pins_subscriber = node->create_subscription<std_msgs::msg::Empty>(
      "/poll_pins", 1,
      [arduino_cmd_msg_publisher](
          const std_msgs::msg::Empty::SharedPtr prompt_msg) {
        std::cout << "Here 4" << std::endl;
        // Create a SerialMessage object
        SerialMessage<uint8_t> inputPinsMsg(MSG_TYPE_UINT8, "inputPins");
        std::vector<uint8_t> msgPacket = inputPinsMsg.pack();
        std_msgs::msg::UInt8MultiArray message;
        message.data.resize(msgPacket.size());
        message.data = msgPacket;
        // Publish the message
        arduino_cmd_msg_publisher->publish(message);
      });

  // Create a timer to call the send_pin_states function 10 times per second
  auto timer = node->create_wall_timer(
      std::chrono::milliseconds(500), [&arduino_cmd_msg_publisher]() {
        update_pin_states(arduino_cmd_msg_publisher);
      });

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
