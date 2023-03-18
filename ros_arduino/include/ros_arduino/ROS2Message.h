#ifndef ROS2_MESSAGE_H
#define ROS2_MESSAGE_H

#include <string>
#include "ros_arduino/ros2MsgTypes.h"

class ROS2Message {
public:
  static const uint8_t START_SIGNAL[3];
  static const int CHECKSUM_LENGTH = 2;

  ROS2Message();
  ROS2Message(MessageType type, const std::string& topic_name, const uint8_t *data, size_t data_length);
  ~ROS2Message();
  MessageType getType() const;
  const std::string& getTopicName() const;
  const uint8_t *getData() const;
  size_t getDataLength() const;
  size_t getMessageLength() const;
  bool unpack(void *data) const;

  template<typename T>
  bool unpackData(const uint8_t* data, size_t data_length, T& output) const;
  bool isValid() const;

private:
  MessageType _message_type;
  std::string _topic_name;
  uint8_t *_data;
  size_t _data_length;

  uint16_t computeChecksum() const;
};

#endif /* ROS2_MESSAGE_H */
