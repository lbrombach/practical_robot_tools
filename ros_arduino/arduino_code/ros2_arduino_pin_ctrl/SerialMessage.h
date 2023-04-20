#ifndef SERIAL_MESSAGE_H
#define SERIAL_MESSAGE_H

#include <stdint.h>
#include <string>
#include <vector>

#include "SerialMessageTypes.h"
#include "serialization.h"

// The base class for ROS2 messages for a custom communication format
template <class T> class SerialMessage {
public:
  SerialMessage();
  SerialMessage(const SerialMessage &other);
  SerialMessage(MessageType type, const std::string &topic);
  ~SerialMessage();

  bool operator==(const SerialMessage<T> &other);

  MessageType type;
  std::string topic;
  // int topic_length;
  std::vector<T> data;

  // pack message into serialized format
  //// 3 start bytes, 2 type bytes, 2 topic length bytes, n topic name bytes, 2
  ///data length bytes, 1 data byte, checksum
  std::vector<uint8_t> pack();
  SerialMessage<T> unpack(const std::vector<uint8_t> &packet);
};

#endif
