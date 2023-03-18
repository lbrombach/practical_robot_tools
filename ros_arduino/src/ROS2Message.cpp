#include "ros_arduino/ROS2Message.h"
#include <cstring>

const uint8_t ROS2Message::START_SIGNAL[3] = {0xAA, 0xAA, 0xAA};

ROS2Message::ROS2Message() :
  _message_type(MSG_TYPE_NONE),
  _topic_name(""),
  _data(nullptr),
  _data_length(0)
{}

ROS2Message::ROS2Message(MessageType type, const std::string& topic_name, const uint8_t* data, size_t data_length) :
  _message_type(type),
  _topic_name(topic_name),
  _data(new uint8_t[data_length]),
  _data_length(data_length)
{
  memcpy(_data, data, data_length);
}

ROS2Message::~ROS2Message()
{
  if (_data != nullptr) {
    delete[] _data;
  }
}

MessageType ROS2Message::getType() const
{
  return _message_type;
}

const std::string& ROS2Message::getTopicName() const
{
  return _topic_name;
}

const uint8_t* ROS2Message::getData() const
{
  return _data;
}

size_t ROS2Message::getDataLength() const
{
  return _data_length;
}

size_t ROS2Message::getMessageLength() const
{
  // Compute the length of the message, including the header and checksum
  return _data_length+2;
}

bool ROS2Message::unpack(void* data) const
{
  switch (_message_type)
  {
//    case MSG_TYPE_BOOL:
//      return unpackData(_data, _data_length, *(bool*)data);
 //   case MSG_TYPE_INT8:
//      return unpackData(_data, _data_length, *(int8_t*)data);
//    case MSG_TYPE_UINT8:
//      return unpackData(_data, _data_length, *(uint8_t*)data);
//    case MSG_TYPE_INT32:
//     return unpackData(_data, _data_length, *(int32_t*)data);
//    case MSG_TYPE_FLOAT32:
//      return unpackData(_data, _data_length, *(float*)data);
//    case MSG_TYPE_FLOAT64:
//      return unpackData(_data, _data_length, *(double*)data);
//    case MSG_TYPE_STRING:
//      return unpackString(_data, _data_length, (char*)data);
//   case MSG_TYPE_ARRAY:
//      return unpackArray(_data, _data_length, (uint8_t*)data);
    default:
      // Error handling for unknown message type
      return false;
  }
}

bool ROS2Message::isValid() const
{
  // Compute the expected checksum value
  uint16_t expected_checksum = computeChecksum();

  // Extract the actual checksum value from the message data
  uint16_t actual_checksum = (_data[_data_length - CHECKSUM_LENGTH] << 8) | _data[_data_length - 1];

  // Compare the expected and actual checksums
  return expected_checksum == actual_checksum;
}

uint16_t ROS2Message::computeChecksum() const
{
  // Initialize the checksum to 0
  uint16_t checksum = 0;

  // Compute the checksum over the entire message header and data
  size_t message_length = getMessageLength() - CHECKSUM_LENGTH;
  for (size_t i = 0; i < message_length; i++) {
    checksum += _data[i];
  }

  // Return the final checksum value
  return checksum;
}

template<typename T>
bool ROS2Message::unpackData(const uint8_t* data, size_t data_length, T& output) const
{
  if (data_length != sizeof(T))
  {
    return false; // Incorrect data length
  }

  memcpy(&output, data, sizeof(T));
  return true;
}