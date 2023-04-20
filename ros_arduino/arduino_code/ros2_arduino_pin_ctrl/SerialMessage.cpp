#include "SerialMessage.h"

#include <iostream>

using namespace std;

template class SerialMessage<bool>;
template class SerialMessage<uint8_t>;
template class SerialMessage<int8_t>;
template class SerialMessage<int16_t>;
template class SerialMessage<int32_t>;
//template class SerialMessage<float>;


template <class T>
SerialMessage<T>::SerialMessage()
{
}

template <class T>
SerialMessage<T>::SerialMessage(const SerialMessage &other)
{
  type = other.type;
  topic = other.topic;
  data.resize(other.data.size());
  std::copy(other.data.begin(), other.data.end(), data.begin());
}

template <class T>
SerialMessage<T>::SerialMessage(MessageType _type, const std::string &topic_name) : type(_type), topic(topic_name)
{
}

template <class T>
SerialMessage<T>::~SerialMessage() {}

template <class T>
bool SerialMessage<T>::operator==(const SerialMessage<T> &other)
{
  return type == other.type &&
         topic == other.topic &&
         data == other.data;
}

template <class T>
std::vector<uint8_t> SerialMessage<T>::pack()
{
  std::vector<uint8_t> message;
  //msg format is: type(2 bytes) + topic_length(2 bytes) + topic(n bytes) + data_length(2 bytes) + data(n bytes)

  uint16_t _type = (uint16_t)type;
  message.push_back((_type >> 8) & 0xff); // high byte first
  message.push_back(_type & 0xff);        // low byte second
  uint16_t topic_length = (uint16_t)topic.length();
  message.push_back((topic_length >> 8) & 0xff); // high byte first
  message.push_back(topic_length & 0xff);        // low byte second
  message.insert(message.end(), topic.begin(), topic.end());

  if (type != MessageType::MSG_TYPE_EMPTY)
    serialize<T>(type, data, message);

  return message;
}

template <class T>
SerialMessage<T> SerialMessage<T>::unpack(const std::vector<uint8_t> &message)
{
  SerialMessage temp;

  // Check message type
  uint16_t _type = ((uint16_t)message[0] << 8) | (uint16_t)message[1];

  switch (_type & 0xff)
  {
    case 0:
      temp.type = MessageType::MSG_TYPE_EMPTY;
      break;
    case 1:
      temp.type = MessageType::MSG_TYPE_BOOL;
      break;
    case 2:
      temp.type = MessageType::MSG_TYPE_INT8;
      break;
    case 3:
      temp.type = MessageType::MSG_TYPE_UINT8;
      break;
    case 4:
      temp.type = MessageType::MSG_TYPE_INT16;
      break;
    case 5:
      temp.type = MessageType::MSG_TYPE_INT32;
      break;

    default:
      temp.type = MessageType::MSG_TYPE_NONE;
      break;
  }

  // Read topic length and topic name
  int topic_length = ((uint16_t)message[2] << 8) | (uint16_t)message[3];

  temp.topic = std::string(message.begin() + 4, message.begin() + 4 + topic_length);

  int data_length = ((uint16_t)message[4 + topic_length] << 8) | (uint16_t)message[4 + topic_length + 1];
  int dataStart = 4 + topic_length + 2;

  if (temp.type != MessageType::MSG_TYPE_EMPTY) {
    unserialize(temp.type, temp.data, message, dataStart, data_length);
  } else
    temp.topic = "emptyMsg";

  return temp;
}
