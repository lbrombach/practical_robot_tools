#include "ros_arduino/SerialMessage.h"
#include "ros_arduino/serialization.h"
#include <iostream>

using namespace std;

template class SerialMessage<bool>;
template class SerialMessage<uint8_t>;
template class SerialMessage<int8_t>;
template class SerialMessage<int16_t>;
template class SerialMessage<int32_t>;
//template class SerialMessage<float>;

template <class T>
const uint8_t SerialMessage<T>::START_SIGNAL[3] = {0xAA, 0xAA, 0xAA};

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
    std::cout << "CHECKING " << std::endl;
    std::cout << bool(type == other.type) << std::endl;
    std::cout << bool(topic == other.topic) << std::endl;
    std::cout << bool(data == other.data) << std::endl;
    return type == other.type &&
           topic == other.topic &&
           data == other.data;
}

template <class T>
std::vector<uint8_t> SerialMessage<T>::pack()
{
    std::vector<uint8_t> packet;
    packet.insert(packet.end(), SerialMessage::START_SIGNAL, SerialMessage::START_SIGNAL + 3);
    uint16_t _type = (uint16_t)type;
    packet.push_back((_type >> 8) & 0xff); // high byte first
    packet.push_back(_type & 0xff);        // low byte second
    uint16_t topic_length = (uint16_t)topic.length();
    packet.push_back((topic_length >> 8) & 0xff); // high byte first
    packet.push_back(topic_length & 0xff);        // low byte second
    packet.insert(packet.end(), topic.begin(), topic.end());

    if(type != MessageType::MSG_TYPE_EMPTY)
        serialize<T>(type, data, packet);

    uint16_t checksum = 0;
    for (auto i : packet)
    {
        checksum += i;
    }

    packet.push_back((checksum >> 8) & 0xff);
    packet.push_back(checksum & 0xff);
    return packet;
}

template <class T>
SerialMessage<T> SerialMessage<T>::unpack(const std::vector<uint8_t> &packet)
{
    SerialMessage temp;
    // Check start signal
    if (packet[0] != SerialMessage::START_SIGNAL[0] ||
        packet[1] != SerialMessage::START_SIGNAL[1] ||
        packet[2] != SerialMessage::START_SIGNAL[2])
    {
        std::cout << "Error - bad packet start" << std::endl;
        temp.type = MessageType::MSG_TYPE_NONE;
    }

    // Verify checksum
    uint16_t checksum = 0;
    int packetSize = packet.size();
    for (int i = 0; i < packet.size() - 2; i++)
    {
        checksum += packet[i];
    }
    cout << dec << endl;

    uint16_t received_checksum = ((uint16_t)packet[packet.size() - 2] << 8) | (uint16_t)packet[packet.size() - 1];
    if (checksum != received_checksum)
    {
        std::cout << "Error - bad checksum . Got " << received_checksum << "  expected  " << checksum << std::endl;
        temp.type = MessageType::MSG_TYPE_NONE;
    }

    // Check message type
    uint16_t _type = ((uint16_t)packet[3] << 8) | (uint16_t)packet[4];

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
        cout << "Error - not a valid type :" << _type << endl;
        temp.type = MessageType::MSG_TYPE_NONE;
        break;
    }

    // Read topic length and topic name
    int topic_length = ((uint16_t)packet[5] << 8) | (uint16_t)packet[6];

    temp.topic = std::string(packet.begin() + 7, packet.begin() + 7 + topic_length);

    // Read data length and data
    int data_length = ((uint16_t)packet[7 + topic_length] << 8) | (uint16_t)packet[7 + topic_length + 1];
    int dataStart = 7 + topic_length + 2;

    if(type != MessageType::MSG_TYPE_EMPTY)
        unserialize(temp.type, temp.data, packet, dataStart, data_length);

    return temp;
}

template <class T>
void SerialMessage<T>::show() const
{
    cout << "Topic      : " << topic
         << "\nType       : " << type
         << "\nData Length: " << data.size()
         << "\nData       : ";

    for (auto i : data)
        cout
            << (int)i << " ";
    cout << "\n....................................................................." << endl;
}