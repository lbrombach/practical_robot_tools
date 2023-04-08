#include "ros_arduino/SerialMessage.h"
#include "ros_arduino/serialization.h"
#include <iostream>

using namespace std;

template class SerialMessage<bool>;

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

// Function to convert boolean
// into string
std::string btos(bool x)
{
    if (x)
    {
        return "1";
    }
    return "0";
}

template <class T>
std::vector<uint8_t> SerialMessage<T>::pack()
{
    cout << getNumBytesPerElement(MSG_TYPE_EMPTY) << " "
         << getNumBytesPerElement(MSG_TYPE_BOOL) << " "
         << getNumBytesPerElement(MSG_TYPE_INT8) << " "
         << getNumBytesPerElement(MSG_TYPE_UINT8) << " "
         << getNumBytesPerElement(MSG_TYPE_INT16) << endl;

    std::vector<uint8_t> packet;
    packet.insert(packet.end(), SerialMessage::START_SIGNAL, SerialMessage::START_SIGNAL + 3);
    uint16_t _type = (uint16_t)type;
    packet.push_back((_type >> 8) & 0xff); // high byte first
    packet.push_back(_type & 0xff);        // low byte second
    uint16_t topic_length = (uint16_t)topic.length();
    packet.push_back((topic_length >> 8) & 0xff); // high byte first
    packet.push_back(topic_length & 0xff);        // low byte second
    packet.insert(packet.end(), topic.begin(), topic.end());
    serialize<T>(type, data, packet);
    //  uint16_t data_length = data.size();          // always 1 for bool
    //  packet.push_back((data_length >> 8) & 0xff); // high byte first
    //  packet.push_back(data_length & 0xff);        // low byte second

    //   T d;
    //////////////
    // TODO make work for all types and ////
    //  std::cout << "packing data like: ";
    //  for (auto i : data)
    //  {
    //      d = i | 0x00;
    //      std::cout << btos(d) << " ";
    //      packet.push_back(d);
    // }
    ///////////////////
    //  std::cout << std::endl;

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
    std::cout << "Unpacking... " << std::endl;
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
        cout << hex << (int)packet[i] << " ";
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

    default:
        cout << "Error - not a valid type :" << _type << endl;
        temp.type = MessageType::MSG_TYPE_NONE;
        break;
    }

    /////////////////////////////////////////////////////////////////////////////////  make work for variable data length
    cout << "1: type = " << temp.type << endl;

    // Read topic length and topic name
    temp.topic_length = ((uint16_t)packet[5] << 8) | (uint16_t)packet[6];
    cout << "2: topic len = " << temp.topic_length << endl;

    temp.topic = std::string(packet.begin() + 7, packet.begin() + 7 + temp.topic_length);
    cout << "packet size 3: " << packet.size() << endl;

    // Read data length and data

    int data_length = ((uint16_t)packet[7 + temp.topic_length] << 8) | (uint16_t)packet[7 + temp.topic_length + 1];
    int dataStart = 7 + temp.topic_length + 2;

    for (int i = dataStart; i < dataStart + data_length; i++)
    {
        temp.data.push_back(packet[i] != 0);
        std::cout << i << " : " << (int)packet[i] << std::endl;
    }
    std::cout << "dataStart  .. data rawLength vs data.size()... : " << dataStart << " .. " << data_length << " vs " << temp.data.size() << " and val " << temp.data[18] << std::endl;
    cout << ".........................................." << endl;
    cout << "temp in unpack:" << endl;
    temp.show();

    // Update object state
    // temp.type = MessageType::MSG_TYPE_BOOL;
    // topic = topic_name;
    // data = data_value;
    return temp;
}
// 3 start bytes, 2 type bytes, 2 topic length bytes, n topic name bytes, 2 data length bytes, 1 data byte, checksum

// Packed bytes: aa aa aa 00 01 00 09 74 65 73 74 5f 62 6f 6f 6c 00 01 01 d5 05
// 3                start            type     topic length           topic name                  data length       data          checksum
//                   0,2              3,4        5,6                   7, 7+len-1              7+len, 7+len+1     7+len+2    7+len+3, 7+len+4
//                   0,2              3,4        5,6                   7, 15                         16, 17         18            19, 20
//                  aa aa aa         00 01      00 09          74 65 73 74 5f 62 6f 6f 6c           01 00           01             d5 05

template <class T>
void SerialMessage<T>::show() const
{
    cout << "Topic      : " << topic
         << "\nType       : " << type
         << "\nData Length: " << data.size()
         << "\nData       : ";

        for (auto i : data)
            cout
         << i << " ";
    cout << "\n....................................................................." << endl;
}