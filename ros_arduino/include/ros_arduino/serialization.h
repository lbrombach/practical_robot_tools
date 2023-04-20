#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <vector>
#include <stdint.h>
#include "ros_arduino/SerialMessageTypes.h"

using namespace std;

const int getNumBytesPerElement(MessageType type);
template <typename T>
bool serialize(MessageType type, std::vector<T> &data, std::vector<uint8_t> &packet);
template <typename T>
bool unserialize(MessageType type, std::vector<T> &data, std::vector<uint8_t> packet, int start, int numBytes);

#endif

// #ifndef SERIALIZATION_H
// #define SERIALIZATION_H

// #include <vector>
// #include <stdint.h>
// #include "ros_arduino/SerialMessageTypes.h"
// #include <iostream>


// using namespace std;

// const int getNumBytesPerElement(MessageType type)
// {
//     switch (type)
//     {
//     case MessageType::MSG_TYPE_EMPTY:
//         return 0;
//         break;
//     case MessageType::MSG_TYPE_BOOL:
//         return 1;
//         break;
//     case MessageType::MSG_TYPE_INT8:
//         return 1;
//         break;
//     case MessageType::MSG_TYPE_UINT8:
//         return 1;
//         break;
//     case MessageType::MSG_TYPE_INT16:
//         return 2;
//         break;
//     case MessageType::MSG_TYPE_INT32:
//         return 4;
//         break;
//     case MessageType::MSG_TYPE_FLOAT32:
//         return 4;
//         break;
//     case MessageType::MSG_TYPE_FLOAT64:
//         return 8;
//         break;

//     default:
//         return 0;
//         break;
//     }
// }

// // adds data_length (2 bytes) and data (n bytes per element) to packet
// template <typename T>
// bool serialize(MessageType type, std::vector<T> &data, std::vector<uint8_t> &packet)
// {
//     int numBytesPerData = getNumBytesPerElement(type);
//     cout<< "numBytesPerData: " << numBytesPerData << endl;
//     uint16_t data_length = data.size() * numBytesPerData;
//     packet.push_back((data_length >> 8) & 0xff); // high byte first
//     packet.push_back(data_length & 0xff);        // low byte second
//     for (auto i : data)
//     {
//         if (numBytesPerData >= 4)
//         {
//             packet.push_back((i >> 24) & 0xff); // high byte first
//             packet.push_back((i >> 16) & 0xff);
//         }
//         if (numBytesPerData >= 2)
//         {
//             packet.push_back((i >> 8) & 0xff);        
//         }
//         if (numBytesPerData >= 1)
//         {
//             packet.push_back(i & 0xff); // low byte last
//             cout<<(int)packet.back()<< " ";
//         }
//     }
//     return true;
// }

// // unpacks packet into data vector
// // start is index of packet where the data begins
// // numBytes is the number of bytes in packet that contains data.
// // The end data.size() should equal numBytes/numBytesPerData
// template <typename T>
// bool unserialize(MessageType type, std::vector<T> &data, std::vector<uint8_t> packet, int start, int numBytes)
// {
//     int numBytesPerData = getNumBytesPerElement(type);

//     for (int i = start; i < start + numBytes; i += numBytesPerData)
//     {
//         T d;

//         if (numBytesPerData == 0)
//         {
//             return true;
//         }
//         else if (numBytesPerData > 0 && numBytesPerData <= sizeof(T))
//         {
//             d = 0;
            
//             for (int j = 0; j < numBytesPerData; j++)
//             {
//                 int place = numBytesPerData - j - 1;
//                 d |= static_cast<T>(packet[i + j]) << (8 * place);
//             }
//         }
//         else
//         {
//             // Unsupported numBytesPerData
//             return false;
//         }

//         data.push_back(d);
//     }
//     cout<<endl;
//     return true;
// }

// #endif