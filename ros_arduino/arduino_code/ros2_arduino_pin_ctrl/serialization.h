#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <vector>
#include <stdint.h>
#include "SerialMessageTypes.h"

using namespace std;

int getNumBytesPerElement(MessageType type);
template <typename T>
bool serialize(MessageType type, std::vector<T> &data, std::vector<uint8_t> &packet);
template <typename T>
bool unserialize(MessageType type, std::vector<T> &data, std::vector<uint8_t> packet, int start, int numBytes);

#endif
