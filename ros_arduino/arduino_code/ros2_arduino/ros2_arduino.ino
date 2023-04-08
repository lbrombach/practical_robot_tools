#include <ArduinoSTL.h>
#include <SerialMessageTypes.h>
//#include <serialization.h>
#include <SerialMessage.h>
#include <Arduino.h>

#include "SerialMessage.h"
#include "SerialMessageTypes.h"
#include <vector>

void setup() {
  Serial.begin(9600);
  
  // Create empty message test
  SerialMessage<bool> emptyMsg(MSG_TYPE_EMPTY, "test_empty");
  show(emptyMsg);
  
  // Bool test
  SerialMessage<bool> boolMsg(MSG_TYPE_BOOL, "test_bool");
  boolMsg.data.resize(5);
  boolMsg.data.assign({0, 1, 1, 0, 1});
  show(boolMsg);
  std::vector<uint8_t> boolPacket = boolMsg.pack();
  SerialMessage<bool> boolMsgUnpack;
  boolMsgUnpack.unpack(boolPacket);
  show(boolMsgUnpack);
  
  // Uint8 test
  SerialMessage<uint8_t> uint8Msg(MSG_TYPE_UINT8, "test_uint8");
  uint8Msg.data.resize(11);
  uint8Msg.data.assign({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 255});
  show(uint8Msg);
  std::vector<uint8_t> uint8Packet = uint8Msg.pack();
  SerialMessage<uint8_t> uint8MsgUnpack;
  uint8MsgUnpack.unpack(uint8Packet);
  show(uint8MsgUnpack);
  
  // Int8 test
  SerialMessage<int8_t> int8Msg(MSG_TYPE_INT8, "test_int8");
  int8Msg.data.resize(11);
  int8Msg.data.assign({-128, 1, -2, 3, 4, 0, 6, 7, -8, 9, 127});
  show(int8Msg);
  std::vector<uint8_t> int8Packet = int8Msg.pack();
  SerialMessage<int8_t> int8MsgUnpack;
  int8MsgUnpack.unpack(int8Packet);
  show(int8MsgUnpack);
  
  // Int16 test
  SerialMessage<int16_t> int16Msg(MSG_TYPE_INT16, "test_int16");
  int16Msg.data.resize(11);
  int16Msg.data.assign({INT16_MIN, 768, 1, 2, 3, 4, 0, 6, 7, 8, 9, 32, INT16_MAX});
  show(int16Msg);
  std::vector<uint8_t> int16Packet = int16Msg.pack();
  SerialMessage<int16_t> int16MsgUnpack;
  int16MsgUnpack.unpack(int16Packet);
  show(int16MsgUnpack);
  
  // Int32 test
  SerialMessage<int32_t> int32Msg(MSG_TYPE_INT32, "test_int32");
  int32Msg.data.resize(11);
  int32Msg.data.assign({INT32_MIN, 768, 1, 2, 3, 4, 0, 6, 7, 8, 9, 32, INT32_MAX});
  show(int32Msg);
  std::vector<uint8_t> int32Packet = int32Msg.pack();
  SerialMessage<int32_t> int32MsgUnpack;
  int32MsgUnpack.unpack(int32Packet);
  show(int32MsgUnpack);
}

void loop() {
  // Do nothing
}

template <class T>
void show(T msg) 
{
    Serial.print("Topic      : ");
    Serial.println(msg.topic.c_str());
    Serial.print("Type       : ");
    Serial.println(msg.type);
    Serial.print("Data Length: ");
    Serial.println(msg.data.size());
    Serial.print("Data       : ");

    for (auto i : msg.data)
    {
        Serial.print((int)i);
        Serial.print(" ");
    }

    Serial.println("\n.....................................................................");
}
