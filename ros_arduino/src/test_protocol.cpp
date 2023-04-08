#include "ros_arduino/SerialMessage.h"
#include "ros_arduino/SerialMessageTypes.h"
#include <rclcpp/rclcpp.hpp>

using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a SerialMessage object
  SerialMessage<bool> testBool(MSG_TYPE_BOOL, "test_bool");
  testBool.show();
  testBool.data.push_back(true);
  testBool.show();

  // Pack the SerialMessage object into a vector
  vector<uint8_t> packet = testBool.pack();

  // Unpack the vector into a new SerialMessage object
  SerialMessage<bool> resultBool;
  resultBool = resultBool.unpack(packet);

  // Print the original and unpacked SerialMessage objects
  cout << "Original SerialMessage unpacked: " << endl;
  testBool.show();
  cout << "Unpacked SerialMessage: " << endl;
  resultBool.show();

  resultBool.data[0] = false;
  resultBool.data.push_back(1);
  cout << "Changed SerialMessage: " << endl;
  resultBool.show();

  SerialMessage<bool> copiedMsg(resultBool);
  cout << "copied: " << endl;
  copiedMsg.show();
  cout << "###########################################" << endl;

  // Create a SerialMessage object
  SerialMessage<uint8_t> a(MSG_TYPE_UINT8, "test_uint8");
  a.data.resize(11);
  a.data.assign({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 255});
  a.show();

  // Pack the SerialMessage object into a vector to send over serial
  packet = a.pack();

  // Unpack the vector into a new SerialMessage object
  SerialMessage<uint8_t> b;
  b = b.unpack(packet);
  b.show();


  // // Create a SerialMessage object
  // SerialMessage<_Float32> f1(MSG_TYPE_FLOAT32, "test_float32");
  // f1.data.resize(11);
  // f1.data.assign({123.22, 1, 2, 0, 4, 5, 6, 7, 8, 9, 3256.88});
  // f1.show();

  // // Pack the SerialMessage object into a vector to send over serial
  // packet = a.pack();

  // // Unpack the vector into a new SerialMessage object
  // SerialMessage<uint8_t> f2;
  // f2 = f2.unpack(packet);
  // f2.show();


  rclcpp::shutdown();
  return 0;
}

// 3 start bytes, 2 type bytes, 2 topic length bytes, n topic name bytes, 2 data length bytes, 1 data byte, checksum

// Packed bytes: aa aa aa 00 01 00 09 74 65 73 74 5f 62 6f 6f 6c 00 01 01 d5 05
//                   0,2              3,4        5,6                   7, 7+len-1              7+len, 7+len+1     7+len+2    7+len+3, 7+len+4
//                   0,2              3,4        5,6                   7, 15                         16, 17         18            19, 20
// 3                start            type     topic length           topic name                  data length       data          checksum
//                  aa aa aa         00 01      00 09          74 65 73 74 5f 62 6f 6f 6c           01 00           01             d5 05
