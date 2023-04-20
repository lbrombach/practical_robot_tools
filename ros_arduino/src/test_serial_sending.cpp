#include "ros_arduino/SerialMessage.h"
#include "ros_arduino/SerialMessageTypes.h"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"


using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_serial_sending");

  // Create a publisher for the UInt8MultiArray topic
  auto publisher = node->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_pub_test", 1);

 


// Create a timer to publish a message every 1 second
    auto timer_callback = [publisher]() -> void 
    {
        std_msgs::msg::UInt8MultiArray message;
        vector<uint8_t> msgPacket;

        cout<<"Enter a msg type 1, 2, or 3 to publish the message (bool, uint8, int8): ";
        int i;
        cin>>i;

        if(i==1){
          // Create a SerialMessage object
          SerialMessage<bool> testBool(MSG_TYPE_BOOL, "bool_tst");
          testBool.show();
          testBool.data.push_back(true);
          testBool.data.push_back(false);
          testBool.data.push_back(true);
          testBool.data.push_back(true);
          testBool.data.push_back(false);
          testBool.data.push_back(false);
          testBool.data.push_back(false);
          testBool.show();

           // Pack the SerialMessage object into a vector
          msgPacket = testBool.pack();
          message.data.resize(msgPacket.size()); // Resize the data vector to match the array size
          message.data = msgPacket;

        }else if(i==2){
          // Create a SerialMessage object
          SerialMessage<int8_t> testInt8(MSG_TYPE_INT8, "int8_tst");
          testInt8.data.push_back(-2);
          testInt8.data.push_back(0);
          testInt8.data.push_back(127);
          testInt8.data.push_back(-55);
          testInt8.data.push_back(-128);
          testInt8.show();

           // Pack the SerialMessage object into a vector
          msgPacket = testInt8.pack();
          message.data.resize(msgPacket.size()); // Resize the data vector to match the array size
          message.data = msgPacket;
          

          }else if(i==3){
          // Create a SerialMessage object
          SerialMessage<uint8_t> testUInt8(MSG_TYPE_UINT8, "uint8_tst");
          testUInt8.data.push_back(2);
          testUInt8.data.push_back(0);
          testUInt8.data.push_back(77);
          testUInt8.data.push_back(3);
          testUInt8.data.push_back(125);
          testUInt8.data.push_back(255);
          testUInt8.show();

           // Pack the SerialMessage object into a vector
          msgPacket = testUInt8.pack();
          message.data.resize(msgPacket.size()); // Resize the data vector to match the array size
          message.data = msgPacket;

          }else{
            cout<<"Invalid input"<<endl;
            return;
          }
                  

        // Publish the message
        publisher->publish(message);
    };
    auto timer = node->create_wall_timer(std::chrono::seconds(3), timer_callback);

    rclcpp::spin(node);


  rclcpp::shutdown();
  return 0;
}

