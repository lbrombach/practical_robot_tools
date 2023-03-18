#include <ROS2Message.h>

#ifndef LED_BUILTIN // To support some boards (eg. some esp32 boards)
#define LED_BUILTIN 13
#endif 


void subscribeLed(std_msgs::Bool* msg, void* arg)
{
  (void)(arg);

  digitalWrite(LED_BUILTIN, msg->data);
}

class LedSub : public ros2::Node
{
public:
  LedSub()
  
};

void setup() 
{

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() 
{
  static LedSub LedNode;

  ros2::spin(&LedNode);
}
