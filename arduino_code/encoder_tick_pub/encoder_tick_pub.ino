/*
  encoder_tick_pub.ino is a ROS (Robot Operating System) publisher node that runs on
  and publishes an std_msgs::Int16 (-32,768 to 32,767) wheel encoder ticks
  for both left and right wheels. Tested on arduino nano every

  The QuadratureEncoderInt16.h file needs to be copied into a folder called QuadratureEncoderInt16 under the
  libraries folder that is used by the Arduino environment so that it becomes accessible to Arduino programs.
  Lloyd Brombach, November 2020
  lbrombach2@gmail.com
*/

#include <ros.h>
#include "QuadratureEncoderInt16.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#define PUBLISH_INTERVAL 50  //milliseconds
#define LEFT_ENCODER_A 3
#define LEFT_ENCODER_B 2
#define RIGHT_ENCODER_A 6
#define RIGHT_ENCODER_B 7

//create QuadratueEncoder object from QuadratureEncoderInt16.h
QuadratureEncoder leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
QuadratureEncoder rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

std_msgs::Int16 leftCount;
std_msgs::Int16 rightCount;

ros::NodeHandle nh;

ros::Publisher pubLeft("leftWheel", &leftCount);
ros::Publisher pubRight("rightWheel", &rightCount);

void setup()
{
  nh.initNode();
  nh.advertise(pubLeft);
  nh.advertise(pubRight);
  
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP); 
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP); 
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP); 
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), HandleInterruptLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), HandleInterruptLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), HandleInterruptRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), HandleInterruptRightB, CHANGE);
}

void loop()
{
  static long previousMilliseconds = 0;

  unsigned long currentMilliseconds = millis();
  unsigned long milliSecsSinceLastUpdate = abs(currentMilliseconds - previousMilliseconds);

  if (milliSecsSinceLastUpdate > PUBLISH_INTERVAL)
  {
    // save the last time we updated
    previousMilliseconds = currentMilliseconds;

    pubLeft.publish(&leftCount);
    pubRight.publish(&rightCount);
  }

  nh.spinOnce();
  delay(20);
}

void HandleInterruptLeftA()
{
  leftEncoder.OnAChanged();
  leftCount.data = (int)leftEncoder.getPosition();
}

void HandleInterruptLeftB()
{
  leftEncoder.OnBChanged();
  leftCount.data = (int)leftEncoder.getPosition();

}
void HandleInterruptRightA()
{
  rightEncoder.OnAChanged();
  rightCount.data = (int)rightEncoder.getPosition();
}

void HandleInterruptRightB()
{
  rightEncoder.OnBChanged();
  rightCount.data = (int)rightEncoder.getPosition();
}
