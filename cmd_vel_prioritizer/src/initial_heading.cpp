// I think a good way to test would be to run two instances of rqt_robot_steering and one instance of the turtlesim_node.
//just change the topic name being published to turtle1/cmd_vel (or whatever the turtle listens to.. I think thats it)

#include "ros/ros.h"
#include <iostream>
//include the message types the node will use
#include "geometry_msgs/Twist.h"



using namespace std;
//int priority = 0;

//make global variables to store the velocity messages
//to see whats in this message type, check out http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
//then notice that it holds two instaces of another message type called vector3 http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Vector3.html
//so for the autoCmd..you would access individual data member like variable in an object in an object
// so to get forward velocity command its autoCmd.linear.x
geometry_msgs::Twist autoCmd;
geometry_msgs::Twist teleCmd;

//create flags so we can keep track of when we have new messages
bool newAutoMsgFlag = false;
bool newTeleMsgFlag = false;

//create a variable to hold the timestamp of the latest teleop command recieved
//The timestamp comes in a special ros format, but is easily changeable to a simple double
//that represents the number of seconds since the ros system has started. You'll see how below
double lastTeleMsgRecieved = -1;

//the callback parameter should be the same as the data type the message will contain
//If your run the gui steering program (rosrun rqt_robot_steering rqt_robot_steering)
//then do 'rostopic info messageName'  (in this case we could choose whatever message the steering node is generating, 
// so if just plain  cmd_vel use 'rostopic info cmd_vel').  
// The output will show you the message data type and some other stuff. In this case, it shows 'Type: geometry_msgs/Twist'
// geometry_msgs/Twist with a slash is a file name. To declare variables and parameters, we use the scope operator :: as above
void auto_cmdCB(const geometry_msgs::Twist & msg){
  ROS_INFO("auto_cmdCB heard: [%s]", msg->data.c_str());
  //**** now actually copy the data from the message. I *think* you can copy the whole message with =
  // instead of copying individual data member but check the output. For quick checks I personally find iostream and cout<< easier than ROS_INFO
  priority = 1;

  //set the new message flag to true so the main loop knows when to bother processing the message
  newAutoMsgFlag = true;
}

////fix the parameter on this one and also copy the data to tele_cmd
void teleop_cmdCB(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("auto_cmdCB heard: [%s]", msg->data.c_str());
  priority = 2;
  // after you copy the data, get the current time with ros::Time::now() the convert that to seconds with .toSec()
  lastTeleMsgRecieved = ros::Time::now().toSec();

  //set the new message flag to true so the main loop knows when to bother processing the message
  newTeleMsgFlag = true;
}

//create a helper method to calculate how many seconds has elapsed since whatever timestamp you pass to it
double getSecondsSince(double timeStamp){
  return ros::Time::now().toSec() - timeStamp;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_selector");
  ros::NodeHandle node;
  geometry_msgs::Twist twist;

  //Subscribe topics
  ros::Subscriber auto_cmd = node.subscribe("auto/cmd_vel", 1, &auto_cmdCB);
  ros::Subscriber teleop_cmd = node.subscribe("teleop/cmd_vel", 1, &teleop_cmdCB);
  
  //Initialize publish topics
  ros::Publisher pub = node.advertise<geometry_msgs::Twist>("hoverboard_velocity_controller/cmd_vel", 1);

  ////the first problem you'll run into there is that the program will run through this logic then exit. 
  //We ultimately need a loop so I'll template one below
  ////Republishing based on priority
  //if(priority == 2){
  //  pub.publish(auto_cmd);
  //} else if (priority == 1){
  //  pub.publish(teleop_cmd);
  //} else {
  //  //If you receive at the same time, teleop should be priority. 
  //  pub.publish(teleop_cmd);
  //}

  //declare a variable called loop_rate of a special ros type called Rate. 
  //This is ultimately how many times per second we want the loop to run. We want this to run fairly fast
  //to minimize delays, I think 100 times per second is plenty. May test to see if we can reduce to 50 or less.
  ros::Rate loop_rate(100);
  while (ros::ok() )
  {
    // this is the part where the callbacks are checked for new messages. If there are messages, the callback functions happen during the spin
    ros::spinOnce();
    
    
    //after the spin, we have the latest of both messages stored but they may be old. 
    //Check the message flags to determine which, if any should be re-broadcast

    //if there is a new tele message, that always gets broadcast and forget everything else
    if(newTeleMsgFlag == true){
      //we need to pass a ros message to the publisher, not the name of the subscriber
      pub.publish(teleCmd);
    }
    //before we publish the auto commands, we want a little delay after the last tele command
    //in case the operator needs a few seconds to press a power button or e-stop or whatever
    //I think 10 seconds is a good amount to start with
    //this if() tests if we even have a new auto message AND that it has been at least 10 seconds since the last teleop message
    else if (newAutoMsgFlag == true && getSecondsSince(lastTeleMsgRecieved) > 10 ){
      pub.publish(autoCmd);     
    }    

      
    //always reset the flags to false. We let the callbacks set it to true IF they get a new message
    newAutoMsgFlag = false;
    newTeleMsgFlag = false;

    // after the rest of the loop has executed, the loop_rate variabe has a function that can sleep the loop until 
    // enough time has passed to keep the frequency of the loop at 100 times per second. This is better than trying to add
    // a fixed sleep() time because this method automatically adjusts even if execution times change
    loop_rate.sleep();
  }



  return 0;
}
