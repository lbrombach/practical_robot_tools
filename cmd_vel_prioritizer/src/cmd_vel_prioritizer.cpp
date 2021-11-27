/*********************************************************************
* cmd_vel_prioritizer.cpp is a utility to select one of several
* geometry_msgs::Twist messages (often named cmd_vel) and only 
* republish the one with the highest priority. 
* This could, for example, allow teleop commands to override
* autonomously issued commands in a hurry without fighting the
* motion planner.
* 
*
* 
* Part of the practical_robot_utils collection at
* https://github.com/lbrombach/practical_robot_tools.git
*
* Author: Lloyd Brombach
* lbrombach2@gmail.com
* November 2021
**********************************************************************/

#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "vector"



using namespace std;


//helper method to calculate how many seconds has elapsed since whatever timestamp you pass to it
double getSecondsSince(double timeStamp){
  return ros::Time::now().toSec() - timeStamp;
}

//for holding data about the Twist messages received
struct Command{
	string topic;
	geometry_msgs::Twist cmd;
	double lastMsgRecieved; //time of the last received message
	double lockTime;  		//how long to prevent lower-priority topics from processing after lastMsgRecieved on this topic 
	bool newMsgFlag;  		//resets to false at end of every cycle of the main loop
	
	//constructor
	Command(string topic, double lockTime = 10){
		topic = topic;
		this->lastMsgRecieved = -1;
		this->lockTime = lockTime;
		newMsgFlag = false;
	};
	
	//returns whether this particular object has a relevent message
	bool wantsControl(){
		return getSecondsSince(lastMsgRecieved) <= lockTime;
	}
};
vector<Command> commands;

void setNewMsg(const geometry_msgs::Twist & msg, int index){
	commands[index].cmd = msg;
	commands[index].lastMsgRecieved = ros::Time::now().toSec();
	commands[index].newMsgFlag = true;
}

void input1CB(const geometry_msgs::Twist & msg){
	setNewMsg(msg, 0);
}
void input2CB(const geometry_msgs::Twist & msg){
	setNewMsg(msg, 1);
}
void input3CB(const geometry_msgs::Twist & msg){
	setNewMsg(msg, 2);
}
void input4CB(const geometry_msgs::Twist & msg){
	setNewMsg(msg, 3);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_vel_prioritizer");
	ros::NodeHandle node;
	  
	//get params
	double frequency = 30;
	double lockTime = .2;
	std::string outputTopic, topic1, topic2, topic3, topic4;
	ros::param::param<double>("/cmd_vel_prioritizer/pub_frequency", frequency, 30.0);
	ros::param::param<std::string>("/cmd_vel_prioritizer/output_topic", outputTopic, "cmd_vel");
	ros::param::param<std::string>("/cmd_vel_prioritizer/input_topic1", topic1, "teleop/cmd_vel");
	ros::param::param<std::string>("/cmd_vel_prioritizer/input_topic2", topic2, "none");
	ros::param::param<std::string>("/cmd_vel_prioritizer/input_topic3", topic3, "none");
	ros::param::param<std::string>("/cmd_vel_prioritizer/input_topic4", topic4, "none");
	
	
	//set our publisher	
	ROS_INFO("Starting cmd_vel_prioritizer node with output topic: %s", outputTopic.c_str()); 
	ros::Publisher cmdPub = node.advertise<geometry_msgs::Twist>(outputTopic, 1);
  
	
	//set subscribers. input1 must not be "none"
	ros::param::param<double>("/cmd_vel_prioritizer/input1_locktime", lockTime, 10);
	ROS_INFO("Starting cmd_vel_prioritizer node with input_topic1 topic: %s", topic1.c_str()); 
	commands.push_back(Command(topic1, lockTime));
	ros::Subscriber input1 = node.subscribe(topic1, 1, &input1CB);
	ros::Subscriber input2;
	ros::Subscriber input3;
	ros::Subscriber input4;

	
	//the rest of the subscribers are all added to commands vector for index number
	//consistency, but only registered as a subscriber if the topic is not "none"
	ros::param::param<double>("/cmd_vel_prioritizer/input2_locktime", lockTime, 0.2);
	ROS_INFO("Starting cmd_vel_prioritizer node with input_topic2 topic: %s", topic2.c_str()); 
	commands.push_back(Command(topic2, lockTime));
	if(topic2!="none")
		input2 = node.subscribe(topic2, 1, &input2CB);	
	
	
	ros::param::param<double>("/cmd_vel_prioritizer/input3_locktime", lockTime, 0.2);
	ROS_INFO("Starting cmd_vel_prioritizer node with input_topic3 topic: %s", topic3.c_str()); 
	commands.push_back(Command(topic3, lockTime));
	if(topic3!="none")
		input3 = node.subscribe(topic3, 1, &input3CB);	
	
	
	ros::param::param<double>("/cmd_vel_prioritizer/input4_locktime", lockTime, 0.2);
	ROS_INFO("Starting cmd_vel_prioritizer node with input_topic4 topic: %s", topic4.c_str()); 
	commands.push_back(Command(topic4, lockTime));
	if(topic4!="none")
		input4 = node.subscribe(topic4, 1, &input4CB);	
	

  ros::Rate loop_rate(frequency);
  while (ros::ok() )
  {
    ros::spinOnce();
    
	//iterate through vector, republishing only the highest priority message
    bool cmdSentFlag = false;
    for(int i = 0; i<commands.size() && !cmdSentFlag; i++){
		if(!cmdSentFlag && commands[i].wantsControl()){
			if(commands[i].newMsgFlag){
				cmdPub.publish(commands[i].cmd);
			}						
			cmdSentFlag = true;
		}
		commands[i].newMsgFlag = false; 	//reset all newMsgFlags to start again
	}

    loop_rate.sleep();
  }



  return 0;
}
