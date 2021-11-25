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

//make global variables to store the velocity messages
geometry_msgs::Twist autoCmd;
geometry_msgs::Twist teleCmd;

//helper method to calculate how many seconds has elapsed since whatever timestamp you pass to it
double getSecondsSince(double timeStamp){
  return ros::Time::now().toSec() - timeStamp;
}

struct Command{
	string topic;
	geometry_msgs::Twist cmd;
	double lastMsgRecieved;
	double lockTime;
	bool newMsgFlag;
	
	Command(string topic, double lockTime = 10){
		topic = topic;
		this->lastMsgRecieved = -1;
		this->lockTime = lockTime;
		newMsgFlag = false;
	};
	
	bool wantsControl(){
		cout<<topic<<" wants control: "<<(getSecondsSince(lastMsgRecieved) <= lockTime);
		cout<<"  "<<getSecondsSince(lastMsgRecieved)<<"   "<<lockTime<<endl;
		return getSecondsSince(lastMsgRecieved) <= lockTime;
	}
	
	void setCommand(geometry_msgs::Twist &newCommand){
		cout<<topic<<" new command set: "<<endl;
		cmd = newCommand;
		lastMsgRecieved = ros::Time::now().toSec();
		newMsgFlag = true;
	}
	
};
vector<Command> commands;

void input1CB(const geometry_msgs::Twist & msg){
	geometry_msgs::Twist newCmd;
	newCmd.linear.x = msg.linear.x;
	newCmd.linear.y = msg.linear.y;
	newCmd.angular.z = msg.angular.z;
	commands[0].setCommand(newCmd);
	cout<<"GOT1"<<endl;
}
void input2CB(const geometry_msgs::Twist & msg){
	geometry_msgs::Twist newCmd;
	newCmd.linear.x = msg.linear.x;
	newCmd.linear.y = msg.linear.y;
	newCmd.angular.z = msg.angular.z;
	commands[1].setCommand(newCmd);
}
void input3CB(const geometry_msgs::Twist & msg){
	geometry_msgs::Twist newCmd;
	newCmd.linear.x = msg.linear.x;
	newCmd.linear.y = msg.linear.y;
	newCmd.angular.z = msg.angular.z;
	commands[2].setCommand(newCmd);
}
void input4CB(const geometry_msgs::Twist & msg){
	geometry_msgs::Twist newCmd;
	newCmd.linear.x = msg.linear.x;
	newCmd.angular.z = msg.angular.z;
	commands[3].setCommand(newCmd);
	cout<<"GOT4"<<endl;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_vel_prioritizer");
	ros::NodeHandle node;
	  
	//get params and build vector of topics to monitor
	double frequency;
	double lockTime;
	std::string topic_name;
	ros::param::param<double>("/cmd_vel_prioritizer/pub_frequency", frequency, 100.0);
	cout<<"freq = " <<frequency<<endl;
	ROS_INFO("Starting cmd_vel_prioritizer node with pub frequency: %f", frequency);
	
	//first our publisher
	ros::param::param<std::string>("/cmd_vel_prioritizer/output_topic", topic_name, "/cmd_vel");
	ROS_INFO("Starting cmd_vel_prioritizer node with output topic: %s", topic_name.c_str()); 
	ros::Publisher cmdPub = node.advertise<geometry_msgs::Twist>(topic_name, 1);
  
	
	//Subscribe topics
	ros::param::param<std::string>("/cmd_vel_prioritizer/input_topic1", topic_name, "teleop/cmd_vel");
	ros::param::param<double>("/cmd_vel_prioritizer/input1_locktime", lockTime, 10);
	ROS_INFO("Starting cmd_vel_prioritizer node with input_topic1 topic: %s", topic_name.c_str()); 
	commands.push_back(Command(topic_name, lockTime));
	ros::Subscriber input1 = node.subscribe(topic_name, 1, &input1CB);
	ros::Subscriber input2;
	ros::Subscriber input3;
	ros::Subscriber input4;

	
	ros::param::param<std::string>("/cmd_vel_prioritizer/input_topic2", topic_name, "none");
		ros::param::param<double>("/cmd_vel_prioritizer/input2_locktime", lockTime, 0.2);
		ROS_INFO("Starting cmd_vel_prioritizer node with input_topic2 topic: %s", topic_name.c_str()); 
		commands.push_back(Command(topic_name, lockTime));
		input2 = node.subscribe(topic_name, 1, &input2CB);	
	
	ros::param::param<std::string>("/cmd_vel_prioritizer/input_topic3", topic_name, "none");
		ros::param::param<double>("/cmd_vel_prioritizer/input3_locktime", lockTime, 0.2);
		ROS_INFO("Starting cmd_vel_prioritizer node with input_topic3 topic: %s", topic_name.c_str()); 
		commands.push_back(Command(topic_name, lockTime));
		input3 = node.subscribe(topic_name, 1, &input3CB);	
	
	ros::param::param<std::string>("/cmd_vel_prioritizer/input_topic4", topic_name, "none");
		ros::param::param<double>("/cmd_vel_prioritizer/input4_locktime", lockTime, 0.2);
		ROS_INFO("Starting cmd_vel_prioritizer node with input_topic4 topic: %s", topic_name.c_str()); 
		commands.push_back(Command(topic_name, lockTime));
		input4 = node.subscribe(topic_name, 1, &input4CB);	
	

  ros::Rate loop_rate(frequency);
  while (ros::ok() )
  {
    ros::spinOnce();
    
    bool cmdSentFlag = false;
    for(int i = 0; i<commands.size(); i++){
		cout<<i<< " "<<endl;
		if(!cmdSentFlag && commands[i].wantsControl()){
			cout<<i<<" PUBS"<<endl;
			if(commands[i].newMsgFlag){
				cmdPub.publish(commands[i].cmd);
			}						
			cmdSentFlag = true;
		}
		commands[i].newMsgFlag = false;			
	}

    loop_rate.sleep();
  }



  return 0;
}
