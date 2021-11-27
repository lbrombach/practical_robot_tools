# cmd_vel_prioritizer
A utility node to allow different geometry_msgs::Twist messages to have different priorities. <br> 
Author: Lloyd Brombach <br> 
lbrombach2@gmail.com  <br> 
November 2021
<br><hr><br> 

 
## Description and Operation:
The node subsribes to up to four Twist messages and only republishes the one with the highest priority.
This could, for example, allow teleop commands to override autonomously issued commands without fighting them. <br> 
<br> 

## Usage:
With a topic name parameter and a delay parameter for each topic, it is easier to use a launch file than to rosrun from the command line. See the cmd_vel_prioritizer.launch for an example/template.
<br> 

## Subscriptions
- topic1 (default: teleop/cmd_vel) ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
    - The highest priority topic

- topic2 (default: none) ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
    - The 2nd highest priority topic
    
- topic3 (default: none) ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
    - The third highest priority topic
    
- topic4 (default: "move_base/cmd_vel") ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
    - The lowest priority topic
    
<br><br> 

## Publications
- output_topic (default: "cmd_vel") ([geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))
    - The topic that gets republished

<br><br> 

## Parameters
- output_topic (string, default: cmd_vel)
    - use to change the output topic

- input_topic1 (string, default: teleop/cmd_vel)
    - the name of the topic that should have the highest priority
    
- input1_locktime (double, default: 4.0)
    - the amount of time following last valid topic1 message to ignore lower priority inputs

- input_topic2 (string, default: none)
    - the name of the topic that should have the second highest priority
    
- input2_locktime (double, default: 0.2)
    - the amount of time following last valid topic2 message to ignore lower priority inputs
    
- input_topic3 (string, default: none)
    - the name of the topic that should have the third highest priority

- input3_locktime (double, default: 0.2)
    - the amount of time following last valid topic3 message to ignore lower priority inputs    

- input_topic4 (string, default: move_base/cmd_vel)
    - the name of the topic that should have the lowest priority
    
- input4_locktime (double, default: 0.2)
    - Not relevant because topic4 is the lowest priority topic

