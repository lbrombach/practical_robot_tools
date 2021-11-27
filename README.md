# practical_robot_tools
A collection of ROS packages to handle miscellaneous robot tasks and system management. If you are a reader of my book Practical Robotics in C++, you will almost certainly find some useful tools here as example for how to code some things we didn't get to cover in the book. 


 
## Purpose/Motivation:
I've created a lot of tools over the years, but have mostly embedded them in other programs. I thought it was time to start separating them to make them useful for generic robots. More will be added as I have time to make generic tools I use within my custom robots. 

 
## Installation:
cd ~/catkin_ws/src <br> 
git clone https://github.com/lbrombach/practical_robot_tools.git <br> 
cd .. <br> 
catkin_make <br>  
(or use build tool of your choosing)

<br>
<br>

# Nodes: 

# rplidar_motor_control
<br> 

## Description:
The RPLidar's motor is always running by default. This can prematurely wear the bearings and reduce robot run-time if running when not needed. This, along with some base robot management packages provided by you, allows for the automatic starting and stopping of the RPLidar motor when certain nodes are started or stopped (Default are rviz and move_base, but user-settable). The "base robot management package" may be as simple as a launch file or script that starts the rplidar_node and the rplidar_motor_control node. 
## Documentation:
[rplidar_motor_control](rplidar_motor_control/readme.md)


<br><hr>
# cmd_vel_prioritizer
<br>

## Description:
A utility node to allow different geometry_msgs::Twist messages to have different priorities. If two or more are published at the same time, only the message with the highest priority is republished in order to avoid two node "fighting" for control of the robot.
## Documentation
[cmd_vel_prioritizer](cmd_vel_prioritizer/readme.md)
<br><hr>

## Contributing, bug reports, etc:
Please use the bug reporting system for bugs and feature requests. With many projects, a job, a family, I can't promise
to get to feature requests very quickly, but am definitely listening for feedback to make improvement and will prioritize bugs. I am open to
pull requests if you'd like to contribute. I can be contacted by email at lbrombach2@gmail.com. 
