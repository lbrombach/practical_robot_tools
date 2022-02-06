#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "ros_arduino/device_manager.h"

//hoverboard topic callback - stores time of most recent non-zero message recieved
void updateHoverboardTopic(const geometry_msgs::Twist &cmdVel)
{
    if (cmdVel.linear.x != 0 || cmdVel.angular.z != 0)
    {
        lastHoverboardCmdTime = ros::Time::now().toSec();
    }
}

//light blink topic callback - just store time of most recent received from move_base
void updateLightBlinkTopic(const geometry_msgs::Twist &cmdVel)
{
    lastLightBlinkCmdTime = ros::Time::now().toSec();
}

int main(int argc, char **argv)
{
    //normal ROS node setup: Register node with master,  advertise publisher
    ros::init(argc, argv, "arduino_interface_manager_node");
    ros::NodeHandle node;

    ros::Subscriber subHoverboardTopic = node.subscribe(hoverboardTopic, 1, updateHoverboardTopic);
    ros::Subscriber subLightBlinkTOpic = node.subscribe(lightBlinkTopic, 1, updateLightBlinkTopic);
    ros::Publisher pubHoverboardRequired = node.advertise<std_msgs::Bool>("hoverboard_required", 1);
    ros::Publisher pubLightRequired = node.advertise<std_msgs::Bool>("light_required", 1);
    ros::Publisher pubLightBlinkRequired = node.advertise<std_msgs::Bool>("light_blink_required", 1);
    ros::Publisher pubLidarRequired = node.advertise<std_msgs::Bool>("lidar_required", 1);

    //create an empty Bool message
    std_msgs::Bool msg;

    //set the frequency the loop will run
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        //check the subscribers for new message and process callback functions as needed
        ros::spinOnce();

        //check if hoverboard control board is required
        msg.data = hoverboardRequired();
        pubHoverboardRequired.publish(msg);

        //light is required any time the system is on
        msg.data = true;
        pubLightRequired.publish(msg);

        //check if light is required to be blinking
        msg.data = lightBlinkRequired();
        pubLightBlinkRequired.publish(msg);

        //check if lidar is needed
        msg.data = lidarRequired();
        pubLidarRequired.publish(msg);

        //sleep for however long is necessary to maintain the loop_rate
        loop_rate.sleep();
    }

    return 0;
}