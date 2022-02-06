#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H
#include "ros/ros.h"
#include <vector>

using namespace std;

// if non-zero messages on this topic are recieved, hoverboard base is needed
const string hoverboardTopic = "hoverboard_velocity_controller/cmd_vel";

// if any message on this topic is received, robot is in autonomous mode and light must blink
const string lightBlinkTopic = "move_base/cmd_vel";

// if these nodes are active, lidar is lkely needed
const string lidarNode1 = "move_base";
const string lidarNode2 = "rviz";

// how long to keep a device on after last topic message is recieved
const int hoverboardTimeoutSeconds = 120;
const int lightBlinkTimeoutSeconds = 10;

// for tracking timestamps
double lastHoverboardCmdTime = -1;
double lastLightBlinkCmdTime = -1;

// helper to get seconds passed since value passed in
int getSecondsSince(double lastTimeInSeconds) { return (int)(ros::Time::now().toSec() - lastTimeInSeconds); }

// helper to check if string is found in a list of nodes
bool isPresent(std::string str) {
    // get list of running nodes from ros master
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);

    // check if string is present in node names
    for (int i = 0; i < nodes.size(); i++) {
        if (nodes[i].find(str) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool hoverboardRequired() {
    if (getSecondsSince(lastHoverboardCmdTime) < hoverboardTimeoutSeconds) {
        return true;
    }
    return false;
}

bool lightBlinkRequired() {
    if (getSecondsSince(lastLightBlinkCmdTime) < lightBlinkTimeoutSeconds) {
        return true;
    }
    return false;
}

bool lidarRequired() {
    // check if designated nodes are running
    return isPresent(lidarNode1) || isPresent(lidarNode2);
};

#endif