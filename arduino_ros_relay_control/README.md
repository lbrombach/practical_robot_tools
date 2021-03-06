# arduino_ros_relay_control

# NOTE:
## This project was written for a specific application and a lot of things that should be parameters or configurable through a file are hard-coded until I have time to evolve the project. Still, I think it is useful as is or very minor changes (mostly a few string constants) would allow for easily re-purposing to handle different tasks. Use the existing code as-is, or take the three different existing behaviors and replicate the parts you need. 

**Overview**

The robot has 5 devices that need control, but each have different criteria and different actions needed to control them. We have:
- A [hacked hoverboard control board](https://github.com/alex-makarov/hoverboard-firmware-hack-FOC): This goes to sleep after some time and needs its power button momentarily pressed to wake it up. This project does that by pulsing a relay whenever specific cmd_vel messages are active and the [hoverboard_driver node](https://github.com/alex-makarov/hoverboard-driver) is reporting that the board is not connected.
- A Safety Light: The light is required to be steadily on whenever the system is on, but must blink if the robot is in autonomous mode. This package turns the light on any time the node is running, but "steadily on" can be interrupted by the blink requirement, which is determined by monitoring the move_base/cmd_vel topic to determine if the robot is in autonomous mode.
- A Lidar: The robot is online 24/7 but the lidar is not always needed. The relay is used to cut and restore power to the lidar based on whether certain nodes are active. By default these are move_base and rviz.
- Two relays for fans that are simply commanded on whenever the Arduino is powered. In the future hopefully this package will include modulating fan speed with a mosfet based on temperature.
<br>

This package uses a ROS node (arduino_interface_manager_node) to determine what hardware/behavior is required. The node then publishes simple boolean messages for the Arduino. The arduino has to know what action is required to power the device (relay steady on, relay pulsed to mimic pressing a power button, or relay toggling on/off to make a blinking action). 


<br><hr>

# Below is the original documentation provided to the team using the package as-is
## Overview
This package contains a ROS node and Arduino code to control relays.

The arduino_interface_manager_node determines what hardware needs to be powered at any given moment by monitoring several topics and checking if several nodes are active. The node then publishes boolean messages which an Arduino subscribes to, and activates relays based on the value of each message. Rosserial is used to interface data transmission between ROS and the Arduino's serial interface. This package also contains a folder with the code that runs on the Arduino.

![ROS Arduino overview](./images/overview.jpeg)
<br><br>

## arduino_interface_manager_node 
**Description**

It is the job of this node to publish boolean messages that indicate whether a given device is required or not. If the device is required by some part of the system, a boolen true message is published, else the node publishes false on that topic. There is a separate topic for each device that the Arduino controls. The method of determining whether a device is required varies by device. Currently, the four device_required messages and their decision criteria are as follows:

<br>

**Subscribes**
- topic1 default: /hoverboard_velocity_controller/cmd_vel ([geometry_msgs::Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)) <br> 
    The hard-coded hoverboardTopic topic. Used to determine if the system requires the hoverboard control board to be powered on

- topic2 default: /move_base/cmd_vel ([geometry_msgs::Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)) <br> 
    The hard-coded lightBlinkTopic topic. Used to determine if the safety light should be blinking

- *Not a subscription, but to determine if the lidar is required this node fetches a list of currently running nodes and checks if either of two designated nodes are running. These hard-coded const strings are currently "move_base" and "rviz" but instructions to change are in the Details section below.

**Publishes**
- topic hoverboard_required ([std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)) <br> 
    True when the the hoverboard control board required. Goes false after hoverboardTimeoutSeconds have passed without a non-zero command on the hoverboardTopic topic.
- topic light_required ([std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)) <br> 
    True when the safety light is required (his is any tie the system is powered so this is always true when this node is running)
- topic light_blink_required ([std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))  <br> 
    True when the safety light should be blinking instead of steady (should blink when in autonomous mode)
- topic lidar_required ([std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))  <br> 
    True when the lidar is required

**Details** 

This node is intended to work with an Arduino Nano Every that is running the [ros_arduino.ino program](./arduino_code/ros_arduino/ros_arduino.ino) in the arduino_code folder in this package. Subscriber topic names are hard-coded as const strings in [device_manager.h](./include/arduino_ros_relay_control/device_manager.h) and can be changed there if necessary. Timeout values (hoverboardTimeoutSeconds & lightBlinkTimeoutSeconds) can also be changed here. Recompiling is necessary after changes. <br>

![device_manager.h](./images/device_manager_h.jpeg)
<br>

**TODO**

 In the future topic names, node names, and timeout values should be parameterized for flexibility that does not require recompiling.

<hr>

## rosserial_python
**Description**

rosserial_python is not part of this package, but is required to handle message-passing between ROS and the Arduino. All that is needed is to run it (easiest from launch file) with the Arduino's port set as a parameter. Example launch file entry: <br> 
![rosserial_python launch entry](./images/rosserial_launch.jpeg)

<hr>

## Arduino Nano Every
**Description**

The Arduino's job is to handle powering devices on and off via common 5v relay modules. It subscribes to boolean messages (issued by the  arduino_interface_manager_node) and generally ensures a device or action is active if its message is true. There is a timeout where it turns devices off if no messages are recieved for a time. Currently there are three relays controlled by the Arduino:
- Hoverboard Power Button Relay: The arduino compares the hoverboard_required message with the hoverboard_connected message and if they do no match, it pulses the power button relay to "press" hoverboard control board power button.
- Safety Light Relay: The Arduino blinks the light_blink_required is true, otherwise it keeps it on as long as light_required is true. 
- Lidar Power Relay: The Arduino turns on power to the lidar device with lidar_required is true
- (there are two more but no logical control has been implemented for them and the arduino simply turns them on when powered)
<br>

**Wiring**

The Arduino receives and sends data via a USB cable to the computer. Although power to Arduino (requires 5v) can come from the USB cable, we observed that the voltage was sagging enough that is was not able to operate the relay module. This problem was solved by running 5v to the Arduino and relay module directly from the 5V supply on the robot. We connected digital pins from the Arduino to the input pins on the relay module. The relay contacts were then connected to the various devices. <br> 

![Arduino Wiring](./images/wiring.jpeg)
<br>

**The Code**

Arduino programs are commonly called "sketches." The sketch that runs on the Arduino for this package is a sketch called  [ros_arduino.ino](./arduino_code/ros_arduino/ros_arduino.ino) and can be found in the arduino_code folder. For instructions on how to use the Arduino IDE, how to upload sketches, etc, see the Arduino tutorials at https://docs.arduino.cc/learn.
<br>

**Pin Numbers and other constants**

Pin numbers and constants are declared at the top of the ros_arduino.ino file and can be changed to suit needs. The constant CONNECTION_TIMEOUT_SECONDS is to ensure that relays are shut off if this much time has elapsed since a message has been received on a given topic (for example if the ros master shuts down while the devices are on). The sketch will have to be re-uploaded to the Arduino for changes to take effect.    
<br>

![Arduino Pin Numbers](./images/arduino_consts.jpeg)
<br>

**Subscribes**
- topic hoverboard_required ([std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)) <br> 
- topic light_required ([std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)) <br> 
- topic light_blink_required ([std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))  <br> 
- topic lidar_required ([std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))  <br> 

**Publishes**
- none
<br>

**Running the Arduino**

Once the program has been uploaded to the Arduino, it begins to run automatically as soon as the Arduino is powered on. There are no commands needed to start the Arduino, but rosserial_python must be running in order to facilitate the exchange of data from ROS to the Arduino - see the rosserial_python entry above.
<br><br>

## Launching the components
For this package to all work together the arduino_interface_manager_node, rosserial_python, and the Arduino all have to be running. The Arduino runs automatically when powered. Here is an example launch file entry to start the arduino_interface_manager_node and rosserial_python. 
<br>

![ros_arduino launch](./images/launch.png)
<br>

**TODO**

Currently, there are two relay for fans. It would be good if temperature sensors were added and the relays were controlled to maintain some temperature instead of leaving the fans on always. It would be even better if these two relays were replaced with transistors so fan speed could be modulated with demand instead of simple on/off control. 





