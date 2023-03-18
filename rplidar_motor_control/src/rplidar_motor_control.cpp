#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

const int SECONDS_BETWEEN_TRIES = 5;
rclcpp::Clock myClock;
double lastScan = -1;

void updateRPLidarScan(const sensor_msgs::msg::LaserScan &scan)
{
  lastScan = myClock.now().seconds();
}

bool isScanning()
{
  return myClock.now().seconds() - lastScan < 1;
}

int getSecondsSince(double lastTime)
{
  return myClock.now().seconds() - lastTime;
}

bool isPresent(const std::vector<std::string> &nodes, const std::string &str)
{
  for (const auto &node : nodes)
  {
    std::cout << node << " & " << str << " ? ";
    if (node.find(str) != std::string::npos)
    {
      std::cout << " YES" << std::endl;
      return true;
    }
  }
  std::cout << " NOPE" << std::endl;
  return false;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rplidar_motor_control");
  auto stopLidarMotor = node->create_client<std_srvs::srv::Empty>("/stop_motor");
  auto startLidarMotor = node->create_client<std_srvs::srv::Empty>("/start_motor");

  std::string topic_name;
  node->declare_parameter("topic_name", "/scan");
  node->get_parameter("topic_name", topic_name);
  auto subRPLidar = node->create_subscription<sensor_msgs::msg::LaserScan>(topic_name, 1, updateRPLidarScan);

  std::string node1;
  std::string node2;
  node->declare_parameter("node1", "/rviz");
  node->get_parameter("node1", node1);
  node->declare_parameter("node2", "/move_base");
  node->get_parameter("node2", node2);

  RCLCPP_INFO(node->get_logger(), "Starting rplidar_motor_control node with scan topic: %s", topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "Starting rplidar_motor_control node with node1 name: %s", node1.c_str());
  RCLCPP_INFO(node->get_logger(), "Starting rplidar_motor_control node with node2 name: %s", node2.c_str());

  std::cout << "######### " << myClock.now().seconds() << " ########" << std::endl;
  double lastCall = myClock.now().seconds();

  rclcpp::Rate loop_rate(1);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    const double now = myClock.now().seconds();

    auto node_graph = node->get_node_graph_interface();
    std::vector<std::string> nodes = node_graph->get_node_names();

    for (const auto &node : nodes)
    {
      std::cout << node << std::endl;
    }
    std::cout << "........." << std::endl;

    bool lidarRequired = isPresent(nodes, node1) || isPresent(nodes, node2);

    std::cout << "lidarRequired ? " << lidarRequired << " ... " << isScanning() << "  ... " << getSecondsSince(lastScan) << std::endl;

    // start lidar motor if needed and not already scanning
    if (lidarRequired && !isScanning() && getSecondsSince(lastCall) > SECONDS_BETWEEN_TRIES && rclcpp::service::exists("start_motor"))
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling motor_start");
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      auto future_result = startLidarMotor->async_send_request(request);
      lastCall = now();
    }
    // turn lidar motor off if scanning connected and doesn't need to be
    else if (!lidarRequired && IsScanning() && get_seconds_since(lastCall) > SECONDS_BETWEEN_TRIES && rclcpp::service::exists("stop_motor"))
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling motor_stop");
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      auto future_result = stopLidarMotor->async_send_request(request);
      lastCall = now();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate)));
  }
}
