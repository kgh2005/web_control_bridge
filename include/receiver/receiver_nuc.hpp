#ifndef RECEIVER_NUC_HPP
#define RECEIVER_NUC_HPP

#include <rclcpp/rclcpp.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <map>

#include <web_control_bridge/msg/imuflag_msg.hpp>
#include <web_control_bridge/msg/node_manager_msg.hpp>

struct nuc
{
  int set;
  int imu;
  int vision;
};

class ReceiverNucNode : public rclcpp::Node
{
public:
  ReceiverNucNode();
  ~ReceiverNucNode();

private:
  int sock_fd_ = -1;


  web_control_bridge::msg::ImuflagMsg msg;
  web_control_bridge::msg::NodeManagerMsg nm_msg;

  rclcpp::Publisher<web_control_bridge::msg::ImuflagMsg>::SharedPtr imuflag_publisher_;
  rclcpp::Publisher<web_control_bridge::msg::NodeManagerMsg>::SharedPtr nodemanager_publisher_;
  
  rclcpp::TimerBase::SharedPtr timer_nuc_;
  void handle_nuc_message();

  // std::map<int, RobotData> data_by_id;
};

#endif // RECEIVER_NUC_HPP
