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


// NUC 전용
// ============================================================
#include <web_control_bridge/msg/imuflag_msg.hpp>
#include <QProcess>

struct nuc
{
  int set;
};
//
// ============================================================

class ReceiverNucNode : public rclcpp::Node
{
public:
  ReceiverNucNode();
  ~ReceiverNucNode();

private:
  int sock_fd_ = -1;

  // NUC 전용
  // ============================================================
  rclcpp::Publisher<web_control_bridge::msg::ImuflagMsg>::SharedPtr imuflag_publisher_;
  rclcpp::TimerBase::SharedPtr timer_nuc_;
  void handle_nuc_message();
  //
  // ============================================================

  // std::map<int, RobotData> data_by_id;
};

#endif // RECEIVER_NUC_HPP
