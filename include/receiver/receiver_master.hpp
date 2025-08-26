#ifndef RECEIVER_MASTER_HPP
#define RECEIVER_MASTER_HPP

#include <rclcpp/rclcpp.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <map>

// master pc 전용
// ============================================================
#include <web_control_bridge/msg/robot1receiver_msg.hpp>
#include <web_control_bridge/msg/robot2receiver_msg.hpp>
#include <web_control_bridge/msg/robot3receiver_msg.hpp>
#include <web_control_bridge/msg/robot4receiver_msg.hpp>

struct RobotData
{
  int id;
  double yaw;
  double roll;
  double pitch;
  double robot_x;
  double robot_y;
  double ball_x;
  double ball_y;
  int ik_x;
  int ik_y;
  int ball_flag;
};
//
// ===============================================================

class ReceiverMasterNode : public rclcpp::Node
{
public:
  ReceiverMasterNode();
  ~ReceiverMasterNode();

private:
  int sock_fd_ = -1;

  // master pc 전용
  // ============================================================
  rclcpp::Publisher<web_control_bridge::msg::Robot1receiverMsg>::SharedPtr robot1receiver_publisher_;
  rclcpp::Publisher<web_control_bridge::msg::Robot2receiverMsg>::SharedPtr robot2receiver_publisher_;
  rclcpp::Publisher<web_control_bridge::msg::Robot3receiverMsg>::SharedPtr robot3receiver_publisher_;
  rclcpp::Publisher<web_control_bridge::msg::Robot4receiverMsg>::SharedPtr robot4receiver_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  void handle_message();
  //
  // ============================================================

  // std::map<int, RobotData> data_by_id;
};

#endif // RECEIVER_MASTER_HPP
