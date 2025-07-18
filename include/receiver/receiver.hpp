#ifndef RECEIVER_HPP
#define RECEIVER_HPP

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
#include <humanoid_interfaces/msg/robot1receiver_msg.hpp>
#include <humanoid_interfaces/msg/robot2receiver_msg.hpp>
#include <humanoid_interfaces/msg/robot3receiver_msg.hpp>
#include <humanoid_interfaces/msg/robot4receiver_msg.hpp>
//

// master pc 전용
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
};
//
// ===============================================================


// NUC 전용
// ============================================================
// #include <humanoid_interfaces/msg/imuflag_msg.hpp>

// struct nuc
// {
//   int set;
// };
//
// ============================================================

class ReceiverNode : public rclcpp::Node
{
public:
  ReceiverNode();
  ~ReceiverNode();

private:
  int sock_fd_ = -1;

  // master pc 전용
  // ============================================================
  rclcpp::Publisher<humanoid_interfaces::msg::Robot1receiverMsg>::SharedPtr robot1receiver_publisher_;
  rclcpp::Publisher<humanoid_interfaces::msg::Robot2receiverMsg>::SharedPtr robot2receiver_publisher_;
  rclcpp::Publisher<humanoid_interfaces::msg::Robot3receiverMsg>::SharedPtr robot3receiver_publisher_;
  rclcpp::Publisher<humanoid_interfaces::msg::Robot4receiverMsg>::SharedPtr robot4receiver_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  void handle_message();
  //
  // ============================================================

  // NUC 전용
  // ============================================================
  // rclcpp::Publisher<humanoid_interfaces::msg::ImuflagMsg>::SharedPtr imuflag_publisher_;
  // rclcpp::TimerBase::SharedPtr timer_nuc_;
  // void handle_nuc_message();
  //
  // ============================================================

  // std::map<int, RobotData> data_by_id;
};

#endif // RECEIVER_HPP
