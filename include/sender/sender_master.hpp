#ifndef SENDER_MASTER_HPP
#define SENDER_MASTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <QUdpSocket>
#include <QString>
#include <QDebug>

// master pc 전용
// ============================================================
#include "web_control_bridge/msg/robot1sender_msg.hpp"
#include "web_control_bridge/msg/robot2sender_msg.hpp"
#include "web_control_bridge/msg/robot3sender_msg.hpp"
#include "web_control_bridge/msg/robot4sender_msg.hpp"

// master pc 전용
struct master
{
  int set;
};
//
// ============================================================

class SenderMasterNode : public rclcpp::Node
{
public:
  SenderMasterNode();
  ~SenderMasterNode();

private:
  QUdpSocket *socket;

  // master pc 전용
  // ============================================================
  rclcpp::Subscription<web_control_bridge::msg::Robot1senderMsg>::SharedPtr robot1sender_sub_;
  rclcpp::Subscription<web_control_bridge::msg::Robot2senderMsg>::SharedPtr robot2sender_sub_;
  rclcpp::Subscription<web_control_bridge::msg::Robot3senderMsg>::SharedPtr robot3sender_sub_;
  rclcpp::Subscription<web_control_bridge::msg::Robot4senderMsg>::SharedPtr robot4sender_sub_;
  void robot1senderCallback(const web_control_bridge::msg::Robot1senderMsg::SharedPtr msg);
  void robot2senderCallback(const web_control_bridge::msg::Robot2senderMsg::SharedPtr msg);
  void robot3senderCallback(const web_control_bridge::msg::Robot3senderMsg::SharedPtr msg);
  void robot4senderCallback(const web_control_bridge::msg::Robot4senderMsg::SharedPtr msg);
  void sendMessage_master(const QString &receiverIP, quint16 receiverPort, int yaw_set);
  //
  // ============================================================
};

#endif // SENDER_MASTER_HPP
