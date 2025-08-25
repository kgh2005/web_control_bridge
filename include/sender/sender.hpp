#ifndef SENDER_HPP
#define SENDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <QUdpSocket>
#include <QString>
#include <QDebug>

// NUC 전용
// ============================================================
// #include "web_control_bridge/msg/imu_msg.hpp"
// #include "web_control_bridge/msg/robocuplocalization25.hpp"
// #include "web_control_bridge/msg/ik_coord_msg.hpp"

// struct RobotData
// {
//   int id;
//   double yaw;
//   double roll;
//   double pitch;
//   double robot_x;
//   double robot_y;
//   double ball_x;
//   double ball_y;
//   int ik_x;
//   int ik_y;
//   int ball_flag;
// };
//
// ============================================================

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

class SenderNode : public rclcpp::Node
{
public:
  SenderNode();
  ~SenderNode();

private:
  // NUC 전용
  // ===========================================================
  // double roll = 0.0;
  // double pitch = 0.0;
  // double yaw = 0.0;

  // double robot_x = 0.0;
  // double robot_y = 0.0;
  // double ball_x = 0.0;
  // double ball_y = 0.0;
  // int ikX = 0;
  // int ikY = 0;
  // int ball_flag = 0;

  //
  // ============================================================

  QUdpSocket *socket;

  // NUC 전용
  // ===========================================================
  // rclcpp::Subscription<web_control_bridge::msg::ImuMsg>::SharedPtr imu_sub_;
  // rclcpp::Subscription<web_control_bridge::msg::Robocuplocalization25>::SharedPtr localization_sub_;
  // rclcpp::Subscription<web_control_bridge::msg::IkCoordMsg>::SharedPtr ik_sub_;
  // rclcpp::Subscription<web_control_bridge::msg::Robocupvision25>::SharedPtr vision_sub_;
  // void visionCallback(const web_control_bridge::msg::Robocupvision25::SharedPtr msg);
  // void ikCallback(const web_control_bridge::msg::IkCoordMsg::SharedPtr msg);
  // void sendMessage(const QString &receiverIP, quint16 receiverPort);
  // void imuCallback(const web_control_bridge::msg::ImuMsg::SharedPtr msg);
  // void localizationCallback(const web_control_bridge::msg::Robocuplocalization25::SharedPtr msg);
  //
  // ============================================================

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

#endif // SENDER_HPP
