#ifndef SENDER_HPP
#define SENDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <QUdpSocket>
#include <QString>
#include <QDebug>

// NUC 전용
// ============================================================
// #include "humanoid_interfaces/msg/imu_msg.hpp"
// #include "humanoid_interfaces/msg/robocuplocalization25.hpp"

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
// };
//
// ============================================================

// master pc 전용
// ============================================================
#include "humanoid_interfaces/msg/robot1sender_msg.hpp"

// master pc 전용
struct master1
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
  //
  // ============================================================

  QUdpSocket *socket;

  // NUC 전용
  // ===========================================================
  // rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  // rclcpp::Subscription<humanoid_interfaces::msg::Robocuplocalization25>::SharedPtr localization_sub_;
  // void sendMessage(const QString &receiverIP, quint16 receiverPort);
  // void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  // void localizationCallback(const humanoid_interfaces::msg::Robocuplocalization25::SharedPtr msg);
  //
  // ============================================================

  // master pc 전용
  // ============================================================
  int yaw_set = 0;
  rclcpp::Subscription<humanoid_interfaces::msg::Robot1senderMsg>::SharedPtr robot1sender_sub_;
  void robot1senderCallback(const humanoid_interfaces::msg::Robot1senderMsg::SharedPtr msg);
  void sendMessage_master1(const QString &receiverIP, quint16 receiverPort);
  //
  // ============================================================
};

#endif // SENDER_HPP
