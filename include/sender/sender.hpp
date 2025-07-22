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
// #include "humanoid_interfaces/msg/ik_coord_msg.hpp"

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
#include "humanoid_interfaces/msg/robot1sender_msg.hpp"
#include "humanoid_interfaces/msg/robot2sender_msg.hpp"
#include "humanoid_interfaces/msg/robot3sender_msg.hpp"
#include "humanoid_interfaces/msg/robot4sender_msg.hpp"

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
  // rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  // rclcpp::Subscription<humanoid_interfaces::msg::Robocuplocalization25>::SharedPtr localization_sub_;
  // rclcpp::Subscription<humanoid_interfaces::msg::IkCoordMsg>::SharedPtr ik_sub_;
  // rclcpp::Subscription<humanoid_interfaces::msg::Robocupvision25>::SharedPtr vision_sub_;
  // void visionCallback(const humanoid_interfaces::msg::Robocupvision25::SharedPtr msg);
  // void ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg);
  // void sendMessage(const QString &receiverIP, quint16 receiverPort);
  // void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  // void localizationCallback(const humanoid_interfaces::msg::Robocuplocalization25::SharedPtr msg);
  //
  // ============================================================

  // master pc 전용
  // ============================================================
  rclcpp::Subscription<humanoid_interfaces::msg::Robot1senderMsg>::SharedPtr robot1sender_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Robot2senderMsg>::SharedPtr robot2sender_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Robot3senderMsg>::SharedPtr robot3sender_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Robot4senderMsg>::SharedPtr robot4sender_sub_;
  void robot1senderCallback(const humanoid_interfaces::msg::Robot1senderMsg::SharedPtr msg);
  void robot2senderCallback(const humanoid_interfaces::msg::Robot2senderMsg::SharedPtr msg);
  void robot3senderCallback(const humanoid_interfaces::msg::Robot3senderMsg::SharedPtr msg);
  void robot4senderCallback(const humanoid_interfaces::msg::Robot4senderMsg::SharedPtr msg);
  void sendMessage_master(const QString &receiverIP, quint16 receiverPort, int yaw_set);
  //
  // ============================================================
};

#endif // SENDER_HPP
