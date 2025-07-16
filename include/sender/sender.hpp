#ifndef SENDER_HPP
#define SENDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <QUdpSocket>
#include <QString>
#include <QDebug>

#include "humanoid_interfaces/msg/imu_msg.hpp"
#include "humanoid_interfaces/msg/robocuplocalization25.hpp"

class SenderNode : public rclcpp::Node
{
public:
  SenderNode();
  ~SenderNode();

  void sendMessage(const QString &receiverIP, quint16 receiverPort);

private:
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double ball_x = 0.0;
  double ball_y = 0.0;

  QUdpSocket *socket;
  rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Robocuplocalization25>::SharedPtr localization_sub_;

  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void localizationCallback(const humanoid_interfaces::msg::Robocuplocalization25::SharedPtr msg);
};

#endif // SENDER_HPP
