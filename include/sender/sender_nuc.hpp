#ifndef SENDER_NUC_HPP
#define SENDER_NUC_HPP

#include <rclcpp/rclcpp.hpp>
#include <QUdpSocket>
#include <QString>
#include <QDebug>
#include <string>

#include "humanoid_interfaces/msg/imu_msg.hpp"
#include "humanoid_interfaces/msg/robocuplocalization25.hpp"
#include "humanoid_interfaces/msg/ik_coord_msg.hpp"
#include "humanoid_interfaces/msg/robocupvision25.hpp"

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

class SenderNucNode : public rclcpp::Node
{
public:
  SenderNucNode();
  ~SenderNucNode();

private:
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double ball_x = 0.0;
  double ball_y = 0.0;
  int ikX = 0;
  int ikY = 0;
  int ball_flag = 0;

  QUdpSocket *socket;

  std::string master_ip_;

  rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Robocuplocalization25>::SharedPtr localization_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::IkCoordMsg>::SharedPtr ik_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Robocupvision25>::SharedPtr vision_sub_;
  void visionCallback(const humanoid_interfaces::msg::Robocupvision25::SharedPtr msg);
  void ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg);
  void sendMessage(const QString &receiverIP, quint16 receiverPort);
  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void localizationCallback(const humanoid_interfaces::msg::Robocuplocalization25::SharedPtr msg);
  void get_params();
};

#endif // SENDER_NUC_HPP
