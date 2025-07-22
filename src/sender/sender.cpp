#include "sender/sender.hpp"

SenderNode::SenderNode() : Node("sender_node")
{
  socket = new QUdpSocket();

  // NUC 전용
  // ===========================================================
  // imu_sub_ = this->create_subscription<humanoid_interfaces::msg::ImuMsg>(
  //     "Imu", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
  //     std::bind(&SenderNode::imuCallback, this, std::placeholders::_1));
  // localization_sub_ = this->create_subscription<humanoid_interfaces::msg::Robocuplocalization25>(
  //     "/localization", 10, std::bind(&SenderNode::localizationCallback, this, std::placeholders::_1));
  // ik_sub_ = this->create_subscription<humanoid_interfaces::msg::IkCoordMsg>(
  //     "/ikcoordinate", 10, std::bind(&SenderNode::ikCallback, this, std::placeholders::_1));
  // vision_sub_ = this->create_subscription<humanoid_interfaces::msg::Robocupvision25>(
  //     "/vision", 100, std::bind(&SenderNode::visionCallback, this, std::placeholders::_1));
  //
  // ============================================================

  // master pc 전용
  // ============================================================
  robot1sender_sub_ = this->create_subscription<humanoid_interfaces::msg::Robot1senderMsg>(
      "/robot1sender", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SenderNode::robot1senderCallback, this, std::placeholders::_1));

  robot2sender_sub_ = this->create_subscription<humanoid_interfaces::msg::Robot2senderMsg>(
      "/robot2sender", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SenderNode::robot2senderCallback, this, std::placeholders::_1));

  robot3sender_sub_ = this->create_subscription<humanoid_interfaces::msg::Robot3senderMsg>(
      "/robot3sender", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SenderNode::robot3senderCallback, this, std::placeholders::_1));

  robot4sender_sub_ = this->create_subscription<humanoid_interfaces::msg::Robot4senderMsg>(
      "/robot4sender", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SenderNode::robot4senderCallback, this, std::placeholders::_1));
  //
  // ============================================================
}

SenderNode::~SenderNode()
{
  delete socket;
}

// NUC 전용
//  ===========================================================
// void SenderNode::visionCallback(const humanoid_interfaces::msg::Robocupvision25::SharedPtr msg)
// {
//   if (msg->ball_cam_x != 0) ball_flag = 1;
//   sendMessage("172.100.5.71", 2222);
// }
//
// void SenderNode::ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg)
// {
//   ikX = msg->x;
//   ikY = msg->y;

//   sendMessage("172.100.5.71", 2222);
// }
//
//  void SenderNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
//  {
//    roll = msg->roll;
//    pitch = msg->pitch;
//    yaw = msg->yaw;

//   sendMessage("172.100.5.71", 2222);
// }

// void SenderNode::localizationCallback(const humanoid_interfaces::msg::Robocuplocalization25::SharedPtr msg)
// {
//   robot_x = msg->robot_x;
//   robot_y = msg->robot_y;
//   ball_x = msg->ball_x;
//   ball_y = msg->ball_y;

//   sendMessage("172.100.5.71", 2222);
// }

// void SenderNode::sendMessage(const QString &receiverIP, quint16 receiverPort)
// {
//   RobotData data;
//   data.id = 1; // 로봇 ID
//   data.yaw = yaw;
//   data.roll = roll;
//   data.pitch = pitch;
//   data.robot_x = robot_x;
//   data.robot_y = robot_y;
//   data.ball_x = ball_x;
//   data.ball_y = ball_y;
//   data.ik_x = ikX;
//   data.ik_y = ikY;
//   data.ball_flag = ball_flag;

//   RCLCPP_INFO(this->get_logger(), "roll: %.2f, pitch: %.2f, yaw: %.2f, robot_x: %.2f, robot_y: %.2f, ball_x: %.2f, ball_y: %.2f",
//               data.roll, data.pitch, data.yaw, data.robot_x, data.robot_y, data.ball_x, data.ball_y);

//   QByteArray buffer(reinterpret_cast<const char *>(&data), sizeof(RobotData));
//   socket->writeDatagram(buffer, QHostAddress(receiverIP), receiverPort);

//   qDebug() << "send";
//   ball_flag = 0;
// }
//
// ============================================================

// master pc 전용
// ============================================================
void SenderNode::sendMessage_master(const QString &receiverIP, quint16 receiverPort, int yaw_set)
{
  master msg;
  msg.set = yaw_set;
  RCLCPP_INFO(this->get_logger(), "%d", msg.set);

  QByteArray buffer(reinterpret_cast<const char *>(&msg), sizeof(master));
  socket->writeDatagram(buffer, QHostAddress(receiverIP), receiverPort);

  qDebug() << "send";
}

void SenderNode::robot1senderCallback(const humanoid_interfaces::msg::Robot1senderMsg::SharedPtr msg)
{
  // master pc 전용 메시지 처리
  int yaw_set1 = msg->set;

  RCLCPP_INFO(this->get_logger(), "Received set value: %d", yaw_set1);

  sendMessage_master("172.100.1.161", 2222, yaw_set1);
}
void SenderNode::robot2senderCallback(const humanoid_interfaces::msg::Robot2senderMsg::SharedPtr msg)
{
  // master pc 전용 메시지 처리
  int yaw_set2 = msg->set;

  RCLCPP_INFO(this->get_logger(), "Received set value: %d", yaw_set2);

  sendMessage_master("172.100.1.161", 2222, yaw_set2);
}
void SenderNode::robot3senderCallback(const humanoid_interfaces::msg::Robot3senderMsg::SharedPtr msg)
{
  // master pc 전용 메시지 처리
  int yaw_set3 = msg->set;

  RCLCPP_INFO(this->get_logger(), "Received set value: %d", yaw_set3);

  sendMessage_master("172.100.1.161", 2222, yaw_set3);
}
void SenderNode::robot4senderCallback(const humanoid_interfaces::msg::Robot4senderMsg::SharedPtr msg)
{
  // master pc 전용 메시지 처리
  int yaw_set4 = msg->set;

  RCLCPP_INFO(this->get_logger(), "Received set value: %d", yaw_set4);

  sendMessage_master("172.100.1.161", 2222, yaw_set4);
}
//
// ============================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
