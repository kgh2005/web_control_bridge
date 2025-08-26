#include "sender/sender_master.hpp"

SenderMasterNode::SenderMasterNode() : Node("sender_master_node")
{
  socket = new QUdpSocket();

  // master pc 전용
  // ============================================================
  robot1sender_sub_ = this->create_subscription<web_control_bridge::msg::Robot1senderMsg>(
      "/robot1sender", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SenderMasterNode::robot1senderCallback, this, std::placeholders::_1));

  robot2sender_sub_ = this->create_subscription<web_control_bridge::msg::Robot2senderMsg>(
      "/robot2sender", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SenderMasterNode::robot2senderCallback, this, std::placeholders::_1));

  robot3sender_sub_ = this->create_subscription<web_control_bridge::msg::Robot3senderMsg>(
      "/robot3sender", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SenderMasterNode::robot3senderCallback, this, std::placeholders::_1));

  robot4sender_sub_ = this->create_subscription<web_control_bridge::msg::Robot4senderMsg>(
      "/robot4sender", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SenderMasterNode::robot4senderCallback, this, std::placeholders::_1));
  //
  // ============================================================
}

SenderMasterNode::~SenderMasterNode()
{
  delete socket;
}

// master pc 전용
// ============================================================
void SenderMasterNode::sendMessage_master(const QString &receiverIP, quint16 receiverPort, int yaw_set)
{
  master msg;
  msg.set = yaw_set;
  RCLCPP_INFO(this->get_logger(), "%d", msg.set);

  QByteArray buffer(reinterpret_cast<const char *>(&msg), sizeof(master));
  socket->writeDatagram(buffer, QHostAddress(receiverIP), receiverPort);

  qDebug() << "send";
}

void SenderMasterNode::robot1senderCallback(const web_control_bridge::msg::Robot1senderMsg::SharedPtr msg)
{
  // master pc 전용 메시지 처리
  int yaw_set1 = msg->set;

  RCLCPP_INFO(this->get_logger(), "Received set value: %d", yaw_set1);

  sendMessage_master("172.100.1.161", 2222, yaw_set1);
}
void SenderMasterNode::robot2senderCallback(const web_control_bridge::msg::Robot2senderMsg::SharedPtr msg)
{
  // master pc 전용 메시지 처리
  int yaw_set2 = msg->set;

  RCLCPP_INFO(this->get_logger(), "Received set value: %d", yaw_set2);

  sendMessage_master("172.100.1.161", 2222, yaw_set2);
}
void SenderMasterNode::robot3senderCallback(const web_control_bridge::msg::Robot3senderMsg::SharedPtr msg)
{
  // master pc 전용 메시지 처리
  int yaw_set3 = msg->set;

  RCLCPP_INFO(this->get_logger(), "Received set value: %d", yaw_set3);

  sendMessage_master("172.100.1.161", 2222, yaw_set3);
}
void SenderMasterNode::robot4senderCallback(const web_control_bridge::msg::Robot4senderMsg::SharedPtr msg)
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
  auto node = std::make_shared<SenderMasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
