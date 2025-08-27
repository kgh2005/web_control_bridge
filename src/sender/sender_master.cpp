#include "sender/sender_master.hpp"

SenderMasterNode::SenderMasterNode() : Node("sender_master_node")
{
  socket = new QUdpSocket();

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

  get_params();
  RCLCPP_INFO(this->get_logger(), " robot1_ip: %s", robot1_ip_.c_str());
  RCLCPP_INFO(this->get_logger(), " robot2_ip: %s", robot2_ip_.c_str());
  RCLCPP_INFO(this->get_logger(), " robot3_ip: %s", robot3_ip_.c_str());
  RCLCPP_INFO(this->get_logger(), " robot4_ip: %s", robot4_ip_.c_str());
}

SenderMasterNode::~SenderMasterNode()
{
  delete socket;
}

void SenderMasterNode::get_params()
{
  this->declare_parameter<std::string>("robot1_ip", "");
  this->declare_parameter<std::string>("robot2_ip", "");
  this->declare_parameter<std::string>("robot3_ip", "");
  this->declare_parameter<std::string>("robot4_ip", "");

  this->get_parameter("robot1_ip", robot1_ip_);
  this->get_parameter("robot2_ip", robot2_ip_);
  this->get_parameter("robot3_ip", robot3_ip_);
  this->get_parameter("robot4_ip", robot4_ip_);
}

void SenderMasterNode::sendMessage_master(const QString &receiverIP, quint16 receiverPort)
{
  QByteArray buffer(reinterpret_cast<const char *>(&send_msg_), sizeof(master));
  socket->writeDatagram(buffer, QHostAddress(receiverIP), receiverPort);

  qDebug() << "send";
}

void SenderMasterNode::robot1senderCallback(const web_control_bridge::msg::Robot1senderMsg::SharedPtr msg)
{
  send_msg_.set = msg->set;
  send_msg_.imu = msg->imu;
  send_msg_.vision = msg->vision;

  RCLCPP_INFO(this->get_logger(), "set: %d, imu: %d, vision: %d", send_msg_.set, send_msg_.imu, send_msg_.vision);

  sendMessage_master(QString::fromStdString(robot1_ip_), 2222);
}
void SenderMasterNode::robot2senderCallback(const web_control_bridge::msg::Robot2senderMsg::SharedPtr msg)
{
  send_msg_.set = msg->set;
  send_msg_.imu = msg->imu;
  send_msg_.vision = msg->vision;

  RCLCPP_INFO(this->get_logger(), "set: %d, imu: %d, vision: %d", send_msg_.set, send_msg_.imu, send_msg_.vision);

  sendMessage_master(QString::fromStdString(robot2_ip_), 2222);
}
void SenderMasterNode::robot3senderCallback(const web_control_bridge::msg::Robot3senderMsg::SharedPtr msg)
{
  send_msg_.set = msg->set;
  send_msg_.imu = msg->imu;
  send_msg_.vision = msg->vision;

  RCLCPP_INFO(this->get_logger(), "set: %d, imu: %d, vision: %d", send_msg_.set, send_msg_.imu, send_msg_.vision);

  sendMessage_master(QString::fromStdString(robot3_ip_), 2222);
}
void SenderMasterNode::robot4senderCallback(const web_control_bridge::msg::Robot4senderMsg::SharedPtr msg)
{
  send_msg_.set = msg->set;
  send_msg_.imu = msg->imu;
  send_msg_.vision = msg->vision;

  RCLCPP_INFO(this->get_logger(), "set: %d, imu: %d, vision: %d", send_msg_.set, send_msg_.imu, send_msg_.vision);

  sendMessage_master(QString::fromStdString(robot4_ip_), 2222);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SenderMasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
