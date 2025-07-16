#include "sender/sender.hpp"

SenderNode::SenderNode() : Node("sender_node")
{
  socket = new QUdpSocket();

  imu_sub_ = this->create_subscription<humanoid_interfaces::msg::ImuMsg>(
    "Imu", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
    std::bind(&SenderNode::imuCallback, this, std::placeholders::_1));
  localization_sub_ = this->create_subscription<humanoid_interfaces::msg::Robocuplocalization25>(
    "RoboCupLocalization25", 10, std::bind(&SenderNode::localizationCallback, this, std::placeholders::_1));  
}

SenderNode::~SenderNode()
{
  delete socket;
}

void SenderNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  roll = msg->roll;
  pitch = msg->pitch;
  yaw = msg->yaw;

  sendMessage("192.168.0.203", 3333);
}

void SenderNode::localizationCallback(const humanoid_interfaces::msg::Robocuplocalization25::SharedPtr msg)
{
  robot_x = msg->robot_x;
  robot_y = msg->robot_y;
  ball_x = msg->ball_x;
  ball_y = msg->ball_y;

  sendMessage("192.168.0.203", 3333);
}

void SenderNode::sendMessage(const QString &receiverIP, quint16 receiverPort)
{
  QString message = QString("roll=%1,pitch=%2,yaw=%3,robot_x=%4,robot_y=%5,ball_x=%6,ball_y=%7")
                      .arg(roll, 0, 'f', 2)
                      .arg(pitch, 0, 'f', 2)
                      .arg(yaw, 0, 'f', 2)
                      .arg(robot_x, 0, 'f', 2)
                      .arg(robot_y, 0, 'f', 2)
                      .arg(ball_x, 0, 'f', 2)
                      .arg(ball_y, 0, 'f', 2);

  QByteArray data = message.toUtf8();
  QHostAddress address(receiverIP);
  socket->writeDatagram(data, address, receiverPort);

  qDebug() << "Message sent to" << receiverIP << ":" << receiverPort;
  qDebug() << "Sent data:" << message;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}