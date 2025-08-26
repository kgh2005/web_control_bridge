#include "sender/sender_nuc.hpp"

SenderNucNode::SenderNucNode() : Node("sender_nuc_node")
{
  socket = new QUdpSocket();

  // NUC 전용
  // ===========================================================
  imu_sub_ = this->create_subscription<humanoid_interfaces::msg::ImuMsg>(
      "Imu", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&SenderNucNode::imuCallback, this, std::placeholders::_1));
  localization_sub_ = this->create_subscription<humanoid_interfaces::msg::Robocuplocalization25>(
      "/localization", 10, std::bind(&SenderNucNode::localizationCallback, this, std::placeholders::_1));
  ik_sub_ = this->create_subscription<humanoid_interfaces::msg::IkCoordMsg>(
      "/ikcoordinate", 10, std::bind(&SenderNucNode::ikCallback, this, std::placeholders::_1));
  vision_sub_ = this->create_subscription<humanoid_interfaces::msg::Robocupvision25>(
      "/vision", 100, std::bind(&SenderNucNode::visionCallback, this, std::placeholders::_1));
  //
  // ============================================================
}

SenderNucNode::~SenderNucNode()
{
  delete socket;
}

// NUC 전용
//  ===========================================================
void SenderNucNode::visionCallback(const humanoid_interfaces::msg::Robocupvision25::SharedPtr msg)
{
  if (msg->ball_cam_x != 0)
    ball_flag = 1;
  sendMessage("172.100.5.71", 2222);
}

void SenderNucNode::ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg)
{
  ikX = msg->x;
  ikY = msg->y;

  sendMessage("172.100.5.71", 2222);
}

void SenderNucNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  roll = msg->roll;
  pitch = msg->pitch;
  yaw = msg->yaw;

  sendMessage("172.100.5.71", 2222);
}

void SenderNucNode::localizationCallback(const humanoid_interfaces::msg::Robocuplocalization25::SharedPtr msg)
{
  robot_x = msg->robot_x;
  robot_y = msg->robot_y;
  ball_x = msg->ball_x;
  ball_y = msg->ball_y;

  sendMessage("172.100.5.71", 2222);
}

void SenderNucNode::sendMessage(const QString &receiverIP, quint16 receiverPort)
{
  RobotData data;
  data.id = 1; // 로봇 ID
  data.yaw = yaw;
  data.roll = roll;
  data.pitch = pitch;
  data.robot_x = robot_x;
  data.robot_y = robot_y;
  data.ball_x = ball_x;
  data.ball_y = ball_y;
  data.ik_x = ikX;
  data.ik_y = ikY;
  data.ball_flag = ball_flag;

  RCLCPP_INFO(this->get_logger(), "roll: %.2f, pitch: %.2f, yaw: %.2f, robot_x: %.2f, robot_y: %.2f, ball_x: %.2f, ball_y: %.2f",
              data.roll, data.pitch, data.yaw, data.robot_x, data.robot_y, data.ball_x, data.ball_y);

  QByteArray buffer(reinterpret_cast<const char *>(&data), sizeof(RobotData));
  socket->writeDatagram(buffer, QHostAddress(receiverIP), receiverPort);

  qDebug() << "send";
  ball_flag = 0;
}
//
// ============================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SenderNucNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
