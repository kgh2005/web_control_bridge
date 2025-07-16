#include "receiver/receiver.hpp"

ReceiverNode::ReceiverNode() : Node("receiver_node")
{
  // UDP 소켓 생성, AF_INET(IPv4체계 사용), SOCK_DGRAM(UDP 통신 사용)
  sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

  // 소켓 생성 실패시 -1 반환
  if (sock_fd_ < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "소켓 생성 실패");
    return;
  }

  // 모든 인터페이스에서 수신하도록 수정
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(2222);
  addr.sin_addr.s_addr = INADDR_ANY; // 이렇게 변경

  // 바인딩 실패 시 더 자세한 오류 정보 출력
  if (bind(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "바인딩 실패: %s", strerror(errno));
    close(sock_fd_);
    sock_fd_ = -1;
    throw std::runtime_error("Socket binding failed");
  }

  // 메세지 처리할 타이머 생성
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ReceiverNode::handle_message, this));
}

ReceiverNode::~ReceiverNode()
{
  if (sock_fd_ >= 0)
  {
    close(sock_fd_);
    sock_fd_ = -1; // 소켓 핸들을 무효화
  }
}

void ReceiverNode::handle_message()
{
  char buffer[sizeof(RobotData)] = {0};  // 정확한 구조체 크기만큼만 받음
  struct sockaddr_in sender_addr;
  socklen_t sender_addr_len = sizeof(sender_addr);

  ssize_t bytes_received = recvfrom(sock_fd_, buffer, sizeof(buffer), MSG_DONTWAIT,
                                    (struct sockaddr *)&sender_addr, &sender_addr_len);

  if (bytes_received != sizeof(RobotData)) {
    RCLCPP_WARN(this->get_logger(), "수신 데이터 크기 불일치: %ld bytes", bytes_received);
    return;
  }

  std::string sender_ip = inet_ntoa(sender_addr.sin_addr);

  // 바이너리 데이터를 구조체로 해석
  RobotData data;
  std::memcpy(&data, buffer, sizeof(RobotData)); //frame_id

  // data_by_id[data.id] = data;

  // 출력
  RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
  RCLCPP_INFO(this->get_logger(), "Received from %s", sender_ip.c_str());
  RCLCPP_INFO(this->get_logger(), "ID       = %d", data.id);
  RCLCPP_INFO(this->get_logger(), "yaw      = %.2f", data.yaw);
  RCLCPP_INFO(this->get_logger(), "roll     = %.2f", data.roll);
  RCLCPP_INFO(this->get_logger(), "pitch    = %.2f", data.pitch);
  RCLCPP_INFO(this->get_logger(), "robot_x  = %.2f", data.robot_x);
  RCLCPP_INFO(this->get_logger(), "robot_y  = %.2f", data.robot_y);
  RCLCPP_INFO(this->get_logger(), "ball_x   = %.2f", data.ball_x);
  RCLCPP_INFO(this->get_logger(), "ball_y   = %.2f", data.ball_y);
  RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReceiverNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}