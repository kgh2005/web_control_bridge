#include "receiver/receiver_nuc.hpp"

ReceiverNucNode::ReceiverNucNode() : Node("receiver_nuc_node")
{
  // master pc 전용
  // ============================================================
  // robot1receiver_publisher_ = this->create_publisher<web_control_bridge::msg::Robot1receiverMsg>("/robot1receiver", 10);
  // robot2receiver_publisher_ = this->create_publisher<web_control_bridge::msg::Robot2receiverMsg>("/robot2receiver", 10);
  // robot3receiver_publisher_ = this->create_publisher<web_control_bridge::msg::Robot3receiverMsg>("/robot3receiver", 10);
  // robot4receiver_publisher_ = this->create_publisher<web_control_bridge::msg::Robot4receiverMsg>("/robot4receiver", 10);
  //
  // =============================================================

  // NUC 전용
  // ============================================================
  imuflag_publisher_ = this->create_publisher<web_control_bridge::msg::ImuflagMsg>("/imuflag", 10);
  //
  // ============================================================

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

  // NUC 전용
  // ============================================================
  // 메세지 처리할 타이머 생성
  timer_nuc_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&ReceiverNucNode::handle_nuc_message, this)); // NUC 전용 타이머
  // ============================================================
}

ReceiverNucNode::~ReceiverNucNode()
{
  if (sock_fd_ >= 0)
  {
    close(sock_fd_);
    sock_fd_ = -1; // 소켓 핸들을 무효화
  }
}

// NUC 전용
// ============================================================
void ReceiverNucNode::handle_nuc_message()
{
  char buffer[sizeof(nuc)] = {0}; // 정확한 구조체 크기만큼만 받음
  struct sockaddr_in sender_addr;
  socklen_t sender_addr_len = sizeof(sender_addr);

  ssize_t bytes_received = recvfrom(sock_fd_, buffer, sizeof(buffer), MSG_DONTWAIT,
                                    (struct sockaddr *)&sender_addr, &sender_addr_len);

  if (bytes_received != sizeof(nuc))
  {
    RCLCPP_WARN(this->get_logger(), "수신 데이터 크기 불일치: %ld bytes", bytes_received);
    return;
  }

  std::string sender_ip = inet_ntoa(sender_addr.sin_addr);

  // 바이너리 데이터를 구조체로 해석
  nuc data;
  std::memcpy(&data, buffer, sizeof(nuc));

  RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
  RCLCPP_INFO(this->get_logger(), "Received from %s", sender_ip.c_str());
  RCLCPP_INFO(this->get_logger(), "set      = %d", data.set);
  RCLCPP_INFO(this->get_logger(), "--------------------------------------------");

  web_control_bridge::msg::ImuflagMsg msg;
  msg.set = data.set;
  imuflag_publisher_->publish(msg);
}
//
// =============================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReceiverNucNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}