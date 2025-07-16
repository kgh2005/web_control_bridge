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

  // 소켓 주소들을 저장하는 구조체(sockaddr_in) 이미 라이브러리에 정의되어 있음
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;                         // AF_INET(IPv4체계 사용)
  addr.sin_port = htons(2222);                       // 포트설정
  addr.sin_addr.s_addr = inet_addr("192.168.0.192"); // 수신할 ip주소 설정

  // 소켓 바인딩 - 네트워크를 특정 ip 주소와 포트에 연결하는 과정
  if (bind(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "2222번 포트 바인딩 실패");
    close(sock_fd_);
    sock_fd_ = -1;                                     // 소켓 핸들을 무효화
    throw std::runtime_error("Socket binding failed"); // return 대신 예외 처리
  }

  RCLCPP_INFO(this->get_logger(), "ip : 192.168.0.192");
  RCLCPP_INFO(this->get_logger(), "port : 2222 에 대한 신호 대기중...");

  // 메세지 처리할 타이머 생성
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
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
  char buffer[1024] = {0};
  struct sockaddr_in sender_addr;
  socklen_t sender_addr_len = sizeof(sender_addr);

  ssize_t bytes_received = recvfrom(sock_fd_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT,
                                    (struct sockaddr *)&sender_addr, &sender_addr_len);

  if (bytes_received <= 0)
    return;

  std::string sender_ip = inet_ntoa(sender_addr.sin_addr);
  std::string received_data(buffer, bytes_received);

  // 파싱용 맵에 key=value 저장
  std::map<std::string, double> data_map;
  std::stringstream ss(received_data);
  std::string token;

  while (std::getline(ss, token, ','))
  {
    size_t pos = token.find('=');
    if (pos != std::string::npos)
    {
      std::string key = token.substr(0, pos);
      try
      {
        double value = std::stod(token.substr(pos + 1));
        data_map[key] = value;
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "값 변환 실패: %s", token.c_str());
      }
    }
  }

  // 변수 추출 (없으면 0.0 디폴트)
  double roll = data_map["roll"];
  double pitch = data_map["pitch"];
  double yaw = data_map["yaw"];
  double robot_x = data_map["robot_x"];
  double robot_y = data_map["robot_y"];
  double ball_x = data_map["ball_x"];
  double ball_y = data_map["ball_y"];

  // 출력
  RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
  RCLCPP_INFO(this->get_logger(), "Received from %s", sender_ip.c_str());
  RCLCPP_INFO(this->get_logger(), "roll     = %.2f", roll);
  RCLCPP_INFO(this->get_logger(), "pitch    = %.2f", pitch);
  RCLCPP_INFO(this->get_logger(), "yaw      = %.2f", yaw);
  RCLCPP_INFO(this->get_logger(), "robot_x  = %.2f", robot_x);
  RCLCPP_INFO(this->get_logger(), "robot_y  = %.2f", robot_y);
  RCLCPP_INFO(this->get_logger(), "ball_x   = %.2f", ball_x);
  RCLCPP_INFO(this->get_logger(), "ball_y   = %.2f", ball_y);
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