#ifndef RECEIVER_HPP
#define RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> 
#include <cstring>  


class ReceiverNode : public rclcpp::Node
{
public:
  ReceiverNode();
  ~ReceiverNode();

private:
  int sock_fd_;
  rclcpp::TimerBase::SharedPtr timer_;

  void handle_message();
};

#endif // RECEIVER_HPP
