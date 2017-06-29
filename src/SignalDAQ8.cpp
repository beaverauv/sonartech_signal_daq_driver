#include <sonartech_signal_daq_driver/SignalDAQ8.hpp>

SignalDAQ8::SignalDAQ8(const ros::NodeHandle &nh) : nh_(nh) {
  memset(&board_cmd_addr, 0, sizeof(board_cmd_addr));
  memset(&self_cmd_addr, 0, sizeof(self_cmd_addr));
  memset(&board_data_addr, 0, sizeof(board_data_addr));

  // No Socket , address and port to send to
  board_cmd_addr.sin_family = AF_INET;
  board_cmd_addr.sin_addr.s_addr = inet_addr(board_ip_);
  board_cmd_addr.sin_port = htons(cmd_port_);

  // socket, port to send commands from
  self_cmd_addr.sin_family = AF_INET;
  self_cmd_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  self_cmd_addr.sin_port = htons(cmd_port_);

  // socket, port to listen for data
  board_data_addr.sin_family = AF_INET;
  board_data_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  board_data_addr.sin_port = htons(data_port_);
}

SignalDAQ8::~SignalDAQ8() {
  close(socket_cmd_fd_);
  close(socket_data_fd_);
}

int SignalDAQ8::sendCommand(short data) {
  if (sendto(socket_cmd_fd_, &data, sizeof(data), 0,
             (struct sockaddr *)&board_cmd_addr, sizeof(board_cmd_addr)) < 0) {
    die("Failed to send command");
  }
}

int SignalDAQ8::initializeSockets() {

  if ((socket_cmd_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    die("Failed to get socket file descriptor");
  }

  if ((socket_data_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    die("Failed to get socket file descriptor");
  }

  if (bind(socket_cmd_fd_, (struct sockaddr *)&self_cmd_addr,
           sizeof(self_cmd_addr)) < 0) {
    die("Failed to bind socket self_cmd_addr");
  }

  if (bind(socket_data_fd_, (struct sockaddr *)&board_data_addr,
           sizeof(board_data_addr)) < 0) {
    die("Failed to bind socket board_data_addr");
  }
}

int SignalDAQ8::testBoard() {}

void SignalDAQ8::die(std::string message) {
  ROS_ERROR("%s with error: %s", message.c_str(), strerror(errno));
  exit(1);
}
