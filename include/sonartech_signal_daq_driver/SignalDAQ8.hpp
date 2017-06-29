#ifndef SONARTECHSIGNALDAQ_HPP_
#define SONARTECHSIGNALDAQ_HPP_
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <ros/ros.h>

#include <sonartech_signal_daq_driver/SamplesArray.h>

class SignalDAQ8 {
public:
  SignalDAQ8(const ros::NodeHandle &nh);
  ~SignalDAQ8();

  int testBoard();

  int initializeSockets();

  int sendCommand(short data);

  sonartech_signal_daq_driver::SamplesArray createMessageFromData(char* data, int data_len);

  void die(std::string message);

  ros::NodeHandle nh_;

  struct sockaddr_in board_cmd_addr, self_cmd_addr;
  struct sockaddr_in board_data_addr;

  int socket_data_fd_, socket_cmd_fd_;

  static constexpr const char *board_ip_ = "192.168.0.73";
  static constexpr const char *self_ip_ = "192.168.0.10";

  static constexpr int cmd_port_ = 8004;
  static constexpr int data_port_ = 9004;

  static constexpr int DATA_BUF_LEN = 8030;
  static constexpr int NUM_SAMPLES = 500;
};

#endif
