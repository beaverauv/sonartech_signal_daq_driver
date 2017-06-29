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

int SignalDAQ8::testBoard() {
  ROS_INFO("Sending start packet...");

  sendCommand(1);

  ROS_INFO("Waiting for 10 seconds to receive data");

  struct timeval read_timeout;

  read_timeout.tv_sec = 10;
  read_timeout.tv_usec = 0;
  int recv_len;

  char buf[DATA_BUF_LEN];

  socklen_t slen = sizeof board_data_addr;


  setsockopt(socket_data_fd_, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

  if ((recv_len = recvfrom(socket_data_fd_, buf, DATA_BUF_LEN, 0, (struct sockaddr *) &board_data_addr, &slen)) < 0){
    sendCommand(2);
    die("Failed to open udp port 9004 for listening for data");
  }

  ROS_INFO("Got data with a length of %d", recv_len);

  sendCommand(2);

  // std::cout << createMessageFromData(buf, sizeof(buf));
  sonartech_signal_daq_driver::SamplesArray samples = createMessageFromData(buf, sizeof(buf));

  for (int i = 0; i < NUM_SAMPLES; i++){
    ROS_INFO("SAMPLE NUM %d", i);
    ROS_INFO("CHANNEL 1: %d", samples.samples[i].chnl1);
    ROS_INFO("CHANNEL 2: %d", samples.samples[i].chnl2);
    ROS_INFO("CHANNEL 3: %d", samples.samples[i].chnl3);
    ROS_INFO("CHANNEL 4: %d", samples.samples[i].chnl4);
    ROS_INFO("CHANNEL 5: %d", samples.samples[i].chnl5);
    ROS_INFO("CHANNEL 6: %d", samples.samples[i].chnl6);
    ROS_INFO("CHANNEL 7: %d", samples.samples[i].chnl7);
    ROS_INFO("CHANNEL 8: %d", samples.samples[i].chnl8);

  }
  // for (int i = 0; i < DATA_BUF_LEN; i++){
  //   ROS_INFO
  // }

  //ROS_INFO("%s", buf);

  // sendCommand(2);

}

sonartech_signal_daq_driver::SamplesArray SignalDAQ8::createMessageFromData(char* data, int data_len){
  if (data_len != DATA_BUF_LEN){
    ROS_ERROR("TRIED TO CREATE MESSAGE FROM DATA WITH WRONG SIZE");
    exit(1);
  }

  sonartech_signal_daq_driver::SamplesArray sample_array;

  //i_data is for the data index
  //i_sample is for the sample number index
  for (int i_data = 29, i_sample = 0; i_data < data_len && i_sample < NUM_SAMPLES; i_sample++){
    sample_array.samples[i_sample].chnl1 = (data[i_data+2] << 8) | data[i_data+1];
    i_data += 2;
    sample_array.samples[i_sample].chnl2 = (data[i_data+2] << 8) | data[i_data+1];
    i_data += 2;
    sample_array.samples[i_sample].chnl3 = (data[i_data+2] << 8) | data[i_data+1];
    i_data += 2;
    sample_array.samples[i_sample].chnl4 = (data[i_data+2] << 8) | data[i_data+1];
    i_data += 2;
    sample_array.samples[i_sample].chnl5 = (data[i_data+2] << 8) | data[i_data+1];
    i_data += 2;
    sample_array.samples[i_sample].chnl6 = (data[i_data+2] << 8) | data[i_data+1];
    i_data += 2;
    sample_array.samples[i_sample].chnl7 = (data[i_data+2] << 8) | data[i_data+1];
    i_data += 2;
    sample_array.samples[i_sample].chnl8 = (data[i_data+2] << 8) | data[i_data+1];
    i_data += 2;
  }

  return sample_array;
}

void SignalDAQ8::die(std::string message) {
  ROS_ERROR("%s with error: %s", message.c_str(), strerror(errno));
  exit(1);
}
