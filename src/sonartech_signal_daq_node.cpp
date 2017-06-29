#include <sonartech_signal_daq_driver/SignalDAQ8.hpp>

#include <ros/ros.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "signal_daq");

  ros::NodeHandle nh;

  SignalDAQ8 signaldaq(nh);

  signaldaq.initializeSockets();

  signaldaq.testBoard();
  //signaldaq.sendCommand(2);
}
