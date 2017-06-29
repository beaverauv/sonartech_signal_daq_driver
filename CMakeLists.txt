cmake_minimum_required(VERSION 2.8.3)

project(sonartech_signal_daq_driver)


add_definitions("-std=c++14")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Sample.msg
  SamplesArray.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)


add_executable(sonartech_signal_daq_node
  src/sonartech_signal_daq_node.cpp
  src/SignalDAQ8.cpp
)

target_link_libraries(sonartech_signal_daq_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(sonartech_signal_daq_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
