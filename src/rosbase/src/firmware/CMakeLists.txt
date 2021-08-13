cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(rosbase_arduino
  SRCS basecontroller.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD nano
  PORT /dev/ttyS0
  SERIAL picocom @SERIAL_PORT@ -b 9600 -l
)