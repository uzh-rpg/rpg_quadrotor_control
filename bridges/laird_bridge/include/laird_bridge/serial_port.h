/*
 * serial_port.h
 *
 *  Created on: Nov 15, 2018
 *      Author: jkohler based on ffontana's work
 */

#pragma once



#include "ros/ros.h"

#include "mavlink/v1.0/rpg/mavlink.h"

// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>

// random includes
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <thread>

namespace laird_bridge
{

class LairdBridge;
// forward declaration

class SerialPort
{
public:
  SerialPort() {};
  SerialPort(std::shared_ptr<LairdBridge> laird_bridge);
  virtual ~SerialPort();

  bool connect(const std::string port, const int baud);
  void disconnect();

  bool setup(const int baud);
  void serialThread();
  void sendMavlinkMessage(mavlink_message_t mavlink_msg);

  std::shared_ptr<LairdBridge>  laird_bridge_;
  int serial_port_fd_;
  std::thread serial_thread_;
  bool serial_thread_should_exit_;
};

} /* namespace laird_bridge */
