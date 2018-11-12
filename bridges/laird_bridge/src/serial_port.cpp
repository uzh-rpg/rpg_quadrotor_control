/*
 * serial_port.cpp
 *
 *  Created on: Apr 30, 2014
 *  Revised on: Nov 15, 2018 by jkohler
 *      Author: ffontana
 */

#include "laird_bridge/serial_port.h"
#include "laird_bridge/laird_bridge.h"
#include <sys/poll.h>
#include <termios.h>


namespace laird_bridge
{

SerialPort::SerialPort(std::shared_ptr<LairdBridge>& laird_bridge)
{
  serial_thread_should_exit_ = false;
  serial_port_fd_ = 0;
  laird_bridge_ = laird_bridge;
}

SerialPort::~SerialPort()
{
  disconnect();
}

bool SerialPort::connect(const std::string port, const int baud)
{
  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  ROS_INFO("[%s] Connect to serial port", ros::this_node::getName().c_str());

  if (serial_port_fd_ == -1)
  {
    ROS_ERROR("%s Could not open serial port %s", ros::this_node::getName().c_str(), port.c_str());
    return false;
  }

  if (!setup(baud))
  {
    close(serial_port_fd_);
    ROS_ERROR("%s Could not set configuration of serial port", ros::this_node::getName().c_str());
    return false;
  }
  serial_thread_ = std::thread(SerialPort::serial_thread_, this);
  return true;
}

void SerialPort::disconnect()
{
  serial_thread_should_exit_ = true;
  // TODO wait for thread to finish
  close(serial_port_fd_);

}

bool SerialPort::setup(const int baud)
{

  /* set speed config */
  int speed;
  switch (baud)
  {
    case 115200:
      speed = B115200;
      break;
    case 230400:
      speed = B230400;
      break;
    case 460800:
      speed = B460800;
      break;
    case 921600:
      speed = B921600;
      break;
    default:
      ROS_ERROR("%s: Desired baud rate %d not supported, aborting.", ros::this_node::getName().c_str(), baud);
      return false;
      break;
  }

  // clear config
  fcntl(serial_port_fd_, F_SETFL, 0);
  // read non blocking
  fcntl(serial_port_fd_, F_SETFL, FNDELAY);

  struct termios uart_config;
  /* Fill the struct for the new configuration */
  tcgetattr(serial_port_fd_, &uart_config);

  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
  uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  //
  // No line processing:
  // echo off
  // echo newline off
  // canonical mode off,
  // extended input processing off
  // signal chars off
  //
  uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  //
  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  //
  uart_config.c_cflag &= ~(CSIZE | PARENB);
  uart_config.c_cflag |= CS8;

  if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0)
  {
    ROS_ERROR("%s: Could not set desired baud rate of %d Baud", ros::this_node::getName().c_str(), baud);
    return false;
  }

  // Finally, apply the configuration
  // TODO tcsetattr(_uart_fd, TCSAFLUSH, &uart_config)
  if (tcsetattr(serial_port_fd_, TCSAFLUSH, &uart_config) < 0)
  {
    ROS_ERROR("%s Could not set configuration of serial port", ros::this_node::getName().c_str());
    return false;
  }
  return true;
}

void SerialPort::serialThread()
{
  int prev_droped_packages = 0;
  int droped_packages = 0;

  struct pollfd fds[1];
  fds[0].fd = serial_port_fd_;
  fds[0].events = POLLIN;

  while (!serial_thread_should_exit_ && ros::ok())
  {
    mavlink_message_t mavlink_msg;
    mavlink_status_t status;
    uint8_t buf[32];

    if (poll(fds, 1, 500) > 0)
    {
      if (fds[0].revents & POLLIN)
      {
        ssize_t nread = read(serial_port_fd_, buf, sizeof(buf));

        for (ssize_t i = 0; i < nread; i++)
        {
          if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &mavlink_msg, &status))
          {
            laird_bridge_->handleMavlinkMessage(mavlink_msg);
          }
          droped_packages += status.packet_rx_drop_count;
        }

        /* to avoid reading very small chunks wait before polling again */
        if (nread < (ssize_t)sizeof(buf))
        {
          usleep(500); // sleep 0.5 ms
        }
      }
    }

    // Some Debugging output about dropping packets
    if (prev_droped_packages != droped_packages)
    {
      ROS_DEBUG("[%s] Dropped packages, ctr = %d", ros::this_node::getName().c_str(), droped_packages);
    }
    prev_droped_packages = droped_packages;
  }
  return;
}

void SerialPort::sendMavlinkMessage(mavlink_message_t mavlink_msg)
{
  static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;

  mavlink_finalize_message_chan(&mavlink_msg, mavlink_msg.sysid, mavlink_msg.compid, MAVLINK_COMM_0, mavlink_msg.len,
                                mavlink_crcs[mavlink_msg.msgid]);

  static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int messageLength = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);
  int written = write(serial_port_fd_, (char*)buffer, messageLength);
  // if you tcflush on the odorid you will loose a lot of packets (ffontana)
  //tcflush(serial_port_fd_, TCOFLUSH);
  if (messageLength != written)
    ROS_ERROR("%s: Wrote %d bytes but should have written %d", ros::this_node::getName().c_str(), written,
              messageLength);
}

} /* namespace laird_bridge */
