#include "ctrl_bridge/serial.hpp"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/poll.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <chrono>

#include "ctrl_bridge/msgs_defs.hpp"

namespace ctrl_bridge {

SerialPort::SerialPort()
{}

SerialPort::SerialPort(const std::string port, const int baudrate) :
  SerialPort()
{
  if(!start(port, baudrate))
    ROS_WARN("[%s] Could not open port %s with baudrate %d!",
      ros::this_node::getName().c_str(),
      port.c_str(), baudrate);
}

SerialPort::~SerialPort()
{
  stop();
}

bool SerialPort::start(const std::string port, const int baudrate)
{
  ROS_INFO("[%s] Connecting to serial port %s",
    ros::this_node::getName().c_str(), port.c_str());
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  usleep(10000);

  if(serial_fd_ == -1)
  {
    ROS_ERROR("[%s] Could not open serial port %s",
      ros::this_node::getName().c_str(), port.c_str());
    return false;
  }

  ROS_INFO("[%s] Configuring serial port %s",
    ros::this_node::getName().c_str(), port.c_str());

  if(!configure(baudrate))
  {
    ROS_ERROR("[%s] Could not configure serial port %s",
      ros::this_node::getName().c_str(), port.c_str());
    return false;
  }

  // Clear buffer brute force way.
  char trash_buffer[BUFFERSIZE];
  while(read(serial_fd_, trash_buffer, BUFFERSIZE)>0);

  should_exit_ = false;
  serial_thread_ = std::thread(&SerialPort::serialThread, this);

  return true;
}

bool SerialPort::stop()
{
  should_exit_ = true;

  if(serial_thread_.joinable())
    serial_thread_.join();

  if(serial_fd_)
    close(serial_fd_);

  return true;
}

bool SerialPort::isOpen() const
{
  return fcntl(serial_fd_, F_GETFD) != -1 || errno != EBADF;
}


bool SerialPort::configure(const int baudrate)
{
  int baud;
  switch(baudrate)
  {
    case 115200:
      baud = B115200;
      break;
    case 230400:
      baud = B230400;
      break;
    case 460800:
      baud = B460800;
      break;
    case 921600:
      baud = B921600;
      break;
    default:
      ROS_ERROR("[%s] Desired baud rate %d not supported.",
        ros::this_node::getName().c_str(), baudrate);
      return false;
  }

  // clear config
  fcntl(serial_fd_, F_SETFL, 0);
  // read non blocking
  fcntl(serial_fd_, F_SETFL, FNDELAY);

  struct termios uart_config;
  
  // Get config
  tcgetattr(serial_fd_, &uart_config);

  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  uart_config.c_oflag &=
    ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  uart_config.c_iflag &=
    ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  // No line processing:
  // echo off
  // echo newline off
  // canonical mode off,
  // extended input processing off
  // signal chars off
  uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  uart_config.c_cflag &= ~(CSIZE | PARENB);
  uart_config.c_cflag |= CS8;

  if(cfsetispeed(&uart_config, baud) < 0 || cfsetospeed(&uart_config, baud) < 0)
  {
    ROS_ERROR("[%s] Could not set desired baud rate %d!",
      ros::this_node::getName().c_str(), baudrate);
    return false;
  }

  // Finally, apply the configuration
  if (tcsetattr(serial_fd_, TCSAFLUSH, &uart_config) < 0)
  {
    ROS_ERROR("[%s] Could not set configuration of serial port!",
      ros::this_node::getName().c_str());
    return false;
  }
  return true;
}

void SerialPort::serialThread()
{
  while(!should_exit_ && ros::ok())
  {
    size_t last_length = received_length_;
    const ssize_t bytes_read  = read(
      serial_fd_,
      &receive_buffer_[received_length_],
      BUFFERSIZE - received_length_);

    if(bytes_read > 0) received_length_ += bytes_read;
    if(!received_length_) continue;

    // ROS_INFO("[%s] Read %d bytes, buffer is %zu!",
    //   ros::this_node::getName().c_str(), bytes_read, received_length_);

    // If there is nothing to do, poll again.
    if(received_length_ == 0u) continue;

    // If we don't have a container start yet, move to the first delimiter.
    for(size_t idx = 0; idx < received_length_; ++idx)
    {
      // If we have a delimiter...
      if(receive_buffer_[idx] == msgs::SERIAL_CONTAINER_DELIMITER)
      {
        // ... and catch the double delimiter case
        while((idx+1) < received_length_ &&
            receive_buffer_[idx+1] == msgs::SERIAL_CONTAINER_DELIMITER)
        {
          ++idx;
        }
        char tmp[BUFFERSIZE];
        received_length_ -= idx;
        last_length = 0u;
        memcpy(tmp, &receive_buffer_[idx], received_length_);
        memcpy(receive_buffer_, tmp, received_length_);
        break;
      }
    }

    // If there is nothing to do, poll again.
    if(received_length_ == 0u) continue;

    // If we haven't found a start, trash all received data
    if(receive_buffer_[0] != msgs::SERIAL_CONTAINER_DELIMITER)
    {
      received_length_ = 0u;
      continue;
    }

    // Now we have a delimiter at the beginning and can search for the end
    for(size_t idx = std::max((size_t)1, last_length); idx < received_length_; ++idx)
    {
      if(receive_buffer_[idx] == msgs::SERIAL_CONTAINER_DELIMITER)
      {
        // End found, copy into output buffer and notify possible receiver.
        const std::lock_guard<std::mutex> guard(buffer_mtx);
        memcpy(received_message_, &receive_buffer_[1], idx-1u);
        received_message_length_ = idx-1u;
        rc_cv_.notify_all();

        char tmp[BUFFERSIZE];
        received_length_ -= idx;
        last_length = 0;
        memcpy(tmp, &receive_buffer_[idx], received_length_);
        memcpy(receive_buffer_, tmp, received_length_);
        break;
      }
    }

    // If we have a start but no end in a full buffer, something went wrong.
    // We trash the data and wait for new data.
    if(received_length_ == BUFFERSIZE)
    {
      received_length_ = 0;
      continue;
    }
    std::unique_lock<std::mutex> lck(proc_mtx_);
    proc_cv_.wait_for(lck, std::chrono::milliseconds(100));
  }
}


ssize_t SerialPort::send(const char* const buffer, const size_t length)
{
  const ssize_t bytes_written = write(serial_fd_, buffer, length);
  if(bytes_written < 0)
  {
    ROS_ERROR("[%s] Could not write to serial port!",
      ros::this_node::getName().c_str());
  }
  else if((size_t)bytes_written < length)
  {
    ROS_ERROR("[%s] Could only write %zu of %zu bytes!",
      ros::this_node::getName().c_str(), (size_t)bytes_written, (size_t)length);
  }

  return (size_t)bytes_written;
}

size_t SerialPort::receive(char* const buffer)
{
  if(!received_message_length_)
  {
    std::unique_lock<std::mutex> lck(rc_mtx_);
    rc_cv_.wait_for(lck, std::chrono::milliseconds(2));
  }

  const size_t return_length = received_message_length_;
  if(received_message_length_)
  {
    const std::lock_guard<std::mutex> guard(buffer_mtx);
    memcpy(buffer, received_message_, received_message_length_);
    received_message_length_ = 0u;
  }

  proc_cv_.notify_all();
  return return_length;
}



} // namespace ctrl_brdige
