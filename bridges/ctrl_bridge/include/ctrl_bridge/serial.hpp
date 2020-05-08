#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <queue>
#include <array>

#include <ros/ros.h>
#include "ctrl_bridge/msgs_defs.hpp"

namespace ctrl_bridge {

class SerialPort {
public:
  SerialPort();
  SerialPort(const std::string port, const int baudrate);
  ~SerialPort();

  bool start(const std::string port, const int baudrate);
  bool stop();
  bool isOpen() const;

  ssize_t send(const char* const buffer, const size_t length);
  size_t receive(char* const buffer);

  static constexpr size_t BUFFERSIZE = 1024;

private:
  bool configure(const int baudrate);
  void serialThread();

  int serial_fd_{-1};
  bool should_exit_{false};

  std::thread serial_thread_;

  char receive_buffer_[BUFFERSIZE];
  size_t received_length_{0u};

  char received_message_[BUFFERSIZE];
  size_t received_message_length_{0u};

  std::mutex rc_mtx_;
  std::mutex proc_mtx_;
  std::mutex buffer_mtx;
  std::condition_variable rc_cv_;
  std::condition_variable proc_cv_;
};

} // namespace ctrl_brdige