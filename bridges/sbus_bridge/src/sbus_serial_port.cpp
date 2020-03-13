#include "sbus_bridge/sbus_serial_port.h"

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <deque>

#include <ros/ros.h>

namespace sbus_bridge {

SBusSerialPort::SBusSerialPort()
    : receiver_thread_(),
      receiver_thread_should_exit_(false),
      serial_port_fd_(-1) {}

SBusSerialPort::SBusSerialPort(const std::string& port,
                               const bool start_receiver_thread)
    : receiver_thread_(),
      receiver_thread_should_exit_(false),
      serial_port_fd_(-1) {
  setUpSBusSerialPort(port, start_receiver_thread);
}

SBusSerialPort::~SBusSerialPort() { disconnectSerialPort(); }

bool SBusSerialPort::setUpSBusSerialPort(const std::string& port,
                                         const bool start_receiver_thread) {
  if (!connectSerialPort(port)) {
    return false;
  }

  if (start_receiver_thread) {
    if (!startReceiverThread()) {
      return false;
    }
  }

  return true;
}

bool SBusSerialPort::connectSerialPort(const std::string& port) {
  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  ROS_INFO("[%s] Connect to serial port", ros::this_node::getName().c_str());

  if (serial_port_fd_ == -1) {
    ROS_ERROR("[%s] Could not open serial port %s",
              ros::this_node::getName().c_str(), port.c_str());
    return false;
  }

  if (!configureSerialPortForSBus()) {
    close(serial_port_fd_);
    ROS_ERROR("[%s] Could not set necessary configuration of serial port",
              ros::this_node::getName().c_str());
    return false;
  }

  return true;
}

void SBusSerialPort::disconnectSerialPort() {
  stopReceiverThread();

  close(serial_port_fd_);
}

bool SBusSerialPort::startReceiverThread() {
  // Start watchdog thread
  try {
    receiver_thread_ =
        std::thread(&SBusSerialPort::serialPortReceiveThread, this);
  } catch (...) {
    ROS_ERROR("[%s] Could not successfully start SBUS receiver thread.",
              ros::this_node::getName().c_str());
    return false;
  }

  return true;
}

bool SBusSerialPort::stopReceiverThread() {
  if (!receiver_thread_.joinable()) {
    return true;
  }

  receiver_thread_should_exit_ = true;

  // Wait for receiver thread to finish
  receiver_thread_.join();

  return true;
}

bool SBusSerialPort::configureSerialPortForSBus() const {
  // clear config
  fcntl(serial_port_fd_, F_SETFL, 0);
  // read non blocking
  fcntl(serial_port_fd_, F_SETFL, FNDELAY);

  struct termios2 uart_config;
  /* Fill the struct for the new configuration */
  ioctl(serial_port_fd_, TCGETS2, &uart_config);

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
  uart_config.c_iflag &=
      ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  //
  // No line processing:
  // echo off
  // echo newline off
  // canonical mode off,
  // extended input processing off
  // signal chars off
  //
  uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  // Turn off character processing
  // Turn off odd parity
  uart_config.c_cflag &= ~(CSIZE | PARODD | CBAUD);

  // Enable parity generation on output and parity checking for input.
  uart_config.c_cflag |= PARENB;
  // Set two stop bits, rather than one.
  uart_config.c_cflag |= CSTOPB;
  // No output processing, force 8 bit input
  uart_config.c_cflag |= CS8;
  // Enable a non standard baud rate
  uart_config.c_cflag |= BOTHER;

  // Set custom baud rate of 100'000 bits/s necessary for sbus
  const speed_t spd = 100000;
  uart_config.c_ispeed = spd;
  uart_config.c_ospeed = spd;

  if (ioctl(serial_port_fd_, TCSETS2, &uart_config) < 0) {
    ROS_ERROR("[%s] could not set configuration of serial port",
              ros::this_node::getName().c_str());
    return false;
  }

  return true;
}

void SBusSerialPort::transmitSerialSBusMessage(const SBusMsg& sbus_msg) const {
  static uint8_t buffer[kSbusFrameLength_];

  // SBUS header
  buffer[0] = kSbusHeaderByte_;

  // 16 channels of 11 bit data
  buffer[1] = (uint8_t)((sbus_msg.channels[0] & 0x07FF));
  buffer[2] = (uint8_t)((sbus_msg.channels[0] & 0x07FF) >> 8 |
                        (sbus_msg.channels[1] & 0x07FF) << 3);
  buffer[3] = (uint8_t)((sbus_msg.channels[1] & 0x07FF) >> 5 |
                        (sbus_msg.channels[2] & 0x07FF) << 6);
  buffer[4] = (uint8_t)((sbus_msg.channels[2] & 0x07FF) >> 2);
  buffer[5] = (uint8_t)((sbus_msg.channels[2] & 0x07FF) >> 10 |
                        (sbus_msg.channels[3] & 0x07FF) << 1);
  buffer[6] = (uint8_t)((sbus_msg.channels[3] & 0x07FF) >> 7 |
                        (sbus_msg.channels[4] & 0x07FF) << 4);
  buffer[7] = (uint8_t)((sbus_msg.channels[4] & 0x07FF) >> 4 |
                        (sbus_msg.channels[5] & 0x07FF) << 7);
  buffer[8] = (uint8_t)((sbus_msg.channels[5] & 0x07FF) >> 1);
  buffer[9] = (uint8_t)((sbus_msg.channels[5] & 0x07FF) >> 9 |
                        (sbus_msg.channels[6] & 0x07FF) << 2);
  buffer[10] = (uint8_t)((sbus_msg.channels[6] & 0x07FF) >> 6 |
                         (sbus_msg.channels[7] & 0x07FF) << 5);
  buffer[11] = (uint8_t)((sbus_msg.channels[7] & 0x07FF) >> 3);
  buffer[12] = (uint8_t)((sbus_msg.channels[8] & 0x07FF));
  buffer[13] = (uint8_t)((sbus_msg.channels[8] & 0x07FF) >> 8 |
                         (sbus_msg.channels[9] & 0x07FF) << 3);
  buffer[14] = (uint8_t)((sbus_msg.channels[9] & 0x07FF) >> 5 |
                         (sbus_msg.channels[10] & 0x07FF) << 6);
  buffer[15] = (uint8_t)((sbus_msg.channels[10] & 0x07FF) >> 2);
  buffer[16] = (uint8_t)((sbus_msg.channels[10] & 0x07FF) >> 10 |
                         (sbus_msg.channels[11] & 0x07FF) << 1);
  buffer[17] = (uint8_t)((sbus_msg.channels[11] & 0x07FF) >> 7 |
                         (sbus_msg.channels[12] & 0x07FF) << 4);
  buffer[18] = (uint8_t)((sbus_msg.channels[12] & 0x07FF) >> 4 |
                         (sbus_msg.channels[13] & 0x07FF) << 7);
  buffer[19] = (uint8_t)((sbus_msg.channels[13] & 0x07FF) >> 1);
  buffer[20] = (uint8_t)((sbus_msg.channels[13] & 0x07FF) >> 9 |
                         (sbus_msg.channels[14] & 0x07FF) << 2);
  buffer[21] = (uint8_t)((sbus_msg.channels[14] & 0x07FF) >> 6 |
                         (sbus_msg.channels[15] & 0x07FF) << 5);
  buffer[22] = (uint8_t)((sbus_msg.channels[15] & 0x07FF) >> 3);

  // SBUS flags
  // (bit0 = least significant bit)
  // bit0 = ch17 = digital channel (0x01)
  // bit1 = ch18 = digital channel (0x02)
  // bit2 = Frame lost, equivalent red LED on receiver (0x04)
  // bit3 = Failsafe activated (0x08)
  // bit4 = n/a
  // bit5 = n/a
  // bit6 = n/a
  // bit7 = n/a
  buffer[23] = 0x00;
  if (sbus_msg.digital_channel_1) {
    buffer[23] |= 0x01;
  }
  if (sbus_msg.digital_channel_2) {
    buffer[23] |= 0x02;
  }
  if (sbus_msg.frame_lost) {
    buffer[23] |= 0x04;
  }
  if (sbus_msg.failsafe) {
    buffer[23] |= 0x08;
  }

  // SBUS footer
  buffer[24] = kSbusFooterByte_;

  const int written = write(serial_port_fd_, (char*)buffer, kSbusFrameLength_);
  // tcflush(serial_port_fd_, TCOFLUSH); // There were rumors that this might
  // not work on Odroids...
  if (written != kSbusFrameLength_) {
    ROS_ERROR("[%s] Wrote %d bytes but should have written %d",
              ros::this_node::getName().c_str(), written, kSbusFrameLength_);
  }
}

void SBusSerialPort::serialPortReceiveThread() {
  struct pollfd fds[1];
  fds[0].fd = serial_port_fd_;
  fds[0].events = POLLIN;

  uint8_t init_buf[10];
  while (read(serial_port_fd_, init_buf, sizeof(init_buf)) > 0) {
    // On startup, as long as we receive something, we keep reading to ensure
    // that the first byte of the first poll is the start of an SBUS message
    // and not some arbitrary byte.
    // This should help to get the framing in sync in the beginning.
    usleep(100);
  }

  std::deque<uint8_t> bytes_buf;

  while (!receiver_thread_should_exit_) {
    // Buffer to read bytes from serial port. We make it large enough to
    // potentially contain 4 sbus messages but its actual size probably does
    // not matter too much
    uint8_t read_buf[4 * kSbusFrameLength_];

    if (poll(fds, 1, kPollTimeoutMilliSeconds_) > 0) {
      if (fds[0].revents & POLLIN) {
        const ssize_t nread = read(serial_port_fd_, read_buf, sizeof(read_buf));

        for (ssize_t i = 0; i < nread; i++) {
          bytes_buf.push_back(read_buf[i]);
        }

        bool valid_sbus_message_received = false;
        uint8_t sbus_msg_bytes[kSbusFrameLength_];
        while (bytes_buf.size() >= kSbusFrameLength_) {
          // Check if we have a potentially valid SBUS message
          // A valid SBUS message must have to correct header and footer byte
          // as well as zeros in the four most significant bytes of the flag
          // byte (byte 23)
          if (bytes_buf.front() == kSbusHeaderByte_ &&
              !(bytes_buf[kSbusFrameLength_ - 2] & 0xF0) &&
              bytes_buf[kSbusFrameLength_ - 1] == kSbusFooterByte_) {
            for (uint8_t i = 0; i < kSbusFrameLength_; i++) {
              sbus_msg_bytes[i] = bytes_buf.front();
              bytes_buf.pop_front();
            }

            valid_sbus_message_received = true;
          } else {
            // If it is not a valid SBUS message but has a correct header byte
            // we need to pop it to prevent staying in this loop forever
            bytes_buf.pop_front();
            ROS_WARN("[%s] SBUS message framing not in sync",
                     ros::this_node::getName().c_str());
          }

          // If not, pop front elements until we have a valid header byte
          while (!bytes_buf.empty() && bytes_buf.front() != kSbusHeaderByte_) {
            bytes_buf.pop_front();
          }
        }

        if (valid_sbus_message_received) {
          // Sometimes we read more than one sbus message at the same time
          // By running the loop above for as long as possible before handling
          // the received sbus message we achieve to only process the latest one
          const SBusMsg received_sbus_msg = parseSbusMessage(sbus_msg_bytes);
          handleReceivedSbusMessage(received_sbus_msg);
        }
      }
    }
  }

  return;
}

SBusMsg SBusSerialPort::parseSbusMessage(
    uint8_t sbus_msg_bytes[kSbusFrameLength_]) const {
  SBusMsg sbus_msg;

  sbus_msg.timestamp = ros::Time::now();

  // Decode the 16 regular channels
  sbus_msg.channels[0] =
      (((uint16_t)sbus_msg_bytes[1]) | ((uint16_t)sbus_msg_bytes[2] << 8)) &
      0x07FF;
  sbus_msg.channels[1] = (((uint16_t)sbus_msg_bytes[2] >> 3) |
                          ((uint16_t)sbus_msg_bytes[3] << 5)) &
                         0x07FF;
  sbus_msg.channels[2] =
      (((uint16_t)sbus_msg_bytes[3] >> 6) | ((uint16_t)sbus_msg_bytes[4] << 2) |
       ((uint16_t)sbus_msg_bytes[5] << 10)) &
      0x07FF;
  sbus_msg.channels[3] = (((uint16_t)sbus_msg_bytes[5] >> 1) |
                          ((uint16_t)sbus_msg_bytes[6] << 7)) &
                         0x07FF;
  sbus_msg.channels[4] = (((uint16_t)sbus_msg_bytes[6] >> 4) |
                          ((uint16_t)sbus_msg_bytes[7] << 4)) &
                         0x07FF;
  sbus_msg.channels[5] =
      (((uint16_t)sbus_msg_bytes[7] >> 7) | ((uint16_t)sbus_msg_bytes[8] << 1) |
       ((uint16_t)sbus_msg_bytes[9] << 9)) &
      0x07FF;
  sbus_msg.channels[6] = (((uint16_t)sbus_msg_bytes[9] >> 2) |
                          ((uint16_t)sbus_msg_bytes[10] << 6)) &
                         0x07FF;
  sbus_msg.channels[7] = (((uint16_t)sbus_msg_bytes[10] >> 5) |
                          ((uint16_t)sbus_msg_bytes[11] << 3)) &
                         0x07FF;
  sbus_msg.channels[8] =
      (((uint16_t)sbus_msg_bytes[12]) | ((uint16_t)sbus_msg_bytes[13] << 8)) &
      0x07FF;
  sbus_msg.channels[9] = (((uint16_t)sbus_msg_bytes[13] >> 3) |
                          ((uint16_t)sbus_msg_bytes[14] << 5)) &
                         0x07FF;
  sbus_msg.channels[10] = (((uint16_t)sbus_msg_bytes[14] >> 6) |
                           ((uint16_t)sbus_msg_bytes[15] << 2) |
                           ((uint16_t)sbus_msg_bytes[16] << 10)) &
                          0x07FF;
  sbus_msg.channels[11] = (((uint16_t)sbus_msg_bytes[16] >> 1) |
                           ((uint16_t)sbus_msg_bytes[17] << 7)) &
                          0x07FF;
  sbus_msg.channels[12] = (((uint16_t)sbus_msg_bytes[17] >> 4) |
                           ((uint16_t)sbus_msg_bytes[18] << 4)) &
                          0x07FF;
  sbus_msg.channels[13] = (((uint16_t)sbus_msg_bytes[18] >> 7) |
                           ((uint16_t)sbus_msg_bytes[19] << 1) |
                           ((uint16_t)sbus_msg_bytes[20] << 9)) &
                          0x07FF;
  sbus_msg.channels[14] = (((uint16_t)sbus_msg_bytes[20] >> 2) |
                           ((uint16_t)sbus_msg_bytes[21] << 6)) &
                          0x07FF;
  sbus_msg.channels[15] = (((uint16_t)sbus_msg_bytes[21] >> 5) |
                           ((uint16_t)sbus_msg_bytes[22] << 3)) &
                          0x07FF;

  // Decode flags from byte 23 (see comments in sendSBusMessage function)
  // By default, flags are set to false in SBusMsg constructor, so we only need
  // to set them true if necessary
  if (sbus_msg_bytes[23] & (1 << 0)) {
    sbus_msg.digital_channel_1 = true;
  }

  if (sbus_msg_bytes[23] & (1 << 1)) {
    sbus_msg.digital_channel_2 = true;
  }

  if (sbus_msg_bytes[23] & (1 << 2)) {
    sbus_msg.frame_lost = true;
  }

  if (sbus_msg_bytes[23] & (1 << 3)) {
    sbus_msg.failsafe = true;
  }

  return sbus_msg;
}

}  // namespace sbus_bridge
