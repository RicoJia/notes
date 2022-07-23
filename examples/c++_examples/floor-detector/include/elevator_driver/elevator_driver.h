// Copyright (c) 2022 by Diligent Robotics. All Rights Reserved.
// See LICENSE file in the project root for full license information.
#pragma once

#include <fcntl.h>
#include <linux/serial.h>
#include <mutex>
#include <poll.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define DR_CRC16 0xA001
#define HEIGHT_OFFSET 3
#define PRESSURE_OFFSET 7
#define RD_LENGTH 13

namespace elevator_driver {

class ElevatorDriver {
public:
  ElevatorDriver() {
    indx_ = 0;
    height = 0.0;
    pressure = 0.0;
    fd = -1;
    memset(buff_, 0, sizeof(buff_));
  }

  ~ElevatorDriver() { closePort(); }
  int connect(char* device);

  uint16_t calcCRC(const size_t& dataLen);

  void parseSingleByte(const uint8_t& btToParse);

  void readSerial();

  float getPressure() { return pressure; }
  float getHeight() { return height; }

  void closePort() {
    if (fd > 0) {
      close(fd);
      fd = -1;
    }
  }

private:
  uint8_t buff_[20];
  float height;
  float pressure;
  size_t indx_;
  int fd;
};
}