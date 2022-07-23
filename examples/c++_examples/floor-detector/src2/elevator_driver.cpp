#include "elevator_driver/elevator_driver.h"

using namespace elevator_driver;

int ElevatorDriver::connect(char* device) {
  struct termios tios;
  speed_t speed;
  int flags;
  // flags = O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL;
  flags = O_RDWR;

  fd = open(device, flags);
  if (fd == -1) {
    fprintf(stderr, "ERROR, cannot open device %s\n", device);
  }
  memset(&tios, 0, sizeof(struct termios));
  speed = B115200;
  if ((cfsetispeed(&tios, speed) < 0) || (cfsetospeed(&tios, speed) < 0)) {
    close(fd);
    return -1;
  }
  // tios.c_cflag |= (CREAD | CLOCAL);
  // tios.c_cflag &= ~CSIZE;
  // tios.c_cflag |= CS8;
  // tios.c_cflag &=~ CSTOPB;
  // tios.c_cflag &=~ PARENB;
  // tios.c_iflag &= ~INPCK;
  // tios.c_iflag &= ~(IXON | IXOFF | IXANY);
  // tios.c_oflag &=~ OPOST;
  // tios.c_cc[VMIN] = 0;
  // tios.c_cc[VTIME] = 0;
    tios.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tios.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tios.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tios.c_cflag |= CS8; // 8 bits per byte (most common)
  tios.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tios.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tios.c_lflag &= ~ICANON;
  tios.c_lflag &= ~ECHO; // Disable echo
  tios.c_lflag &= ~ECHOE; // Disable erasure
  tios.c_lflag &= ~ECHONL; // Disable new-line echo
  tios.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tios.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tios.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tios.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tios.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  tios.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tios.c_cc[VMIN] = 0;

  if (tcsetattr(fd, TCSANOW, &tios) < 0) {
    close(fd);
    fd = -1;
    return -1;
  }
  return 0;
}

uint16_t ElevatorDriver::calcCRC(const size_t& dataLen) {
  uint16_t out = 0xFFFF;

  for (size_t i = 0; i < dataLen; i++) {
    out ^= buff_[i];
    for (int brd = 0; brd < 8; brd++) {
      if (out & 1) {
        out = (out >> 1) ^ DR_CRC16;
      } else {
        out = (out >> 1);
      }
    }
  }
  return out;
}

void ElevatorDriver::parseSingleByte(const uint8_t& btToParse) {
  if (indx_ == 0) {
    if (btToParse == 0x64) {
      buff_[indx_] = btToParse;
      indx_++;
    }
  } else if(indx_ == 1) {
    if (btToParse == 0x72) {
      buff_[indx_] = btToParse;
      indx_++;
    } else {
      indx_ = 0;
    }
  } else if (indx_ < RD_LENGTH - 1) {
    buff_[indx_] = btToParse;
    indx_++;
  } else if (indx_ < RD_LENGTH) {
    buff_[indx_] = btToParse;
    indx_++;
    uint16_t crcCalc = calcCRC(indx_ - 2);
    uint16_t crcRcv = 0;
    crcRcv = ((uint16_t)buff_[indx_ - 2])| ((uint16_t)buff_[indx_ - 1] << 8);
    if (crcCalc == crcRcv) {
      uint8_t* mvH = (uint8_t*)&height;
      mvH[0] = buff_[HEIGHT_OFFSET + 0];
      mvH[1] = buff_[HEIGHT_OFFSET + 1];
      mvH[2] = buff_[HEIGHT_OFFSET + 2];
      mvH[3] = buff_[HEIGHT_OFFSET + 3];
      uint8_t* mvP = (uint8_t*)&pressure;
      mvP[0] = buff_[PRESSURE_OFFSET + 0];
      mvP[1] = buff_[PRESSURE_OFFSET + 1];
      mvP[2] = buff_[PRESSURE_OFFSET + 2];
      mvP[3] = buff_[PRESSURE_OFFSET + 3];
    }
    indx_ = 0;
  }
}

void ElevatorDriver::readSerial() {
	uint8_t buffIn[20];
  tcflush(fd, TCIOFLUSH);
  int rc = read(fd, buffIn, RD_LENGTH);
  if (rc < 0) {
    return;
  }
  for (int i = 0; i < rc; i++) {
    parseSingleByte(buffIn[i]);
  }
}
