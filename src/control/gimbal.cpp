#include "control/gimbal.hpp"
#include <cstdlib>
#include <errno.h>         // Error integer and strerror() function
#include <fcntl.h>         // Contains file controls like O_RDWR
#include <spdlog/spdlog.h> // For spdlog
#include <string.h>        // For strerror
#include <termios.h>       // Contains POSIX terminal control definitions
#include <unistd.h>        // write(), read(), close()

namespace mini_infantry {

Gimbal::Gimbal() {
  serial_fd_ = open(serial_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (serial_fd_ < 0) {
    spdlog::error("Error {} from open: {}", errno, strerror(errno));
    return;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    spdlog::error("Error {} from tcgetattr: {}", errno, strerror(errno));
    close(serial_fd_);
    return;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE;         // Clear all the size bits
  tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;                                                      // Disable canonical mode
  tty.c_lflag &= ~ECHO;                                                        // Disable echo
  tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
  tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
  tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    spdlog::error("Error {} from tcsetattr: {}", errno, strerror(errno));
    close(serial_fd_);
    return;
  }

  spdlog::info("Serial port {} opened successfully.", serial_name_);
}

void Gimbal::enableMotor(MotorAddress &addres) {
  if (serial_fd_ < 0) {
    spdlog::error("Cannot send command, serial port not open.");
    return;
  }

  // 字节序列: 01 f3 ab 01 00 6b
  unsigned char command[] = {static_cast<unsigned char>(addres), 0xF3, 0xAB, 0x01, 0x00, 0x6B};
  int bytes_written = write(serial_fd_, command, sizeof(command));

  if (bytes_written != sizeof(command)) {
    spdlog::error("Error writing to serial port: {} bytes written, expected {} bytes. Error: {}", bytes_written, sizeof(command),
                  strerror(errno));
  } else {
    spdlog::debug("Command sent successfully: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}", command[0], command[1], command[2], command[3],
                  command[4], command[5]);
  }
}
void Gimbal::rotateAngle(MotorAddress &addres, double &angle, RoateDirect direct, uint16_t velocity = 5000, uint8_t acc = 255) {
  if (serial_fd_ < 0) {
    spdlog::error("Cannot send command, serial port not open.");
    return;
  }

  if (velocity > 5000) {
    spdlog::warn("velocity is bigger than max limit 5000, set the velocity: {} to max limit: 5000", velocity);
    velocity = 5000;
  }
  uint32_t pulse_count = static_cast<uint32_t>((static_cast<uint64_t>(std::abs(angle)) * pulses_per_revolution_) / 360);
  unsigned char command[] = {static_cast<unsigned char>(addres),
                             0xFD,
                             static_cast<unsigned char>(direct),
                             static_cast<unsigned char>((velocity >> 8) & 0xFF),
                             static_cast<unsigned char>((velocity >> 0) & 0xFF),
                             static_cast<unsigned char>(acc),
                             static_cast<unsigned char>((pulse_count >> 24) & 0xFF),
                             static_cast<unsigned char>((pulse_count >> 16) & 0xFF),
                             static_cast<unsigned char>((pulse_count >> 8) & 0xFF),
                             static_cast<unsigned char>((pulse_count >> 0) & 0xFF),
                             0x00,
                             0x00,
                             0x6B

  };
  int bytes_written = write(serial_fd_, command, sizeof(command));

  if (bytes_written != sizeof(command)) {
    spdlog::error("Error writing to serial port: {} bytes written, expected {} bytes. Error: {}", bytes_written, sizeof(command),
                  strerror(errno));
  } else {
    spdlog::debug("Command sent successfully: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
                  command[0], command[1], command[2], command[3], command[4], command[5], command[6], command[7], command[8], command[9],
                  command[10], command[11], command[12]);
  }
  
}

} // namespace mini_infantry
