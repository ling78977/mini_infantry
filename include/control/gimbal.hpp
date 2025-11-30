#pragma once

#include "motor.hpp"
#include <cstdint>
#include <string>
namespace mini_infantry {

enum MotorAddress { Yaw = 1, Pitch = 2 };
enum RoateDirect {Forward=0,Reverse=1};
class Gimbal {
public:
  Gimbal(); // 构造函数
  void enableMotor(MotorAddress &addres);
  void rotateAngle(MotorAddress& addres,double &angle,RoateDirect direct,uint16_t velocity,uint8_t acc);

private:
  std::string serial_name_ = "/dev/ttyAMA0";
  int serial_fd_; // 串口文件描述符
  int pulses_per_revolution_=51200;
};
} // namespace mini_infantry