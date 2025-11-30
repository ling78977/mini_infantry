#pragma once
#include "control/motor.hpp"
#include <stdexcept>

#define d_x 0.125f
#define d_y 0.130f

namespace mini_infantry {

class MotionSolver {
private:
  // 私有构造函数：初始化电机指针为nullptr
  MotionSolver();

  // 私有析构函数：不负责释放外部传入的电机指针（由外部管理）
  ~MotionSolver();

  // 禁止拷贝构造和赋值
  MotionSolver(const MotionSolver &) = delete;
  MotionSolver &operator=(const MotionSolver &) = delete;

  // 电机指针（由外部传入）
  ::mini_infantry::Motor *motor_front_left_ = nullptr;
  ::mini_infantry::Motor *motor_front_right_ = nullptr;
  ::mini_infantry::Motor *motor_back_left_ = nullptr;
  ::mini_infantry::Motor *motor_back_right_ = nullptr;

  // 单例实例
  static MotionSolver *instance_;

public:
  // 获取单例实例
  static MotionSolver *getInstance();

  // 释放单例实例
  static void releaseInstance();

  // 外部传入电机指针（必须在使用其他功能前调用）
  static void setMotors(::mini_infantry::Motor *motor_front_left, ::mini_infantry::Motor *motor_front_right, ::mini_infantry::Motor *motor_back_left, ::mini_infantry::Motor *motor_back_right);

  // 运动解算主函数
  static void solve(float vx, float vy, float v_yaw);
};

} // namespace mini_infantry