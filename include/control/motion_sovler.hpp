#pragma once
#include "motor.hpp"

#define d_x 0.125f
#define d_y 0.130f


namespace mini_infantry {

class MotionSolver {
private:
  // 私有构造函数：初始化电机指针为nullptr
  MotionSolver() : motor_front_left_(nullptr), motor_front_right_(nullptr), motor_back_left_(nullptr), motor_back_right_(nullptr) {}

  // 私有析构函数：不负责释放外部传入的电机指针（由外部管理）
  ~MotionSolver() = default;

  // 禁止拷贝构造和赋值
  MotionSolver(const MotionSolver &) = delete;
  MotionSolver &operator=(const MotionSolver &) = delete;

  // 电机指针（由外部传入）
  Motor *motor_front_left_ = nullptr;
  Motor *motor_front_right_ = nullptr;
  Motor *motor_back_left_ = nullptr;
  Motor *motor_back_right_ = nullptr;

  // 单例实例
  inline static MotionSolver *instance_ = nullptr;

public:
  // 获取单例实例
  static MotionSolver *getInstance() {
    if (instance_ == nullptr) {
      instance_ = new MotionSolver();
    }
    return instance_;
  }

  // 释放单例实例
  static void releaseInstance() {
    delete instance_;
    instance_ = nullptr;
  }

  // 外部传入电机指针（必须在使用其他功能前调用）
  static void setMotors(Motor *motor_front_left, Motor *motor_front_right, Motor *motor_back_left, Motor *motor_back_right) {
    MotionSolver *solver = getInstance();
    if (solver->motor_front_left_ != nullptr) {
      // 可选：防止重复设置电机指针
      spdlog::critical("Motors already set. Cannot reinitialize.");
    }
    if (!motor_front_left || !motor_front_right || !motor_back_left || !motor_back_right) {
      spdlog::critical("Motor pointers cannot be null.");
    }
    solver->motor_front_left_ = motor_front_left;
    solver->motor_front_right_ = motor_front_right;
    solver->motor_back_left_ = motor_back_left;
    solver->motor_back_right_ = motor_back_right;
  }

  // 运动解算主函数
  static void solve(float vx, float vy, float v_yaw) {
    MotionSolver *solver = getInstance();
    if (!solver->motor_front_left_ || !solver->motor_front_right_ || !solver->motor_back_left_ || !solver->motor_back_right_) {
      spdlog::critical("Motors not initialized. Call setMotors() first.");
    }
    float lf_speed, rf_speed, lb_speed, rb_speed;

    lf_speed = vx - vy - (d_x + d_y) * v_yaw;
    rf_speed = vx + vy + (d_x + d_y) * v_yaw;
    lb_speed = vx + vy - (d_x + d_y) * v_yaw;
    rb_speed = vx - vy + (d_x + d_y) * v_yaw;

    float max_ = fmax(fabs(lf_speed), fabs(rf_speed));
    max_       = fmax(max_, fabs(lb_speed));
    max_       = fmax(max_, fabs(rb_speed));

    if (max_ > 2) {
        lf_speed /= max_;
        rf_speed /= max_;
        lb_speed /= max_;
        rb_speed /= max_;
    }
    int pwm_value_front_left  = solver->motor_front_left_->pidCalculate(lf_speed, solver->motor_front_left_->getEncoderSpeed());
    int pwm_value_front_right = solver->motor_front_right_->pidCalculate(rf_speed, solver->motor_front_right_->getEncoderSpeed());
    int pwm_value_back_left   = solver->motor_back_left_->pidCalculate(lb_speed, solver->motor_back_left_->getEncoderSpeed());
    int pwm_value_back_right  = solver->motor_back_right_->pidCalculate(rb_speed, solver->motor_back_right_->getEncoderSpeed());

    solver->motor_front_left_->controlSetPwm(pwm_value_front_left);
    solver->motor_front_right_->controlSetPwm(pwm_value_front_right);
    solver->motor_back_left_->controlSetPwm(pwm_value_back_left);
    solver->motor_back_right_->controlSetPwm(pwm_value_back_right);
  }

  // 设置电机参数
//   static void setMotorParams(uint8_t motor_id, float speed, float acceleration) {
//     MotionSolver *solver = getInstance();
//     if (!solver->motor1_) {
//       throw std::runtime_error("Motors not initialized. Call setMotors() first.");
//     }
//     switch (motor_id) {
//     case 1:
//       // solver->motor1_->setSpeed(speed);
//       // solver->motor1_->setAcceleration(acceleration);
//       break;
//     case 2:
//       // solver->motor2_->setSpeed(speed);
//       break;
//     // ... (其他电机处理)
//     default:
//       throw std::out_of_range("Invalid motor ID (1-4).");
//     }
//   }

//   // 获取电机状态
//   static int getMotorStatus(uint8_t motor_id) {
//     MotionSolver *solver = getInstance();
//     if (!solver->motor1_) {
//       throw std::runtime_error("Motors not initialized. Call setMotors() first.");
//     }
//     switch (motor_id) {
//     case 1:
//       return 0; // 示例：返回电机1状态
//     case 2:
//       return 0; // 示例：返回电机2状态
//     // ... (其他电机处理)
//     default:
//       throw std::out_of_range("Invalid motor ID (1-4).");
//     }
//   }
};

} // namespace mini_infantry