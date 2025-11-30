#pragma once
#include "util/periodic_timer.hpp" // 引入 PeriodicTimer 类
#include "pid_controller.hpp" // 引入 PidController 类
#include "rotary_encoder.hpp" // 引入 RotaryEncoder 类
#include <atomic>
#include <boost/asio.hpp>
#include <cmath>
#include <functional>
#include <memory> // For std::unique_ptr
#include <numeric>
#include <vector>
#include <wiringPi.h>
#include <spdlog/spdlog.h>


#define PI 3.14159265358979323846f
#define DEG2RAD(x) ((x) * PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / PI)
#define SINGLE_TURN_PULSE 11
#define REDUCTION_RATIO 30
#define WHEEL_DIAMETER 0.08f

#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)

namespace mini_infantry {
// 获取当前时间的毫秒数（自系统启动或 epoch 以来）
long long get_current_ms();

class Motor {
public:
  Motor(boost::asio::io_context &io_ctx, int pwm_pin, int encoder_pinA, int encoder_pinB, int motor_in1, int motor_in2);
  void pidInit(double kp = 100.0, double ki = 100.0, double kd = 100.0, double max_iout = 99.0);
  void pidSet(double kp, double ki, double kd);
  void runAutoCalcSpeed(int speed_calc_period_ms = 15);
  void controlSetPwm(int pwm_value);
  ~Motor();
  int encoderGetSteps() const;
  int pidCalculate(double target, double current);
  void encoderUpdateSpeed();
  float getEncoderSpeed() const;
  void controlSetForwoardIsFlip(bool need_flip);
  void encoderSetEncoderIsFlip(bool need_flip);
  void stopSpeedCalculation();

private:
  void controlForwardRotate();
  void controlBackwardRotate();

  bool forward_need_flip_ = false;

  std::atomic<float> encoder_speed_{0.0f};
  long long last_update_time_ms_ = 0; // ms

  // For speed smoothing
  const size_t speed_buffer_size_ = 3; // Window size for weighted average
  std::vector<float> speed_buffer_;
  size_t speed_buffer_index_ = 0;

  int pwm_pin_ = -1;
  int motor_in1_ = -1;
  int motor_in2_ = -1;

  boost::asio::io_context &io_ctx_; // Reference to the io_context

  PidController pid_controller_; // PidController 实例
  RotaryEncoder rotary_encoder_; // RotaryEncoder 实例

  std::unique_ptr<PeriodicTimer> speed_timer_; // Using the new PeriodicTimer
};

} // namespace mini_infantry