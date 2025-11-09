#pragma once
#include "pid_controller.hpp" // 引入 PidController 类
#include "rotary_encoder.hpp" // 引入 RotaryEncoder 类
#include <atomic>
#include <boost/asio.hpp>
#include <cmath>
#include <functional>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <wiringPi.h>

// 简单的日志宏，方便后续替换为更复杂的日志系统
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#define PWM_PIN_WIR_1 1
#define PWM_PIN_WIR_23 23
#define PWM_PIN_WIR_24 24
#define PWM_PIN_WIR_26 26


#define PWM_PIN_BCM_12 12
#define PWM_PIN_BCM_13 13
#define PWM_PIN_BCM_18 18
#define PWM_PIN_BCM_19 19

#define PI 3.14159265358979323846f
#define DEG2RAD(x) ((x) * PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / PI)
#define SINGLE_TURN_PULSE 11
#define REDUCTION_RATIO 30
#define WHEEL_DIAMETER 0.08f

#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)

namespace mini_infantry {
// 获取当前时间的毫秒数（自系统启动或 epoch 以来）
inline long long get_current_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
class Motor {
public:
  Motor(boost::asio::io_context &io_ctx, int pwm_pin, int encoder_pinA, int encoder_pinB, int motor_in1, int motor_in2)
      : speed_timer_(io_ctx), pid_controller_(), rotary_encoder_(encoder_pinA, encoder_pinB) { // 初始化 PidController 和 RotaryEncoder
    pwm_pin_ = pwm_pin;
    motor_in1_ = motor_in1;
    motor_in2_ = motor_in2;

    pinMode(motor_in1, OUTPUT);
    pinMode(motor_in2, OUTPUT);

    pinMode(pwm_pin_, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(100);
    pwmSetClock(13);
    // Initialize motor direction pins to LOW to ensure motor is off
    digitalWrite(motor_in1, LOW);
    digitalWrite(motor_in2, LOW);
    pwmWrite(pwm_pin_, 0);
    speed_buffer_.resize(speed_buffer_size_, 0.0f);

    LOG_INFO("Motor initialized with pwm_pin: " << pwm_pin_ << ", encoder_pinA: " << encoder_pinA << ", encoder_pinB: " << encoder_pinB);
  }
  void pidInit(double kp = 100.0, double ki = 100.0, double kd = 100.0, double max_iout = 99.0) {
    pid_controller_.pidInit(kp, ki, kd, max_iout);
  }
  void pidSet(double kp, double ki, double kd) {
    pid_controller_.pidSet(kp, ki, kd);
  }
  void run(bool auto_calc_speed = false, int speed_calc_period_ms = 15) {
    if (auto_calc_speed) {
      speed_timer_.expires_after(std::chrono::milliseconds(speed_calc_period_ms));
      speed_timer_.async_wait(std::bind(&Motor::speedUpdateHandler, this, std::placeholders::_1, speed_calc_period_ms));
    }
  }
  void controlSetPwm(int pwm_value) {
    pwm_value > 0 ? controlForwardRotate() : controlBackwardRotate();

    pwmWrite(pwm_pin_, abs(pwm_value) > 99 ? 99 : abs(pwm_value));
  }

  ~Motor() {
    // Turn off motor
    pwmWrite(pwm_pin_, 0);
    digitalWrite(motor_in1_, LOW);
    digitalWrite(motor_in2_, LOW);
  }
  int encoderGetSteps() const { return rotary_encoder_.getSteps(); }

  int pidCalculate(double target, double current) {
    return pid_controller_.pidCalculate(target, current);
  }

  void encoderUpdateSpeed() {
    static bool is_first_update_ = true;
    if (is_first_update_) {
      is_first_update_ = false;
      last_update_time_ms_ = get_current_ms();
    }
    auto this_steps = rotary_encoder_.getSteps(); // Get steps from RotaryEncoder
    // LOG_INFO("Encoder steps: " << this_steps); // Add log for debugging
    rotary_encoder_.resetSteps(); // Reset steps in RotaryEncoder
    auto this_time_ms = get_current_ms();
    auto time_diff_ms = this_time_ms - last_update_time_ms_;
    last_update_time_ms_ = this_time_ms;
    float encode_number_of_turns;
    encode_number_of_turns = this_steps / (float)(SINGLE_TURN_PULSE * REDUCTION_RATIO);

    auto encode_distance = encode_number_of_turns * WHEEL_CIRCUMFERENCE;

    auto speed = std::isnan(encode_distance / time_diff_ms * 1000.0f) ? 0.0f : (encode_distance / time_diff_ms * 1000.0f);
    if (rotary_encoder_.getEncoderIsFlip()) { // Use RotaryEncoder's flip state
      speed = -speed;
    }
    // Sliding window average filter
    speed_buffer_[speed_buffer_index_] = speed;
    speed_buffer_index_ = (speed_buffer_index_ + 1) % speed_buffer_size_;

    // Weighted average filter
    float filtered_speed = 0.0f;
    if (speed_buffer_size_ < 3) {
        float sum = std::accumulate(speed_buffer_.begin(), speed_buffer_.end(), 0.0f);
        filtered_speed = sum / speed_buffer_size_;
    } else {
        float latest_speed = speed_buffer_[(speed_buffer_index_ - 1 + speed_buffer_size_) % speed_buffer_size_];
        float prev_speed = speed_buffer_[(speed_buffer_index_ - 2 + speed_buffer_size_) % speed_buffer_size_];
        float prev_prev_speed = speed_buffer_[(speed_buffer_index_ - 3 + speed_buffer_size_) % speed_buffer_size_]; // Corrected index
        filtered_speed = latest_speed * 0.5f + prev_speed * 0.3f + prev_prev_speed * 0.2f;
    }

    encoder_speed_.store(filtered_speed);
    // std::cout << "Encoder speed: " << filtered_speed << std::endl;
  }
  float getEncoderSpeed() const { return encoder_speed_.load(); }

  void controlSetForwoardIsFlip(bool need_flip) { forward_need_flip_ = need_flip; }

  void encoderSetEncoderIsFlip(bool need_flip) { rotary_encoder_.setEncoderIsFlip(need_flip); } // Delegate to RotaryEncoder


  void stopSpeedCalculation() { speed_timer_.cancel(); }

private:
  void speedUpdateHandler(const boost::system::error_code &ec, int period_ms) {
    if (ec) {
      if (ec == boost::asio::error::operation_aborted)
        return; // Timer was cancelled, normal exit
      LOG_ERROR("Speed timer error: " << ec.message());
      return;
    }

    encoderUpdateSpeed();

    // Reschedule the timer
    speed_timer_.expires_at(speed_timer_.expiry() + std::chrono::milliseconds(period_ms));
    speed_timer_.async_wait(std::bind(&Motor::speedUpdateHandler, this, std::placeholders::_1, period_ms));
  }

  void controlForwardRotate() {
    if (forward_need_flip_) {
      digitalWrite(motor_in1_, LOW);
      digitalWrite(motor_in2_, HIGH);
    } else {
      digitalWrite(motor_in1_, HIGH);
      digitalWrite(motor_in2_, LOW);
    }
  }
  void controlBackwardRotate() {
    if (forward_need_flip_) {
      digitalWrite(motor_in1_, HIGH);
      digitalWrite(motor_in2_, LOW);
    } else {
      digitalWrite(motor_in1_, LOW);
      digitalWrite(motor_in2_, HIGH);
    }
  }

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

  PidController pid_controller_; // PidController 实例
  RotaryEncoder rotary_encoder_; // RotaryEncoder 实例

  boost::asio::steady_timer speed_timer_;
};

} // namespace mini_infantry