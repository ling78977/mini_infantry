#include "control/motor.hpp"
#include "util/periodic_timer.hpp"
#include "pid_controller.hpp"
#include "rotary_encoder.hpp"
#include <atomic>
#include <boost/asio.hpp>
#include <cmath>
#include <functional>
#include <memory> // For std::unique_ptr
#include <numeric>
#include <vector>
#include <wiringPi.h>
#include <spdlog/spdlog.h>
#include <chrono>

namespace mini_infantry {

// 获取当前时间的毫秒数（自系统启动或 epoch 以来）
long long get_current_ms() {
  return ::std::chrono::duration_cast<::std::chrono::milliseconds>(::std::chrono::steady_clock::now().time_since_epoch()).count();
}

Motor::Motor(boost::asio::io_context &io_ctx, int pwm_pin, int encoder_pinA, int encoder_pinB, int motor_in1, int motor_in2)
    : io_ctx_(io_ctx), pid_controller_(), rotary_encoder_(encoder_pinA, encoder_pinB) { // 初始化 PidController 和 RotaryEncoder
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

  spdlog::info("Motor initialized with pwm_pin: {}, encoder_pinA: {}, encoder_pinB: {}", pwm_pin_, encoder_pinA, encoder_pinB);
}

void Motor::pidInit(double kp, double ki, double kd, double max_iout) {
  pid_controller_.pidInit(kp, ki, kd, max_iout);
}

void Motor::pidSet(double kp, double ki, double kd) { pid_controller_.pidSet(kp, ki, kd); }

void Motor::runAutoCalcSpeed(int speed_calc_period_ms) {

  speed_timer_ = std::make_unique<PeriodicTimer>(io_ctx_, std::bind(&Motor::encoderUpdateSpeed, this),
                                                 ::std::chrono::milliseconds(speed_calc_period_ms));
  speed_timer_->start();
}

void Motor::controlSetPwm(int pwm_value) {
  pwm_value > 0 ? controlForwardRotate() : controlBackwardRotate();

  pwmWrite(pwm_pin_, abs(pwm_value) > 99 ? 99 : abs(pwm_value));
}

Motor::~Motor() {
  // Turn off motor
  pwmWrite(pwm_pin_, 0);
  digitalWrite(motor_in1_, LOW);
  digitalWrite(motor_in2_, LOW);
}

int Motor::encoderGetSteps() const { return rotary_encoder_.getSteps(); }

int Motor::pidCalculate(double target, double current) { return pid_controller_.pidCalculate(target, current); }

// encoderUpdateSpeed() is now the callback for PeriodicTimer, no longer takes boost::system::error_code
void Motor::encoderUpdateSpeed() {
  static bool is_first_update_ = true;
  if (is_first_update_) {
    is_first_update_ = false;
    last_update_time_ms_ = get_current_ms();
  }
  auto this_steps = rotary_encoder_.getSteps(); // Get steps from RotaryEncoder
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
  
}
float Motor::getEncoderSpeed() const { return encoder_speed_.load(); }

void Motor::controlSetForwoardIsFlip(bool need_flip) { forward_need_flip_ = need_flip; }

void Motor::encoderSetEncoderIsFlip(bool need_flip) { rotary_encoder_.setEncoderIsFlip(need_flip); } // Delegate to RotaryEncoder

void Motor::stopSpeedCalculation() {
  if (speed_timer_) {
    speed_timer_->stop();
  }
}

void Motor::controlForwardRotate() {
  if (forward_need_flip_) {
    digitalWrite(motor_in1_, LOW);
    digitalWrite(motor_in2_, HIGH);
  } else {
    digitalWrite(motor_in1_, HIGH);
    digitalWrite(motor_in2_, LOW);
  }
}
void Motor::controlBackwardRotate() {
  if (forward_need_flip_) {
    digitalWrite(motor_in1_, HIGH);
    digitalWrite(motor_in2_, LOW);
  } else {
    digitalWrite(motor_in1_, LOW);
    digitalWrite(motor_in2_, HIGH);
  }
}

} // namespace mini_infantry