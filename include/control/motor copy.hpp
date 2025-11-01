#pragma once
// #include "pid_controller.hpp"
// #include "rotary_encoder.hpp"
#include <atomic>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <wiringPi.h>
#define PWM_PIN_WIR_1 1
#define PWM_PIN_WIR_26 26
#define PWM_PIN_WIR_23 23
#define PWM_PIN_WIR_24 24

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
enum State { Idle = 0, Ccw1 = 1, Ccw2 = 2, Ccw3 = 3, Cw1 = 4, Cw2 = 5, Cw3 = 6 };

// 转换结果枚举
enum TransitionResult {
  StateIdle = 0,
  StateCcw1 = 1,
  StateCcw2 = 2,
  StateCcw3 = 3,
  StateCw1 = 4,
  StateCw2 = 5,
  StateCw3 = 6,
  StepPlus = 7,
  StepMinus = 8
};
struct PidData {
  float kp;
  float ki;
  float kd;

  float max_out = 99;
  float max_iout;

  float set_val;
  float fdb_val;

  int out; // pwm -100~100
  float p_out;
  float i_out;
  float d_out;
  float last_error;
  float last_last_error;
  float current_error;
};
// 获取当前时间的毫秒数（自系统启动或 epoch 以来）
inline long long get_current_ms() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);              // 使用单调时钟，避免系统时间被修改影响
  return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000; // 秒转毫秒 + 纳秒转毫秒
}
class Motor {
public:
  Motor(int pwm_pin, int encoder_pinA, int encoder_pinB, int motor_in1, int motor_in2) {
    pwm_pin_ = pwm_pin;
    encoder_pinA_ = encoder_pinA;
    encoder_pinB_ = encoder_pinB;
    motor_in1_ = motor_in1;
    motor_in2_ = motor_in2;
    if (!isGpioInitialized_) {
      if (wiringPiSetup() == -1) {
        throw std::runtime_error("Failed to initialize wiringPi");
      } else {
        isGpioInitialized_ = true;
        std::cout << "wiringPi initialized successfully" << std::endl;
      }
    } else {
      std::cout << "wiringPi already initialized" << std::endl;
    }
    pinMode(motor_in1, OUTPUT);
    pinMode(motor_in2, OUTPUT);
    pinMode(encoder_pinA_, INPUT);
    pinMode(encoder_pinB_, INPUT);

    pinMode(pwm_pin_, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(100);
    pwmSetClock(13);
    digitalWrite(motor_in1, HIGH);
    digitalWrite(motor_in2, HIGH);
    // digitalWrite(motor_in2, LOW);
    // 启用上拉电阻
    pullUpDnControl(encoder_pinA_, PUD_UP);
    pullUpDnControl(encoder_pinB_, PUD_UP);
    pwmWrite(pwm_pin_, 0);
  }
  void pidInit(float kp = 100.0, float ki = 100.0, float kd = 100.0, float max_iout = 99999.0) {
    pid_data_.kp = kp;
    pid_data_.ki = ki;
    pid_data_.kd = kd;
    pid_data_.max_iout = max_iout;
    pid_data_.last_error = 0;
    pid_data_.last_last_error = 0;
    pid_data_.current_error = 0;
    is_pid_initialized_ = true;
  }
  void pidSet(float kp, float ki, float kd) {
    pid_data_.p_out = kp;
    pid_data_.i_out = ki;
    pid_data_.d_out = kd;
    is_pid_initialized_ = true;
  }
  void run() {
    if (!is_pid_initialized_) {
      throw std::runtime_error("PID not initialized!");
    }
    encoderUpdateEdge();
    std::cout << "Rotary Encoder configured on pins " << encoder_pinA_ << " and " << encoder_pinB_ << std::endl;
  }
  void controlSetPwm(int pwm_value) {
    pwm_value > 0 ? controlForwardRotate() : controlBackwardRotate();

    pwmWrite(pwm_pin_, abs(pwm_value) > 99 ? 99 : abs(pwm_value));
  }

  ~Motor() {
    // 关闭PWM
    pwmWrite(pwm_pin_, 0);
    // 关闭电机
    digitalWrite(motor_in1_, LOW);
    digitalWrite(motor_in2_, LOW);
  }
  int getSteps() const { return encoder_steps_.load(); }

  int pidCalculate(float target, float current) {

    pid_data_.last_last_error = pid_data_.last_error;
    pid_data_.last_error = pid_data_.current_error;
    // pid_data_.current_error   = target - current;

    pid_data_.set_val = target;
    pid_data_.fdb_val = current;
    pid_data_.current_error = target - current;

    pid_data_.p_out = pid_data_.kp * (pid_data_.current_error - pid_data_.last_error);
    pid_data_.i_out = pid_data_.ki * pid_data_.current_error;
    pid_data_.d_out = pid_data_.kd * (pid_data_.current_error - 2 * pid_data_.last_error + pid_data_.last_last_error);
    pid_data_.out = pid_data_.p_out + pid_data_.i_out + pid_data_.d_out;

    pidLimitMax();
    return pid_data_.out;
  }

  void encoderUpdateSpeed() {
    static bool is_first_update_ = true;
    if (is_first_update_) {
      is_first_update_ = false;
      last_update_time_ms_ = get_current_ms();
      //   return 0.0f;
    }
    auto this_steps = encoder_steps_.load();
    encoder_steps_.store(0);
    auto this_time_ms = get_current_ms();
    auto time_diff_ms = this_time_ms - last_update_time_ms_;
    last_update_time_ms_ = this_time_ms;
    auto encode_number_of_turns = this_steps / (float)(SINGLE_TURN_PULSE * REDUCTION_RATIO);
    auto encode_distance = encode_number_of_turns * WHEEL_CIRCUMFERENCE;
    auto speed = std::isnan(encode_distance / time_diff_ms / 1000.0f) ? 0.0f : (encode_distance / time_diff_ms / 1000.0f);
    if (encoder_need_flip_) {
      speed = -speed;
    }
    encoder_speed_.store(speed);
    // return encoder_speed_;
  }
  float getEncoderSpeed() const { return encoder_speed_.load(); }

  void controlSetForwoardIsFlip(bool need_flip) { forward_need_flip_ = need_flip; }

  void encoderSetEncoderIsFlip(bool need_flip) { encoder_need_flip_ = need_flip; }

public:
  void encoderAChanged() {
    encoderUpdateEdge();
    encoderChangeState(encoder_current_edge_);
  }

  void encoderBChanged() {
    encoderUpdateEdge();
    encoderChangeState(encoder_current_edge_);
  }

private:
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

  void encoderUpdateEdge() {
    int aVal = digitalRead(encoder_pinA_) == 0 ? 1 : 0;
    int bVal = digitalRead(encoder_pinB_) == 0 ? 1 : 0;
    encoder_current_edge_ = (aVal << 1) | bVal;
  }
  void encoderChangeState(int edgeVal) {
    State state = encoder_current_state_.load();
    int newStateInt = transitions_[state][edgeVal];
    if (newStateInt == StepPlus) {
      // 顺时针步进
      encoder_steps_++;
      //   std::cout << "Step Plus Detected" << std::endl;
    } else if (newStateInt == StepMinus) {
      // 逆时针步进
      encoder_steps_--;
      //   std::cout << "Step Minus Detected" << std::endl;
    } else {
      // 普通状态转换
      encoder_current_state_ = static_cast<State>(newStateInt);
      return;
    }
    // 重置状态
    encoder_current_state_ = Idle;
  }

  void pidLimitMax() {
    pid_data_.out = pid_data_.out > pid_data_.max_out ? pid_data_.max_out : pid_data_.out;
    pid_data_.out = pid_data_.out < -pid_data_.max_out ? -pid_data_.max_out : pid_data_.out;
  }

  // 状态转换表
  static constexpr int transitions_[7][4] = {
      // 00, 01, 10, 11
      {StateIdle, StateCcw1, StateCw1, StateIdle},  // Idle:   idle, ccw1, cw1, idle
      {StateIdle, StateCcw1, StateCcw3, StateCcw2}, // Ccw1:   idle, ccw1, ccw3, ccw2
      {StateIdle, StateCcw1, StateCcw3, StateCcw2}, // Ccw2:   idle, ccw1, ccw3, ccw2
      {StepMinus, StateIdle, StateCcw3, StateCcw2}, // Ccw3:   -1, idle, ccw3, ccw2
      {StateIdle, StateCw3, StateCw1, StateCw2},    // Cw1:    idle, cw3, cw1, cw2
      {StateIdle, StateCw3, StateCw1, StateCw2},    // Cw2:    idle, cw3, cw1, cw2
      {StepPlus, StateCw3, StateIdle, StateCw2}     // Cw3:    +1, cw3, idle, cw2
  };

  bool forward_need_flip_ = false;

  inline static bool isGpioInitialized_ = false;

  // 编码器核心状态变量
  std::atomic<int16_t> encoder_steps_{0};
  std::atomic<State> encoder_current_state_{Idle};
  std::atomic<int> encoder_current_edge_{0};
  std::atomic<float> encoder_speed_{0.0f};
  long long last_update_time_ms_ = 0; // ms
  bool encoder_need_flip_ = false;

  int pwm_pin_ = -1;
  int encoder_pinA_ = -1;
  int encoder_pinB_ = -1;
  int motor_in1_ = -1;
  int motor_in2_ = -1;

  PidData pid_data_{};
  bool is_pid_initialized_ = false;
};

} // namespace mini_infantry