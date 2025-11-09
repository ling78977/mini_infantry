#pragma once
// #include "pid_controller.hpp"
// #include "rotary_encoder.hpp"
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
  Motor(boost::asio::io_context &io_ctx, int pwm_pin, int encoder_pinA, int encoder_pinB, int motor_in1, int motor_in2)
      : speed_timer_(io_ctx) {
    // Assign an instance index from the pool
    instance_index_ = -1;
    for (int i = 0; i < MAX_MOTORS; ++i) {
      if (motor_instances_[i] == nullptr) {
        instance_index_ = i;
        motor_instances_[i] = this;
        break;
      }
    }
    if (instance_index_ == -1) {
      throw std::runtime_error("Maximum number of Motor instances reached.");
    }

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
    pullUpDnControl(encoder_pinA_, PUD_UP);
    pullUpDnControl(encoder_pinB_, PUD_UP);
    pwmWrite(pwm_pin_, 0);
    speed_buffer_.resize(speed_buffer_size_, 0.0f);
  }
  void pidInit(float kp = 100.0, float ki = 100.0, float kd = 100.0, float max_iout = 99.0) {
    pid_data_.kp = kp;
    pid_data_.ki = ki;
    pid_data_.kd = kd;
    pid_data_.max_iout = max_iout;
    pid_data_.last_error = 0;
    pid_data_.last_last_error = 0;
    pid_data_.current_error = 0;
    pid_data_.i_out = 0;
    is_pid_initialized_ = true;
  }
  void pidSet(float kp, float ki, float kd) {
    pid_data_.kp = kp;
    pid_data_.ki = ki;
    pid_data_.kd = kd;

    is_pid_initialized_ = true;
  }
  void run(bool auto_calc_speed = false, int speed_calc_period_ms = 15) {
    if (!is_pid_initialized_) {
      throw std::runtime_error("PID not initialized!");
    }
    encoderUpdateEdge();

    if (wiringPiISR(encoder_pinA_, INT_EDGE_BOTH, isr_A_funcs_[instance_index_]) != 0) {
      throw std::runtime_error("Failed to setup ISR for pin A");
    }
    if (wiringPiISR(encoder_pinB_, INT_EDGE_BOTH, isr_B_funcs_[instance_index_]) != 0) {
      throw std::runtime_error("Failed to setup ISR for pin B");
    }
    std::cout << "Rotary Encoder started on pins " << encoder_pinA_ << " and " << encoder_pinB_ << std::endl;

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
    if (instance_index_ != -1) {
      // Unregister ISRs
      wiringPiISR(encoder_pinA_, INT_EDGE_SETUP, nullptr);
      wiringPiISR(encoder_pinB_, INT_EDGE_SETUP, nullptr);

      // Release the instance from the pool
      motor_instances_[instance_index_] = nullptr;
      instance_index_ = -1;
    }
    // Turn off motor
    pwmWrite(pwm_pin_, 0);
    digitalWrite(motor_in1_, LOW);
    digitalWrite(motor_in2_, LOW);
  }
  int encoderGetSteps() const { return encoder_steps_.load(); }

  int pidCalculate(float target, float current) {
    // 1. Calculate current error
    pid_data_.current_error = target - current;
    pid_data_.set_val = target;
    pid_data_.fdb_val = current;

    // 2. Calculate Proportional term
    pid_data_.p_out = pid_data_.kp * pid_data_.current_error;

    // 3. Calculate Integral term with anti-windup
    pid_data_.i_out += pid_data_.ki * pid_data_.current_error;
    // Clamp the integral term
    if (pid_data_.i_out > pid_data_.max_iout) {
      pid_data_.i_out = pid_data_.max_iout;
    } else if (pid_data_.i_out < -pid_data_.max_iout) {
      pid_data_.i_out = -pid_data_.max_iout;
    }

    // 4. Calculate Derivative term
    pid_data_.d_out = pid_data_.kd * (pid_data_.current_error - pid_data_.last_error);

    // 5. Calculate total output
    pid_data_.out = static_cast<int>(pid_data_.p_out + pid_data_.i_out + pid_data_.d_out);

    // 6. Update error for next iteration
    pid_data_.last_error = pid_data_.current_error;

    // 7. Apply output limits
    pidLimitMax();

    // --- Original User Code ---
    // pid_data_.last_last_error = pid_data_.last_error;
    // pid_data_.last_error = pid_data_.current_error;

    // pid_data_.set_val = target;
    // pid_data_.fdb_val = current;
    // pid_data_.current_error = target - current;

    // pid_data_.p_out = pid_data_.kp * (pid_data_.current_error - pid_data_.last_error);
    // pid_data_.i_out = pid_data_.ki * pid_data_.current_error;
    // pid_data_.d_out = pid_data_.kd * (pid_data_.current_error - 2 * pid_data_.last_error + pid_data_.last_last_error);
    // pid_data_.out = pid_data_.p_out + pid_data_.i_out + pid_data_.d_out;

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
    // std::cout << "time_diff_ms: " << time_diff_ms << ", this_steps: " << this_steps << std::endl;
    last_update_time_ms_ = this_time_ms;
    float encode_number_of_turns;
    encode_number_of_turns = this_steps / (float)(SINGLE_TURN_PULSE * REDUCTION_RATIO);

    auto encode_distance = encode_number_of_turns * WHEEL_CIRCUMFERENCE;

    auto speed = std::isnan(encode_distance / time_diff_ms * 1000.0f) ? 0.0f : (encode_distance / time_diff_ms * 1000.0f);
    // std::cout << "encode_number_of_turns: " << encode_number_of_turns << ", encode_distance: " << encode_distance << "this speed: " <<
    // speed<<", this steps: "<<this_steps
    // << std::endl;
    if (encoder_need_flip_) {
      speed = -speed;
    }
    // Sliding window average filter
    speed_buffer_[speed_buffer_index_] = speed;
    speed_buffer_index_ = (speed_buffer_index_ + 1) % speed_buffer_size_;

    float sum = std::accumulate(speed_buffer_.begin(), speed_buffer_.end(), 0.0f);
    float filtered_speed = sum / speed_buffer_size_;

    encoder_speed_.store(filtered_speed);
    // return encoder_speed_;
  }
  float getEncoderSpeed() const { return encoder_speed_.load(); }

  void controlSetForwoardIsFlip(bool need_flip) { forward_need_flip_ = need_flip; }

  void encoderSetEncoderIsFlip(bool need_flip) { encoder_need_flip_ = need_flip; }


  void stopSpeedCalculation() { speed_timer_.cancel(); }

private:
  void speedUpdateHandler(const boost::system::error_code &ec, int period_ms) {
    if (ec) {
      if (ec == boost::asio::error::operation_aborted)
        return; // Timer was cancelled, normal exit
      std::cerr << "Speed timer error: " << ec.message() << std::endl;
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
      // std::cout << "Step Plus Detected" << std::endl;
      encoder_steps_++;
      //   std::cout << "Step Plus Detected" << std::endl;
    } else if (newStateInt == StepMinus) {
      // 逆时针步进
      // std::cout << "Step Minus Detected" << std::endl;
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
  void encoderAChanged() {
    // Original state machine logic
    encoderUpdateEdge();
    encoderChangeState(encoder_current_edge_);
  }

  void encoderBChanged() {
    // Original state machine logic
    encoderUpdateEdge();
    encoderChangeState(encoder_current_edge_);
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

  // For speed smoothing
  const size_t speed_buffer_size_ = 3; // Average over 3 samples
  std::vector<float> speed_buffer_;
  size_t speed_buffer_index_ = 0;

  int pwm_pin_ = -1;
  int encoder_pinA_ = -1;
  int encoder_pinB_ = -1;
  int motor_in1_ = -1;
  int motor_in2_ = -1;
  int instance_index_ = -1;

  PidData pid_data_{};
  bool is_pid_initialized_ = false;

  boost::asio::steady_timer speed_timer_;

  // --- Static members for ISR dispatching ---
  static const int MAX_MOTORS = 4;
  inline static Motor *motor_instances_[MAX_MOTORS] = {nullptr};

  // ISR handler declarations
  static void ISR_0A();
  static void ISR_0B();
  static void ISR_1A();
  static void ISR_1B();
  static void ISR_2A();
  static void ISR_2B();
  static void ISR_3A();
  static void ISR_3B();

  inline static void (*isr_A_funcs_[MAX_MOTORS])() = {ISR_0A, ISR_1A, ISR_2A, ISR_3A};
  inline static void (*isr_B_funcs_[MAX_MOTORS])() = {ISR_0B, ISR_1B, ISR_2B, ISR_3B};
};

// --- ISR handler definitions ---
inline void Motor::ISR_0A() {
  if (motor_instances_[0])
    motor_instances_[0]->encoderAChanged();
}
inline void Motor::ISR_0B() {
  if (motor_instances_[0])
    motor_instances_[0]->encoderBChanged();
}
inline void Motor::ISR_1A() {
  if (motor_instances_[1])
    motor_instances_[1]->encoderAChanged();
}
inline void Motor::ISR_1B() {
  if (motor_instances_[1])
    motor_instances_[1]->encoderBChanged();
}
inline void Motor::ISR_2A() {
  if (motor_instances_[2])
    motor_instances_[2]->encoderAChanged();
}
inline void Motor::ISR_2B() {
  if (motor_instances_[2])
    motor_instances_[2]->encoderBChanged();
}
inline void Motor::ISR_3A() {
  if (motor_instances_[3])
    motor_instances_[3]->encoderAChanged();
}
inline void Motor::ISR_3B() {
  if (motor_instances_[3])
    motor_instances_[3]->encoderBChanged();
}

} // namespace mini_infantry