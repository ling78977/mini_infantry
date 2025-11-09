#pragma once

#include <atomic>
#include <cmath>
#include <wiringPi.h>
#include <iostream> // For LOG_INFO/ERROR

// 简单的日志宏，方便后续替换为更复杂的日志系统
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl

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

class RotaryEncoder {
public:
  RotaryEncoder(int encoder_pinA, int encoder_pinB)
      : encoder_pinA_(encoder_pinA), encoder_pinB_(encoder_pinB) {
    // Assign an instance index from the pool
    instance_index_ = -1;
    for (int i = 0; i < MAX_ENCODERS; ++i) {
      if (encoder_instances_[i] == nullptr) {
        instance_index_ = i;
        encoder_instances_[i] = this;
        break;
      }
    }
    if (instance_index_ == -1) {
      LOG_ERROR("Maximum number of RotaryEncoder instances reached.");
      throw std::runtime_error("Maximum number of RotaryEncoder instances reached.");
    }

    pinMode(encoder_pinA_, INPUT);
    pinMode(encoder_pinB_, INPUT);
    pullUpDnControl(encoder_pinA_, PUD_UP);
    pullUpDnControl(encoder_pinB_, PUD_UP);
    updateEdge(); // Initialize encoder edge state

    if (wiringPiISR(encoder_pinA_, INT_EDGE_BOTH, isr_A_funcs_[instance_index_]) != 0) {
      throw std::runtime_error("Failed to setup ISR for pin A on RotaryEncoder");
    }
    if (wiringPiISR(encoder_pinB_, INT_EDGE_BOTH, isr_B_funcs_[instance_index_]) != 0) {
      throw std::runtime_error("Failed to setup ISR for pin B on RotaryEncoder");
    }
    LOG_INFO("RotaryEncoder initialized and ISRs registered on pins " << encoder_pinA_ << " and " << encoder_pinB_ << " for instance " << instance_index_ << ". Configured pins: A=" << encoder_pinA << ", B=" << encoder_pinB);
  }

  ~RotaryEncoder() {
    if (instance_index_ != -1) {
      // Unregister ISRs
      wiringPiISR(encoder_pinA_, INT_EDGE_SETUP, nullptr);
      wiringPiISR(encoder_pinB_, INT_EDGE_SETUP, nullptr);

      // Release the instance from the pool
      encoder_instances_[instance_index_] = nullptr;
      instance_index_ = -1;
    }
  }

  int getSteps() const { return encoder_steps_.load(); }
  void resetSteps() { encoder_steps_.store(0); }
  void setEncoderIsFlip(bool need_flip) { encoder_need_flip_ = need_flip; }
  bool getEncoderIsFlip() const { return encoder_need_flip_; }
  int getEncoderPinA() const { return encoder_pinA_; } // Added getter
  int getEncoderPinB() const { return encoder_pinB_; } // Added getter

  // ISR handlers to be called by external dispatchers
  void pinAChanged() {
    // LOG_INFO("pinAChanged triggered for encoder " << instance_index_); // Add log for debugging
    updateEdge();
    changeState(encoder_current_edge_);
  }

  void pinBChanged() {
    // LOG_INFO("pinBChanged triggered for encoder " << instance_index_); // Add log for debugging
    updateEdge();
    changeState(encoder_current_edge_);
  }

private:
  void updateEdge() {
    int aVal = digitalRead(encoder_pinA_) == 0 ? 1 : 0;
    int bVal = digitalRead(encoder_pinB_) == 0 ? 1 : 0;
    encoder_current_edge_ = (aVal << 1) | bVal;
  }

  void changeState(int edgeVal) {
    State state = encoder_current_state_.load();
    int newStateInt = transitions_[state][edgeVal];
    if (newStateInt == StepPlus) {
      encoder_steps_++;
    } else if (newStateInt == StepMinus) {
      encoder_steps_--;
    } else {
      encoder_current_state_ = static_cast<State>(newStateInt);
      return;
    }
    encoder_current_state_ = Idle;
  }
  
  int encoder_pinA_;
  int encoder_pinB_;
  int instance_index_ = -1; // Instance index for ISR dispatching

  // 编码器核心状态变量
  std::atomic<int16_t> encoder_steps_{0};
  std::atomic<State> encoder_current_state_{Idle};
  std::atomic<int> encoder_current_edge_{0};
  bool encoder_need_flip_ = false;

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
  // --- Static members for ISR dispatching ---
  static const int MAX_ENCODERS = 4; // Max number of encoder instances
  inline static RotaryEncoder *encoder_instances_[MAX_ENCODERS] = {nullptr};

  // ISR handler declarations
  static void ISR_0A();
  static void ISR_0B();
  static void ISR_1A();
  static void ISR_1B();
  static void ISR_2A();
  static void ISR_2B();
  static void ISR_3A();
  static void ISR_3B();

  inline static void (*isr_A_funcs_[MAX_ENCODERS])() = {ISR_0A, ISR_1A, ISR_2A, ISR_3A};
  inline static void (*isr_B_funcs_[MAX_ENCODERS])() = {ISR_0B, ISR_1B, ISR_2B, ISR_3B};
};

// --- ISR handler definitions ---
inline void RotaryEncoder::ISR_0A() {
  if (encoder_instances_[0])
    encoder_instances_[0]->pinAChanged();
}
inline void RotaryEncoder::ISR_0B() {
  if (encoder_instances_[0])
    encoder_instances_[0]->pinBChanged();
}
inline void RotaryEncoder::ISR_1A() {
  if (encoder_instances_[1])
    encoder_instances_[1]->pinAChanged();
}
inline void RotaryEncoder::ISR_1B() {
  if (encoder_instances_[1])
    encoder_instances_[1]->pinBChanged();
}
inline void RotaryEncoder::ISR_2A() {
  if (encoder_instances_[2])
    encoder_instances_[2]->pinAChanged();
}
inline void RotaryEncoder::ISR_2B() {
  if (encoder_instances_[2])
    encoder_instances_[2]->pinBChanged();
}
inline void RotaryEncoder::ISR_3A() {
  if (encoder_instances_[3])
    encoder_instances_[3]->pinAChanged();
}
inline void RotaryEncoder::ISR_3B() {
  if (encoder_instances_[3])
    encoder_instances_[3]->pinBChanged();
}

} // namespace mini_infantry