#pragma once

#include <atomic>
#include <cmath>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <wiringPi.h>

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
  RotaryEncoder(int encoder_pinA, int encoder_pinB);
  ~RotaryEncoder();

  int getSteps() const;
  void resetSteps();
  void setEncoderIsFlip(bool need_flip);
  bool getEncoderIsFlip() const;
  int getEncoderPinA() const;
  int getEncoderPinB() const;

  void pinAChanged();
  void pinBChanged();

private:
  void updateEdge();
  void changeState(int edgeVal);

  int encoder_pinA_;
  int encoder_pinB_;
  int instance_index_ = -1;

  std::atomic<int16_t> encoder_steps_{0};
  std::atomic<State> encoder_current_state_{Idle};
  std::atomic<int> encoder_current_edge_{0};
  bool encoder_need_flip_ = false;

  static constexpr int transitions_[7][4] = {
      {StateIdle, StateCcw1, StateCw1, StateIdle},
      {StateIdle, StateCcw1, StateCcw3, StateCcw2},
      {StateIdle, StateCcw1, StateCcw3, StateCcw2},
      {StepMinus, StateIdle, StateCcw3, StateCcw2},
      {StateIdle, StateCw3, StateCw1, StateCw2},
      {StateIdle, StateCw3, StateCw1, StateCw2},
      {StepPlus, StateCw3, StateIdle, StateCw2}};

  static const int MAX_ENCODERS = 4;
  static RotaryEncoder *encoder_instances_[MAX_ENCODERS];

  static void ISR_0A();
  static void ISR_0B();
  static void ISR_1A();
  static void ISR_1B();
  static void ISR_2A();
  static void ISR_2B();
  static void ISR_3A();
  static void ISR_3B();

  static void (*isr_A_funcs_[MAX_ENCODERS])();
  static void (*isr_B_funcs_[MAX_ENCODERS])();
}; // Don't forget the semicolon after class definition

} // namespace mini_infantry