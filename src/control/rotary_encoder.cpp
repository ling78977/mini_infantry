#include "control/rotary_encoder.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <wiringPi.h>

namespace mini_infantry {

// --- Static members for ISR dispatching ---
RotaryEncoder *RotaryEncoder::encoder_instances_[RotaryEncoder::MAX_ENCODERS] = {nullptr};

void (*RotaryEncoder::isr_A_funcs_[RotaryEncoder::MAX_ENCODERS])() = {RotaryEncoder::ISR_0A, RotaryEncoder::ISR_1A, RotaryEncoder::ISR_2A, RotaryEncoder::ISR_3A};
void (*RotaryEncoder::isr_B_funcs_[RotaryEncoder::MAX_ENCODERS])() = {RotaryEncoder::ISR_0B, RotaryEncoder::ISR_1B, RotaryEncoder::ISR_2B, RotaryEncoder::ISR_3B};

RotaryEncoder::RotaryEncoder(int encoder_pinA, int encoder_pinB) : encoder_pinA_(encoder_pinA), encoder_pinB_(encoder_pinB) {
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
    spdlog::critical("Maximum number of RotaryEncoder instances reached.");
    throw std::runtime_error("Maximum number of RotaryEncoder instances reached.");
  }

  pinMode(encoder_pinA_, INPUT);
  pinMode(encoder_pinB_, INPUT);
  pullUpDnControl(encoder_pinA_, PUD_UP);
  pullUpDnControl(encoder_pinB_, PUD_UP);
  updateEdge(); // Initialize encoder edge state

  if (wiringPiISR(encoder_pinA_, INT_EDGE_BOTH, isr_A_funcs_[instance_index_]) != 0) {
    spdlog::critical("Failed to setup ISR for pin A on RotaryEncoder");
    throw std::runtime_error("Failed to setup ISR for pin B on RotaryEncoder");
  }
  if (wiringPiISR(encoder_pinB_, INT_EDGE_BOTH, isr_B_funcs_[instance_index_]) != 0) {
    spdlog::critical("Failed to setup ISR for pin B on RotaryEncoder");
    throw std::runtime_error("Failed to setup ISR for pin B on RotaryEncoder");
  }
  spdlog::info("RotaryEncoder initialized and ISRs registered on pins {} and {} for instance {}. Configured pins: A={}, B={}",
               encoder_pinA_, encoder_pinB_, instance_index_, encoder_pinA, encoder_pinB);
}

RotaryEncoder::~RotaryEncoder() {
  if (instance_index_ != -1) {
    // Unregister ISRs
    wiringPiISR(encoder_pinA_, INT_EDGE_SETUP, nullptr);
    wiringPiISR(encoder_pinB_, INT_EDGE_SETUP, nullptr);

    // Release the instance from the pool
    encoder_instances_[instance_index_] = nullptr;
    instance_index_ = -1;
  }
}

int RotaryEncoder::getSteps() const { return encoder_steps_.load(); }
void RotaryEncoder::resetSteps() { encoder_steps_.store(0); }
void RotaryEncoder::setEncoderIsFlip(bool need_flip) { encoder_need_flip_ = need_flip; }
bool RotaryEncoder::getEncoderIsFlip() const { return encoder_need_flip_; }
int RotaryEncoder::getEncoderPinA() const { return encoder_pinA_; }
int RotaryEncoder::getEncoderPinB() const { return encoder_pinB_; }

void RotaryEncoder::pinAChanged() {
  // spdlog::info("pinAChanged triggered for encoder {}", instance_index_); // Add log for debugging
  updateEdge();
  changeState(encoder_current_edge_);
}

void RotaryEncoder::pinBChanged() {
  // spdlog::info("pinBChanged triggered for encoder {}", instance_index_); // Add log for debugging
  updateEdge();
  changeState(encoder_current_edge_);
}

void RotaryEncoder::updateEdge() {
  int aVal = digitalRead(encoder_pinA_) == 0 ? 1 : 0;
  int bVal = digitalRead(encoder_pinB_) == 0 ? 1 : 0;
  encoder_current_edge_ = (aVal << 1) | bVal;
}

void RotaryEncoder::changeState(int edgeVal) {
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

// --- ISR handler definitions ---
void RotaryEncoder::ISR_0A() {
  if (encoder_instances_[0])
    encoder_instances_[0]->pinAChanged();
}
void RotaryEncoder::ISR_0B() {
  if (encoder_instances_[0])
    encoder_instances_[0]->pinBChanged();
}
void RotaryEncoder::ISR_1A() {
  if (encoder_instances_[1])
    encoder_instances_[1]->pinAChanged();
}
void RotaryEncoder::ISR_1B() {
  if (encoder_instances_[1])
    encoder_instances_[1]->pinBChanged();
}
void RotaryEncoder::ISR_2A() {
  if (encoder_instances_[2])
    encoder_instances_[2]->pinAChanged();
}
void RotaryEncoder::ISR_2B() {
  if (encoder_instances_[2])
    encoder_instances_[2]->pinBChanged();
}
void RotaryEncoder::ISR_3A() {
  if (encoder_instances_[3])
    encoder_instances_[3]->pinAChanged();
}
void RotaryEncoder::ISR_3B() {
  if (encoder_instances_[3])
    encoder_instances_[3]->pinBChanged();
}

} // namespace mini_infantry