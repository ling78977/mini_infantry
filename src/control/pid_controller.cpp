#include "control/pid_controller.hpp"
#include <spdlog/spdlog.h>
#include <algorithm> // For std::clamp

namespace mini_infantry {

PidController::PidController() {
  pid_data_.kp = 0.0;
  pid_data_.ki = 0.0;
  pid_data_.kd = 0.0;
  pid_data_.max_iout = 0.0; // 默认值，需要通过pidInit或pidSet设置
  reset();
  spdlog::info("PidController initialized.");
}

void PidController::pidInit(double kp, double ki, double kd, double max_iout) {
  pid_data_.kp = kp;
  pid_data_.ki = ki;
  pid_data_.kd = kd;
  pid_data_.max_iout = max_iout;
  reset();
  spdlog::info("PidController parameters initialized: Kp={}, Ki={}, Kd={}, MaxIOut={}", kp, ki, kd, max_iout);
}

void PidController::pidSet(double kp, double ki, double kd) {
  pid_data_.kp = kp;
  pid_data_.ki = ki;
  pid_data_.kd = kd;
  // LOG_INFO("PidController parameters set: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd);
}

int PidController::pidCalculate(double set_val, double fdb_val) {
  pid_data_.current_error = set_val - fdb_val;

  pid_data_.p_out = pid_data_.kp * pid_data_.current_error;

  pid_data_.i_out += pid_data_.ki * pid_data_.current_error;
  pid_data_.i_out = std::clamp(pid_data_.i_out, -pid_data_.max_iout, pid_data_.max_iout);

  pid_data_.d_out = pid_data_.kd * (pid_data_.current_error - pid_data_.last_error);

  pid_data_.out = static_cast<int>(pid_data_.p_out + pid_data_.i_out + pid_data_.d_out);
  pid_data_.out = std::clamp(pid_data_.out, -static_cast<int>(pid_data_.max_out), static_cast<int>(pid_data_.max_out));

  pid_data_.last_error = pid_data_.current_error;

  return pid_data_.out;
}

void PidController::reset() {
  pid_data_.out = 0;
  pid_data_.p_out = 0.0;
  pid_data_.i_out = 0.0;
  pid_data_.d_out = 0.0;
  pid_data_.last_error = 0.0;
  pid_data_.current_error = 0.0;
  spdlog::info("PidController state reset.");
}

const PidData& PidController::getPidData() const {
  return pid_data_;
}

} // namespace mini_infantry