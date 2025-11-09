#ifndef MINI_INFANTRY_PID_CONTROLLER_HPP
#define MINI_INFANTRY_PID_CONTROLLER_HPP

#include <algorithm> // For std::clamp
#include <iostream> // For LOG_INFO/LOG_ERROR (if needed, or pass a logger)

namespace mini_infantry {

// 简单的日志宏，方便后续替换为更复杂的日志系统
// 注意：这里重新定义是为了pid_controller.hpp可以独立使用，
// 实际项目中应使用统一的日志系统
// 假设 motor.hpp 会先被包含，其中定义了 LOG_INFO 和 LOG_ERROR
// 如果 pid_controller.hpp 需要独立使用，则需要在此处添加 std:: 前缀
#ifndef LOG_INFO
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl
#endif
#ifndef LOG_ERROR
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#endif

/**
 * @brief 存储PID控制器参数和状态的数据结构。
 */
struct PidData {
  double kp;
  double ki;
  double kd;

  double max_out = 99.0; // 最大输出限制
  double max_iout;       // 积分项最大输出限制

  int out;             // 最终输出 (PWM值 -100~100)
  double p_out;        // 比例项输出
  double i_out;        // 积分项输出
  double d_out;        // 微分项输出
  double last_error;   // 上一次误差
  double current_error; // 当前误差
};

/**
 * @brief PID控制器类，封装了PID算法的初始化、设置和计算逻辑。
 */
class PidController {
public:
  /**
   * @brief 构造函数，初始化PID控制器。
   */
  PidController() {
    pid_data_.kp = 0.0;
    pid_data_.ki = 0.0;
    pid_data_.kd = 0.0;
    pid_data_.max_iout = 0.0; // 默认值，需要通过pidInit或pidSet设置
    reset();
    LOG_INFO("PidController initialized.");
  }

  /**
   * @brief 初始化PID参数。
   * @param kp 比例增益。
   * @param ki 积分增益。
   * @param kd 微分增益。
   * @param max_iout 积分项最大输出限制。
   */
  void pidInit(double kp, double ki, double kd, double max_iout = 100.0) {
    pid_data_.kp = kp;
    pid_data_.ki = ki;
    pid_data_.kd = kd;
    pid_data_.max_iout = max_iout;
    reset();
    LOG_INFO("PidController parameters initialized: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << ", MaxIOut=" << max_iout);
  }

  /**
   * @brief 设置PID参数。
   * @param kp 比例增益。
   * @param ki 积分增益。
   * @param kd 微分增益。
   */
  void pidSet(double kp, double ki, double kd) {
    pid_data_.kp = kp;
    pid_data_.ki = ki;
    pid_data_.kd = kd;
    LOG_INFO("PidController parameters set: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd);
  }

  /**
   * @brief 计算PID输出。
   * @param set_val 设定值。
   * @param fdb_val 反馈值。
   * @return 计算出的控制输出 (PWM值)。
   */
  int pidCalculate(double set_val, double fdb_val) {
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

  /**
   * @brief 重置PID控制器状态，清除误差和积分项。
   */
  void reset() {
    pid_data_.out = 0;
    pid_data_.p_out = 0.0;
    pid_data_.i_out = 0.0;
    pid_data_.d_out = 0.0;
    pid_data_.last_error = 0.0;
    pid_data_.current_error = 0.0;
    LOG_INFO("PidController state reset.");
  }

  /**
   * @brief 获取当前的PID数据。
   * @return PidData结构体的常量引用。
   */
  const PidData& getPidData() const {
    return pid_data_;
  }

private:
  PidData pid_data_; // PID参数和状态
};

} // namespace mini_infantry

#endif // MINI_INFANTRY_PID_CONTROLLER_HPP