#ifndef MINI_INFANTRY_PID_CONTROLLER_HPP
#define MINI_INFANTRY_PID_CONTROLLER_HPP

#include <algorithm> // For std::clamp
#include <spdlog/spdlog.h> // For spdlog

namespace mini_infantry {

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
  PidController();

  void pidInit(double kp, double ki, double kd, double max_iout = 100.0);

  void pidSet(double kp, double ki, double kd);

  int pidCalculate(double set_val, double fdb_val);

  void reset();

  const PidData& getPidData() const;

private:
  PidData pid_data_; // PID参数和状态
};

} // namespace mini_infantry

#endif // MINI_INFANTRY_PID_CONTROLLER_HPP