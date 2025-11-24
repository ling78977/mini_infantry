#include "motion_sovler.hpp"
#include "motor.hpp"
#include "pid_controller.hpp"
#include "rotary_encoder.hpp"
#include "util/periodic_timer.hpp"
#include "util/logger_init.hpp" // Include logger_init.hpp

#include <boost/asio.hpp>
#include <chrono>
#include <random>
#include <thread>
#include <wiringPi.h>
#include <yaml-cpp/yaml.h> // 引入yaml-cpp库
#include <spdlog/spdlog.h> // Include spdlog

using namespace mini_infantry;

// 全局io_context和信号集，用于优雅退出
boost::asio::io_context io_context;
boost::asio::signal_set signals(io_context, SIGINT, SIGTERM);

// 电机指针
std::unique_ptr<Motor> motor_fl;
std::unique_ptr<Motor> motor_fr;
std::unique_ptr<Motor> motor_bl;
std::unique_ptr<Motor> motor_br;

// 主控制定时器
std::unique_ptr<PeriodicTimer> main_control_timer;

void signal_handler(const boost::system::error_code &error, int signal_number) {
  if (!error) {
    spdlog::warn("Received signal {}, shutting down...", signal_number);
    // 停止所有电机速度计算
    if (motor_fl)
      motor_fl->stopSpeedCalculation();
    if (motor_fr)
      motor_fr->stopSpeedCalculation();
    if (motor_bl)
      motor_bl->stopSpeedCalculation();
    if (motor_br)
      motor_br->stopSpeedCalculation();

    // 停止主控制定时器
    if (main_control_timer)
      main_control_timer->stop();

    // 停止io_context
    io_context.stop();
  }
}

void main_control_loop() {
  // 圆形轨迹参数
  static float radius = 0.5f; // 圆形轨迹的半径 (米)
  static float circle_linear_speed = 0.2f; // 沿圆周的线速度 (米/秒)
  static float current_angle = 0.0f; // 当前角度 (弧度)

  // 控制周期 (从main函数中获取，这里假设为50ms)
  static float control_period_s = 50.0f / 1000.0f; // 50ms = 0.05s

  // 计算当前时刻的vx和vy，使小车沿圆周运动
  // vx = -V * sin(angle)
  // vy = V * cos(angle)
  float target_vx = -circle_linear_speed * sin(current_angle);
  float target_vy = circle_linear_speed * cos(current_angle);
  float target_v_yaw = 0.0f; // 朝向不变

  // 平滑因子，控制速度过渡的快慢
  static float smoothing_factor = 0.1f; // 每次更新10%的差距

  // 当前速度 (用于平滑过渡)
  static float current_vx = 0.0f;
  static float current_vy = 0.0f;
  static float current_v_yaw = 0.0f;

  // 平滑过渡当前速度到目标速度
  current_vx += (target_vx - current_vx) * smoothing_factor;
  current_vy += (target_vy - current_vy) * smoothing_factor;
  current_v_yaw += (target_v_yaw - current_v_yaw) * smoothing_factor;

  MotionSolver::solve(current_vx, current_vy, current_v_yaw);

  spdlog::info("Target: vx={}, vy={}, v_yaw={}", target_vx, target_vy, target_v_yaw);
  spdlog::info("Current: vx={}, vy={}, v_yaw={}", current_vx, current_vy, current_v_yaw);
  // 打印当前电机速度
  spdlog::info("Current FL Speed: {}, FR Speed: {}, BL Speed: {}, BR Speed: {}",
               motor_fl->getEncoderSpeed(), motor_fr->getEncoderSpeed(),
               motor_bl->getEncoderSpeed(), motor_br->getEncoderSpeed());

  // 更新角度，使小车继续沿圆周运动
  // 角度变化 = (线速度 / 半径) * 时间
  current_angle += (circle_linear_speed / radius) * control_period_s;
  // 保持角度在0到2PI之间
  if (current_angle > 2 * M_PI) {
    current_angle -= 2 * M_PI;
  }
}

int main(int argc, char* argv[]) {
  // 初始化spdlog
  // 提取程序名作为日志器名称
  std::string program_name = (argc > 0) ? std::string(argv[0]) : "unknown_program";
  // 移除路径部分，只保留程序名
  size_t last_slash_idx = program_name.find_last_of("/\\");
  if (std::string::npos != last_slash_idx) {
      program_name = program_name.substr(last_slash_idx + 1);
  }
  util::init_logger("application.log", program_name, spdlog::level::info, spdlog::level::info);

  // 初始化WiringPi库
  if (wiringPiSetup() == -1) {
    spdlog::error("WiringPi initialization failed!");
    return 1;
  }

  // 注册信号处理函数
  signals.async_wait(signal_handler);

  try {
    // 加载YAML配置文件
    YAML::Node config = YAML::LoadFile("/home/jaren/projects/mini_infantry/config/pid_cinfig.yaml");

    // 获取电机引脚配置
    const YAML::Node &motor_pins = config["MotorPins"];

    // 前左电机
    motor_fl = std::make_unique<Motor>(
        io_context, motor_pins["motor_front_left"]["pwm_pin"].as<int>(),
        motor_pins["motor_front_left"]["encoder_pinA"].as<int>(),
        motor_pins["motor_front_left"]["encoder_pinB"].as<int>(),
        motor_pins["motor_front_left"]["motor_in1"].as<int>(),
        motor_pins["motor_front_left"]["motor_in2"].as<int>());

    // 前右电机
    motor_fr = std::make_unique<Motor>(
        io_context, motor_pins["motor_front_right"]["pwm_pin"].as<int>(),
        motor_pins["motor_front_right"]["encoder_pinA"].as<int>(),
        motor_pins["motor_front_right"]["encoder_pinB"].as<int>(),
        motor_pins["motor_front_right"]["motor_in1"].as<int>(),
        motor_pins["motor_front_right"]["motor_in2"].as<int>());

    // 后左电机
    motor_bl = std::make_unique<Motor>(
        io_context, motor_pins["motor_back_left"]["pwm_pin"].as<int>(),
        motor_pins["motor_back_left"]["encoder_pinA"].as<int>(),
        motor_pins["motor_back_left"]["encoder_pinB"].as<int>(),
        motor_pins["motor_back_left"]["motor_in1"].as<int>(),
        motor_pins["motor_back_left"]["motor_in2"].as<int>());

    // 后右电机
    motor_br = std::make_unique<Motor>(
        io_context, motor_pins["motor_back_right"]["pwm_pin"].as<int>(),
        motor_pins["motor_back_right"]["encoder_pinA"].as<int>(),
        motor_pins["motor_back_right"]["encoder_pinB"].as<int>(),
        motor_pins["motor_back_right"]["motor_in1"].as<int>(),
        motor_pins["motor_back_right"]["motor_in2"].as<int>());

    // 设置MotionSolver的电机
    MotionSolver::setMotors(motor_fl.get(), motor_fr.get(), motor_bl.get(),
                            motor_br.get());

    // 获取电机PID配置
    const YAML::Node &motors_config = config["Motors"];

    // 初始化PID参数和翻转设置
    auto init_motor_config = [&](Motor &motor, const YAML::Node &motor_node) {
      double kp = motor_node["kp"].as<double>();
      double ki = motor_node["ki"].as<double>();
      double kd = motor_node["kd"].as<double>();
      bool forward_is_flip = motor_node["forward_is_flip"].as<bool>();
      bool encoder_is_flip = motor_node["encoder_is_flip"].as<bool>();

      motor.pidInit(kp, ki, kd);
      motor.controlSetForwoardIsFlip(forward_is_flip);
      motor.encoderSetEncoderIsFlip(encoder_is_flip);
    };

    init_motor_config(*motor_fl, motors_config["motor_front_left"]);
    init_motor_config(*motor_fr, motors_config["motor_front_right"]);
    init_motor_config(*motor_bl, motors_config["motor_back_left"]);
    init_motor_config(*motor_br, motors_config["motor_back_right"]);

    // 启动电机速度自动计算
    motor_fl->runAutoCalcSpeed();
    motor_fr->runAutoCalcSpeed();
    motor_bl->runAutoCalcSpeed();
    motor_br->runAutoCalcSpeed();

    // 设置主控制循环定时器 (例如，每50ms执行一次)
    main_control_timer = std::make_unique<PeriodicTimer>(
        io_context, std::bind(main_control_loop), std::chrono::milliseconds(1000));
    main_control_timer->start();

    spdlog::info("Motion solver test started. Press Ctrl+C to exit.");

    // 运行io_context，这将阻塞直到所有任务完成或io_context停止
    io_context.run();

  } catch (const YAML::Exception &e) {
    spdlog::error("YAML parsing error: {}", e.what());
    return 1;
  } catch (const std::exception &e) {
    spdlog::error("Error: {}", e.what());
    return 1;
  }

  // MotionSolver的单例实例在程序退出时会自动释放，因为我们使用了智能指针管理Motor对象，
  // 并且MotionSolver的析构函数是default的，不负责释放外部传入的Motor指针。
  // 如果MotionSolver内部有需要手动释放的资源，则需要调用MotionSolver::releaseInstance();
  // 但目前看来，MotionSolver只持有Motor的裸指针，不负责其生命周期。
  // 因此，这里不需要显式调用MotionSolver::releaseInstance()。

  spdlog::info("Motion solver test finished.");
  return 0;
}