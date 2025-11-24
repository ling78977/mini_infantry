#include "motor.hpp"
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>
#include <boost/asio.hpp>
#include "yaml-cpp/yaml.h"

// 移除旧的 sigint_handler，使用 boost::asio::signal_set 替代

int main() {
  // signal(SIGINT, sigint_handler); // 移除此行

  try {
    boost::asio::io_context io_ctx;

    if (wiringPiSetup() == -1) {
      LOG_ERROR("Failed to initialize wiringPi");
      return 1;
    }
    LOG_INFO("wiringPi initialized successfully");

    YAML::Node config = YAML::LoadFile("/home/jaren/projects/mini_infantry/config/pid_cinfig.yaml");

    mini_infantry::Motor motor1(io_ctx, config["MotorPins"]["motor_front_left"]["pwm_pin"].as<int>(),
                                 config["MotorPins"]["motor_front_left"]["encoder_pinA"].as<int>(),
                                 config["MotorPins"]["motor_front_left"]["encoder_pinB"].as<int>(),
                                 config["MotorPins"]["motor_front_left"]["motor_in1"].as<int>(),
                                 config["MotorPins"]["motor_front_left"]["motor_in2"].as<int>());
    mini_infantry::Motor motor2(io_ctx, config["MotorPins"]["motor_front_right"]["pwm_pin"].as<int>(),
                                 config["MotorPins"]["motor_front_right"]["encoder_pinA"].as<int>(),
                                 config["MotorPins"]["motor_front_right"]["encoder_pinB"].as<int>(),
                                 config["MotorPins"]["motor_front_right"]["motor_in1"].as<int>(),
                                 config["MotorPins"]["motor_front_right"]["motor_in2"].as<int>());
    mini_infantry::Motor motor3(io_ctx, config["MotorPins"]["motor_back_left"]["pwm_pin"].as<int>(),
                                 config["MotorPins"]["motor_back_left"]["encoder_pinA"].as<int>(),
                                 config["MotorPins"]["motor_back_left"]["encoder_pinB"].as<int>(),
                                 config["MotorPins"]["motor_back_left"]["motor_in1"].as<int>(),
                                 config["MotorPins"]["motor_back_left"]["motor_in2"].as<int>());
    mini_infantry::Motor motor4(io_ctx, config["MotorPins"]["motor_back_right"]["pwm_pin"].as<int>(),
                                 config["MotorPins"]["motor_back_right"]["encoder_pinA"].as<int>(),
                                 config["MotorPins"]["motor_back_right"]["encoder_pinB"].as<int>(),
                                 config["MotorPins"]["motor_back_right"]["motor_in1"].as<int>(),
                                 config["MotorPins"]["motor_back_right"]["motor_in2"].as<int>());

    std::vector<mini_infantry::Motor *> motors = {&motor1, &motor2, &motor3, &motor4};

    for (auto &motor : motors) {
      motor->pidInit();
      motor->runAutoCalcSpeed(); // Default: auto speed calculation is OFF
    }

    int val1 = 0, step1 = 1;
    int val2 = 0, step2 = 1;
    int val3 = 0, step3 = 1;
    int val4 = 0, step4 = 1;

    while (1) {
      // val1 += step1;
      // if (val1 >= 100 || val1 <= 0)
      //   step1 = -step1;
      // motor1.controlSetPwm(val1);
      LOG_INFO("motor1 steps: " << motor1.encoderGetSteps());

      // val2 += step2;
      // if (val2 >= 100 || val2 <= 0)
      //   step2 = -step2;
      // motor2.controlSetPwm(val2);
      LOG_INFO("motor2 steps: " << motor2.encoderGetSteps());

      // val3 += step3;
      // if (val3 >= 100 || val3 <= 0)
      //   step3 = -step3;
      // motor3.controlSetPwm(val3);
      LOG_INFO("motor3 steps: " << motor3.encoderGetSteps());

      // val4 += step4;
      // if (val4 >= 100 || val4 <= 0)
      //   step4 = -step4;
      // motor4.controlSetPwm(val4);
      LOG_INFO("motor4 steps: " << motor4.encoderGetSteps());

      std::this_thread::sleep_for(std::chrono::milliseconds(5
    ));
    }

    // Handle graceful shutdown
    boost::asio::signal_set signals(io_ctx, SIGINT, SIGTERM);
    signals.async_wait([&](const boost::system::error_code &, int) {
      LOG_INFO("\nCaught signal, shutting down gracefully...");
      io_ctx.stop();
    });

    // Run the event loop
    io_ctx.run();

  } catch (const std::exception &e) {
    LOG_ERROR("Exception: " << e.what());
    return 1;
  }

  return 0;
}