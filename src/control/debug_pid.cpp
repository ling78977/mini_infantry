// Standard Library
#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

// Third-party Libraries
#include "mqtt/async_client.h"
#include "nlohmann/json.hpp"
#include "yaml-cpp/yaml.h"
#include <boost/asio.hpp>

// Project Headers
#include "motion_sovler.hpp"

using json = nlohmann::json;

// 定义电机ID枚举
enum MotorID { FRONT_LEFT = 0, FRONT_RIGHT = 1, BACK_LEFT = 2, BACK_RIGHT = 3 };

// MQTT and Threading
const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("pid_debugger");
const std::string TOPIC_CONTROL("pid/control");
const std::string TOPIC_STATUS("pid/status");
const int QOS = 1;

std::atomic<MotorID> target_motor_id(FRONT_LEFT);
std::atomic<double> target_speed(0.0);
std::mutex motor_data_mutex;

// MQTT Callback handler
class callback : public virtual mqtt::callback {
  mqtt::async_client &cli_;
  std::vector<mini_infantry::Motor *> &motors_;

  void connected(const std::string &cause) override {
    std::cout << "\nConnection successful." << std::endl;
    std::cout << "\nSubscribing to topic '" << TOPIC_CONTROL << "' for client " << CLIENT_ID << " using QoS" << QOS << "\n" << std::endl;
    cli_.subscribe(TOPIC_CONTROL, QOS);
  }

  void connection_lost(const std::string &cause) override {
    std::cout << "\nConnection lost" << std::endl;
    if (!cause.empty())
      std::cout << "\tcause: " << cause << std::endl;
  }

  void message_arrived(mqtt::const_message_ptr msg) override {
    try {
      auto j = json::parse(msg->get_payload_str());

      uint8_t motor_idx = j["motor_id"];
      double kp = j["kp"];
      double ki = j["ki"];
      double kd = j["kd"];
      double speed = j["speed"];
      bool forward_is_flip = j["forward_is_flip"];
      bool encoder_is_flip = j["encoder_is_flip"];
      bool save_params = j.value("save", false);

      std::lock_guard<std::mutex> lock(motor_data_mutex);
      if (motor_idx < motors_.size()) {
        target_motor_id = static_cast<MotorID>(motor_idx);
        target_speed = speed;
        motors_[motor_idx]->pidSet(kp, ki, kd);
        motors_[motor_idx]->controlSetForwoardIsFlip(forward_is_flip);
        motors_[motor_idx]->encoderSetEncoderIsFlip(encoder_is_flip);

        if (save_params) {
          std::cout << "Saving parameters for motor " << motor_idx << "..." << std::endl;
          YAML::Node config = YAML::LoadFile("/home/jaren/projects/mini_infantry/config/pid_cinfig.yaml");
          std::string motor_name;
          switch (static_cast<MotorID>(motor_idx)) {
          case FRONT_LEFT:
            motor_name = "motor_front_left";
            break;
          case FRONT_RIGHT:
            motor_name = "motor_front_right";
            break;
          case BACK_LEFT:
            motor_name = "motor_back_left";
            break;
          case BACK_RIGHT:
            motor_name = "motor_back_right";
            break;
          }
          if (!motor_name.empty()) {
            config["Motors"][motor_name]["kp"] = kp;
            config["Motors"][motor_name]["ki"] = ki;
            config["Motors"][motor_name]["kd"] = kd;
            config["Motors"][motor_name]["forward_is_flip"] = forward_is_flip;
            config["Motors"][motor_name]["encoder_is_flip"] = encoder_is_flip;
            std::ofstream fout("/home/jaren/projects/mini_infantry/config/pid_cinfig.yaml");
            fout << config;
            fout.close();
            std::cout << "Parameters saved." << std::endl;
          }
        }
      } else {
        std::cerr << "Invalid motor index received: " << static_cast<int>(motor_idx) << std::endl;
      }
    } catch (const json::parse_error &e) {
      std::cerr << "JSON parse error: " << e.what() << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Error processing message: " << e.what() << std::endl;
    }
  }

public:
  callback(mqtt::async_client &cli, std::vector<mini_infantry::Motor *> &motors) : cli_(cli), motors_(motors) {}
};

int main() {
  boost::asio::io_context io_ctx;

  mini_infantry::Motor motor_front_left(io_ctx, PWM_PIN_WIR_1, 0, 2, 8, 9);
  mini_infantry::Motor motor_front_right(io_ctx, PWM_PIN_WIR_23, 13, 14, 21, 22);
  mini_infantry::Motor motor_back_left(io_ctx, PWM_PIN_WIR_26, 10, 6, 31, 11);
  mini_infantry::Motor motor_back_right(io_ctx, PWM_PIN_WIR_24, 27, 29, 25, 28);

  std::vector<mini_infantry::Motor *> motors = {&motor_front_left, &motor_front_right, &motor_back_left, &motor_back_right};

  YAML::Node config = YAML::LoadFile("/home/jaren/projects/mini_infantry/config/pid_cinfig.yaml");
  motor_front_left.controlSetForwoardIsFlip(config["Motors"]["motor_front_left"]["forward_is_flip"].as<bool>());
  motor_front_left.encoderSetEncoderIsFlip(config["Motors"]["motor_front_left"]["encoder_is_flip"].as<bool>());
  motor_front_right.controlSetForwoardIsFlip(config["Motors"]["motor_front_right"]["forward_is_flip"].as<bool>());
  motor_front_right.encoderSetEncoderIsFlip(config["Motors"]["motor_front_right"]["encoder_is_flip"].as<bool>());
  motor_back_left.controlSetForwoardIsFlip(config["Motors"]["motor_back_left"]["forward_is_flip"].as<bool>());
  motor_back_left.encoderSetEncoderIsFlip(config["Motors"]["motor_back_left"]["encoder_is_flip"].as<bool>());
  motor_back_right.controlSetForwoardIsFlip(config["Motors"]["motor_back_right"]["forward_is_flip"].as<bool>());
  motor_back_right.encoderSetEncoderIsFlip(config["Motors"]["motor_back_right"]["encoder_is_flip"].as<bool>());

  motors[FRONT_LEFT]->pidInit(config["Motors"]["motor_front_left"]["kp"].as<double>(),
                              config["Motors"]["motor_front_left"]["ki"].as<double>(),
                              config["Motors"]["motor_front_left"]["kd"].as<double>());
  motors[FRONT_RIGHT]->pidInit(config["Motors"]["motor_front_right"]["kp"].as<double>(),
                               config["Motors"]["motor_front_right"]["ki"].as<double>(),
                               config["Motors"]["motor_front_right"]["kd"].as<double>());
  motors[BACK_LEFT]->pidInit(config["Motors"]["motor_back_left"]["kp"].as<double>(), config["Motors"]["motor_back_left"]["ki"].as<double>(),
                             config["Motors"]["motor_back_left"]["kd"].as<double>());
  motors[BACK_RIGHT]->pidInit(config["Motors"]["motor_back_right"]["kp"].as<double>(),
                              config["Motors"]["motor_back_right"]["ki"].as<double>(),
                              config["Motors"]["motor_back_right"]["kd"].as<double>());

  for (auto &motor : motors) {
    // if (motor != &motor_back_right) {
    //   continue;
    // }
    // motor->setDirectMode(true);
    motor->run(true); // Start with auto speed calculation enabled
  }

  mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
  callback cb(client, motors);
  client.set_callback(cb);

  auto connOpts = mqtt::connect_options_builder().clean_session().will(mqtt::message("test", "LWT", QOS)).finalize();

  try {
    std::cout << "Connecting to the MQTT server..." << std::endl;
    client.connect(connOpts)->wait();
  } catch (const mqtt::exception &exc) {
    std::cerr << "Error: " << exc.what() << std::endl;
    return 1;
  }

  std::cout << "MQTT client connected. Starting event loop." << std::endl;

  boost::asio::steady_timer pid_timer(io_ctx);
  int pid_period_ms = 20; // PID calculation interval

  std::function<void(const boost::system::error_code &)> pid_task_handler;
  pid_task_handler = [&](const boost::system::error_code &ec) {
    if (ec) {
      if (ec == boost::asio::error::operation_aborted)
        return;
      std::cerr << "PID timer error: " << ec.message() << std::endl;
      return;
    }

    MotorID current_target_motor;
    double current_target_speed;
    {
      std::lock_guard<std::mutex> lock(motor_data_mutex);
      current_target_motor = target_motor_id.load();
      current_target_speed = target_speed.load();
    }

    if (client.is_connected()) {
      for (size_t i = 0; i < motors.size(); ++i) {
        double speed_to_set = (i == current_target_motor) ? current_target_speed : 0.0;
        int pwm_value = motors[i]->pidCalculate(speed_to_set, motors[i]->getEncoderSpeed());
        motors[i]->controlSetPwm(pwm_value);

        if (i == current_target_motor) {

          std::cout << "Motor " << i << " speed: " << speed_to_set << " pwm: " << pwm_value << std::endl;

          json status_msg;
          status_msg["motor_id"] = i;
          status_msg["encoder_speed"] = motors[i]->getEncoderSpeed();
          status_msg["target_speed"] = speed_to_set;
          client.publish(TOPIC_STATUS, status_msg.dump(), QOS, false);
        }
      }
    }

    pid_timer.expires_at(pid_timer.expiry() + std::chrono::milliseconds(pid_period_ms));
    pid_timer.async_wait(pid_task_handler);
  };

  // Start timers
  pid_timer.expires_after(std::chrono::milliseconds(pid_period_ms));
  pid_timer.async_wait(pid_task_handler);

  // Handle graceful shutdown
  boost::asio::signal_set signals(io_ctx, SIGINT, SIGTERM);
  signals.async_wait([&](const boost::system::error_code &, int) {
    std::cout << "\nCaught signal, shutting down gracefully..." << std::endl;
    io_ctx.stop();
  });

  // Run the event loop
  io_ctx.run();

  // Cleanup
  std::cout << "Disconnecting from the MQTT server..." << std::endl;
  if (client.is_connected()) {
    client.disconnect()->wait();
  }
  std::cout << "Clean up and exit." << std::endl;

  return 0;
}