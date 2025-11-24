// Standard Library
#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <mutex>
#include <vector>

// Third-party Libraries
#include "mqtt/async_client.h"
#include "nlohmann/json.hpp"
#include <boost/asio.hpp>
#include <fstream>
#include <spdlog/spdlog.h> // Include spdlog
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>
// Project Headers
#include "motor.hpp"
#include "util/logger_init.hpp" // Include logger_init.hpp

using json = nlohmann::json;

// 定义电机ID枚举
enum MotorID { FRONT_LEFT = 0, FRONT_RIGHT = 1, BACK_LEFT = 2, BACK_RIGHT = 3 };

// MQTT and Threading
// const std::string SERVER_ADDRESS("tcp://localhost:1883"); // 从配置文件加载
// const std::string CLIENT_ID("pid_debugger"); // 从配置文件加载
// const std::string TOPIC_CONTROL("pid/control"); // 从配置文件加载
// const std::string TOPIC_STATUS("pid/status"); // 从配置文件加载
// const int QOS = 1; // 从配置文件加载

std::atomic<MotorID> target_motor_id(FRONT_LEFT);
std::atomic<double> target_speed(0.0);
std::mutex motor_data_mutex;

// MQTT Callback handler
class callback : public virtual mqtt::callback {
  mqtt::async_client &cli_;
  std::vector<mini_infantry::Motor *> &motors_;
  const std::string topic_control_;
  const std::string client_id_;
  const int qos_;
  YAML::Node config_node_; // Store the config node for saving parameters

  void connected(const std::string &cause) override {
    spdlog::info("Connection successful.");
    spdlog::info("Subscribing to topic '{}' for client {} using QoS {}", topic_control_, client_id_, qos_);
    cli_.subscribe(topic_control_, qos_);
  }

  void connection_lost(const std::string &cause) override {
    spdlog::warn("Connection lost");
    if (!cause.empty())
      spdlog::warn("\tcause: {}", cause);
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
          spdlog::info("Saving parameters for motor {}...", motor_idx);
          // Use the stored config_node_
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
            config_node_["Motors"][motor_name]["kp"] = kp;
            config_node_["Motors"][motor_name]["ki"] = ki;
            config_node_["Motors"][motor_name]["kd"] = kd;
            config_node_["Motors"][motor_name]["forward_is_flip"] = forward_is_flip;
            config_node_["Motors"][motor_name]["encoder_is_flip"] = encoder_is_flip;
            std::ofstream fout("/home/jaren/projects/mini_infantry/config/pid_cinfig.yaml");
            fout << config_node_; // Write the modified config_node_
            fout.close();
            spdlog::info("Parameters saved.");
          }
        }
      } else {
        spdlog::error("Invalid motor index received: {}", static_cast<int>(motor_idx));
      }
    } catch (const json::parse_error &e) {
      spdlog::error("JSON parse error: {}", e.what());
    } catch (const std::exception &e) {
      spdlog::error("Error processing message: {}", e.what());
    }
  }

public:
  callback(mqtt::async_client &cli, std::vector<mini_infantry::Motor *> &motors, const YAML::Node &config)
      : cli_(cli), motors_(motors), topic_control_(config["MqttConfig"]["topic_control"].as<std::string>()),
        client_id_(config["MqttConfig"]["client_id"].as<std::string>()), qos_(config["MqttConfig"]["qos"].as<int>()), config_node_(config) {
  }
};

int main(int argc, char *argv[]) {
  // Initialize spdlog
  // 提取程序名作为日志器名称
  std::string program_name = (argc > 0) ? std::string(argv[0]) : "unknown_program";
  // 移除路径部分，只保留程序名
  size_t last_slash_idx = program_name.find_last_of("/\\");
  if (std::string::npos != last_slash_idx) {
    program_name = program_name.substr(last_slash_idx + 1);
  }
  util::init_logger("application.log", program_name, spdlog::level::info, spdlog::level::info);

  boost::asio::io_context io_ctx;

  if (wiringPiSetup() == -1) {
    spdlog::error("Failed to initialize wiringPi");
    return 1;
  }
  spdlog::info("wiringPi initialized successfully");

  YAML::Node main_config = YAML::LoadFile("/home/jaren/projects/mini_infantry/config/pid_cinfig.yaml");
  int speed_buffer_size = main_config["MotorConfig"]["speed_buffer_size"].as<int>();

  mini_infantry::Motor motor_front_left(io_ctx, main_config["MotorPins"]["motor_front_left"]["pwm_pin"].as<int>(),
                                        main_config["MotorPins"]["motor_front_left"]["encoder_pinA"].as<int>(),
                                        main_config["MotorPins"]["motor_front_left"]["encoder_pinB"].as<int>(),
                                        main_config["MotorPins"]["motor_front_left"]["motor_in1"].as<int>(),
                                        main_config["MotorPins"]["motor_front_left"]["motor_in2"].as<int>());
  mini_infantry::Motor motor_front_right(io_ctx, main_config["MotorPins"]["motor_front_right"]["pwm_pin"].as<int>(),
                                         main_config["MotorPins"]["motor_front_right"]["encoder_pinA"].as<int>(),
                                         main_config["MotorPins"]["motor_front_right"]["encoder_pinB"].as<int>(),
                                         main_config["MotorPins"]["motor_front_right"]["motor_in1"].as<int>(),
                                         main_config["MotorPins"]["motor_front_right"]["motor_in2"].as<int>());
  mini_infantry::Motor motor_back_left(io_ctx, main_config["MotorPins"]["motor_back_left"]["pwm_pin"].as<int>(),
                                       main_config["MotorPins"]["motor_back_left"]["encoder_pinA"].as<int>(),
                                       main_config["MotorPins"]["motor_back_left"]["encoder_pinB"].as<int>(),
                                       main_config["MotorPins"]["motor_back_left"]["motor_in1"].as<int>(),
                                       main_config["MotorPins"]["motor_back_left"]["motor_in2"].as<int>());
  mini_infantry::Motor motor_back_right(io_ctx, main_config["MotorPins"]["motor_back_right"]["pwm_pin"].as<int>(),
                                        main_config["MotorPins"]["motor_back_right"]["encoder_pinA"].as<int>(),
                                        main_config["MotorPins"]["motor_back_right"]["encoder_pinB"].as<int>(),
                                        main_config["MotorPins"]["motor_back_right"]["motor_in1"].as<int>(),
                                        main_config["MotorPins"]["motor_back_right"]["motor_in2"].as<int>());

  std::vector<mini_infantry::Motor *> motors = {&motor_front_left, &motor_front_right, &motor_back_left, &motor_back_right};

  // YAML::Node config = YAML::LoadFile("/home/jaren/projects/mini_infantry/config/pid_cinfig.yaml"); // 已经定义过
  motor_front_left.controlSetForwoardIsFlip(main_config["Motors"]["motor_front_left"]["forward_is_flip"].as<bool>());
  motor_front_left.encoderSetEncoderIsFlip(main_config["Motors"]["motor_front_left"]["encoder_is_flip"].as<bool>());
  motor_front_right.controlSetForwoardIsFlip(main_config["Motors"]["motor_front_right"]["forward_is_flip"].as<bool>());
  motor_front_right.encoderSetEncoderIsFlip(main_config["Motors"]["motor_front_right"]["encoder_is_flip"].as<bool>());
  motor_back_left.controlSetForwoardIsFlip(main_config["Motors"]["motor_back_left"]["forward_is_flip"].as<bool>());
  motor_back_left.encoderSetEncoderIsFlip(main_config["Motors"]["motor_back_left"]["encoder_is_flip"].as<bool>());
  motor_back_right.controlSetForwoardIsFlip(main_config["Motors"]["motor_back_right"]["forward_is_flip"].as<bool>());
  motor_back_right.encoderSetEncoderIsFlip(main_config["Motors"]["motor_back_right"]["encoder_is_flip"].as<bool>());

  motors[FRONT_LEFT]->pidInit(main_config["Motors"]["motor_front_left"]["kp"].as<double>(),
                              main_config["Motors"]["motor_front_left"]["ki"].as<double>(),
                              main_config["Motors"]["motor_front_left"]["kd"].as<double>());
  motors[FRONT_RIGHT]->pidInit(main_config["Motors"]["motor_front_right"]["kp"].as<double>(),
                               main_config["Motors"]["motor_front_right"]["ki"].as<double>(),
                               main_config["Motors"]["motor_front_right"]["kd"].as<double>());
  motors[BACK_LEFT]->pidInit(main_config["Motors"]["motor_back_left"]["kp"].as<double>(),
                             main_config["Motors"]["motor_back_left"]["ki"].as<double>(),
                             main_config["Motors"]["motor_back_left"]["kd"].as<double>());
  motors[BACK_RIGHT]->pidInit(main_config["Motors"]["motor_back_right"]["kp"].as<double>(),
                              main_config["Motors"]["motor_back_right"]["ki"].as<double>(),
                              main_config["Motors"]["motor_back_right"]["kd"].as<double>());

  for (auto &motor : motors) {
    // if (motor != &motor_back_right) {
    //   continue;
    // }
    // motor->setDirectMode(true);
    motor->runAutoCalcSpeed(10); // Start with auto speed calculation enabled
  }

  const std::string SERVER_ADDRESS = main_config["MqttConfig"]["server_address"].as<std::string>();
  const std::string CLIENT_ID = main_config["MqttConfig"]["client_id"].as<std::string>();
  const std::string TOPIC_STATUS = main_config["MqttConfig"]["topic_status"].as<std::string>();
  const int QOS = main_config["MqttConfig"]["qos"].as<int>();

  mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
  callback cb(client, motors, main_config); // Pass main_config to the callback constructor
  client.set_callback(cb);

  auto connOpts = mqtt::connect_options_builder().clean_session().will(mqtt::message("test", "LWT", QOS)).finalize();

  try {
    spdlog::info("Connecting to the MQTT server...");
    client.connect(connOpts)->wait();
  } catch (const mqtt::exception &exc) {
    spdlog::error("Error: {}", exc.what());
    return 1;
  }

  spdlog::info("MQTT client connected. Starting event loop.");

  boost::asio::steady_timer pid_timer(io_ctx);
  int pid_period_ms = 15; // PID calculation interval

  std::function<void(const boost::system::error_code &)> pid_task_handler;
  pid_task_handler = [&](const boost::system::error_code &ec) {
    if (ec) {
      if (ec == boost::asio::error::operation_aborted)
        return;
      spdlog::error("PID timer error: {}", ec.message());
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

          // spdlog::info("Motor {} speed: {} pwm: {}", i, speed_to_set, pwm_value);

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
    spdlog::info("Caught signal, shutting down gracefully...");
    io_ctx.stop();
  });

  // Run the event loop
  io_ctx.run();

  // Cleanup
  spdlog::info("Disconnecting from the MQTT server...");
  if (client.is_connected()) {
    client.disconnect()->wait();
  }
  spdlog::info("Clean up and exit.");

  return 0;
}