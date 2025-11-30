#include "control/gimbal.hpp"
#include "util/logger_init.hpp" // 确保包含正确的头文件
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::seconds

int main() {
    util::init_logger("log/test/debug_gimbal_motor.log", "debug_gimbal_motor",spdlog::level::level_enum::debug,spdlog::level::level_enum::debug); // 使用正确的命名空间和参数
    spdlog::info("Starting gimbal motor debug test.");

    mini_infantry::Gimbal gimbal;

    // Give some time for serial port to initialize
    std::this_thread::sleep_for(std::chrono::seconds(2)); 

    // Test enabling motor 1
    // Test Yaw motor
    mini_infantry::MotorAddress yaw_motor_address = mini_infantry::MotorAddress::Yaw;
    spdlog::info("Attempting to enable Yaw Motor...");
    gimbal.enableMotor(yaw_motor_address);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    double yaw_angle = 1;
    spdlog::info("Attempting to rotate Yaw Motor by {} degrees...", yaw_angle);
    gimbal.rotateAngle(yaw_motor_address, yaw_angle, mini_infantry::RoateDirect::Forward, 200, 0);
   
    auto a=yaw_angle;
    int step=1;
    while (true) {
        a+=step;
   gimbal.rotateAngle(yaw_motor_address, a, mini_infantry::RoateDirect::Forward, 200, 0);
   if(std::abs(a)>=10){
    step=-step;
   }
   std::this_thread::sleep_for(std::chrono::milliseconds(10));
   }


    spdlog::info("Gimbal motor debug test finished.");

    return 0;
}