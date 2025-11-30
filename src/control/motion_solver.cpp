#include "control/motion_sovler.hpp"
#include <cmath> // For fmax and fabs
#include <spdlog/spdlog.h> // For spdlog

namespace mini_infantry {

// 私有构造函数：初始化电机指针为nullptr
MotionSolver::MotionSolver() : motor_front_left_(nullptr), motor_front_right_(nullptr), motor_back_left_(nullptr), motor_back_right_(nullptr) {}

// 私有析构函数：不负责释放外部传入的电机指针（由外部管理）
MotionSolver::~MotionSolver() = default;


// 单例实例
::mini_infantry::MotionSolver *::mini_infantry::MotionSolver::instance_ = nullptr;

// 获取单例实例
::mini_infantry::MotionSolver *::mini_infantry::MotionSolver::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new MotionSolver();
    }
    return instance_;
}

// 释放单例实例
void MotionSolver::releaseInstance() {
    delete instance_;
    instance_ = nullptr;
}

// 外部传入电机指针（必须在使用其他功能前调用）
void MotionSolver::setMotors(Motor *motor_front_left, Motor *motor_front_right, Motor *motor_back_left, Motor *motor_back_right) {
    MotionSolver *solver = getInstance();
    if (solver->motor_front_left_ != nullptr) {
        // 可选：防止重复设置电机指针
        spdlog::critical("Motors already set. Cannot reinitialize.");
        throw ::std::runtime_error("Motors already set. Cannot reinitialize.");
    }
    if (!motor_front_left || !motor_front_right || !motor_back_left || !motor_back_right) {
        spdlog::critical("Motor pointers cannot be null.");
        throw ::std::runtime_error("Motor pointers cannot be null.");
    }
    solver->motor_front_left_ = motor_front_left;
    solver->motor_front_right_ = motor_front_right;
    solver->motor_back_left_ = motor_back_left;
    solver->motor_back_right_ = motor_back_right;
}

// 运动解算主函数
void MotionSolver::solve(float vx, float vy, float v_yaw) {
    MotionSolver *solver = getInstance();
    if (!solver->motor_front_left_ || !solver->motor_front_right_ || !solver->motor_back_left_ || !solver->motor_back_right_) {
        spdlog::critical("Motors not initialized. Call setMotors() first.");
        throw ::std::runtime_error("Motors not initialized. Call setMotors() first.");
    }
    float lf_speed, rf_speed, lb_speed, rb_speed;

    lf_speed = vx - vy - (d_x + d_y) * v_yaw;
    rf_speed = vx + vy + (d_x + d_y) * v_yaw;
    lb_speed = vx + vy - (d_x + d_y) * v_yaw;
    rb_speed = vx - vy + (d_x + d_y) * v_yaw;

    float max_ = fmax(fabs(lf_speed), fabs(rf_speed));
    max_ = fmax(max_, fabs(lb_speed));
    max_ = fmax(max_, fabs(rb_speed));

    if (max_ > 2) {
        lf_speed /= max_;
        rf_speed /= max_;
        lb_speed /= max_;
        rb_speed /= max_;
    }
    int pwm_value_front_left = solver->motor_front_left_->pidCalculate(lf_speed, solver->motor_front_left_->getEncoderSpeed());
    int pwm_value_front_right = solver->motor_front_right_->pidCalculate(rf_speed, solver->motor_front_right_->getEncoderSpeed());
    int pwm_value_back_left = solver->motor_back_left_->pidCalculate(lb_speed, solver->motor_back_left_->getEncoderSpeed());
    int pwm_value_back_right = solver->motor_back_right_->pidCalculate(rb_speed, solver->motor_back_right_->getEncoderSpeed());

    solver->motor_front_left_->controlSetPwm(pwm_value_front_left);
    solver->motor_front_right_->controlSetPwm(pwm_value_front_right);
    solver->motor_back_left_->controlSetPwm(pwm_value_back_left);
    solver->motor_back_right_->controlSetPwm(pwm_value_back_right);
}

} // namespace mini_infantry