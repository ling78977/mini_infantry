#pragma once

#include <boost/asio.hpp>
#include <functional>
#include <chrono>
#include <iostream>

namespace mini_infantry {

// 简单的日志宏，方便后续替换为更复杂的日志系统
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl

class PeriodicTimer {
public:
    // 构造函数，接收io_context、回调函数和周期
    PeriodicTimer(boost::asio::io_context& io_ctx, std::function<void()> callback, std::chrono::milliseconds period)
        : timer_(io_ctx), callback_(callback), period_(period) {}

    // 启动定时器
    void start() {
        timer_.expires_after(period_);
        timer_.async_wait(std::bind(&PeriodicTimer::handleTimeout, this, std::placeholders::_1));
        LOG_INFO("PeriodicTimer started with period: " << period_.count() << "ms");
    }

    // 停止定时器
    void stop() {
        timer_.cancel();
        LOG_INFO("PeriodicTimer stopped.");
    }

private:
    // 定时器回调处理函数
    void handleTimeout(const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted) {
            // 定时器被取消，正常退出
            return;
        }
        if (ec) {
            LOG_ERROR("PeriodicTimer error: " << ec.message());
            return;
        }

        // 执行用户回调
        if (callback_) {
            callback_();
        }

        // 重新调度定时器
        timer_.expires_at(timer_.expiry() + period_);
        timer_.async_wait(std::bind(&PeriodicTimer::handleTimeout, this, std::placeholders::_1));
    }

    boost::asio::steady_timer timer_;
    std::function<void()> callback_;
    std::chrono::milliseconds period_;
};

} // namespace mini_infantry