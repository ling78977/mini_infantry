#include "motor.hpp"
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>
#include <boost/asio.hpp>

void sigint_handler(int signum) {
  printf("Received Ctrl+C (SIGINT), cleaning up...\n");
  exit(EXIT_FAILURE);
}

int main() {
  signal(SIGINT, sigint_handler);

  try {
    boost::asio::io_context io_ctx;

    mini_infantry::Motor motor1(io_ctx, PWM_PIN_WIR_1, 0, 2, 8, 9);
    mini_infantry::Motor motor2(io_ctx, PWM_PIN_WIR_23, 13, 14, 21, 22);
    mini_infantry::Motor motor3(io_ctx, PWM_PIN_WIR_26, 10, 6, 31, 11);
    mini_infantry::Motor motor4(io_ctx, PWM_PIN_WIR_24, 27, 29, 25, 28);

    std::vector<mini_infantry::Motor *> motors = {&motor1, &motor2, &motor3, &motor4};

    for (auto &motor : motors) {
      motor->pidInit();
      motor->run(); // Default: auto speed calculation is OFF
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
      std::cout << "motor1 steps: " << motor1.encoderGetSteps() << std::endl;

      // val2 += step2;
      // if (val2 >= 100 || val2 <= 0)
      //   step2 = -step2;
      // motor2.controlSetPwm(val2);
      std::cout << "motor2 steps: " << motor2.encoderGetSteps() << std::endl;

      // val3 += step3;
      // if (val3 >= 100 || val3 <= 0)
      //   step3 = -step3;
      // motor3.controlSetPwm(val3);
      std::cout << "motor3 steps: " << motor3.encoderGetSteps() << std::endl;

      // val4 += step4;
      // if (val4 >= 100 || val4 <= 0)
      //   step4 = -step4;
      // motor4.controlSetPwm(val4);
      std::cout << "motor4 steps: " << motor4.encoderGetSteps() << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(5
    ));
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}