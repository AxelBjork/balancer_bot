#include "../xbox_controller.h"
#include <chrono>
#include <iostream>
#include <thread>

int main() {
  XboxController pad;

  for (int i = 0; i < 100; ++i) { // poll for ~5s
    pad.update();
    std::cout << "LeftY=" << pad.leftY() << "  RightY=" << pad.rightY()
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
