#include <fmt/format.h>
#include <thread>
#include <chrono>
#include <array>

#include "simitl.h"

#include <cctype>
#include <iostream>

std::thread t{};
bool running = true;

BetaflightInput betaflightInput = {};
BetaflightOutput betaflightOutput = {};

uint64_t currentFrame = 1U;
uint64_t framePrintOsd = 1600U;
uint64_t framePrintMotors = 400U;
uint64_t frameRestart = 2000U;

void printOsdToCli()
{
  if(currentFrame % framePrintOsd != 0){
    return;
  }

  fmt::print("\n");
  for (int l = 0; l < 16; l++)
  {
    for (int c = 0; c < 30; c++)
    {
      uint8_t v = betaflightOutput.osd[(l * 30) + c];
      if (std::isprint(v))
      {
        std::cout << v;
      }
      else
      {
        std::cout << " ";
      }
    }
    fmt::print("\n");
  }
}

void printMotorPwm()
{
  if(currentFrame % framePrintMotors != 0){
    return;
  }

  fmt::print("\n");
  for (int i = 0; i < 4; i++)
  {
    fmt::print("Motor {}: PWM = {}", i+1, betaflightOutput.motorPwm[i]);
    fmt::print("\n");
  }
}

void initInputDefaults(BetaflightInput& bfInput)
{
  // 1.25 ms 800 fps
  bfInput.deltaSeconds = 0.00125f;

  // RC channels: -1.0 to 1.0 (mapped internally to Betaflight 1000-2000 range)
  bfInput.rcData[0] =  0.0f; // roll    (centered)
  bfInput.rcData[1] =  0.0f; // pitch   (centered)
  bfInput.rcData[2] = -1.0f; // throttle (low)
  bfInput.rcData[3] =  0.0f; // yaw     (centered)
  bfInput.rcData[4] = -1.0f; // arm switch (disarmed)
  bfInput.rcData[5] = -1.0f;
  bfInput.rcData[6] = -1.0f;
  bfInput.rcData[7] = -1.0f;

  bfInput.accelerometer.z = -9.81f;

  bfInput.gyro.y = 17.4533f; // 1000 degress per second
}

void updateThread()
{
  while (running)
  {
    betaflightOutput = BetaflightUpdate(betaflightInput);
    printOsdToCli();
    printMotorPwm();

    // 1.25 ms 800 fps
    std::this_thread::sleep_for(std::chrono::microseconds(1250));
    fmt::print(".");

    currentFrame++;
  }
}

int main() {
  fmt::print("simitl-tester starting...\n");

  initInputDefaults(betaflightInput);

  BetaflightInit("test.bin");

  t = std::thread(updateThread);
  t.join();

  return 0;
}