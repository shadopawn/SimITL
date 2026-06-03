#include <fmt/format.h>
#include <thread>
#include <chrono>
#include <array>

#include "network/packets.h"
#include "simitl.h"

#include <cctype>
#include <iostream>

std::thread t{};
bool running = true;

StateInput stateInput = {};
StateOutput stateOutput = {};

uint64_t currentFrame = 1U;
uint64_t framePrintOsd = 120U;
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
      uint8_t v = stateOutput.osd[(l * 30) + c];
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

void initInputDefaults(StateInput& s)
{
  // 16.6 ms 60 fps
  s.delta = 0.016f;

  // RC channels: -1.0 to 1.0 (mapped internally to Betaflight 1000-2000 range)
  s.rcData[0] =  0.0f; // roll    (centered)
  s.rcData[1] =  0.0f; // pitch   (centered)
  s.rcData[2] = -1.0f; // throttle (low)
  s.rcData[3] =  0.0f; // yaw     (centered)
  s.rcData[4] = -1.0f; // arm switch (disarmed)
  s.rcData[5] = -1.0f;
  s.rcData[6] = -1.0f;
  s.rcData[7] = -1.0f;

  // Identity rotation matrix
  s.rotation[0] = { 1.0f, 0.0f, 0.0f };
  s.rotation[1] = { 0.0f, 1.0f, 0.0f };
  s.rotation[2] = { 0.0f, 0.0f, 1.0f };

  // Motor imbalance
  for (int i = 0; i < 4; i++) {
    s.motorImbalance[i] = { 13.0f, 7.0f, 5.0f };
  }

  // Gyro noise
  s.gyroBaseNoiseAmp  = 0.000287f;
  s.gyrobaseNoiseFreq = 228.0f;

  // Frame harmonics
  s.frameHarmonic1Amp  = 0.02242f;
  s.frameHarmonic1Freq = 275.0f;
  s.frameHarmonic2Amp  = 0.01f;
  s.frameHarmonic2Freq = 326.66f;

  // Battery fully charged 4s
  s.vbat = 16.8f;
}

void updateThread()
{
  while (running)
  {
    simitl_update(stateInput);
    stateOutput = simitl_get_state();
    printOsdToCli();

    // 16.6 ms 60 fps
    std::this_thread::sleep_for(std::chrono::microseconds(16600));
    fmt::print(".");

    currentFrame++;
  }
}

int main() {
  fmt::print("simitl-tester starting...\n");

  initInputDefaults(stateInput);

  BetaflightInit("test.bin");

  t = std::thread(updateThread);
  t.join();

  return 0;
}