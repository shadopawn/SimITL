#ifndef SIM_BETAFLIGHT_INPUT_OUTPUT_H
#define SIM_BETAFLIGHT_INPUT_OUTPUT_H

#include <cstdint>

struct Vec3F{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Vec4F{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float w = 1.0f;
};

struct BatteryState {
  uint8_t batCellCount = 6;
  // current battery voltage
  float batVoltage    = 0.0f; // in V
  // current sagged battery voltage
  float batVoltageSag = 0.0f; // in V
  // current battery capacity
  double batCapacity  = 0.0f; // in mAh

  // current amp draw in amps
  double amperage = 0.0f;
  // current mAh drawn from battery
  double mAhDrawn = 0.0f;
};


struct BetaflightInput{
  float deltaSeconds = 0.0f;

  float rcData[8] {};

  Vec3F gyro {};
  Vec3F accelerometer {};
  Vec4F rotation {};

  BatteryState batteryState {};
};

struct BetaflightOutput{
  bool armed = false;
  int armingDisabledFlags = 0;

  float motorPwm[4];

  bool beeping = false;

  uint8_t osd[16*30] {};
};

#endif