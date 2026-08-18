#include "sim/betaflight_input_output.h"

extern "C" {
  extern void BetaflightInit(const char* eepromFilename);
  extern BetaflightOutput BetaflightUpdate(const BetaflightInput& betaflightInput);
  extern void SetBlackboxValue(uint8_t index, int16_t value);
}