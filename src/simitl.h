#include "sim/betaflight_input_output.h"

extern "C" {
  extern void BetaflightInit(const char* eepromFilename);
  extern BetaflightOutput BetaflightUpdate(const BetaflightInput& betaflightInput);
}