#include "sim/sim.h"
#include "sim/betaflight_input_output.h"
#include <fmt/format.h>
#include <stdlib.h>
#include <stdio.h>

SimITL::Sim* sim = nullptr;

// interface for c lib
extern "C" {
  void BetaflightInit(const char* eepromFilename){
    sim = &SimITL::Sim::getInstance();
    sim->init(eepromFilename);
  }

  BetaflightOutput BetaflightUpdate(const BetaflightInput& betaflightInput){
    return sim->update(betaflightInput);
  }
}
