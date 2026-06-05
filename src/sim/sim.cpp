#include "sim.h"
#include "sim/betaflight_input_output.h"
#include <cstdint>

namespace SimITL{

  Sim& Sim::getInstance() {
    static Sim simulator;
    return simulator;
  }

  Sim::Sim(){
  }

  Sim::~Sim() {
  }

  void wsUpdateThread(Sim * sim){
    sim->wsThreadRunning = true;
    while (sim->running) {
      BF::updateSerial();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    sim->wsThreadRunning = false;
    fmt::print("wsThread end!!\n");
  }

  void Sim::init(const char* eepromFilename) {
    running = true;

    if(!wsThreadRunning){
      fmt::print("Starting ws update thread\n");
      wsThread = std::thread(wsUpdateThread, this);
    }
    
    //reset rc data to valid data...
    BF::resetRcData();

    fmt::print("Initializing betaflight\n");
    BF::setEepromFileName(eepromFilename);
    BF::init();
  }

  BetaflightOutput Sim::update(const BetaflightInput& betaflightInput){
    int64_t stateUpdateDeltaMicros = static_cast<int64_t>(betaflightInput.deltaSeconds * 1000000.0);

    if(stateUpdateDeltaMicros > static_cast<int64_t>(100000)){
      stateUpdateDeltaMicros = static_cast<int64_t>(100000);
    }
    if(stateUpdateDeltaMicros < static_cast<int64_t>(0)){
      stateUpdateDeltaMicros = static_cast<int64_t>(1);
    }
    
    //update rc data
    BF::setRcData(betaflightInput.rcData);
    
    //rc data is updated independently
    return BF::update(stateUpdateDeltaMicros, betaflightInput);
  }
}