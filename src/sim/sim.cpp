#include "sim.h"

#include <chrono>
#include <cstdint>
#include <stdio.h>

namespace SimITL{

  // 20kHz scheduler, is enough to run PID at 8khz
  const int64_t FREQUENCY = 8e3;//20e3;
  const int64_t DELTA_MICROS = 1e6 / FREQUENCY;

  Sim& Sim::getInstance() {
    static Sim simulator;
    return simulator;
  }

  Sim::Sim()
  {
    mPhysics.setSimState(&mSimState);
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

  void Sim::reinitPhysics(const StateInit& stateInit){
    mPhysics.initState(stateInit);
  }

  void Sim::update(const StateInput& stateInput){
    int64_t stateUpdateDeltaMicros = static_cast<int64_t>(stateInput.delta * 1000000.0);

    if(stateUpdateDeltaMicros > static_cast<int64_t>(100000)){
      stateUpdateDeltaMicros = static_cast<int64_t>(100000);
    }
    if(stateUpdateDeltaMicros < static_cast<int64_t>(0)){
      stateUpdateDeltaMicros = static_cast<int64_t>(1);
    }
    
    total_delta += stateUpdateDeltaMicros;

    //update rc data
    BF::setRcData(stateInput.rcData);
    
    //rc data is updated independently
    simStep();
  }

  const StateOutput& Sim::getStateUpdate() const{
    return mSimState.stateOutput;
  }

  void Sim::command(const CommandType cmd){
    mPhysics.updateCommands(cmd);
  };

  void Sim::simStep() {
    for (auto k = 0u; (total_delta - DELTA_MICROS) >= 0; k++) {
      total_delta -= DELTA_MICROS;
      const double dt = static_cast<double>(DELTA_MICROS) / 1e6f;
  
      // updates betaflight data and schedules bf update
      BF::update(DELTA_MICROS, mSimState);
    }
  }

  void Sim::stop(){
    // stopping the ws coms kind of corrupts managed memory of the engine under linux...
    /*
    running = false;

    BF::stopSerial();

    if(wsThread.joinable()){
      wsThread.join();
    }
    */
  }

}