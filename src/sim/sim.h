#ifndef SIMITL_H
#define SIMITL_H

#ifdef _WIN32
#include "winsock2.h"
#endif

#include "sim/bf.h"
#include <thread>
#include <fmt/format.h>

namespace SimITL{

  class Sim {
  public:
    static Sim& getInstance();

    ~Sim();

    // initialize
    void init(const char* eepromFilename);
    // update the simulation according to new inputs
    BetaflightOutput update(const BetaflightInput& betaflightInput);

    bool running = false;
    bool wsThreadRunning = false;

  private:
    std::thread wsThread{};

    // protected for testing
  protected:
    Sim();
    
  };

}

#endif // SIMITL_H