#include <fmt/format.h>
#include "bf.h"
#include "sitl.h" // access to bf internals
#include <array>

namespace SimITL{
  #define USE_QUAT_ORIENTATION

  #ifndef M_PI
  #define M_PI 3.14159265358979
  #endif

  const static auto GYRO_SCALE = 16.4f;
  const static auto RAD2DEG = (180.0f / float(M_PI));
  const static auto ACC_SCALE = (256 / 9.80665f);

  namespace BF {
    
    void resetRcData(){
      //reset rc data to valid data...
      for(int i = 0; i < SIMULATOR_MAX_RC_CHANNELS; i++){
        rcDataCache[i] = 1000U;
      }
    }

    void setRcData(const float (&data)[8])
    {
      uint32_t timeUs = BF::micros_passed & 0xFFFFFFFF;

      // std::array<uint16_t, 8> rcData;
      for (int i = 0; i < 8; i++)
      {
        rcDataCache[i] = uint16_t(1500 + data[i] * 500);
      }
      rcDataReceptionTimeUs = timeUs;
      // BF::rxMspFrameReceive(&rcData[0], 8);
      // hack to trick bf into using sim data...
      BF::rxRuntimeState.channelCount = SIMULATOR_MAX_RC_CHANNELS; // SimITL target.h
      BF::rxRuntimeState.rcReadRawFn = BF::rxRcReadData;
      BF::rxRuntimeState.rcFrameStatusFn = BF::rxRcFrameStatus;
      BF::rxRuntimeState.rxProvider = BF::RX_PROVIDER_UDP;
      //BF::rxRuntimeState.rcFrameTimeUsFn = BF::rxRcFrameTimeUs;
      BF::rxRuntimeState.lastRcFrameTimeUs = timeUs;
    }

    void setEepromFileName(const char* filename){
      fmt::print("BF::setEepromFileName {}\n", filename);
      const size_t maxFileSize = 512;
      EEPROM_FILENAME = new char[maxFileSize];
      std::fill(EEPROM_FILENAME, EEPROM_FILENAME + maxFileSize, 0);
      EEPROM_FILENAME[maxFileSize - 1] = '\0';
      memcpy(EEPROM_FILENAME, filename, strnlen(filename, maxFileSize));
    }

    void updateBattery(const BetaflightInput& betaflightInput){
      BF::setCellCount(betaflightInput.batteryState.batCellCount);
      // voltage
      BF::voltageMeter_t* vMeter = BF::getVoltageMeter();
      vMeter->unfiltered      = static_cast<uint16_t>(betaflightInput.batteryState.batVoltageSag * 1e2);
      vMeter->displayFiltered = static_cast<uint16_t>(betaflightInput.batteryState.batVoltageSag * 1e2);
      vMeter->sagFiltered     = static_cast<uint16_t>(betaflightInput.batteryState.batVoltage    * 1e2);
      // ampere
      BF::currentMeter_t* cMeter = BF::getCurrentMeter();
      cMeter->amperage        = static_cast<int32_t>(betaflightInput.batteryState.amperage * 1e2);
      cMeter->amperageLatest  = static_cast<int32_t>(betaflightInput.batteryState.amperage * 1e2);
      cMeter->mAhDrawn        = static_cast<int32_t>(betaflightInput.batteryState.mAhDrawn);
    }

    void updateGyroAcc(const BetaflightInput& betaflightInput){
      int16_t x, y, z;
      
      // Corrdinate system is adjusted to work with input from the unreal coordinate system
      x = int16_t(BF::constrain(int(betaflightInput.accelerometer.x * ACC_SCALE), -32767, 32767));
      y = int16_t(BF::constrain(int(-betaflightInput.accelerometer.y * ACC_SCALE), -32767, 32767));
      z = int16_t(BF::constrain(int(betaflightInput.accelerometer.z * ACC_SCALE), -32767, 32767));
      BF::virtualAccSet(BF::virtualAccDev, x, y, z);

      x = int16_t(BF::constrain(int(-betaflightInput.gyro.x * GYRO_SCALE * RAD2DEG), -32767, 32767));
      y = int16_t(BF::constrain(int(betaflightInput.gyro.y * GYRO_SCALE * RAD2DEG), -32767, 32767));
      z = int16_t(BF::constrain(int(-betaflightInput.gyro.z * GYRO_SCALE * RAD2DEG), -32767, 32767));
      BF::virtualGyroSet(BF::virtualGyroDev, x, y, z);

      BF::imuSetAttitudeQuat(
          betaflightInput.rotation.w,
          -betaflightInput.rotation.x,
          -betaflightInput.rotation.y,
          betaflightInput.rotation.z);
    }

    // void updateGps(const SimState& simState){
    //   const auto DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS = 1.113195f;
    //   const auto cosLon0 = 0.63141842418f;

    //   // set gps:
    //   vec3 pos;
    //   copy(pos, simState.stateInput.position);

    //   BF::EnableState(BF::GPS_FIX);
    //   BF::gpsSol.numSat = 10;
    //   BF::gpsSol.llh.lat =
    //       int32_t(
    //           pos[2] * 100 /
    //           DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS) +
    //       simState.stateInit.gps.lat;
    //   BF::gpsSol.llh.lon =
    //       int32_t(
    //           pos[0] * 100 /
    //           (cosLon0 *
    //             DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS)) +
    //       simState.stateInit.gps.lon;
    //   BF::gpsSol.llh.altCm = int32_t(pos[1] * 100) + simState.stateInit.gps.alt;
    //   vec3 linearVelocity;
    //   copy(linearVelocity, simState.stateInput.linearVelocity);
    //   BF::gpsSol.groundSpeed = uint16_t(length(linearVelocity) * 100);
    //   BF::GPS_update |= BF::GPS_MSP_UPDATE;
    // }

    void updateOsd(BetaflightOutput& betaflightOutput){
      bool osdChanged = false;
      for (int y = 0; y < VIDEO_LINES; y++) {
        for (int x = 0; x < CHARS_PER_LINE; x++) {
          betaflightOutput.osd[y * CHARS_PER_LINE + x] = BF::osdScreen[y][x];
        }
      }
    }

    BetaflightOutput update(uint64_t dt, const BetaflightInput& betaflightInput){
      bool schedulerExecuted = false;

      BF::micros_passed += dt;
      
      updateBattery(betaflightInput);
      updateGyroAcc(betaflightInput);
      // updateGps(simState);

      if (BF::sleep_timer > 0) {
        BF::sleep_timer -= dt;
        BF::sleep_timer = std::max(int64_t(0), BF::sleep_timer);
      } else {
        BF::scheduler();
        schedulerExecuted = true;
      }

      BetaflightOutput betaflightOutput {};

      updateOsd(betaflightOutput);

      betaflightOutput.armed = (BF::armingFlags & BF::ARMED) == BF::ARMED;
      betaflightOutput.armingDisabledFlags = (int)BF::getArmingDisableFlags();

      betaflightOutput.motorPwm[0] = BF::motorsPwm[0] / 1000.0f;
      betaflightOutput.motorPwm[1] = BF::motorsPwm[1] / 1000.0f;
      betaflightOutput.motorPwm[2] = BF::motorsPwm[2] / 1000.0f;
      betaflightOutput.motorPwm[3] = BF::motorsPwm[3] / 1000.0f;

      betaflightOutput.beeping = BF::getBeeper();

      return betaflightOutput;
    }

    void setDebugValue(uint8_t mode, uint8_t index, int16_t value){
      BF_DEBUG_SET(mode, index, value);
    }

    void updateSerial(){
      BF::updateSerialWs();
    }

    void stopSerial()
    {
      BF::stopSerialWs();
    }
  } // namespace bf
} // namespace SimITL