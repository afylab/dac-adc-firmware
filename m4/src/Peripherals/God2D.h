#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>

#include "Config.h"
#include "Utils/TimingUtil.h"
#include "Utils/shared_memory.h"
#include "unordered_set"

class God2D {
 public:
  static void setup() { initializeRegistry(); }

  static void initializeRegistry() {
    registerMemberFunctionVector(timeSeriesBufferRamp2D,
                                 "2D_TIME_SERIES_BUFFER_RAMP");
    registerMemberFunctionVector(dacLedBufferRamp2D, "2D_DAC_LED_BUFFER_RAMP");
  }

  // timeSeriesBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, adcInterval_us, retrace (0.0f = false, 1.0f = true), snake (0.0f = false, 1.0f = true),
  // numFastDacChannels, [fastDacChannelID, fastDacV0, fastDacVf] *
  // numFastDacChannels, numSlowDacChannels, [slowDacChannelID, slowDacV0,
  // slowDacVf] * numSlowDacChannels, [adcChannelID] * numAdcChannels
  // note that if snake is true, the value of retrace is ignored.
  static OperationResult timeSeriesBufferRamp2D(
      const std::vector<float> &args) {
    // Minimum required arguments:
    // 7 initial params + at least 1 fast DAC channel + 1 slow DAC channel + ADC
    // channels
    if (args.size() < 7 + 3 + 3 + 1) {
      return OperationResult::Failure(
          "Not enough arguments provided for 2D ramp");
    }

    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numStepsFast = static_cast<int>(args[currentIndex++]);
    int numStepsSlow = static_cast<int>(args[currentIndex++]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[currentIndex++]);
    uint32_t adc_interval_us = static_cast<uint32_t>(args[currentIndex++]);
    bool retrace =
        static_cast<bool>(args[currentIndex++]);  // 0.0f = false, 1.0f = true
    
    bool snake =
      static_cast<bool>(args[currentIndex++]);  // 0.0f = false, 1.0f = true

    // Parse Fast DAC Channels
    if (currentIndex >= args.size()) {
      return OperationResult::Failure(
          "Unexpected end of arguments while parsing fast DAC channels");
    }
    int numFastDacChannels = static_cast<int>(args[currentIndex++]);
    if (args.size() < currentIndex + numFastDacChannels * 3) {
      return OperationResult::Failure(
          "Not enough arguments for fast DAC channels");
    }

    int fastDacChannels[numFastDacChannels];
    float fastDacV0s[numFastDacChannels];
    float fastDacVfs[numFastDacChannels];

    for (int i = 0; i < numFastDacChannels; ++i) {
      fastDacChannels[i] = static_cast<int>(args[currentIndex++]);
      fastDacV0s[i] = args[currentIndex++];
      fastDacVfs[i] = args[currentIndex++];
    }

    // Parse Slow DAC Channels
    if (currentIndex >= args.size()) {
      return OperationResult::Failure(
          "Unexpected end of arguments while parsing slow DAC channels");
    }
    int numSlowDacChannels = static_cast<int>(args[currentIndex++]);
    if (args.size() < currentIndex + numSlowDacChannels * 3) {
      return OperationResult::Failure(
          "Not enough arguments for slow DAC channels");
    }

    int slowDacChannels[numSlowDacChannels];
    float slowDacV0s[numSlowDacChannels];
    float slowDacVfs[numSlowDacChannels];

    for (int i = 0; i < numSlowDacChannels; ++i) {
      slowDacChannels[i] = static_cast<int>(args[currentIndex++]);
      slowDacV0s[i] = args[currentIndex++];
      slowDacVfs[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels) {
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int adcChannels[numAdcChannels];
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Validate total number of DAC channels
    if (numFastDacChannels + numSlowDacChannels != numDacChannels) {
      return OperationResult::Failure(
          "Sum of fast and slow DAC channels does not match numDacChannels");
    }

    // Reset ADC before starting the ramp
    ADCController::resetToPreviousConversionTimes();

    float voltageStepSize[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSize[i] = (slowDacVfs[i] - slowDacV0s[i]) / (numStepsSlow - 1);
    }

    float previousVoltageSet[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      previousVoltageSet[i] = slowDacV0s[i];
    }

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();


    // Iterate over slow steps with optional retrace
    for (int slowStep = 0; slowStep < numStepsSlow && !getStopFlag();
         ++slowStep) {
      // Set slow DAC channels to the current slow step voltages
      #if !defined(__NEW_SHIELD__)
      PeripheralCommsController::beginDacTransaction();
      #endif
      for (int i = 0; i < numSlowDacChannels; ++i) {
        DACController::setVoltageNoTransactionNoLdac(slowDacChannels[i],
                                                     previousVoltageSet[i]);
        previousVoltageSet[i] += voltageStepSize[i];
      }
      DACController::toggleLdac();
      #if !defined(__NEW_SHIELD__)
      PeripheralCommsController::endTransaction();
      #endif

      // Determine ramp direction based on retrace flag
      bool isReverse = false;
      if (snake) {
        isReverse = (slowStep % 2 != 0);  // Reverse on odd slow steps
      }

      // Prepare ramp voltages
      float currentV0s[numFastDacChannels];
      float currentVfs[numFastDacChannels];
      if (isReverse) {
        // Swap V0 and Vf for reverse ramp
        for (int i = 0; i < numFastDacChannels; ++i) {
          currentV0s[i] = fastDacVfs[i];
          currentVfs[i] = fastDacV0s[i];
        }
      } else {
        for (int i = 0; i < numFastDacChannels; ++i) {
          currentV0s[i] = fastDacV0s[i];
          currentVfs[i] = fastDacVfs[i];
        }
      }

      OperationResult ramp1Result = OperationResult::Success();
      OperationResult ramp2Result = OperationResult::Success();

      ramp1Result = God::timeSeriesBufferRampBase(
        numFastDacChannels, numAdcChannels, numStepsFast, dac_interval_us,
        adc_interval_us, fastDacChannels, currentV0s, currentVfs,
        adcChannels);

      if (retrace && !snake) {
        ramp2Result = God::timeSeriesBufferRampBase(
          numFastDacChannels, numAdcChannels, numStepsFast, dac_interval_us,
          adc_interval_us, fastDacChannels, currentVfs, currentV0s,
          adcChannels);
      }


      if (!ramp1Result.isSuccess() && !ramp2Result.isSuccess()) {
        return OperationResult::Failure(ramp1Result.getMessage() + "\n" +
                                        ramp2Result.getMessage());
      } else if (!ramp1Result.isSuccess()) {
        return OperationResult::Failure(ramp1Result.getMessage());
      } else if (!ramp2Result.isSuccess()) {
        return OperationResult::Failure(ramp2Result.getMessage());
      }
    }
    
    // Reset ADC before setting channels to idle mode
    ADCController::resetToPreviousConversionTimes();

    // Set ADC channels to idle mode
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
    }

    PeripheralCommsController::dataLedOff();

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("2D RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }


  // dacLedBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, dacSettlingTime_us, retrace (0.0f = false, 1.0f = true), snake (0.0f = false, 1.0f = true),
  // numAdcAverages numFastDacChannels, [fastDacChannelID, fastDacV0, fastDacVf]
  // * numFastDacChannels, numSlowDacChannels, [slowDacChannelID, slowDacV0,
  // slowDacVf] * numSlowDacChannels, [adcChannelID] * numAdcChannels
  // note that if snake is true, the value of retrace is ignored.
  static OperationResult dacLedBufferRamp2D(const std::vector<float> &args) {
    // Minimum required arguments:
    // 7 initial params + at least 1 fast DAC channel + 1 slow DAC channel + ADC
    // channels
    if (args.size() < 7 + 3 + 3 + 1) {
      return OperationResult::Failure(
          "Not enough arguments provided for 2D ramp");
    }

    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numStepsFast = static_cast<int>(args[currentIndex++]);
    int numStepsSlow = static_cast<int>(args[currentIndex++]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[currentIndex++]);
    uint32_t dac_settling_time_us = static_cast<uint32_t>(args[currentIndex++]);
    bool retrace =
        static_cast<bool>(args[currentIndex++]);  // 0.0f = false, 1.0f = true
    bool snake =
        static_cast<bool>(args[currentIndex++]);  // 0.0f = false, 1.0f = true
    
    int numAdcAverages = static_cast<int>(args[currentIndex++]);

    // Parse Fast DAC Channels
    if (currentIndex >= args.size()) {
      return OperationResult::Failure(
          "Unexpected end of arguments while parsing fast DAC channels");
    }
    int numFastDacChannels = static_cast<int>(args[currentIndex++]);
    if (args.size() < currentIndex + numFastDacChannels * 3) {
      return OperationResult::Failure(
          "Not enough arguments for fast DAC channels");
    }

    int fastDacChannels[numFastDacChannels];
    float fastDacV0s[numFastDacChannels];
    float fastDacVfs[numFastDacChannels];

    for (int i = 0; i < numFastDacChannels; ++i) {
      fastDacChannels[i] = static_cast<int>(args[currentIndex++]);
      fastDacV0s[i] = args[currentIndex++];
      fastDacVfs[i] = args[currentIndex++];
    }

    // Parse Slow DAC Channels
    if (currentIndex >= args.size()) {
      return OperationResult::Failure(
          "Unexpected end of arguments while parsing slow DAC channels");
    }
    int numSlowDacChannels = static_cast<int>(args[currentIndex++]);
    if (args.size() < currentIndex + numSlowDacChannels * 3) {
      return OperationResult::Failure(
          "Not enough arguments for slow DAC channels");
    }

    int slowDacChannels[numSlowDacChannels];
    float slowDacV0s[numSlowDacChannels];
    float slowDacVfs[numSlowDacChannels];

    for (int i = 0; i < numSlowDacChannels; ++i) {
      slowDacChannels[i] = static_cast<int>(args[currentIndex++]);
      slowDacV0s[i] = args[currentIndex++];
      slowDacVfs[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels) {
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int adcChannels[numAdcChannels];
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Reset ADC before starting the ramp
    ADCController::resetToPreviousConversionTimes();

    // Validate total number of DAC channels
    if (numFastDacChannels + numSlowDacChannels != numDacChannels) {
      return OperationResult::Failure(
          "Sum of fast and slow DAC channels does not match numDacChannels");
    }

    float voltageStepSize[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSize[i] = (slowDacVfs[i] - slowDacV0s[i]) / (numStepsSlow - 1);
    }

    float previousVoltageSet[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      previousVoltageSet[i] = slowDacV0s[i];
    }

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    // Iterate over slow steps with optional retrace
    for (int slowStep = 0; slowStep < numStepsSlow && !getStopFlag();
         ++slowStep) {
      // Set slow DAC channels to the current slow step voltages
      #if !defined(__NEW_SHIELD__)
      PeripheralCommsController::beginDacTransaction();
      #endif
      for (int i = 0; i < numSlowDacChannels; ++i) {
        DACController::setVoltageNoTransactionNoLdac(slowDacChannels[i],
                                                     previousVoltageSet[i]);
        previousVoltageSet[i] += voltageStepSize[i];
      }
      DACController::toggleLdac();
      #if !defined(__NEW_SHIELD__)
      PeripheralCommsController::endTransaction();
      #endif

      // Determine ramp direction based on retrace flag
      bool isReverse = false;
      if (snake) {
        isReverse = (slowStep % 2 != 0);  // Reverse on odd slow steps
      }

      // Prepare ramp voltages
      float currentV0s[numFastDacChannels];
      float currentVfs[numFastDacChannels];
      if (isReverse) {
        // Swap V0 and Vf for reverse ramp
        for (int i = 0; i < numFastDacChannels; ++i) {
          currentV0s[i] = fastDacVfs[i];
          currentVfs[i] = fastDacV0s[i];
        }
      } else {
        for (int i = 0; i < numFastDacChannels; ++i) {
          currentV0s[i] = fastDacV0s[i];
          currentVfs[i] = fastDacVfs[i];
        }
      }

      OperationResult ramp1Result = OperationResult::Success();
      OperationResult ramp2Result = OperationResult::Success();

      ramp1Result = God::dacLedBufferRampBase(
          numFastDacChannels, numAdcChannels, numStepsFast, numAdcAverages,
          dac_interval_us, dac_settling_time_us, fastDacChannels, currentV0s,
          currentVfs, adcChannels);
      
      if (retrace && !snake) {
        ramp2Result = God::dacLedBufferRampBase(
          numFastDacChannels, numAdcChannels, numStepsFast, numAdcAverages,
          dac_interval_us, dac_settling_time_us, fastDacChannels, currentVfs,
          currentV0s, adcChannels);
      }

      if (!ramp1Result.isSuccess() && !ramp2Result.isSuccess()) {
        return OperationResult::Failure(ramp1Result.getMessage() + "\n" +
                                        ramp2Result.getMessage());
      } else if (!ramp1Result.isSuccess()) {
        return OperationResult::Failure(ramp1Result.getMessage());
      } else if (!ramp2Result.isSuccess()) {
        return OperationResult::Failure(ramp2Result.getMessage());
      }
    }

    // Reset ADC before setting channels to idle mode
    ADCController::resetToPreviousConversionTimes();

    // Set ADC channels to idle mode
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
    }

    PeripheralCommsController::dataLedOff();

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("2D RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }

};