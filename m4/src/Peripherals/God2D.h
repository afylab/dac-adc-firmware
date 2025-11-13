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
  // [dacChannelID] * numDacChannels,
  // [startPoint] * numDacChannels,
  // [fastAxisVector] * numDacChannels,
  // [slowAxisVector] * numDacChannels,
  // [adcChannelID] * numAdcChannels
  //
  // The fast/slow axis vectors define a 2D plane in the N-dimensional DAC phase space.
  // Position(s,f) = startPoint + s*slowAxisVector + f*fastAxisVector where s,f ∈ [0,1]
  // This allows probing arbitrary 2D planar subspaces anywhere in the full DAC phase space.
  static OperationResult timeSeriesBufferRamp2D(
      const std::vector<float> &args) {
    // Minimum required arguments:
    // 8 initial params + numDacChannels + 3*numDacChannels vectors + numAdcChannels
    if (args.size() < 8) {
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

    // Validate we have enough arguments
    if (args.size() < currentIndex + numDacChannels + 3 * numDacChannels + numAdcChannels) {
      return OperationResult::Failure("Insufficient arguments for 2D ramp");
    }

    // Parse DAC channel IDs
    int dacChannels[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Parse start point
    float startPoint[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      startPoint[i] = args[currentIndex++];
    }

    // Parse fast axis vector
    float fastAxisVector[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      fastAxisVector[i] = args[currentIndex++];
    }

    // Parse slow axis vector
    float slowAxisVector[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      slowAxisVector[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels) {
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int adcChannels[numAdcChannels];
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Check voltage bounds for all four corners of the 2D scan rectangle (both calibrated bounds AND global limits)
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      
      // Four corners: startPoint, startPoint+fast, startPoint+slow, startPoint+fast+slow
      float corner1 = startPoint[i];
      float corner2 = startPoint[i] + fastAxisVector[i];
      float corner3 = startPoint[i] + slowAxisVector[i];
      float corner4 = startPoint[i] + fastAxisVector[i] + slowAxisVector[i];
      
      float minVoltage = min(min(corner1, corner2), min(corner3, corner4));
      float maxVoltage = max(max(corner1, corner2), max(corner3, corner4));
      
      if (minVoltage < lowerBound || maxVoltage > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " 2D scan range [" + String(minVoltage, 6) + 
                                        ", " + String(maxVoltage, 6) + 
                                        "]V exceeds bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
    }

    // Reset ADC before starting the ramp
    ADCController::resetToPreviousConversionTimes();

    // Calculate slow axis step sizes for each DAC channel
    float slowStepSize[numDacChannels];
    for (int i = 0; i < numDacChannels; i++) {
      slowStepSize[i] = slowAxisVector[i] / (numStepsSlow - 1);
    }

    // Track current position in phase space (starting at startPoint)
    float currentSlowPosition[numDacChannels];
    for (int i = 0; i < numDacChannels; i++) {
      currentSlowPosition[i] = startPoint[i];
    }

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    // Iterate over slow steps
    for (int slowStep = 0; slowStep < numStepsSlow && !getStopFlag(); ++slowStep) {
      // Determine ramp direction for snake mode
      bool isReverse = false;
      if (snake) {
        isReverse = (slowStep % 2 != 0);  // Reverse on odd slow steps
      }

      // Calculate start and end voltages for fast axis ramp
      // Position = currentSlowPosition + t * fastAxisVector (where t goes from 0 to 1)
      float fastV0s[numDacChannels];
      float fastVfs[numDacChannels];
      
      for (int i = 0; i < numDacChannels; ++i) {
        if (isReverse) {
          fastV0s[i] = currentSlowPosition[i] + fastAxisVector[i];
          fastVfs[i] = currentSlowPosition[i];
        } else {
          fastV0s[i] = currentSlowPosition[i];
          fastVfs[i] = currentSlowPosition[i] + fastAxisVector[i];
        }
      }

      // Execute fast axis ramp
      OperationResult ramp1Result = God::timeSeriesBufferRampBase(
          numDacChannels, numAdcChannels, numStepsFast, dac_interval_us,
          adc_interval_us, dacChannels, fastV0s, fastVfs, adcChannels);

      // Execute retrace if requested (and not in snake mode)
      OperationResult ramp2Result = OperationResult::Success();
      if (retrace && !snake) {
        ramp2Result = God::timeSeriesBufferRampBase(
            numDacChannels, numAdcChannels, numStepsFast, dac_interval_us,
            adc_interval_us, dacChannels, fastVfs, fastV0s, adcChannels);
      }

      // Check for errors
      if (!ramp1Result.isSuccess() && !ramp2Result.isSuccess()) {
        return OperationResult::Failure(ramp1Result.getMessage() + "\n" +
                                        ramp2Result.getMessage());
      } else if (!ramp1Result.isSuccess()) {
        return OperationResult::Failure(ramp1Result.getMessage());
      } else if (!ramp2Result.isSuccess()) {
        return OperationResult::Failure(ramp2Result.getMessage());
      }

      // Advance along slow axis
      for (int i = 0; i < numDacChannels; ++i) {
        currentSlowPosition[i] += slowStepSize[i];
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
  // numAdcAverages,
  // [dacChannelID] * numDacChannels,
  // [startPoint] * numDacChannels,
  // [fastAxisVector] * numDacChannels,
  // [slowAxisVector] * numDacChannels,
  // [adcChannelID] * numAdcChannels
  //
  // The fast/slow axis vectors define a 2D plane in the N-dimensional DAC phase space.
  // Position(s,f) = startPoint + s*slowAxisVector + f*fastAxisVector where s,f ∈ [0,1]
  // This allows probing arbitrary 2D planar subspaces anywhere in the full DAC phase space.
  static OperationResult dacLedBufferRamp2D(const std::vector<float> &args) {
    // Minimum required arguments:
    // 9 initial params + numDacChannels + 3*numDacChannels vectors + numAdcChannels
    if (args.size() < 9) {
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

    // Validate we have enough arguments
    if (args.size() < currentIndex + numDacChannels + 3 * numDacChannels + numAdcChannels) {
      return OperationResult::Failure("Insufficient arguments for 2D ramp");
    }

    // Parse DAC channel IDs
    int dacChannels[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Parse start point
    float startPoint[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      startPoint[i] = args[currentIndex++];
    }

    // Parse fast axis vector
    float fastAxisVector[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      fastAxisVector[i] = args[currentIndex++];
    }

    // Parse slow axis vector
    float slowAxisVector[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      slowAxisVector[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels) {
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int adcChannels[numAdcChannels];
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Check voltage bounds for all four corners of the 2D scan rectangle (both calibrated bounds AND global limits)
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = max(DACController::getLowerBound(ch), DACLimits::lower_voltage_limit[ch]);
      float upperBound = min(DACController::getUpperBound(ch), DACLimits::upper_voltage_limit[ch]);
      
      // Four corners: startPoint, startPoint+fast, startPoint+slow, startPoint+fast+slow
      float corner1 = startPoint[i];
      float corner2 = startPoint[i] + fastAxisVector[i];
      float corner3 = startPoint[i] + slowAxisVector[i];
      float corner4 = startPoint[i] + fastAxisVector[i] + slowAxisVector[i];
      
      float minVoltage = min(min(corner1, corner2), min(corner3, corner4));
      float maxVoltage = max(max(corner1, corner2), max(corner3, corner4));
      
      if (minVoltage < lowerBound || maxVoltage > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " 2D scan range [" + String(minVoltage, 6) + 
                                        ", " + String(maxVoltage, 6) + 
                                        "]V exceeds bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
    }

    // Reset ADC before starting the ramp
    ADCController::resetToPreviousConversionTimes();

    // Calculate slow axis step sizes for each DAC channel
    float slowStepSize[numDacChannels];
    for (int i = 0; i < numDacChannels; i++) {
      slowStepSize[i] = slowAxisVector[i] / (numStepsSlow - 1);
    }

    // Track current position in phase space (starting at startPoint)
    float currentSlowPosition[numDacChannels];
    for (int i = 0; i < numDacChannels; i++) {
      currentSlowPosition[i] = startPoint[i];
    }

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    // Iterate over slow steps
    for (int slowStep = 0; slowStep < numStepsSlow && !getStopFlag(); ++slowStep) {
      // Determine ramp direction for snake mode
      bool isReverse = false;
      if (snake) {
        isReverse = (slowStep % 2 != 0);  // Reverse on odd slow steps
      }

      // Calculate start and end voltages for fast axis ramp
      // Position = currentSlowPosition + t * fastAxisVector (where t goes from 0 to 1)
      float fastV0s[numDacChannels];
      float fastVfs[numDacChannels];
      
      for (int i = 0; i < numDacChannels; ++i) {
        if (isReverse) {
          fastV0s[i] = currentSlowPosition[i] + fastAxisVector[i];
          fastVfs[i] = currentSlowPosition[i];
        } else {
          fastV0s[i] = currentSlowPosition[i];
          fastVfs[i] = currentSlowPosition[i] + fastAxisVector[i];
        }
      }

      // Execute fast axis ramp
      OperationResult ramp1Result = God::dacLedBufferRampBase(
          numDacChannels, numAdcChannels, numStepsFast, numAdcAverages,
          dac_interval_us, dac_settling_time_us, dacChannels, fastV0s,
          fastVfs, adcChannels);

      // Execute retrace if requested (and not in snake mode)
      OperationResult ramp2Result = OperationResult::Success();
      if (retrace && !snake) {
        ramp2Result = God::dacLedBufferRampBase(
            numDacChannels, numAdcChannels, numStepsFast, numAdcAverages,
            dac_interval_us, dac_settling_time_us, dacChannels, fastVfs,
            fastV0s, adcChannels);
      }

      // Check for errors
      if (!ramp1Result.isSuccess() && !ramp2Result.isSuccess()) {
        return OperationResult::Failure(ramp1Result.getMessage() + "\n" +
                                        ramp2Result.getMessage());
      } else if (!ramp1Result.isSuccess()) {
        return OperationResult::Failure(ramp1Result.getMessage());
      } else if (!ramp2Result.isSuccess()) {
        return OperationResult::Failure(ramp2Result.getMessage());
      }

      // Advance along slow axis
      for (int i = 0; i < numDacChannels; ++i) {
        currentSlowPosition[i] += slowStepSize[i];
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