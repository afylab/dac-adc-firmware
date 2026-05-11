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
  static void setup();

  static void initializeRegistry();

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
      const std::vector<float> &args);


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
  static OperationResult dacLedBufferRamp2D(const std::vector<float> &args);

};