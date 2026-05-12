#pragma once

#include <Arduino.h>
#include <Peripherals/ADC/ADCController.h>
#include <Peripherals/DAC/DACController.h>

#include "Config.h"
#include "Utils/TimingUtil.h"
#include "Utils/shared_memory.h"
#include "unordered_set"

#include <vector>
#include <algorithm>

class God {
 public:
  static void setup();

  static void initializeRegistry();

  static OperationResult initialize();

  static OperationResult hardResetCalibrationToDefaults();


  struct BoardUsage {
    uint8_t numBoards;          // how many distinct boards are in use
    std::vector<uint8_t> idx;   // their indexes, sorted (e.g. {0,1})
  };

  static BoardUsage getUsedBoards(const int *adcChannels, int numAdcChannels);

  static OperationResult timeSeriesAdcRead(const std::vector<float>& args);

  // args:
  // numDacChannels, numAdcChannels, numSteps, dacInterval_us, adcInterval_us,
  // dacchannel0, dacv00, dacvf0, dacchannel1, dacv01, dacvf1, ..., adc0, adc1,
  // adc2, ...
  static OperationResult timeSeriesBufferRampBase(
      const std::vector<float>& args);

  static OperationResult prepareTimeSeriesBufferRampHardware(
      int numAdcChannels, uint32_t dac_interval_us, uint32_t adc_interval_us,
      int* adcChannels, uint8_t& adcMask, BoardUsage& boardUsage);

  static OperationResult runPreparedTimeSeriesBufferRamp(
      int numDacChannels, int numAdcChannels, int numSteps,
      uint32_t dac_interval_us, uint32_t adc_interval_us, int* dacChannels,
      float* dacV0s, float* dacVfs, int* adcChannels, uint8_t adcMask);

  static void cleanupTimeSeriesBufferRampHardware(
      int numAdcChannels, int* adcChannels, const BoardUsage& boardUsage);

  // args:
  // numDacChannels, numAdcChannels, numSteps, numAdcAverages, dacInterval_us,
  // dacSettlingTime_us, dacchannel0, dacv00, dacvf0, dacchannel1, dacv01,
  // dacvf1, ..., adc0, adc1, adc2, ...
  static OperationResult dacLedBufferRampBase(
      const std::vector<float>& args);

  static OperationResult prepareDacLedBufferRampHardware(
      int numAdcChannels, int numAdcAverages, uint32_t dac_interval_us,
      uint32_t dac_settling_time_us, int* adcChannels, uint8_t& adcMask,
      BoardUsage& boardUsage);

  static OperationResult runPreparedDacLedBufferRamp(
      int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
      int* dacChannels, float* dacV0s, float* dacVfs, int* adcChannels,
      uint8_t adcMask);

  static void cleanupDacLedBufferRampHardware(
      int numAdcChannels, int* adcChannels, const BoardUsage& boardUsage);

  static OperationResult OwenRampWrapper(std::vector<float> args);

  static OperationResult OwenRampBase(
    int numDacChannels, int numAdcChannels, int numLoops, int numDacStepsPerLoop, int numAdcAverages,
    uint32_t dac_interval_us, int* dacChannels,
    float** dacVoltageLists, int* adcChannels, int specialIndex, int specialWidth, int numStepsPerSpecialRamp, float* specialDacV0s, float* specialDacVfs);


  static OperationResult AWGBufferRampWrapper(std::vector<float> args);

  static OperationResult AWGDacOnlyRampBase(
      int numDacChannels,
      int numSteps,
      uint32_t dac_interval_us,
      int* dacChannels,
      const float* channelMajorVoltages);

  // AWG_WITH_ADC: AWG waveform with ADC reading at each step
  // Format: AWG_WITH_ADC,dacN,adcN,numSteps,dac_interval_us,numCycles,dacChannels...,adcChannels...,voltages...
  // Voltages are channel-major: all points for DAC0, then all for DAC1, etc.
  static OperationResult AWGWithADCWrapper(std::vector<float> args);

  static OperationResult AWGWithADCBase(
      int numDacChannels, int numAdcChannels, int numSteps,
      uint32_t dac_interval_us, int numCycles,
      int* dacChannels, int* adcChannels, const float* channelMajorVoltages);


  static OperationResult AWGBufferRampBase(
      int numDacChannels, int numAdcChannels, int numLoops, int numDacStepsPerLoop, int numAdcAverages,
      uint32_t dac_interval_us, int* dacChannels,
      float** dacVoltageLists, int* adcChannels);






  static OperationResult dacChannelCalibration();

  // args: numDacChannels, numAdcChannels, numDacSteps,
  // numAdcMeasuresPerDacStep, numAdcAverages, numAdcConversionSkips,
  // adcConversionTime_us, {for each dac channel: dac channel, v0_1, vf_1, v0_2,
  // vf_2}, {for each adc channel: adc channel}
  static OperationResult boxcarAverageRamp(const std::vector<float>& args);
};
