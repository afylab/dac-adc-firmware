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
  static void setup() { initializeRegistry(); }

  static void initializeRegistry() {
    registerMemberFunctionVector(timeSeriesBufferRampWrapper,
                                 "TIME_SERIES_BUFFER_RAMP");
    registerMemberFunctionVector(dacLedBufferRampWrapper,
                                 "DAC_LED_BUFFER_RAMP");
    registerMemberFunction(dacChannelCalibration, "DAC_CH_CAL");
    registerMemberFunctionVector(boxcarAverageRamp, "BOXCAR_BUFFER_RAMP");
  }


  struct BoardUsage {
    uint8_t numBoards;          // how many distinct boards are in use
    std::vector<uint8_t> idx;   // their indexes, sorted (e.g. {0,1})
  };

  static BoardUsage getUsedBoards(const int *adcChannels, int numAdcChannels) {
      std::vector<uint8_t> boards;

      for (int i = 0; i < numAdcChannels; ++i) {
          int ch = adcChannels[i];
          if (ch < 0) continue;          // skip invalid values
          uint8_t board = ch / 4;        // 0‑based board index
          if (std::find(boards.begin(), boards.end(), board) == boards.end())
              boards.push_back(board);   // keep only unique board numbers
      }

      std::sort(boards.begin(), boards.end());

      return BoardUsage{ static_cast<uint8_t>(boards.size()), boards };
  }

  // args:
  // numDacChannels, numAdcChannels, numSteps, dacInterval_us, adcInterval_us,
  // dacchannel0, dacv00, dacvf0, dacchannel1, dacv01, dacvf1, ..., adc0, adc1,
  // adc2, ...
  static OperationResult timeSeriesBufferRampWrapper(
      const std::vector<float>& args) {
    if (args.size() < 5) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    int index = 0;

    int numDacChannels = static_cast<int>(args[index++]);
    int numAdcChannels = static_cast<int>(args[index++]);
    int numSteps = static_cast<int>(args[index++]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[index++]);
    uint32_t adc_interval_us = static_cast<uint32_t>(args[index++]);

    // Check if we have enough arguments for all DAC and ADC channels
    if (args.size() !=
        static_cast<size_t>(index + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Allocate memory for DAC and ADC channel information
    int dacChannels[numDacChannels];
    float dacV0s[numDacChannels];
    float dacVfs[numDacChannels];
    int adcChannels[numAdcChannels];

    // Parse DAC channel information
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[index++]);
      dacV0s[i] = static_cast<float>(args[index++]);
      dacVfs[i] = static_cast<float>(args[index++]);
    }

    // Parse ADC channel information
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[index++]);
    }

    return timeSeriesBufferRampBase(numDacChannels, numAdcChannels, numSteps,
                                    dac_interval_us, adc_interval_us,
                                    dacChannels, dacV0s, dacVfs, adcChannels);
  }

  static OperationResult timeSeriesBufferRampBase(
      int numDacChannels, int numAdcChannels, int numSteps,
      uint32_t dac_interval_us, uint32_t adc_interval_us, int* dacChannels,
      float* dacV0s, float* dacVfs, int* adcChannels) {
    if (adc_interval_us < 1 || dac_interval_us < 1) {
      return OperationResult::Failure("Invalid interval");
    }
    if (numSteps < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }
    if (numDacChannels < 1 || numAdcChannels < 1) {
      return OperationResult::Failure("Invalid number of channels");
    }

    int steps = 0;
    int x = 0;

    const int saved_data_size = numSteps * dac_interval_us / adc_interval_us;

    double voltageStepSize[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSize[i] = (dacVfs[i] - dacV0s[i]) / (numSteps - 1);
    }

    double nextVoltageSet[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      nextVoltageSet[i] = dacV0s[i];
    }

    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, LOW);
    static void (*isrFunctions[])() = {
      TimingUtil::adcSyncISR<0>,
      TimingUtil::adcSyncISR<1>,
      TimingUtil::adcSyncISR<2>,
      TimingUtil::adcSyncISR<3>
    };

    BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);

    int numAdcBoards = boardUsage.numBoards;
    std::vector<uint8_t> adcBoards = boardUsage.idx;

    for (int i = 0; i < numAdcBoards; i++) {
      attachInterrupt(digitalPinToInterrupt(ADCController::getDataReadyPin(adcBoards[i])), isrFunctions[i], FALLING);
    }
    #endif

    uint8_t adcMask = 0u;
    #ifdef __NEW_DAC_ADC__
    for (int i = 0; i < numAdcBoards; i++) {
      adcMask |= 1 << i;
    }
    #else
    adcMask = 1;
    #endif

    TimingUtil::setupTimersTimeSeries(dac_interval_us, adc_interval_us);

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    //set initial DAC voltages
    for (int i = 0; i < numDacChannels; i++) {
      DACController::setVoltageNoTransactionNoLdac(dacChannels[i], dacV0s[i]);
      nextVoltageSet[i] += voltageStepSize[i];
    }

    DACController::toggleLdac();
    steps++;

    for (int i = 0; i < numDacChannels; i++) {
      DACController::setVoltageNoTransactionNoLdac(dacChannels[i], nextVoltageSet[i]);
      nextVoltageSet[i] += voltageStepSize[i];
    }

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::setRDYFN(adcChannels[i]);
      #endif
    }

    while ((x < saved_data_size || steps < numSteps) && !getStopFlag()) {
      if (TimingUtil::dacFlag && steps < numSteps) {
        
        if (steps < numSteps - 1) {
          #if !defined(__NEW_SHIELD__)
          PeripheralCommsController::beginDacTransaction();
          #endif
          for (int i = 0; i < numDacChannels; i++) {
            DACController::setVoltageNoTransactionNoLdac(dacChannels[i], nextVoltageSet[i]);
            nextVoltageSet[i] += voltageStepSize[i];
          }
          #if !defined(__NEW_SHIELD__)
          PeripheralCommsController::endTransaction();
          #endif
        }
        
        steps++;
        TimingUtil::dacFlag = false;
      }
      if (TimingUtil::adcFlag == adcMask) {
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::beginAdcTransaction();
        #endif
        double packets[numAdcChannels];
        for (int i = 0; i < numAdcChannels; i++) {
          packets[i] = ADCController::getVoltageDataNoTransaction(adcChannels[i]);
        }
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::endTransaction();
        #endif
        m4SendVoltage(packets, numAdcChannels);
        x++;
        TimingUtil::adcFlag = 0;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::unsetRDYFN(adcChannels[i]);
      #endif
    }

    #ifdef __NEW_DAC_ADC__
    for (int i = 0; i < numAdcBoards; i++) {
      detachInterrupt(digitalPinToInterrupt(ADCController::getDataReadyPin(adcBoards[i])));
    }
    #endif

    PeripheralCommsController::dataLedOff();

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }

  // args:
  // numDacChannels, numAdcChannels, numSteps, numAdcAverages, dacInterval_us,
  // dacSettlingTime_us, dacchannel0, dacv00, dacvf0, dacchannel1, dacv01,
  // dacvf1, ..., adc0, adc1, adc2, ...
  static OperationResult dacLedBufferRampWrapper(
      const std::vector<float>& args) {
    if (args.size() < 10) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    int index = 0;

    int numDacChannels = static_cast<int>(args[index++]);
    int numAdcChannels = static_cast<int>(args[index++]);
    int numSteps = static_cast<int>(args[index++]);
    int numAdcAverages = static_cast<int>(args[index++]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[index++]);
    uint32_t dac_settling_time_us = static_cast<uint32_t>(args[index++]);

    // Check if we have enough arguments for all DAC and ADC channels
    if (args.size() !=
        static_cast<size_t>(index + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Allocate memory for DAC and ADC channel information
    int dacChannels[numDacChannels];
    float dacV0s[numDacChannels];
    float dacVfs[numDacChannels];
    int adcChannels[numAdcChannels];

    // Parse DAC channel information
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[index++]);
      dacV0s[i] = static_cast<float>(args[index++]);
      dacVfs[i] = static_cast<float>(args[index++]);
    }

    // Parse ADC channel information
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[index++]);
    }

    return dacLedBufferRampBase(numDacChannels, numAdcChannels, numSteps,
                                numAdcAverages, dac_interval_us,
                                dac_settling_time_us, dacChannels, dacV0s,
                                dacVfs, adcChannels);
  }

  static OperationResult dacLedBufferRampBase(
      int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
      uint32_t dac_interval_us, uint32_t dac_settling_time_us, int* dacChannels,
      float* dacV0s, float* dacVfs, int* adcChannels) {
    if (dac_settling_time_us < 1 || dac_interval_us < 1 ||
        dac_settling_time_us >= dac_interval_us) {
      return OperationResult::Failure("Invalid interval or settling time");
    }
    if (numAdcAverages < 1) {
      return OperationResult::Failure("Invalid number of ADC averages");
    }
    if (numSteps < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }
    if (numDacChannels < 1 || numAdcChannels < 1) {
      return OperationResult::Failure("Invalid number of channels");
    }
    
    double packets[numAdcChannels];
    int x = 0;

    double voltageStepSize[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSize[i] = (dacVfs[i] - dacV0s[i]) / (numSteps - 1);
    }

    double nextVoltageSet[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      nextVoltageSet[i] = dacV0s[i];
    }

    double numAdcAveragesInv = 1.0 / static_cast<double>(numAdcAverages);

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, LOW);

    static void (*isrFunctions[])() = {
      TimingUtil::adcSyncISR<0>,
      TimingUtil::adcSyncISR<1>,
      TimingUtil::adcSyncISR<2>,
      TimingUtil::adcSyncISR<3>
    };

    std::vector<uint8_t> boards;

    for (int i = 0; i < numAdcChannels; ++i) {
        int ch = adcChannels[i];
        if (ch < 0) continue;          // skip invalid values
        uint8_t board = ch / 4;        // 0‑based board index
        if (std::find(boards.begin(), boards.end(), board) == boards.end())
            boards.push_back(board);   // keep only unique board numbers
    }

    std::sort(boards.begin(), boards.end());

    int numAdcBoards = boards.size();

    for (int i = 0; i < numAdcBoards; i++) {
      attachInterrupt(digitalPinToInterrupt(ADCController::getDataReadyPin(boards[i])), isrFunctions[i], FALLING);
    }


    #endif

    uint8_t adcMask = 0u;
    #ifdef __NEW_DAC_ADC__
    for (int i = 0; i < numAdcBoards; i++) {
      adcMask |= 1 << i;
    }
    #else
    adcMask = 1;
    #endif

    int dacIncrements = 0;

    // set initial DAC voltages
    for (int i = 0; i < numDacChannels; i++) {
      DACController::setVoltageNoTransactionNoLdac(dacChannels[i], dacV0s[i]);
      nextVoltageSet[i] += voltageStepSize[i];
    }
    DACController::toggleLdac();

    delayMicroseconds(dac_settling_time_us);

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::setRDYFN(adcChannels[i]);
      #endif
    }

    TimingUtil::setupTimersDacLed(dac_interval_us, dac_settling_time_us);

    TimingUtil::dacFlag = false;

    int maxDiff = 0;

    x = 0;
    int diff = 0;
    float cycles = 0.0;
    // int numSteps2 = numSteps - 1;

    uint32_t start = DWT->CYCCNT;
    while (x < numSteps && !getStopFlag()) {
      __WFE();
      if (TimingUtil::dacFlag && ++dacIncrements < numSteps) {
        // cycles =  static_cast<float>(DWT->CYCCNT - start);
        // m4SendFloat(&cycles, 1); // send cycles for debugging
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::beginDacTransaction();
        #endif
        for (int i = 0; i < numDacChannels; i++) {
          DACController::setVoltageNoTransactionNoLdac(dacChannels[i], nextVoltageSet[i]);
          nextVoltageSet[i] += voltageStepSize[i];
        }
        DACController::toggleLdac();
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::endTransaction();
        #endif
        TimingUtil::dacFlag = false;


        // float data[2] = { static_cast<float>(dacIncrements), static_cast<float>(x)};
        
        // m4SendFloat(data, 2); // send cycles for debugging
        // cycles =  static_cast<float>(DWT->CYCCNT - start);
        // m4SendFloat(&cycles, 1); // send cycles for debugging
      }
      if (TimingUtil::adcFlag == adcMask) {
        // uint32_t start = DWT->CYCCNT;
        // cycles =  -1.0 * static_cast<float>(DWT->CYCCNT - start);
        // m4SendFloat(&cycles, 1); // send cycles for debugging
        x++;
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::beginAdcTransaction();
        #endif
        for (int i = 0; i < numAdcChannels; i++) {
          double total = 0.0;
          for (int j = 0; j < numAdcAverages; j++) {
            total += ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
          packets[i] =  total * numAdcAveragesInv;
        }
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::endTransaction();
        #endif
        m4SendVoltage(packets, numAdcChannels);

        diff = dacIncrements - x;
        if (diff < 0) diff = -diff;
        if (diff > maxDiff) {
          maxDiff = diff;
        }

        TimingUtil::adcFlag = 0;
        
        // float data[2] = { static_cast<float>(dacIncrements), static_cast<float>(x)};
        // cycles =  -1.0 * static_cast<float>(DWT->CYCCNT - start);
        // m4SendFloat(&cycles, 1); // send cycles for debugging
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::unsetRDYFN(adcChannels[i]);
      #endif
    }

    #ifdef __NEW_DAC_ADC__
    for (int i = 0; i < numAdcBoards; i++) {
      detachInterrupt(digitalPinToInterrupt(ADCController::getDataReadyPin(boards[i])));
    }
    #endif

    PeripheralCommsController::dataLedOff();

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    if (maxDiff > 1) {
      String message = "DAC and ADC are not synchronized. maxDiff: " + String(maxDiff);
      return OperationResult::Failure(message);
    }

    return OperationResult::Success();
  }

  static OperationResult dacChannelCalibration() {
    CalibrationData calibrationData;
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
      DACController::initialize();
      DACController::setCalibration(i, 0, 1);
      DACController::setVoltage(i, 0);
      delay(1);
      float offsetError = ADCController::getVoltage(i);
      DACController::setCalibration(i, offsetError, 1);
      float voltSet = 9.0;
      DACController::setVoltage(i, voltSet);
      delay(1);
      float gainError = (ADCController::getVoltage(i) - offsetError) / voltSet;
      DACController::setCalibration(i, offsetError, gainError);
      DACController::setVoltage(i, 0);
      calibrationData.offset[i] = offsetError;
      calibrationData.gain[i] = gainError;
    }
    m4SendCalibrationData(calibrationData);
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  // args: numDacChannels, numAdcChannels, numDacSteps,
  // numAdcMeasuresPerDacStep, numAdcAverages, numAdcConversionSkips,
  // adcConversionTime_us, {for each dac channel: dac channel, v0_1, vf_1, v0_2,
  // vf_2}, {for each adc channel: adc channel}
  static OperationResult boxcarAverageRamp(const std::vector<float>& args) {
    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numDacSteps = static_cast<int>(args[currentIndex++]);
    int numAdcMeasuresPerDacStep = static_cast<int>(args[currentIndex++]);
    int numAdcAverages = static_cast<int>(args[currentIndex++]);
    int numAdcConversionSkips = static_cast<int>(args[currentIndex++]);
    uint32_t adcConversionTime_us = static_cast<uint32_t>(args[currentIndex++]);

    int dacChannels[numDacChannels];
    float dacV0_1[numDacChannels];
    float dacVf_1[numDacChannels];
    float dacV0_2[numDacChannels];
    float dacVf_2[numDacChannels];

    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[currentIndex++]);
      dacV0_1[i] = args[currentIndex++];
      dacVf_1[i] = args[currentIndex++];
      dacV0_2[i] = args[currentIndex++];
      dacVf_2[i] = args[currentIndex++];
    }

    int adcChannels[numAdcChannels];

    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    uint32_t actualConversionTime_us = ADCController::presetConversionTime(
        adcChannels[0], adcConversionTime_us, numAdcChannels > 1);
    for (int i = 1; i < numAdcChannels; ++i) {
      ADCController::presetConversionTime(adcChannels[i], adcConversionTime_us,
                                          numAdcChannels > 1);
    }

    uint32_t dacPeriod_us = (numAdcMeasuresPerDacStep + numAdcConversionSkips) *
                            (actualConversionTime_us + 5) * numAdcChannels * numAdcAverages;

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    double voltageStepSizeLow[numDacChannels];
    double voltageStepSizeHigh[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSizeLow[i] =
          (dacVf_1[i] - dacV0_1[i]) / static_cast<double>(numDacSteps - 1);
      voltageStepSizeHigh[i] =
          (dacVf_2[i] - dacV0_2[i]) / static_cast<double>(numDacSteps - 1);
    }

    double previousVoltageSetLow[numDacChannels];
    double previousVoltageSetHigh[numDacChannels];

    for (int i = 0; i < numDacChannels; i++) {
      previousVoltageSetLow[i] = dacV0_1[i];
      previousVoltageSetHigh[i] = dacV0_2[i];
    }

    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, LOW);
    static void (*isrFunctions[])() = {
      TimingUtil::adcSyncISR<0>,
      TimingUtil::adcSyncISR<1>,
      TimingUtil::adcSyncISR<2>,
      TimingUtil::adcSyncISR<3>
    };

    BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);

    int numAdcBoards = boardUsage.numBoards;
    std::vector<uint8_t> adcBoards = boardUsage.idx;

    for (int i = 0; i < numAdcBoards; i++) {
      attachInterrupt(digitalPinToInterrupt(ADCController::getDataReadyPin(adcBoards[i])), isrFunctions[i], FALLING);
    }
    #endif

    uint8_t adcMask = 0u;
    #ifdef __NEW_DAC_ADC__
    for (int i = 0; i < numAdcBoards; i++) {
      adcMask |= 1 << i;
    }
    #else
    adcMask = 1;
    #endif

    int steps = 0;
    int totalSteps = 2 * numDacSteps * numAdcAverages;
    int x = 0;
    int total_data_size = totalSteps * numAdcMeasuresPerDacStep;
    int adcGetsSinceLastDacSet = 0;

    for (int i = 0; i < numAdcChannels; ++i) {
      ADCController::startContinuousConversion(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::setRDYFN(adcChannels[i]);
      #endif
    }

    for (int i = 0; i < numDacChannels; i++) {
      double currentVoltage;
      if (steps % 2 == 0) {
        currentVoltage = previousVoltageSetLow[i];
      } else {
        currentVoltage = previousVoltageSetHigh[i];
      }

      DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                   currentVoltage);
    }

    DACController::toggleLdac();
    steps++;

    TimingUtil::setupTimersTimeSeries(dacPeriod_us, actualConversionTime_us);

    while (x < total_data_size && !getStopFlag()) {
      if (TimingUtil::adcFlag == adcMask) {
        if (adcGetsSinceLastDacSet >= numAdcConversionSkips) {
          #if !defined(__NEW_SHIELD__)
          PeripheralCommsController::beginAdcTransaction();
          #endif
          double packets[numAdcChannels];
          for (int i = 0; i < numAdcChannels; i++) {
            packets[i] = ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
          #if !defined(__NEW_SHIELD__)
          PeripheralCommsController::endTransaction();
          #endif
          m4SendVoltage(packets, numAdcChannels);
          x++;
        }
        adcGetsSinceLastDacSet++;
        TimingUtil::adcFlag = 0;
      }
      if (TimingUtil::dacFlag && steps < totalSteps) {
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::beginDacTransaction();
        #endif
        for (int i = 0; i < numDacChannels; i++) {
          double currentVoltage;
          if (steps % (2 * numAdcAverages) != 0) {
            if (steps % 2 == 0) {
              currentVoltage = previousVoltageSetLow[i];
            } else {
              currentVoltage = previousVoltageSetHigh[i];
            }
          } else if (steps % 2 == 0) {
            previousVoltageSetLow[i] += voltageStepSizeLow[i];
            previousVoltageSetHigh[i] += voltageStepSizeHigh[i];
            currentVoltage = previousVoltageSetLow[i];
          } else {
            previousVoltageSetLow[i] += voltageStepSizeLow[i];
            previousVoltageSetHigh[i] += voltageStepSizeHigh[i];
            currentVoltage = previousVoltageSetHigh[i];
          }
          DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                       currentVoltage);
        }
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::endTransaction();
        #endif
        steps++;
        adcGetsSinceLastDacSet = 0;
        TimingUtil::dacFlag = false;
        TIM8->CNT = 0;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::unsetRDYFN(adcChannels[i]);
      #endif
    }

    PeripheralCommsController::dataLedOff();

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }
};
