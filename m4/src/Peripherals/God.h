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
    registerMemberFunction(initialize, "INITIALIZE");
    registerMemberFunction(initialize, "INIT");
    registerMemberFunction(initialize, "INNIT");
    registerMemberFunctionVector(timeSeriesBufferRampWrapper, "TIME_SERIES_BUFFER_RAMP");
    registerMemberFunctionVector(dacLedBufferRampWrapper, "DAC_LED_BUFFER_RAMP");
    registerMemberFunctionVector(AWGBufferRampWrapper, "AWG_BUFFER_RAMP");
    registerMemberFunctionVector(timeSeriesAdcRead, "TIME_SERIES_ADC_READ");
    registerMemberFunction(dacChannelCalibration, "DAC_CH_CAL");
    registerMemberFunctionVector(boxcarAverageRamp, "BOXCAR_BUFFER_RAMP");
    registerMemberFunction(hardResetCalibrationToDefaults, "HARD_RESET_CALIBRATION");
  }

  inline static OperationResult initialize() {
    DACController::initialize();
    ADCController::initialize();
    return OperationResult::Success("INITIALIZATION COMPLETE");
  }

  inline static OperationResult hardResetCalibrationToDefaults() {
    CalibrationData calibrationData;
    m4ReceiveCalibrationData(calibrationData);
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
      calibrationData.gain[i] = 1.0f;
      calibrationData.offset[i] = 0.0f;
      calibrationData.adc_offset[i] = 0x800000; // Default ADC offset
      calibrationData.adc_gain[i] = 0x200000; // Default ADC gain
    }
    calibrationData.adcCalibrated = false;
    m4SendCalibrationData(calibrationData);

    return OperationResult::Success("Calibration data reset to defaults");
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

  inline static OperationResult timeSeriesAdcRead(const std::vector<float>& args) {
    /**************************************************************************/
    // args: num_channels, channel_indexes, conversion_time, total_duration_us
    // ex: TIME_SERIES_ADC_READ 2,0,1,1000000
    //
    // Function takes a vector of floats as input, where:
    // args[0] = number of ADC channels = numAdcChannels
    // args[1] to args[numAdcChannels] = ADC channel indexes
    // args[numAdcChannels + 1] = conversion time in microseconds
    // args[numAdcChannels + 2] = total duration in microseconds
    /**************************************************************************/

    // Check if we have enough arguments
    if (args.size() < 4) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    // Check if the first argument is a valid number of ADC channels
    int numAdcChannels = static_cast<int>(args[0]);
    if (args.size() != static_cast<size_t>(numAdcChannels + 3)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Setting up a vector of ADC channels from arguments directly after numAdcChannels
    std::vector<int> adcChannels_vec;
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels_vec.push_back(static_cast<int>(args[i + 1]));
    }
    int* adcChannels = adcChannels_vec.data();

    // Check to see if the total duration is valid (needs to be at least 82 microseconds)
    uint32_t conversion_time = static_cast<uint32_t>(args[numAdcChannels + 1]);
    uint32_t totalDuration = static_cast<uint32_t>(args[numAdcChannels + 2]);
    if (totalDuration < 82) {
      return OperationResult::Failure("Invalid total duration");
    }

    //Set conversion time for each channel (same conversion time for all channels)
    float realConversionTime = 0;
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::setConversionTime(adcChannels[i], conversion_time);
    }
    realConversionTime = ADCController::getConversionTimeFloat(adcChannels[0]);

    // Determine the maximum number of independent ADCs used
    int adc_usage[4] = {0, 0, 0, 0};
    for (int i = 0; i < numAdcChannels; ++i) {
      int ch = adcChannels[i];
      if (ch < 0) continue;          // skip invalid values
      uint8_t board = ch / 4;        // 0‑based board index
      adc_usage[board]++;
    }

    int max_indep_ADCs = *std::max_element(adc_usage, adc_usage + 4);
    
    //set sampling rate based on +5% conversion time and the number of ADC channels
    const double sample_rate_float = max_indep_ADCs * realConversionTime * 1.5f;
    const int sample_rate = static_cast<int>(sample_rate_float);

    // Calculate the number of data points to save
    const int saved_data_size = totalDuration / sample_rate;

    // Send the number of bytes to expect before starting the ADC read
    m4SendVoltage(&sample_rate_float, 1);

    // Toggle stop flag and turn on data LED
    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, LOW); //Set the sync pin low to prevent ADCs from triggering
    static void (*isrFunctions[])() = { // Array of ISR functions for each ADC board
      TimingUtil::adcSyncISR<0>,
      TimingUtil::adcSyncISR<1>,
      TimingUtil::adcSyncISR<2>,
      TimingUtil::adcSyncISR<3>
    };

    BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);
    int numAdcBoards = boardUsage.numBoards;
    std::vector<uint8_t> adcBoards = boardUsage.idx;

    // Attach interrupts for each ADC board's data ready pin
    for (int i = 0; i < numAdcBoards; i++) {
      attachInterrupt(digitalPinToInterrupt(ADCController::getDataReadyPin(adcBoards[i])), isrFunctions[i], FALLING);
    }
    #endif

    // reset all ADCs before ramp
    ADCController::resetToPreviousConversionTimes();

    // Start continous conversion for each ADC channel
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::setRDYFN(adcChannels[i]);
      #endif
    }

    // initialize ADC mask
    uint8_t adcMask = 0u;
    #ifdef __NEW_DAC_ADC__
    for (int i = 0; i < numAdcBoards; i++) {
      adcMask |= 1 << i;
    }
    #else
    adcMask = 1;
    #endif

    // Set up timers for ADC sampling
    TimingUtil::setupTimersOnlyADC(sample_rate);

    // initialize loop counter
    int x = 0;

    //Begin main loop
    while (x < saved_data_size && !getStopFlag()) {
      __WFE(); // Wait for event (WFE) to reduce CPU usage
      if (TimingUtil::adcFlag == adcMask) {
        double packets[numAdcChannels];
        for (int i = 0; i < numAdcChannels; i++) {
          double v = ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          packets[i] = v;
        }
        m4SendVoltage(packets, numAdcChannels);
        x++;
        TimingUtil::adcFlag = 0;

        #ifdef __NEW_DAC_ADC__
        digitalWrite(adc_sync, LOW);
        #endif
      }
    }

    // Clean up after the loop
    //Disable ADC timer interrupt
    TimingUtil::disableAdcInterrupt();

    // Set the ADCs into idle mode and unset the readyfnc bit (this allows single channel ADC conversions -- ready flag goes high after any ADC has unread data)
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::unsetRDYFN(adcChannels[i]);
      #endif
    }

    // reset all ADCs after ramp
    ADCController::resetToPreviousConversionTimes();

    //Detach hardware interrupt for ready pin on ADCs
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

    // Check voltage bounds before executing ramp
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      
      if (dacV0s[i] < lowerBound || dacV0s[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " start voltage " + String(dacV0s[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacVfs[i] < lowerBound || dacVfs[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " end voltage " + String(dacVfs[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
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

    //prevent user from setting the DAC update rate too fast
    int adc_usage[4] = {0, 0, 0, 0};
    for (int i = 0; i < numAdcChannels; ++i) {
      int ch = adcChannels[i];
      if (ch < 0) continue;          // skip invalid values
      uint8_t board = ch / 4;        // 0‑based board index
      adc_usage[board]++;
    }

    int max_indep_ADCs = *std::max_element(adc_usage, adc_usage + 4);

    // TODO: put in some limits for the time series ramp related to ADC conversion time scaling
    //check to see if buffer ramp is compatible with the current ADC configuration
    float convTimeSum[4] = {0.0, 0.0, 0.0, 0.0};
    uint8_t board_num = 0;
    int chNum = 0;
    for (int i = 0; i < numAdcChannels; i++) {
      chNum = adcChannels[i];
      board_num = chNum / 4; // 0-based board index
      convTimeSum[board_num] += ADCController::getConversionTimeFloat(adcChannels[i]);
    }
    float maxConvTime = *std::max_element(std::begin(convTimeSum), std::end(convTimeSum));
    if(maxConvTime + 300 >= adc_interval_us) {
      return OperationResult::Failure("ADC delay time is too short, please increase it");
    }

    // check dac update frequency compatibility 
    if(max_indep_ADCs <= 0) {
      return OperationResult::Failure("No ADC channels provided");
    } else if (max_indep_ADCs == 1) {
      if (dac_interval_us < 60) {
        return OperationResult::Failure("DAC interval too short, please increase it");
      }
    } else if (max_indep_ADCs == 2) {
      if (dac_interval_us < 120) {
        return OperationResult::Failure("DAC interval too short, please increase it");
      }
    } else if (max_indep_ADCs == 3) {
      if (dac_interval_us < 180) {
        return OperationResult::Failure("DAC interval too short, please increase it");
      }
    } else if (max_indep_ADCs == 4) {
      if (dac_interval_us < 250) {
          return OperationResult::Failure("DAC interval too short, please increase it");
      }
    }

    BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);
    int numAdcBoards = boardUsage.numBoards;
    std::vector<uint8_t> adcBoards = boardUsage.idx;

    for (int i = 0; i < numAdcBoards; i++) {
      attachInterrupt(digitalPinToInterrupt(ADCController::getDataReadyPin(adcBoards[i])), isrFunctions[i], FALLING);
    }
    #endif

    // reset all ADCs before ramp
    ADCController::resetToPreviousConversionTimes();

    uint8_t adcMask = 0u;
    #ifdef __NEW_DAC_ADC__
    for (int i = 0; i < numAdcBoards; i++) {
      adcMask |= 1 << i;
    }
    #else
    adcMask = 1;
    #endif

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    //set initial DAC voltages
    for (int i = 0; i < numDacChannels; i++) {
      DACController::setVoltageNoTransactionNoLdac(dacChannels[i], dacV0s[i]);
      nextVoltageSet[i] += voltageStepSize[i];
    }

    steps++;


    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::setRDYFN(adcChannels[i]);
      #endif
    }

    TimingUtil::setupTimersTimeSeries(dac_interval_us, adc_interval_us);

    //float cycles = 0.0;
    //uint32_t start = DWT->CYCCNT;

    while ((x < saved_data_size || steps < numSteps) && !getStopFlag()) {
      __WFE(); // Wait for event, this will block until an interrupt occurs
      if (TimingUtil::dacFlag && steps < numSteps) {
        if (steps < numSteps - 1) {
          #if !defined(__NEW_SHIELD__)
          PeripheralCommsController::beginDacTransaction();
          #endif
          for (int i = 0; i < numDacChannels; i++) {
            //start = DWT->CYCCNT; // reset cycle counter
            DACController::setVoltageNoTransactionNoLdac(dacChannels[i], nextVoltageSet[i]);
            //cycles = static_cast<float>(DWT->CYCCNT - start);
            nextVoltageSet[i] += voltageStepSize[i];
          }
          #if !defined(__NEW_SHIELD__)
          PeripheralCommsController::endTransaction();
          #endif
        }
        
        steps++;
        TimingUtil::dacFlag = false;
        //m4SendFloat(&cycles, 1); // send cycles for debugging
      }
      if (TimingUtil::adcFlag == adcMask && x < saved_data_size) {
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

        #ifdef __NEW_DAC_ADC__
        digitalWrite(adc_sync, LOW);
        #endif

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

    // reset all ADCs after ramp
    ADCController::resetToPreviousConversionTimes();

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

    // Check voltage bounds before executing ramp
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      
      if (dacV0s[i] < lowerBound || dacV0s[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " start voltage " + String(dacV0s[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacVfs[i] < lowerBound || dacVfs[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " end voltage " + String(dacVfs[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
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

    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

    ADCController::resetToPreviousConversionTimes();

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

    //check to see if buffer ramp is compatible with the current ADC configuration
    float convTimeSum[4] = {0.0, 0.0, 0.0, 0.0};
    uint8_t board_num = 0;
    int chNum = 0;
    for (int i = 0; i < numAdcChannels; i++) {
      chNum = adcChannels[i];
      board_num = chNum / 4; // 0-based board index
      convTimeSum[board_num] += ADCController::getConversionTimeFloat(adcChannels[i]);
    }
    float maxConvTime = *std::max_element(std::begin(convTimeSum), std::end(convTimeSum));
    if(maxConvTime*numAdcAverages + dac_settling_time_us + 180 >= dac_interval_us) {
      return OperationResult::Failure("DAC Interval is too short for specified ADC conversion time, please increase it");
    }
    //sums up conversion times for each ADC on each board and takes the maximum, if then maxConvTime + settling time + 50us 
    //is greater than the DAC interval, then the buffer ramp is not compatible with the current ADC configuration
    //the extra 180us is to account for any additional timing delays

    //We will also throw an error if the settling time is too short:
    if (dac_settling_time_us < 100) {
      return OperationResult::Failure("DAC settling time is too short, please increase it");
    }

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
    dacIncrements++;

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
    
    //float cycles = 0.0;
    //uint32_t start = DWT->CYCCNT;

    while (x < numSteps && !getStopFlag()) {
      __WFE();
      //cycles =  static_cast<float>(DWT->CYCCNT - start);
      //m4SendFloat(&cycles, 1); // send cycles for debugging

      if (TimingUtil::dacFlag && dacIncrements < numSteps) {
        #ifdef __NEW_SHIELD__
        digitalWrite(GPIO_0, LOW);
        #endif
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
        TimingUtil::dacFlag = false;
        dacIncrements++;

        // float data[2] = { static_cast<float>(dacIncrements), static_cast<float>(x)};
        // m4SendFloat(data, 2);

      }
      if (TimingUtil::adcFlag == adcMask) {
        #ifdef __NEW_SHIELD__
        digitalWrite(GPIO_0, HIGH);
        #endif
        // uint32_t start = DWT->CYCCNT;
        // cycles =  -1.0 * static_cast<float>(DWT->CYCCNT - start);
        // m4SendFloat(&cycles, 1);
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
        #ifdef __NEW_DAC_ADC__
        digitalWrite(adc_sync, LOW);
        #endif
        TimingUtil::adcFlag = 0;
        
        // float data[2] = { static_cast<float>(dacIncrements), static_cast<float>(x)};
        // cycles =  -1.0 * static_cast<float>(DWT->CYCCNT - start);
        // m4SendFloat(&cycles, 1);
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

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

    ADCController::resetToPreviousConversionTimes();

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


  static OperationResult AWGBufferRampWrapper(std::vector<float> args) {
    // Expected argument order:
    // [numDacChannels, numAdcChannels, numLoops, numDacStepsPerLoop, numAdcAverages, dac_interval_us, dac_settling_time_us, <dacChannels...>, <adcChannels...>, <dacVoltageLists...>]
    // The number of DAC and ADC channels determines how many channel indices and voltage lists to expect.

    if (args.size() < 7) {
      return OperationResult::Failure("Insufficient arguments for AWGBufferRampWrapper");
    }

    int idx = 0;
    int numDacChannels = static_cast<int>(args[idx++]);
    int numAdcChannels = static_cast<int>(args[idx++]);
    int numLoops = static_cast<int>(args[idx++]);
    int numDacStepsPerLoop = static_cast<int>(args[idx++]);
    int numAdcAverages = static_cast<int>(args[idx++]);
    uint32_t dac_interval_us = static_cast<uint32_t>(args[idx++]);

    // Check for valid channel counts
    if (numDacChannels < 1 || numAdcChannels < 1 || numLoops < 1 || numDacStepsPerLoop < 1 || numAdcAverages < 1) {
      return OperationResult::Failure("Invalid channel or loop/step/average count");
    }

    // Check if enough arguments for channel indices
    if (args.size() < idx + numDacChannels + numAdcChannels) {
      return OperationResult::Failure("Insufficient arguments for channel indices");
    }

    // Parse DAC channel indices
    int dacChannels[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[idx++]);
    }

    // Parse ADC channel indices
    int adcChannels[numAdcChannels];
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[idx++]);
    }

    // Now, expect numDacChannels blocks of numDacStepsPerLoop floats for the DAC voltage lists
    int expectedVoltageListSize = numDacChannels * numDacStepsPerLoop;
    if (args.size() < idx + expectedVoltageListSize) {
      return OperationResult::Failure("Insufficient arguments for DAC voltage lists");
    }

    // Allocate and fill DAC voltage lists
    float* dacVoltageLists[numDacChannels];
    for (int i = 0; i < numDacChannels; ++i) {
      dacVoltageLists[i] = new float[numDacStepsPerLoop];
      for (int j = 0; j < numDacStepsPerLoop; ++j) {
        dacVoltageLists[i][j] = args[idx++];
      }
    }

    // Call the base function
    OperationResult result = AWGBufferRampBase(
      numDacChannels, numAdcChannels, numLoops, numDacStepsPerLoop, numAdcAverages,
      dac_interval_us, dacChannels, dacVoltageLists, adcChannels
    );

    // Clean up allocated memory
    for (int i = 0; i < numDacChannels; ++i) {
      delete[] dacVoltageLists[i];
    }

    return result;
  }


  static OperationResult AWGBufferRampBase(
      int numDacChannels, int numAdcChannels, int numLoops, int numDacStepsPerLoop, int numAdcAverages,
      uint32_t dac_interval_us, int* dacChannels,
      float** dacVoltageLists, int* adcChannels) {
    if (dac_interval_us < 1) {
      return OperationResult::Failure("Invalid interval or settling time");
    }
    if (numAdcAverages < 1) {
      return OperationResult::Failure("Invalid number of ADC averages");
    }
    if (numLoops < 1 || numDacStepsPerLoop < 1) {
      return OperationResult::Failure("Invalid number of loops or steps per loop");
    }
    if (numDacChannels < 1 || numAdcChannels < 1) {
      return OperationResult::Failure("Invalid number of channels");
    }

    // Check voltage bounds before executing ramp
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      
      for (int j = 0; j < numDacStepsPerLoop; j++) {
        float voltage = dacVoltageLists[i][j];
        if (voltage < lowerBound || voltage > upperBound) {
          return OperationResult::Failure("DAC " + String(ch) + 
                                          " voltage[" + String(j) + "] = " + String(voltage, 6) + 
                                          "V out of bounds [" + String(lowerBound, 6) + 
                                          ", " + String(upperBound, 6) + "]");
        }
      }
    }
    
    double packets[numAdcChannels];
    double numAdcAveragesInv = 1.0 / static_cast<double>(numAdcAverages);

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    ADCController::resetToPreviousConversionTimes();

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

    //check to see if buffer ramp is compatible with the current ADC configuration
    float convTimeSum[4] = {0.0, 0.0, 0.0, 0.0};
    uint8_t board_num = 0;
    int chNum = 0;
    for (int i = 0; i < numAdcChannels; i++) {
      chNum = adcChannels[i];
      board_num = chNum / 4; // 0-based board index
      convTimeSum[board_num] += ADCController::getConversionTimeFloat(adcChannels[i]);
    }
    float maxConvTime = *std::max_element(std::begin(convTimeSum), std::end(convTimeSum));
    uint32_t totalDacSweepTime = numDacStepsPerLoop * dac_interval_us;
    if(maxConvTime*numAdcAverages + 180 >= totalDacSweepTime) {
      return OperationResult::Failure("DAC sweep time is too short for specified ADC conversion time, please increase dac_interval_us or reduce numDacStepsPerLoop");
    }

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

    // Initialize timing flags
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

    // Track current position in voltage lists and loop
    int currentLoop = 0;
    int totalDacSteps = numLoops * numDacStepsPerLoop;
    int currentDacStep = 0;
    int currentAdcReads = 0;

    // Set initial DAC voltages (first step of first loop)
    for (int i = 0; i < numDacChannels; i++) {
      DACController::setVoltageNoTransactionNoLdac(dacChannels[i], dacVoltageLists[i][0]);
    }
    currentDacStep++;

    // Start ADC continuous conversion
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      #ifdef __NEW_DAC_ADC__
      ADCController::setRDYFN(adcChannels[i]);
      #endif
    }

    // Setup timers for DAC and ADC events
    TimingUtil::setupTimerOnlyDac(dac_interval_us);
    TimingUtil::dacFlag = false;

    bool done = false;

    // Main event loop using interrupt-based timing
    while (currentLoop < numLoops && !getStopFlag()) {
      __WFE(); // Wait for event (interrupt)
      
      // Handle DAC flag - time to set next DAC voltage
      if (TimingUtil::dacFlag && currentDacStep < totalDacSteps) {
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::beginDacTransaction();
        #endif
        for (int i = 0; i < numDacChannels; i++) {
          float voltage = dacVoltageLists[i][currentDacStep];
          DACController::setVoltageNoTransactionNoLdac(dacChannels[i], voltage);
        }
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::endTransaction();
        #endif
        

        // Check if we've completed a full sweep of voltages for this loop
        if (currentDacStep >= numDacStepsPerLoop) {
          currentDacStep = 0; // Reset to beginning of voltage list for next loop
          done = true; // Mark that we need to read ADC after settling
        }
        
        TimingUtil::dacFlag = false;
        currentDacStep++;
      }
      
      // Handle ADC flag - time to read ADC after settling
      if (done) {
        done = false; // Reset done flag for next ADC read
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::beginAdcTransaction();
        #endif
        for (int i = 0; i < numAdcChannels; i++) {
          double total = 0.0;
          for (int j = 0; j < numAdcAverages; j++) {
            total += ADCController::getVoltage(adcChannels[i]);
          }
          packets[i] = total * numAdcAveragesInv;
        }
        #if !defined(__NEW_SHIELD__)
        PeripheralCommsController::endTransaction();
        #endif
        m4SendVoltage(packets, numAdcChannels);
        
        #ifdef __NEW_DAC_ADC__
        digitalWrite(adc_sync, LOW);
        #endif
        TimingUtil::adcFlag = 0;
        currentAdcReads++;
        currentLoop++; // Each ADC read marks completion of one loop
      }
    }

    // Clean up timers
    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

    // Clean up
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

    ADCController::resetToPreviousConversionTimes();
    PeripheralCommsController::dataLedOff();

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
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

    // Check voltage bounds before executing ramp (both calibrated bounds AND global limits)
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      
      if (dacV0_1[i] < lowerBound || dacV0_1[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " start voltage 1 " + String(dacV0_1[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacVf_1[i] < lowerBound || dacVf_1[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " end voltage 1 " + String(dacVf_1[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacV0_2[i] < lowerBound || dacV0_2[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " start voltage 2 " + String(dacV0_2[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacVf_2[i] < lowerBound || dacVf_2[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " end voltage 2 " + String(dacVf_2[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
    }

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
