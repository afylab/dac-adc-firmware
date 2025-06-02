#pragma once

#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCBoard.h"
#include "Peripherals/OperationResult.h"
#include "Utils/TimingUtil.h"

class ADCController {
 private:
  inline static std::vector<ADCBoard> adc_boards;

 public:
  inline static void initialize() {
    for (auto board : adc_boards) {
      board.initialize();
    }
  }

  inline static void resetToPreviousConversionTimes() {
    for (auto board : adc_boards) {
      board.resetToPreviousConversionTimes();
    }
  }

  inline static void setup() {

    #ifdef __NEW_DAC_ADC__
    pinMode(adc_sync, OUTPUT);
    digitalWrite(adc_sync, LOW);
    #endif


    initializeRegistry();
    for (auto board : adc_boards) {
      board.setup();
      // int drdy = board.getDataReadyPin();
    }
  }

  inline static void setReadyFlag(int i) {
    adc_boards[i].setReadyFlag();
  }

  inline static bool getReadyFlag(int i) { return adc_boards[i].data_ready; }

  inline static void clearReadyFlag(int i) { adc_boards[i].data_ready = false; }

  static void initializeRegistry() {
    registerMemberFunction(readChannelVoltage, "GET_ADC");
    registerMemberFunction(setConversionTime, "CONVERT_TIME");
    registerMemberFunction(setConversionTimeFW, "CONVERT_TIME_FW");
    registerMemberFunction(getConversionTime, "GET_CONVERT_TIME");
    registerMemberFunction(continuousConvertRead,
                               "CONTINUOUS_CONVERT_READ");
    registerMemberFunction(idleMode, "IDLE_MODE");
    registerMemberFunction(getChannelsActive, "GET_CHANNELS_ACTIVE");
    registerMemberFunction(resetAllADCBoards, "RESET");
    registerMemberFunction(talkADC, "TALK");
    registerMemberFunction(adcZeroScaleCal, "ADC_ZERO_SC_CAL");
    registerMemberFunction(adcChannelSystemZeroScaleCal,
                               "ADC_CH_ZERO_SC_CAL");
    registerMemberFunction(adcChannelSystemFullScaleCal,
                               "ADC_CH_FULL_SC_CAL");
    registerMemberFunctionVector(timeSeriesAdcRead, "TIME_SERIES_ADC_READ");
    registerMemberFunction(setRDYFN, "SET_RDYFN");
    registerMemberFunction(unsetRDYFN, "UNSET_RDYFN");
  }

  inline static void addBoard(int cs_pin, int data_ready,
                              int reset_pin) {
    ADCBoard newBoard = ADCBoard(cs_pin, data_ready, reset_pin);
    adc_boards.push_back(newBoard);
  }

  inline static bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) <
               adc_boards.size() * NUM_CHANNELS_PER_ADC_BOARD;
  }

#define getBoardIndexFromGlobalIndex(channel_index) \
  channel_index / NUM_CHANNELS_PER_ADC_BOARD

#define getChannelIndexFromGlobalIndex(channel_index) \
  channel_index % NUM_CHANNELS_PER_ADC_BOARD

  inline static OperationResult readChannelVoltage(int channel_index) {
    if (isChannelIndexValid(channel_index)) {
      return OperationResult::Success(String(getVoltage(channel_index), 9));
    } else {
      return OperationResult::Failure("Invalid channel index");
    }
  }

  #ifdef __NEW_DAC_ADC__
  inline static void toggleSync() {
    digitalWrite(adc_sync, HIGH);
    digitalWrite(adc_sync, LOW);
  }
  #endif

  inline static float getVoltage(int channel_index) {
    return adc_boards[getBoardIndexFromGlobalIndex(channel_index)].readVoltage(
        getChannelIndexFromGlobalIndex(channel_index));
  }

  inline static float getDataReadyPin(int board_index) {
    return adc_boards[board_index].getDataReadyPin();
  }

  // args: num_channels, channel_indexes, total_duration_us
  // ex: TIME_SERIES_ADC_READ 2,0,1,1000000
  inline static OperationResult timeSeriesAdcRead(const std::vector<float>& args) {
    if (args.size() < 2) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    int numAdcChannels = static_cast<int>(args[0]);
    if (args.size() != static_cast<size_t>(numAdcChannels + 2)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }
    std::vector<int> adcChannels;
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels.push_back(static_cast<int>(args[i + 1]));
    }

    uint32_t totalDuration = static_cast<uint32_t>(args[numAdcChannels + 1]);
    if (totalDuration < 82) {
      return OperationResult::Failure("Invalid total duration");
    }

    int x = 0;

    float adcConversionTime = getConversionTimeFloat(adcChannels[0]);

    for (int i = 1; i < numAdcChannels; i++) {
      if (getConversionTimeFloat(adcChannels[i]) != adcConversionTime) {
        return OperationResult::Failure("All channels must have the same "
                                       "conversion time for time series read");
      }
    }

    const int saved_data_size = totalDuration / adcConversionTime;

    setStopFlag(false);
    PeripheralCommsController::dataLedOn();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
    }
    TimingUtil::setupTimersTimeSeries(10000, adcConversionTime);


    while (x < saved_data_size && !getStopFlag()) {
      if (TimingUtil::adcFlag) {
        float packets[numAdcChannels];
        for (int i = 0; i < numAdcChannels; i++) {
          float v =
              getVoltageDataNoTransaction(adcChannels[i]);
          packets[i] = v;
        }
        m4SendFloat(packets, numAdcChannels);
        x++;
        // TimingUtil::adcFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
    }

    PeripheralCommsController::dataLedOff();

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }

  inline static uint32_t getConversionData(int adc_channel) {
    return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        .getConversionData(getChannelIndexFromGlobalIndex(adc_channel));
  }

  //Sets the RDYFN bit in the IO register to 1
  //This ensures the RDY pin goes low once all ADCs cycle through their conversion process in continuous read mode
  inline static OperationResult setRDYFN(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].setRDYFN();
    return OperationResult::Success("Set RDYFN");
  }

  //unsets the RDYFN bit in the IO register to 1
  //This ensures the RDY pin goes low once all ADCs cycle through their conversion process in continuous read mode
  //This needs to be done after fininshing a buffer ramp to take the ADC out of continuous conversion mode
  inline static OperationResult unsetRDYFN(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].unsetRDYFN();
    return OperationResult::Success("Unset RDYFN");
  }

  inline static double getVoltageData(int adc_channel) {
    return ADC2DOUBLE(getConversionData(adc_channel));
  }

  inline static double getVoltageDataNoTransaction(int adc_channel) {
    return ADC2DOUBLE(adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
                          .getConversionDataNoTransaction(
                              getChannelIndexFromGlobalIndex(adc_channel)));
  }

  inline static void startContinuousConversion(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
        .startContinuousConversion(getChannelIndexFromGlobalIndex(adc_channel));
  }

  inline static OperationResult continuousConvertRead(int channel_index,
                                                      uint32_t frequency_us,
                                                      uint32_t duration_us) {
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index");
    }
    if (frequency_us < 1) {
      return OperationResult::Failure("Invalid frequency");
    }
    if (duration_us < 1) {
      return OperationResult::Failure("Invalid duration");
    }
    if (frequency_us > duration_us) {
      return OperationResult::Failure("Frequency must be less than duration");
    }

    std::vector<double> data =
        adc_boards[getBoardIndexFromGlobalIndex(channel_index)]
            .continuousConvert(getChannelIndexFromGlobalIndex(channel_index),
                               frequency_us, duration_us);
    String result = "";
    for (auto d : data) {
      result += String(d, 9) + ",";
    }
    result = result.substring(0, result.length() - 1);

    return OperationResult::Success(result);
  }

  inline static OperationResult idleMode(int adc_channel) {
    adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].idleMode(
        getChannelIndexFromGlobalIndex(adc_channel));
    return OperationResult::Success("Returned ADC " + String(adc_channel) +
                                    " to idle mode");
  }

  inline static OperationResult getChannelsActive() {
    std::vector<int> statuses;
    for (auto board : adc_boards) {
      for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
        if (board.isChannelActive(i)) {
          statuses.push_back(i);
        }
      }
    }
    String output = parseVector(statuses);
    return OperationResult::Success(output == "" ? "NONE" : output);
  }

  template <typename T>
  inline static String parseVector(const std::vector<T>& data) {
    String result = "";
    for (const auto& d : data) {
      result += String(d) + ",";
    }
    return result.substring(0, result.length() - 1);
  }

  inline static OperationResult resetAllADCBoards() {
    for (auto board : adc_boards) {
      board.reset();
    }
    return OperationResult::Success();
  }

  inline static OperationResult talkADC(byte command) {
    String results = "";
    for (auto board : adc_boards) {
      results += String(board.talkADC(command), 9) + "\n";
    }
    return OperationResult::Success(results);
  }

  inline static OperationResult adcZeroScaleCal() {
    for (auto board : adc_boards) {
      board.zeroScaleSelfCalibration();
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  inline static OperationResult adcChannelSystemZeroScaleCal() {
    for (auto board : adc_boards) {
      for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
        board.zeroScaleChannelSystemSelfCalibration(i);
      }
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  inline static OperationResult adcChannelSystemFullScaleCal() {
    for (auto board : adc_boards) {
      for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
        board.fullScaleChannelSystemSelfCalibration(i);
      }
    }
    return OperationResult::Success("CALIBRATION_FINISHED");
  }

  static OperationResult setConversionTime(int adc_channel, int time_us) {
    if (!isChannelIndexValid(adc_channel)) {
      return OperationResult::Failure("Invalid channel index");
    }
    float setpoint =
        adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].setConversionTime(
            getChannelIndexFromGlobalIndex(adc_channel), time_us);
    if (setpoint == -1.0) {
      return OperationResult::Failure(
          "The filter word you selected is not valid.");
    }
    return OperationResult::Success(String(setpoint, 9));
  }

  static OperationResult setConversionTimeFW(int adc_channel, int filter_word) {
    if (!isChannelIndexValid(adc_channel)) {
      return OperationResult::Failure("Invalid channel index");
    }
    float setpoint =
        adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].setConversionTimeFW(
            getChannelIndexFromGlobalIndex(adc_channel), filter_word);
    if (setpoint == -1.0) {
      return OperationResult::Failure(
          "The filter word you selected is not valid.");
    }
    return OperationResult::Success(String(setpoint, 9));
  }

  static float presetConversionTime(int adc_channel, int time_us, bool isMoreThanOneChannelActive) {
    return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].setConversionTimeFloat(
            getChannelIndexFromGlobalIndex(adc_channel), time_us, isMoreThanOneChannelActive);
  }

  static float getConversionTimeFloat(int adc_channel) {
    if (!isChannelIndexValid(adc_channel)) {
      return -1.0;
    }
    return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].getConversionTime(
        getChannelIndexFromGlobalIndex(adc_channel));
  }

  static float getConversionTimeFloat(int adc_channel, bool isMoreThanOneChannelActive) {
    if (!isChannelIndexValid(adc_channel)) {
      return -1.0;
    }
    return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].getConversionTime(
        getChannelIndexFromGlobalIndex(adc_channel), isMoreThanOneChannelActive);
  }

  static OperationResult getConversionTime(int adc_channel) {
    if (!isChannelIndexValid(adc_channel)) {
      return OperationResult::Failure("Invalid channel index");
    }
    float convert_time =
        adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].getConversionTime(
            getChannelIndexFromGlobalIndex(adc_channel));
    return OperationResult::Success(String(convert_time, 9));
  }
};