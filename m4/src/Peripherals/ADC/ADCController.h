#pragma once

#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "Peripherals/ADC/ADCBoard.h"
#include "Peripherals/OperationResult.h"

class ADCController {
 private:
  static std::vector<ADCBoard> adc_boards;

  static int getBoardIndexFromGlobalIndex(int channel_index);
  static int getChannelIndexFromGlobalIndex(int channel_index);

 public:
  static void initialize();
  static void resetToPreviousConversionTimes();
  static void setup();
  static void setReadyFlag(int i);
  static bool getReadyFlag(int i);
  static void clearReadyFlag(int i);
  static void initializeRegistry();
  static void addBoard(int cs_pin, int data_ready, int reset_pin,
                       int board_idx);
  static bool isChannelIndexValid(int channelIndex);
  static bool isSavedChannelIndexValid(int channelIndex);
  static OperationResult readChannelVoltage(int channel_index);
#ifdef __NEW_DAC_ADC__
  static void toggleSync();
#endif
  static float getVoltage(int channel_index);
  static OperationResult setChopping(bool chop);
  static OperationResult getChopping();
  static OperationResult getRevisionRegister(int board_index);
  static OperationResult getChZeroScaleCalibration(int channel_index);
  static OperationResult getChFullScaleCalibration(int channel_index);
  static OperationResult getSavedChZeroScaleCalibration(int channel_index);
  static OperationResult getSavedChFullScaleCalibration(int channel_index);
  static OperationResult setSavedChZeroScaleCalibration(int channel_index,
                                                        uint32_t value);
  static OperationResult setSavedChFullScaleCalibration(int channel_index,
                                                        uint32_t value);
  static OperationResult setChZeroScaleCalibration(int channel_index,
                                                   uint32_t value);
  static OperationResult setChFullScaleCalibration(int channel_index,
                                                   uint32_t value);
  static OperationResult resetToPreviousConversionTimesSerial();
  static float getDataReadyPin(int board_index);
  static uint32_t getConversionData(int adc_channel);
  static OperationResult setRDYFN(int adc_channel);
  static OperationResult unsetRDYFN(int adc_channel);
  static double getVoltageData(int adc_channel);
  static double getVoltageDataNoTransaction(int adc_channel);
  static void startContinuousConversion(int adc_channel);
  static OperationResult continuousConvertRead(int channel_index,
                                               uint32_t frequency_us,
                                               uint32_t duration_us);
  static OperationResult idleMode(int adc_channel);
  static OperationResult getChannelsActive();

  template <typename T>
  static String parseVector(const std::vector<T>& data) {
    String result = "";
    for (const auto& d : data) {
      result += String(d) + ",";
    }
    return result.substring(0, result.length() - 1);
  }

  static OperationResult hardResetAllADCBoards();
  static OperationResult resetAllADCBoards();
  static OperationResult talkADC(byte command);
  static OperationResult adcZeroScaleCal();
  static OperationResult adcChannelSystemZeroScaleCal();
  static OperationResult adcChannelSystemFullScaleCal();
  static OperationResult setConversionTime(int adc_channel, float time_us);
  static OperationResult setConversionTimeFW(int adc_channel, int filter_word);
  static float presetConversionTime(int adc_channel, int time_us,
                                    bool isMoreThanOneChannelActive);
  static float getConversionTimeFloat(int adc_channel);
  static float getConversionTimeFloat(int adc_channel,
                                      bool isMoreThanOneChannelActive);
  static OperationResult getConversionTime(int adc_channel);
};
