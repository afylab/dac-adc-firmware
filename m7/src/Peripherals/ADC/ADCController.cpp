#include "Peripherals/ADC/ADCController.h"

#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Utils/FastGpio.h"
#include "Utils/shared_memory.h"

std::vector<ADCBoard> ADCController::adc_boards;

int ADCController::getBoardIndexFromGlobalIndex(int channel_index) {
  return channel_index / NUM_CHANNELS_PER_ADC_BOARD;
}

int ADCController::getChannelIndexFromGlobalIndex(int channel_index) {
  return channel_index % NUM_CHANNELS_PER_ADC_BOARD;
}

void ADCController::initialize() {
  for (auto& board : adc_boards) {
    board.initialize();
  }
}

void ADCController::resetToPreviousConversionTimes() {
  for (auto& board : adc_boards) {
    board.resetToPreviousConversionTimes();
  }
}

void ADCController::setup() {
#ifdef __NEW_DAC_ADC__
  pinMode(adc_sync, OUTPUT);
  FastGpio::digitalWrite(adc_sync, false);
#endif

  initializeRegistry();
  for (auto& board : adc_boards) {
    board.setup();
  }
}

void ADCController::setReadyFlag(int i) {
  adc_boards[i].setReadyFlag();
}

bool ADCController::getReadyFlag(int i) {
  return adc_boards[i].data_ready;
}

void ADCController::clearReadyFlag(int i) {
  adc_boards[i].data_ready = false;
}

void ADCController::initializeRegistry() {
  registerMemberFunction(readChannelVoltage, "GET_ADC");
  registerMemberFunction(setConversionTime, "CONVERT_TIME");
  registerMemberFunction(setConversionTimeFW, "CONVERT_TIME_FW");
  registerMemberFunction(getConversionTime, "GET_CONVERT_TIME");
  registerMemberFunction(getRevisionRegister, "GET_REVISION_REG");
  registerMemberFunction(continuousConvertRead, "CONTINUOUS_CONVERT_READ");
  registerMemberFunction(idleMode, "IDLE_MODE");
  registerMemberFunction(getChannelsActive, "GET_CHANNELS_ACTIVE");
  registerMemberFunction(resetAllADCBoards, "RESET");
  registerMemberFunction(talkADC, "TALK");
  registerMemberFunction(adcZeroScaleCal, "ADC_ZERO_SC_CAL");
  registerMemberFunction(adcChannelSystemZeroScaleCal, "ADC_CH_ZERO_SC_CAL");
  registerMemberFunction(adcChannelSystemFullScaleCal, "ADC_CH_FULL_SC_CAL");
  registerMemberFunction(setRDYFN, "SET_RDYFN");
  registerMemberFunction(unsetRDYFN, "UNSET_RDYFN");
  registerMemberFunction(getChZeroScaleCalibration, "GET_ZERO_SCALE_CAL");
  registerMemberFunction(getChFullScaleCalibration, "GET_FULL_SCALE_CAL");
  registerMemberFunction(setChZeroScaleCalibration, "SET_ZERO_SCALE_CAL");
  registerMemberFunction(setChFullScaleCalibration, "SET_FULL_SCALE_CAL");
  registerMemberFunction(getSavedChZeroScaleCalibration,
                         "GET_SAVED_ZERO_SCALE_CAL");
  registerMemberFunction(getSavedChFullScaleCalibration,
                         "GET_SAVED_FULL_SCALE_CAL");
  registerMemberFunction(setSavedChZeroScaleCalibration,
                         "SET_SAVED_ZERO_SCALE_CAL");
  registerMemberFunction(setSavedChFullScaleCalibration,
                         "SET_SAVED_FULL_SCALE_CAL");
  registerMemberFunction(resetToPreviousConversionTimesSerial,
                         "RESET_MAINTAIN");
  registerMemberFunction(hardResetAllADCBoards, "HARD_RESET");
  registerMemberFunction(setChopping, "SET_CHOP");
  registerMemberFunction(getChopping, "GET_CHOP");
}

void ADCController::addBoard(int cs_pin, int data_ready, int reset_pin,
                             int board_idx) {
  ADCBoard newBoard = ADCBoard(cs_pin, data_ready, reset_pin, board_idx);
  adc_boards.push_back(newBoard);
}

bool ADCController::isChannelIndexValid(int channelIndex) {
  return channelIndex >= 0 &&
         static_cast<size_t>(channelIndex) <
             adc_boards.size() * NUM_CHANNELS_PER_ADC_BOARD;
}

bool ADCController::isSavedChannelIndexValid(int channelIndex) {
  return channelIndex >= 0 &&
         channelIndex < NUM_ADC_BOARDS * NUM_CHANNELS_PER_ADC_BOARD;
}

OperationResult ADCController::readChannelVoltage(int channel_index) {
  if (isChannelIndexValid(channel_index)) {
    return OperationResult::Success(String(getVoltage(channel_index), 9));
  }
  return OperationResult::Failure("Invalid channel index");
}

#ifdef __NEW_DAC_ADC__
void ADCController::toggleSync() {
  FastGpio::digitalWrite(adc_sync, true);
  FastGpio::digitalWrite(adc_sync, false);
}
#endif

float ADCController::getVoltage(int channel_index) {
  return adc_boards[getBoardIndexFromGlobalIndex(channel_index)].readVoltage(
      getChannelIndexFromGlobalIndex(channel_index));
}

OperationResult ADCController::setChopping(bool chop) {
  for (auto& board : adc_boards) {
    board.chopEnabled = chop;
  }
  return OperationResult::Success();
}

OperationResult ADCController::getChopping() {
  return adc_boards[0].chopEnabled ? OperationResult::Success("true")
                                   : OperationResult::Success("false");
}

OperationResult ADCController::getRevisionRegister(int board_index) {
  if (board_index < 0 ||
      static_cast<size_t>(board_index) >= adc_boards.size()) {
    return OperationResult::Failure("Invalid board index");
  }
  uint8_t revision = adc_boards[board_index].getRevisionRegister();
  return OperationResult::Success(String(revision));
}

OperationResult ADCController::getChZeroScaleCalibration(int channel_index) {
  uint32_t data = adc_boards[getBoardIndexFromGlobalIndex(channel_index)]
                      .getZeroScaleCalibration(
                          getChannelIndexFromGlobalIndex(channel_index));
  return OperationResult::Success(String(data));
}

OperationResult ADCController::getChFullScaleCalibration(int channel_index) {
  uint32_t data = adc_boards[getBoardIndexFromGlobalIndex(channel_index)]
                      .getFullScaleCalibration(
                          getChannelIndexFromGlobalIndex(channel_index));
  return OperationResult::Success(String(data));
}

OperationResult ADCController::getSavedChZeroScaleCalibration(
    int channel_index) {
  if (!isSavedChannelIndexValid(channel_index)) {
    return OperationResult::Failure("Invalid channel index");
  }

  CalibrationData data;
  readCalibrationData(data);
  return OperationResult::Success(String(data.adc_offset[channel_index]));
}

OperationResult ADCController::getSavedChFullScaleCalibration(
    int channel_index) {
  if (!isSavedChannelIndexValid(channel_index)) {
    return OperationResult::Failure("Invalid channel index");
  }

  CalibrationData data;
  readCalibrationData(data);
  return OperationResult::Success(String(data.adc_gain[channel_index]));
}

OperationResult ADCController::setSavedChZeroScaleCalibration(
    int channel_index, uint32_t value) {
  if (!isSavedChannelIndexValid(channel_index)) {
    return OperationResult::Failure("Invalid channel index");
  }

  CalibrationData data;
  readCalibrationData(data);
  data.adc_offset[channel_index] = value;
  updateCalibrationData(data);
  return OperationResult::Success("Saved zero scale calibration");
}

OperationResult ADCController::setSavedChFullScaleCalibration(
    int channel_index, uint32_t value) {
  if (!isSavedChannelIndexValid(channel_index)) {
    return OperationResult::Failure("Invalid channel index");
  }

  CalibrationData data;
  readCalibrationData(data);
  data.adc_gain[channel_index] = value;
  updateCalibrationData(data);
  return OperationResult::Success("Saved full scale calibration");
}

OperationResult ADCController::setChZeroScaleCalibration(int channel_index,
                                                         uint32_t value) {
  OperationResult apply_result =
      applyChZeroScaleCalibration(channel_index, value);
  if (!apply_result.isSuccess()) {
    return apply_result;
  }
  OperationResult save_result =
      setSavedChZeroScaleCalibration(channel_index, value);
  if (!save_result.isSuccess()) {
    return save_result;
  }
  return OperationResult::Success("Set zero scale calibration");
}

OperationResult ADCController::setChFullScaleCalibration(int channel_index,
                                                         uint32_t value) {
  OperationResult apply_result =
      applyChFullScaleCalibration(channel_index, value);
  if (!apply_result.isSuccess()) {
    return apply_result;
  }
  OperationResult save_result =
      setSavedChFullScaleCalibration(channel_index, value);
  if (!save_result.isSuccess()) {
    return save_result;
  }
  return OperationResult::Success("Set full scale calibration");
}

OperationResult ADCController::applyChZeroScaleCalibration(int channel_index,
                                                           uint32_t value) {
  if (!isChannelIndexValid(channel_index)) {
    return OperationResult::Failure("Invalid channel index");
  }
  adc_boards[getBoardIndexFromGlobalIndex(channel_index)]
      .setZeroScaleCalibration(getChannelIndexFromGlobalIndex(channel_index),
                               value);
  return OperationResult::Success("Applied zero scale calibration");
}

OperationResult ADCController::applyChFullScaleCalibration(int channel_index,
                                                           uint32_t value) {
  if (!isChannelIndexValid(channel_index)) {
    return OperationResult::Failure("Invalid channel index");
  }
  adc_boards[getBoardIndexFromGlobalIndex(channel_index)]
      .setFullScaleCalibration(getChannelIndexFromGlobalIndex(channel_index),
                               value);
  return OperationResult::Success("Applied full scale calibration");
}

OperationResult ADCController::resetToPreviousConversionTimesSerial() {
  for (auto& board : adc_boards) {
    board.resetToPreviousConversionTimes();
  }
  return OperationResult::Success("Reset to previous conversion times");
}

float ADCController::getDataReadyPin(int board_index) {
  return adc_boards[board_index].getDataReadyPin();
}

uint32_t ADCController::getConversionData(int adc_channel) {
  return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
      .getConversionData(getChannelIndexFromGlobalIndex(adc_channel));
}

OperationResult ADCController::setRDYFN(int adc_channel) {
  adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].setRDYFN();
  return OperationResult::Success("Set RDYFN");
}

OperationResult ADCController::unsetRDYFN(int adc_channel) {
  adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].unsetRDYFN();
  return OperationResult::Success("Unset RDYFN");
}

double ADCController::getVoltageData(int adc_channel) {
  return ADC2DOUBLE(getConversionData(adc_channel));
}

double ADCController::getVoltageDataNoTransaction(int adc_channel) {
  return ADC2DOUBLE(adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
                        .getConversionDataNoTransaction(
                            getChannelIndexFromGlobalIndex(adc_channel)));
}

void ADCController::startContinuousConversion(int adc_channel) {
  adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].startContinuousConversion(
      getChannelIndexFromGlobalIndex(adc_channel));
}

OperationResult ADCController::continuousConvertRead(int channel_index,
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
      adc_boards[getBoardIndexFromGlobalIndex(channel_index)].continuousConvert(
          getChannelIndexFromGlobalIndex(channel_index), frequency_us,
          duration_us);
  String result = "";
  for (auto d : data) {
    result += String(d, 9) + ",";
  }
  result = result.substring(0, result.length() - 1);

  return OperationResult::Success(result);
}

OperationResult ADCController::idleMode(int adc_channel) {
  adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].idleMode(
      getChannelIndexFromGlobalIndex(adc_channel));
  return OperationResult::Success("Returned ADC " + String(adc_channel) +
                                  " to idle mode");
}

OperationResult ADCController::getChannelsActive() {
  std::vector<int> statuses;
  for (auto& board : adc_boards) {
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      if (board.isChannelActive(i)) {
        statuses.push_back(i);
      }
    }
  }
  String output = parseVector(statuses);
  return OperationResult::Success(output == "" ? "NONE" : output);
}

OperationResult ADCController::hardResetAllADCBoards() {
  for (auto& board : adc_boards) {
    board.hardReset();
  }

  CalibrationData data;
  readCalibrationData(data);
  data.adcCalibrated = false;
  updateCalibrationData(data);

  return OperationResult::Success("All ADC boards have been hard reset");
}

OperationResult ADCController::resetAllADCBoards() {
  for (auto& board : adc_boards) {
    board.reset();
  }
  return OperationResult::Success();
}

OperationResult ADCController::talkADC(byte command) {
  String results = "";
  for (auto& board : adc_boards) {
    results += String(board.talkADC(command), 9) + "\n";
  }
  return OperationResult::Success(results);
}

OperationResult ADCController::adcZeroScaleCal() {
  for (auto& board : adc_boards) {
    board.zeroScaleSelfCalibration();
  }
  return OperationResult::Success("CALIBRATION_FINISHED");
}

OperationResult ADCController::adcChannelSystemZeroScaleCal() {
  for (auto& board : adc_boards) {
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      board.zeroScaleChannelSystemSelfCalibration(i);
    }
  }
  return OperationResult::Success("CALIBRATION_FINISHED");
}

OperationResult ADCController::adcChannelSystemFullScaleCal() {
  for (auto& board : adc_boards) {
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      board.fullScaleChannelSystemSelfCalibration(i);
    }
  }
  return OperationResult::Success("CALIBRATION_FINISHED");
}

OperationResult ADCController::setConversionTime(int adc_channel,
                                                 float time_us) {
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

OperationResult ADCController::setConversionTimeFW(int adc_channel,
                                                   int filter_word) {
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

float ADCController::presetConversionTime(int adc_channel, int time_us,
                                          bool isMoreThanOneChannelActive) {
  return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)]
      .setConversionTimeFloat(getChannelIndexFromGlobalIndex(adc_channel),
                              time_us, isMoreThanOneChannelActive);
}

float ADCController::getConversionTimeFloat(int adc_channel) {
  if (!isChannelIndexValid(adc_channel)) {
    return -1.0;
  }
  return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].getConversionTime(
      getChannelIndexFromGlobalIndex(adc_channel));
}

float ADCController::getConversionTimeFloat(int adc_channel,
                                            bool isMoreThanOneChannelActive) {
  if (!isChannelIndexValid(adc_channel)) {
    return -1.0;
  }
  return adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].getConversionTime(
      getChannelIndexFromGlobalIndex(adc_channel), isMoreThanOneChannelActive);
}

OperationResult ADCController::getConversionTime(int adc_channel) {
  if (!isChannelIndexValid(adc_channel)) {
    return OperationResult::Failure("Invalid channel index");
  }
  float convert_time =
      adc_boards[getBoardIndexFromGlobalIndex(adc_channel)].getConversionTime(
          getChannelIndexFromGlobalIndex(adc_channel));
  return OperationResult::Success(String(convert_time, 9));
}
