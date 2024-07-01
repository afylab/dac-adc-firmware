#pragma once

#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "Peripherals/ADC/ADCBoard.h"
#include "Peripherals/Peripheral.h"

class ADCController : public Peripheral {
 private:
  std::vector<ADCBoard*> adc_boards;

 public:
  ADCController(FunctionRegistry& r, PeripheralCommsController& c)
      : Peripheral(r, c) {}

  OperationResult initialize() override {
    // for (auto channel : adc_channels) {
    //   channel->initialize();
    // }
    // Serial.println("ADC INITIALIZATION COMPLETE");
    return OperationResult::Success();
  }

  void setup() override {
    commsController.setup();
    initializeRegistry();
    for (auto board : adc_boards) {
      board->setup();
    }
  }

  void initializeRegistry() override {
    REGISTER_MEMBER_FUNCTION_1(registry, this, readChannelVoltage, "GET_ADC");
  }

  void addBoard(int data_sync_pin, int data_ready, int reset_pin) {
    ADCBoard* newBoard =
        new ADCBoard(commsController, data_sync_pin, data_ready, reset_pin);
    adc_boards.push_back(newBoard);
  }

  bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) <
               adc_boards.size() * NUM_CHANNELS_PER_ADC_BOARD;
  }

  int getBoardIndexFromGlobalIndex(int channel_index) {
    return channel_index / NUM_CHANNELS_PER_ADC_BOARD;
  }

  int getChannelIndexFromGlobalIndex(int channel_index) {
    return channel_index % NUM_CHANNELS_PER_ADC_BOARD;
  }

  OperationResult readChannelVoltage(int channel_index) {
    if (isChannelIndexValid(channel_index)) {
      return OperationResult::Success(String(
          adc_boards[getBoardIndexFromGlobalIndex(channel_index)]->readVoltage(
              getChannelIndexFromGlobalIndex(channel_index)), 6));
    } else {
      return OperationResult::Failure("Invalid channel index");
    }
  }
};