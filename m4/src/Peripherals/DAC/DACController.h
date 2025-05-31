#pragma once

#include <Arduino.h>
#include <Config.h>
#include <SPI.h>

#include <vector>

#include "DACChannel.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/OperationResult.h"
#include "Utils/TimingUtil.h"
#include "Utils/shared_memory.h"
#include "Peripherals/PeripheralCommsController.h"

class DACController {
 private:
  inline static std::vector<DACChannel> dac_channels;

 public:
  inline static void initializeRegistry() {
    registerMemberFunction(initialize, "INITIALIZE");
    registerMemberFunction(initialize, "INIT");
    registerMemberFunction(
        initialize, "INNIT");  // oi bruv u got a loicense for that DAC? 🇬🇧
    registerMemberFunction(setVoltage, "SET");
    registerMemberFunction(getVoltage, "GET_DAC");
    registerMemberFunction(sendCode, "SET_DAC_CODE");
    registerMemberFunction(setFullScale, "FULL_SCALE");
    registerMemberFunction(inquiryOSG, "INQUIRY_OSG");
    registerMemberFunction(setOSG, "SET_OSG");
    registerMemberFunction(autoRamp1, "RAMP1");
    registerMemberFunction(autoRamp2, "RAMP2");
    registerMemberFunctionVector(autoRampN, "RAMP_N");
    registerMemberFunction(toggleLdacTest, "TOGGLE_LDAC");
  }

  inline static void addChannel(int cs_pin) {
    DACChannel newChannel = DACChannel(cs_pin);
    dac_channels.push_back(newChannel);
  }

  inline static OperationResult initialize() {
    for (auto& channel : dac_channels) {
      channel.initialize();
    }
    return OperationResult::Success("INITIALIZATION COMPLETE");
  }

  inline static void setup() {
    pinMode(ldac, OUTPUT);
    digitalWrite(ldac, HIGH);
    
    initializeRegistry();
    for (auto& channel : dac_channels) {
      channel.setup();
    }
  }

  inline static DACChannel getChannel(int channel_index) {
    if (!isChannelIndexValid(channel_index)) {
      return DACChannel(-1);
    }
    return dac_channels[channel_index];
  }

  inline static bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) < dac_channels.size();
  }

  inline static OperationResult setVoltage(int channel_index, float voltage) {
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel_index));
    }
    DACChannel dac_channel = dac_channels[channel_index];
    if (voltage < dac_channel.getLowerBound() ||
        voltage > dac_channel.getUpperBound()) {
      return OperationResult::Failure("Voltage out of bounds for DAC " +
                                      String(channel_index));
    }

    float v = dac_channel.setVoltage(voltage);
    return OperationResult::Success("DAC " + String(channel_index) +
                                    " UPDATED TO " + String(v, 6) + " V");
  }

  inline static void setVoltageNoTransactionNoLdac(int channel_index, float voltage) {
    if (!isChannelIndexValid(channel_index)) {
      return;
    }
    DACChannel& dac_channel = dac_channels[channel_index];
    if (voltage < dac_channel.getLowerBound() ||
        voltage > dac_channel.getUpperBound()) {
      return;
    }

    dac_channel.setVoltageNoTransactionNoLdac(voltage);
  }

  inline static OperationResult toggleLdacTest() {
    toggleLdac();
    return OperationResult::Success("LDAC TOGGLED");
  }

  inline static void toggleLdac() {
    digitalWrite(ldac, LOW);
    digitalWrite(ldac, HIGH);
  }

  inline static OperationResult getVoltage(int channel_index) {
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel_index));
    }
    return OperationResult::Success(
        String(dac_channels[channel_index].getVoltage(), 6));
  }

  inline static OperationResult setOSG(int channel_index, float offset, float gain) {
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel_index));
    }
    setCalibration(channel_index, offset, gain);

    return OperationResult::Success("OSG SET FOR DAC " + String(channel_index));
  }

  inline static void setCalibration(int channel_index, float offset,
                                    float gain) {
    if (!isChannelIndexValid(channel_index)) {
      return;
    }

    dac_channels[channel_index].setCalibration(offset, gain);
  }

  inline static float getLowerBound(int channel) {
    if (!isChannelIndexValid(channel)) {
      return -1;
    }
    return dac_channels[channel].getLowerBound();
  }

  inline static float getUpperBound(int channel) {
    if (!isChannelIndexValid(channel)) {
      return -1;
    }
    return dac_channels[channel].getUpperBound();
  }

  inline static OperationResult sendCode(int channel, int code) {
    if (!isChannelIndexValid(channel)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel));
    }
    if (code < 0 || code > 1048576) {
      return OperationResult::Failure("CODE OVERRANGE (0-1048576)");
    }
    dac_channels[channel].sendCode(code);
    return OperationResult::Success("DAC " + String(channel) +
                                    " CODE UPDATED TO " + String(code));
  }

  inline static OperationResult setFullScale(int channel, float full_scale) {
    if (!isChannelIndexValid(channel)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel));
    }
    dac_channels[channel].setFullScale(full_scale);
    return OperationResult::Success("FULL_SCALE_UPDATED");
  }

  inline static OperationResult inquiryOSG() {
    String output = "";
    for (auto channel : dac_channels) {
      // output += String(, 4) + "\n";
      float offset = channel.getOffsetError();
      m4SendFloat(&offset, 1);  // Send offset error to M4
    }
    for (auto channel : dac_channels) {
      // output += String(channel.getGainError(), 4) + "\n";
      float gain = channel.getGainError();
      m4SendFloat(&gain, 1);  // Send offset error to M4
    }
    return OperationResult::Success(output);
  }

  inline static OperationResult autoRamp1(int dacChannel, float v0, float vf, int numSteps, u_long settlingTime_us) {
    return autoRampN({1, static_cast<float>(numSteps), static_cast<float>(settlingTime_us), static_cast<float>(dacChannel), v0, vf});
  }

  inline static OperationResult autoRamp2(int dacChannel1, int dacChannel2, float vi1, float vi2, float vf1, float vf2, int numSteps, u_long settlingTime_us) {
    return autoRampN({2, static_cast<float>(numSteps), static_cast<float>(settlingTime_us), static_cast<float>(dacChannel1), vi1, vf1, static_cast<float>(dacChannel2), vi2, vf2});
  }

  inline static OperationResult autoRampN(const std::vector<float>& args) {
    // args: numDacChannels, numSteps, settlingTime_us, then repeated groups of:
    // dacChannel, v0, vf for each DAC.
    if (args.size() < 3) {
      return OperationResult::Failure("Insufficient arguments provided.");
    }

    int numDacs = static_cast<int>(args[0]);
    int numSteps = static_cast<int>(args[1]);
    unsigned long settlingTime_us = static_cast<unsigned long>(args[2]);

    if (args.size() != static_cast<size_t>(3 + numDacs * 3)) {
      return OperationResult::Failure("Argument count does not match number of DAC channels.");
    }

    // Store per-channel parameters with precomputed step size
    struct RampParams {
      int channel;
      double v0;
      double vf;
      double stepSize;
    };
    
    RampParams* rampParams = new RampParams[numDacs];
    int rampParamsCount = 0;
    
    for (int i = 0; i < numDacs; i++) {
      int baseIndex = 3 + i * 3;
      int ch = static_cast<int>(args[baseIndex]);
      double v0 = args[baseIndex + 1];
      double vf = args[baseIndex + 2];

      // Validate channel index.
      if (!isChannelIndexValid(ch)) {
        delete[] rampParams;
        return OperationResult::Failure("Invalid channel index " + String(ch));
      }
      // Validate voltage bounds.
      DACChannel dacCh = dac_channels[ch];
      if (v0 < dacCh.getLowerBound() || v0 > dacCh.getUpperBound() ||
          vf < dacCh.getLowerBound() || vf > dacCh.getUpperBound()) {
        delete[] rampParams;
        return OperationResult::Failure("Voltage out of bounds for DAC " + String(ch));
      }
      
      // Precompute step size
      double stepSize = (vf - v0) / (numSteps - 1);
      rampParams[rampParamsCount++] = {ch, v0, vf, stepSize};
    }

    int currentStep = 0;
    TimingUtil::setupTimerOnlyDac(settlingTime_us);
    
    // Store current voltages for each channel
    double* currentVoltages = new double[rampParamsCount];
    for (int i = 0; i < rampParamsCount; i++) {
      currentVoltages[i] = rampParams[i].v0;
    }

    while (currentStep < numSteps) {
      if (getStopFlag()) {
        break;
      }
      if (TimingUtil::dacFlag) {
        // Update voltage for each channel using the precomputed step size
        for (int i = 0; i < rampParamsCount; i++) {
          const auto& param = rampParams[i];
          dac_channels[param.channel].setVoltage(currentVoltages[i]);
          currentVoltages[i] += param.stepSize;
        }
        currentStep++;
        TimingUtil::dacFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();

    // Construct a summary message.
    String output = "RAMPING ";
    for (int i = 0; i < rampParamsCount; i++) {
      const auto& param = rampParams[i];
      output += "DAC " + String(param.channel) + " FROM " + String(param.v0) + " TO " + String(param.vf) + "; ";
    }
    output += "IN " + String(numSteps) + " STEPS";

    // Free allocated memory
    delete[] currentVoltages;
    delete[] rampParams;

    if (getStopFlag()) {
      setStopFlag(false);
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success(output);
  }
};