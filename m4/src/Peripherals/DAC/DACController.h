#pragma once

#include <Arduino.h>
#include <Config.h>
#include <SPI.h>

#include <vector>

#include "DACChannel.h"
#include "FunctionRegistry.h"
#include "FunctionRegistryMacros.h"
#include "Peripherals/OperationResult.h"

#include "Utils/TimingUtil.h"

class DACController {
 private:
  inline static std::vector<DACChannel> dac_channels;

 public:
  static void initializeRegistry() {
    REGISTER_MEMBER_FUNCTION_0(initialize, "INITIALIZE");
    REGISTER_MEMBER_FUNCTION_0(initialize, "INIT");
    REGISTER_MEMBER_FUNCTION_0(
        initialize, "INNIT");  // oi bruv u got a loicense for that DAC? 🇬🇧
    REGISTER_MEMBER_FUNCTION_2(setVoltage, "SET");
    REGISTER_MEMBER_FUNCTION_1(getVoltage, "GET_DAC");
    REGISTER_MEMBER_FUNCTION_2(sendCode, "SEND_CODE");
    REGISTER_MEMBER_FUNCTION_2(setFullScale, "FULL_SCALE");
    REGISTER_MEMBER_FUNCTION_0(inquiryOSG, "INQUIRY_OSG");
    REGISTER_MEMBER_FUNCTION_5(autoRamp1, "RAMP1");
    REGISTER_MEMBER_FUNCTION_8(autoRamp2, "RAMP2");
  }

  static void addChannel(int cs_pin) {
    DACChannel newChannel = DACChannel(cs_pin);
    dac_channels.push_back(newChannel);
  }

  static OperationResult initialize() {
    for (auto channel : dac_channels) {
      channel.initialize();
    }
    return OperationResult::Success("INITIALIZATION COMPLETE");
  }

  static void setup() {
    initializeRegistry();
    for (auto channel : dac_channels) {
      channel.setup();
    }
  }

  static DACChannel getChannel(int channel_index) {
    if (!isChannelIndexValid(channel_index)) {
      return DACChannel(-1);
    }
    return dac_channels[channel_index];
  }

  static bool isChannelIndexValid(int channelIndex) {
    return channelIndex >= 0 &&
           static_cast<size_t>(channelIndex) < dac_channels.size();
  }

  static OperationResult setVoltage(int channel_index, float voltage) {
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

  static void setVoltageNoTransaction(int channel_index, float voltage) {
    if (!isChannelIndexValid(channel_index)) {
      return;
    }
    DACChannel dac_channel = dac_channels[channel_index];
    if (voltage < dac_channel.getLowerBound() ||
        voltage > dac_channel.getUpperBound()) {
      return;
    }
    
    dac_channel.setVoltageNoTransaction(voltage);
  }

  static void toggleLdac() {
    digitalWrite(ldac, LOW);
    digitalWrite(ldac, HIGH);
  }

  static OperationResult getVoltage(int channel_index) {
    if (!isChannelIndexValid(channel_index)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel_index));
    }
    return OperationResult::Success(String(dac_channels[channel_index].getVoltage(), 6));
  }

  static void setCalibration(int channel_index, float offset, float gain) {
    if (!isChannelIndexValid(channel_index)) {
      return;
    }

    dac_channels[channel_index].setCalibration(offset, gain);
  }

  static float getLowerBound(int channel) {
    if (!isChannelIndexValid(channel)) {
      return -1;
    }
    return dac_channels[channel].getLowerBound();
  }

  static float getUpperBound(int channel) {
    if (!isChannelIndexValid(channel)) {
      return -1;
    }
    return dac_channels[channel].getUpperBound();
  }

  static OperationResult sendCode(int channel, int code) {
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

  static OperationResult setFullScale(int channel, float full_scale) {
    if (!isChannelIndexValid(channel)) {
      return OperationResult::Failure("Invalid channel index " +
                                      String(channel));
    }
    dac_channels[channel].setFullScale(full_scale);
    return OperationResult::Success("FULL_SCALE_UPDATED");
  }

  static OperationResult inquiryOSG() {
    String output = "";
    for (auto channel : dac_channels) {
      output += String(channel.getOffsetError(), 6) + "\n";
    }
    for (auto channel : dac_channels) {
      output += String(channel.getGainError(), 6) + "\n";
    }
    return OperationResult::Success(output);
  }

  static OperationResult autoRamp1(int dacChannel, float v0, float vf, int numSteps, u_long settlingTime_us) {
    if (!isChannelIndexValid(dacChannel)) {
      return OperationResult::Failure("Invalid channel index " + String(dacChannel));
    }
    
    float *voltSetpoints = new float[numSteps];

    for (int i = 0; i < numSteps; i++) {
      voltSetpoints[i] = v0 + (vf - v0) * i / (numSteps - 1);
    }

    int i = 0;

    TimingUtil::setupTimerOnlyDac(settlingTime_us);

    while (i < numSteps) {
      if (TimingUtil::dacFlag) {
        dac_channels[dacChannel].setVoltage(voltSetpoints[i]);
        i++;
        TimingUtil::dacFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();

    delete[] voltSetpoints;


    return OperationResult::Success("RAMPING DAC " + String(dacChannel) + " FROM " + String(v0) + " TO " + String(vf) + " IN " + String(numSteps) + " STEPS");
  }

  static OperationResult autoRamp2(int dacChannel1, int dacChannel2, float vi1, float vi2, float vf1, float vf2, int numSteps, u_long settlingTime_us) {
    if (!isChannelIndexValid(dacChannel1) || !isChannelIndexValid(dacChannel2)) {
      return OperationResult::Failure("Invalid channel index " + String(dacChannel1) + " or " + String(dacChannel2));
    }
    if (vi1 < dac_channels[dacChannel1].getLowerBound() || vi1 > dac_channels[dacChannel1].getUpperBound() ||
        vi2 < dac_channels[dacChannel2].getLowerBound() || vi2 > dac_channels[dacChannel2].getUpperBound() ||
        vf1 < dac_channels[dacChannel1].getLowerBound() || vf1 > dac_channels[dacChannel1].getUpperBound() ||
        vf2 < dac_channels[dacChannel2].getLowerBound() || vf2 > dac_channels[dacChannel2].getUpperBound()) {
      return OperationResult::Failure("VOLTAGE_OVERRANGE");
    }

    float *voltSetpoints1 = new float[numSteps];
    float *voltSetpoints2 = new float[numSteps];

    for (int i = 0; i < numSteps; i++) {
      voltSetpoints1[i] = vi1 + (vf1 - vi1) * i / (numSteps - 1);
      voltSetpoints2[i] = vi2 + (vf2 - vi2) * i / (numSteps - 1);
    }

    int i = 0;

    TimingUtil::setupTimerOnlyDac(settlingTime_us);

    while (i < numSteps) {
      if (TimingUtil::dacFlag) {
        dac_channels[dacChannel1].setVoltage(voltSetpoints1[i]);
        dac_channels[dacChannel2].setVoltage(voltSetpoints2[i]);
        i++;
        TimingUtil::dacFlag = false;
      }
    }

    TimingUtil::disableDacInterrupt();

    delete[] voltSetpoints1;
    delete[] voltSetpoints2;

    return OperationResult::Success("RAMPING DAC " + String(dacChannel1) + " FROM " + String(vi1) + " TO " + String(vf1) + " AND DAC " + String(dacChannel2) + " FROM " + String(vi2) + " TO " + String(vf2) + " IN " + String(numSteps) + " STEPS");
  }
};