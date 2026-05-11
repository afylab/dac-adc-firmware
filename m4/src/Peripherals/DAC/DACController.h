#pragma once

#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "DACChannel.h"
#include "Peripherals/OperationResult.h"
#include "Utils/CalibrationData.h"

class DACController {
 private:
  struct RampParams {
    int channel;
    double v0;
    double vf;
    double stepSize;
  };

  static std::vector<DACChannel> dac_channels;

 public:
  static void initializeRegistry();
  static OperationResult setUpperLimit(int channel, float limit);
  static OperationResult setLowerLimit(int channel, float limit);
  static OperationResult getUpperLimit(int channel);
  static OperationResult getLowerLimit(int channel);
  static void addChannel(int cs_pin);
  static bool initialize();
  static void setup();
  static DACChannel getChannel(int channel_index);
  static bool isChannelIndexValid(int channelIndex);
  static OperationResult setVoltage(int channel_index, float voltage);
  static void setVoltageNoTransactionNoLdac(int channel_index, float voltage);
  static OperationResult toggleLdacTest();
  static void toggleLdac();
  static OperationResult getVoltage(int channel_index);
  static OperationResult setOSG(int channel_index, float offset, float gain);
  static void setCalibration(int channel_index, float offset, float gain);
  static CalibrationData getCalibrationData();
  static float getLowerBound(int channel);
  static float getUpperBound(int channel);
  static OperationResult sendCode(int channel, int code);
  static OperationResult setFullScale(int channel, float full_scale);
  static OperationResult inquiryOSG();
  static OperationResult autoRamp1(int dacChannel, float v0, float vf,
                                   int numSteps, u_long settlingTime_us);
  static OperationResult autoRamp2(int dacChannel1, int dacChannel2, float vi1,
                                   float vi2, float vf1, float vf2,
                                   int numSteps, u_long settlingTime_us);
  static OperationResult autoRampN(const std::vector<float>& args);
};
