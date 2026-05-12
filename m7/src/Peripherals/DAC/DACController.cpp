#include "Peripherals/DAC/DACController.h"

#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Utils/TimingUtil.h"
#include "Utils/shared_memory.h"

float DACLimits::upper_voltage_limit[NUM_DAC_CHANNELS];
float DACLimits::lower_voltage_limit[NUM_DAC_CHANNELS];
bool DACLimits::limits_initialized = false;

std::vector<DACChannel> DACController::dac_channels;

void DACController::initializeRegistry() {
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
  registerMemberFunction(setUpperLimit, "SET_UPPER_LIMIT");
  registerMemberFunction(setLowerLimit, "SET_LOWER_LIMIT");
  registerMemberFunction(getUpperLimit, "GET_UPPER_LIMIT");
  registerMemberFunction(getLowerLimit, "GET_LOWER_LIMIT");
}

OperationResult DACController::setUpperLimit(int channel, float limit) {
  if (!isChannelIndexValid(channel)) {
    return OperationResult::Failure("Invalid channel index " + String(channel));
  }
  DACLimits::upper_voltage_limit[channel] = limit;
  return OperationResult::Success("CH" + String(channel) +
                                  " UPPER LIMIT SET TO " + String(limit, 6) +
                                  " V");
}

OperationResult DACController::setLowerLimit(int channel, float limit) {
  if (!isChannelIndexValid(channel)) {
    return OperationResult::Failure("Invalid channel index " + String(channel));
  }
  DACLimits::lower_voltage_limit[channel] = limit;
  return OperationResult::Success("CH" + String(channel) +
                                  " LOWER LIMIT SET TO " + String(limit, 6) +
                                  " V");
}

OperationResult DACController::getUpperLimit(int channel) {
  if (!isChannelIndexValid(channel)) {
    return OperationResult::Failure("Invalid channel index " + String(channel));
  }
  return OperationResult::Success(String(DACLimits::upper_voltage_limit[channel],
                                         6));
}

OperationResult DACController::getLowerLimit(int channel) {
  if (!isChannelIndexValid(channel)) {
    return OperationResult::Failure("Invalid channel index " + String(channel));
  }
  return OperationResult::Success(String(DACLimits::lower_voltage_limit[channel],
                                         6));
}

void DACController::addChannel(int cs_pin) {
  int channel_index = dac_channels.size();
  DACChannel newChannel = DACChannel(cs_pin, channel_index);
  dac_channels.push_back(newChannel);
}

bool DACController::initialize() {
  for (auto& channel : dac_channels) {
    channel.initialize();
  }
  return true;
}

void DACController::setup() {
  DACLimits::initializeLimits();

  pinMode(ldac, OUTPUT);
  digitalWrite(ldac, HIGH);
  initializeRegistry();
  for (auto& channel : dac_channels) {
    channel.setup();
  }
}

DACChannel DACController::getChannel(int channel_index) {
  if (!isChannelIndexValid(channel_index)) {
    return DACChannel(-1);
  }
  return dac_channels[channel_index];
}

bool DACController::isChannelIndexValid(int channelIndex) {
  return channelIndex >= 0 &&
         static_cast<size_t>(channelIndex) < dac_channels.size();
}

OperationResult DACController::setVoltage(int channel_index, float voltage) {
  if (!isChannelIndexValid(channel_index)) {
    return OperationResult::Failure("Invalid channel index " +
                                    String(channel_index));
  }

  DACChannel& dac_channel = dac_channels[channel_index];
  if (voltage < dac_channel.getHardwareLowerBound() ||
      voltage > dac_channel.getHardwareUpperBound()) {
    return OperationResult::Failure(
        "Voltage out of bounds for DAC " + String(channel_index) + " (" +
        String(voltage) + " V must be between " +
        String(dac_channel.getHardwareLowerBound()) + " and " +
        String(dac_channel.getHardwareUpperBound()) + " V)");
  }

  float v = dac_channel.setVoltage(voltage);
  if (isnan(v)) {
    return OperationResult::Failure("Voltage out of bounds for DAC " +
                                    String(channel_index));
  }
  return OperationResult::Success("DAC " + String(channel_index) +
                                  " UPDATED TO " + String(v, 6) + " V");
}

bool DACController::setVoltageNoTransactionNoLdac(int channel_index,
                                                  float voltage) {
  if (!isChannelIndexValid(channel_index)) {
    return false;
  }

  DACChannel& dac_channel = dac_channels[channel_index];
  if (voltage < dac_channel.getHardwareLowerBound() ||
      voltage > dac_channel.getHardwareUpperBound()) {
    return false;
  }

  return dac_channel.setVoltageNoTransactionNoLdac(voltage);
}

OperationResult DACController::toggleLdacTest() {
  toggleLdac();
  return OperationResult::Success("LDAC TOGGLED");
}

void DACController::toggleLdac() {
  digitalWrite(ldac, LOW);
  digitalWrite(ldac, HIGH);
}

OperationResult DACController::getVoltage(int channel_index) {
  if (!isChannelIndexValid(channel_index)) {
    return OperationResult::Failure("Invalid channel index " +
                                    String(channel_index));
  }
  return OperationResult::Success(
      String(dac_channels[channel_index].getVoltage(), 6));
}

OperationResult DACController::setOSG(int channel_index, float offset,
                                      float gain) {
  if (!isChannelIndexValid(channel_index)) {
    return OperationResult::Failure("Invalid channel index " +
                                    String(channel_index));
  }
  setCalibration(channel_index, offset, gain);

  return OperationResult::Success("OSG SET FOR DAC " + String(channel_index));
}

void DACController::applyCalibration(int channel_index, float offset,
                                     float gain) {
  if (!isChannelIndexValid(channel_index)) {
    return;
  }

  dac_channels[channel_index].setCalibration(offset, gain);
}

void DACController::setCalibration(int channel_index, float offset, float gain) {
  if (!isChannelIndexValid(channel_index)) {
    return;
  }

  applyCalibration(channel_index, offset, gain);
  CalibrationData calibrationData = getCalibrationData();
  updateCalibrationData(calibrationData);
}

CalibrationData DACController::getCalibrationData() {
  CalibrationData calibrationData;
  readCalibrationData(calibrationData);
  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    calibrationData.offset[i] = dac_channels[i].getOffsetError();
    calibrationData.gain[i] = dac_channels[i].getGainError();
  }
  return calibrationData;
}

float DACController::getLowerBound(int channel) {
  if (!isChannelIndexValid(channel)) {
    return -1;
  }
  float calibratedHardwareLower = dac_channels[channel].getHardwareLowerBound();
  return max(calibratedHardwareLower, DACLimits::lower_voltage_limit[channel]);
}

float DACController::getUpperBound(int channel) {
  if (!isChannelIndexValid(channel)) {
    return -1;
  }
  float calibratedHardwareUpper = dac_channels[channel].getHardwareUpperBound();
  return min(calibratedHardwareUpper, DACLimits::upper_voltage_limit[channel]);
}

OperationResult DACController::sendCode(int channel, int code) {
  if (!isChannelIndexValid(channel)) {
    return OperationResult::Failure("Invalid channel index " + String(channel));
  }
  if (code < 0 || code > 1048576) {
    return OperationResult::Failure("CODE OVERRANGE (0-1048576)");
  }
  dac_channels[channel].sendCode(code);
  return OperationResult::Success("DAC " + String(channel) +
                                  " CODE UPDATED TO " + String(code));
}

OperationResult DACController::setFullScale(int channel, float full_scale) {
  if (!isChannelIndexValid(channel)) {
    return OperationResult::Failure("Invalid channel index " + String(channel));
  }
  dac_channels[channel].setFullScale(full_scale);
  return OperationResult::Success("FULL_SCALE_UPDATED");
}

OperationResult DACController::inquiryOSG() {
  String output = "";
  for (auto& channel : dac_channels) {
    float offset = channel.getOffsetError();
    sendFloatResponseToGateway(&offset, 1);
  }
  for (auto& channel : dac_channels) {
    float gain = channel.getGainError();
    sendFloatResponseToGateway(&gain, 1);
  }
  return OperationResult::Success(output);
}

OperationResult DACController::autoRamp1(int dacChannel, float v0, float vf,
                                         int numSteps,
                                         u_long settlingTime_us) {
  return autoRampN({1, static_cast<float>(numSteps),
                    static_cast<float>(settlingTime_us),
                    static_cast<float>(dacChannel), v0, vf});
}

OperationResult DACController::autoRamp2(int dacChannel1, int dacChannel2,
                                         float vi1, float vi2, float vf1,
                                         float vf2, int numSteps,
                                         u_long settlingTime_us) {
  return autoRampN({2, static_cast<float>(numSteps),
                    static_cast<float>(settlingTime_us),
                    static_cast<float>(dacChannel1), vi1, vf1,
                    static_cast<float>(dacChannel2), vi2, vf2});
}

OperationResult DACController::autoRampN(const std::vector<float>& args) {
  if (args.size() < 3) {
    return OperationResult::Failure("Insufficient arguments provided.");
  }

  int numDacs = static_cast<int>(args[0]);
  int numSteps = static_cast<int>(args[1]);
  unsigned long settlingTime_us = static_cast<unsigned long>(args[2]);

  if (numDacs < 1 || numDacs > NUM_DAC_CHANNELS || numSteps < 1 ||
      settlingTime_us < 1) {
    return OperationResult::Failure("Invalid ramp parameters.");
  }

  if (args.size() != static_cast<size_t>(3 + numDacs * 3)) {
    return OperationResult::Failure(
        "Argument count does not match number of DAC channels.");
  }

  RampParams rampParams[NUM_DAC_CHANNELS] = {};
  int rampParamsCount = 0;

  for (int i = 0; i < numDacs; i++) {
    int baseIndex = 3 + i * 3;
    int ch = static_cast<int>(args[baseIndex]);
    double v0 = args[baseIndex + 1];
    double vf = args[baseIndex + 2];

    if (!isChannelIndexValid(ch)) {
      return OperationResult::Failure("Invalid channel index " + String(ch));
    }

    DACChannel& dacCh = dac_channels[ch];
    if (v0 < dacCh.getHardwareLowerBound() ||
        v0 > dacCh.getHardwareUpperBound() ||
        vf < dacCh.getHardwareLowerBound() ||
        vf > dacCh.getHardwareUpperBound()) {
      return OperationResult::Failure("Voltage out of bounds for DAC " +
                                      String(ch));
    }

    double stepSize = numSteps > 1 ? (vf - v0) / (numSteps - 1) : 0.0;
    rampParams[rampParamsCount++] = {ch, v0, vf, stepSize};
  }

  int currentStep = 0;
  TimingUtil::setupTimerOnlyDac(settlingTime_us);

  double currentVoltages[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < rampParamsCount; i++) {
    currentVoltages[i] = rampParams[i].v0;
  }

  while (currentStep < numSteps) {
    if (isWorkerStopRequested()) {
      break;
    }
    if (TimingUtil::dacFlag) {
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

  String output = "RAMPING ";
  for (int i = 0; i < rampParamsCount; i++) {
    const auto& param = rampParams[i];
    output += "DAC " + String(param.channel) + " FROM " + String(param.v0) +
              " TO " + String(param.vf) + "; ";
  }
  output += "IN " + String(numSteps) + " STEPS";

  if (isWorkerStopRequested()) {
    clearWorkerStopRequest();
    return OperationResult::Failure("RAMPING_STOPPED");
  }

  return OperationResult::Success(output);
}
