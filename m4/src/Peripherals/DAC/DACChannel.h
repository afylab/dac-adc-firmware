#pragma once

#include <Arduino.h>
#include <Peripherals/PeripheralCommsController.h>
#include "Utils/shared_memory.h"

#include "Config.h"

class DACChannel {
 private:
  float gain_error;
  float gain_error_inverse;
  float offset_error;
  int cs_pin;
  int channel_index;
  float voltage_upper_bound;
  float voltage_lower_bound;
  float full_scale = 10.0;

  PeripheralCommsController commsController;

 public:
  DACChannel(int cs_pin, int channel_index = -1);
  
  void setChannelIndex(int index);
  
  int getChannelIndex() const;

  // initialize is the command INITIALIZE, setup is called in main::setup
  void initialize();

  // initialize is the command INITIALIZE, setup is called in main::setup
  void setup();

  float setVoltage(float v);

  void setVoltageNoTransactionNoLdac(float v);

  void setCalibration(float offset, float gain);

  void setFullScale(float full_scale);

  float getHardwareLowerBound();

  float getHardwareUpperBound();

  float getOffsetError();
  float getGainError();

  float sendCode(int decimal);

  float getVoltage();

 private:
  void voltageToDecimal(float v, byte *DB1, byte *DB2, byte *DB3);

  void intToThreeBytes(int decimal, byte *DB1, byte *DB2, byte *DB3);

  // This gives a 16 bit integer (between +/- 2^16)
  int threeByteToInt(byte DB1, byte DB2, byte DB3);

  float threeByteToVoltage(byte DB1, byte DB2, byte DB3);
};
