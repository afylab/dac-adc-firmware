#pragma once

#include <Arduino.h>
#include <Peripherals/PeripheralCommsController.h>
#include "Utils/shared_memory.h"

#include "Config.h"

class DACChannel {
 private:
  float gain_error;
  float offset_error;
  int cs_pin;
  float voltage_upper_bound;
  float voltage_lower_bound;
  float full_scale = 10.0;

  PeripheralCommsController commsController;

 public:
  DACChannel(int cs_pin) : commsController(cs_pin) {
    this->cs_pin = cs_pin;
    offset_error = 0.0;
    gain_error = 1.0;
    voltage_upper_bound = full_scale * gain_error + offset_error;
    voltage_lower_bound = -full_scale * gain_error + offset_error;
  }

  // initialize is the command INITIALIZE, setup is called in main::setup
  void initialize() {
    byte bytesToSend[3] = {
        32, 0,
        2};  // Write to control register; Reserved byte; Unclamp DAC from GND
    commsController.transferDAC(bytesToSend, 3);
    setVoltage(0.0);
  }

  // initialize is the command INITIALIZE, setup is called in main::setup
  void setup() {
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }

  float setVoltage(float v) {
    byte b1;
    byte b2;
    byte b3;

    voltageToDecimal(v / gain_error - offset_error, &b1, &b2, &b3);

    byte bytesToSend[3] = {b1, b2, b3};

    commsController.transferDAC(bytesToSend,
                             3);  // send command byte to DAC; MS data bits,
                                  // DAC2; LS 8 data bits, DAC2

    digitalWrite(ldac, LOW);
    digitalWrite(ldac, HIGH);

    return gain_error * (threeByteToVoltage(b1, b2, b3) + offset_error);
  }

  void setVoltageNoTransactionNoLdac(float v) {
    byte b1;
    byte b2;
    byte b3;

    voltageToDecimal(v / gain_error - offset_error, &b1, &b2, &b3);

    byte bytesToSend[3] = {b1, b2, b3};

    commsController.transferDACNoTransaction(bytesToSend,
                             3);  // send command byte to DAC; MS data bits,
                                  // DAC2; LS 8 data bits, DAC2
  }

  void setCalibration(float offset, float gain) {
    this->offset_error = offset;
    this->gain_error = gain;
    voltage_upper_bound = full_scale * gain_error + offset_error;
    voltage_lower_bound = -full_scale * gain_error + offset_error;
  }

  void setFullScale(float full_scale) {
    this->full_scale = full_scale;
    voltage_upper_bound = full_scale * gain_error + offset_error;
    voltage_lower_bound = -full_scale * gain_error + offset_error;
  }

  float getLowerBound() { return voltage_lower_bound; }

  float getUpperBound() { return voltage_upper_bound; }

  float getOffsetError() { return offset_error; }
  float getGainError() { return gain_error; }

  float sendCode(int decimal) {
    byte b1;
    byte b2;
    byte b3;

    intToThreeBytes(decimal, &b1, &b2, &b3);

    byte bytesToSend[3] = {b1, b2, b3};

    commsController.transferDAC(bytesToSend,
                             3);  // send command byte to DAC; MS data bits,
                                  // DAC2; LS 8 data bits, DAC2

    digitalWrite(ldac, LOW);

    digitalWrite(ldac, HIGH);

    return gain_error * (threeByteToVoltage(b1, b2, b3) + offset_error);
  }

  float getVoltage() {
    byte bytesToSend[3] = {144, 0, 0};
    byte data[3];
    commsController.transferDAC(bytesToSend, 3);
    delayMicroseconds(2);
    commsController.transferDAC(data, 3);

    float voltage = threeByteToVoltage(data[0], data[1], data[2]);
    return gain_error * (voltage + offset_error);
  }

 private:
  void voltageToDecimal(float v, byte *DB1, byte *DB2, byte *DB3) {
    int decimal;
    if (v >= 0) {
      decimal = v * 524287 / full_scale;
    } else {
      decimal = v * 524288 / full_scale + 1048576;
    }
    intToThreeBytes(decimal, DB1, DB2, DB3);
  }

  void intToThreeBytes(int decimal, byte *DB1, byte *DB2, byte *DB3) {
    *DB1 = (byte)((decimal >> 16) | 16);
    *DB2 = (byte)((decimal >> 8) & 255);
    *DB3 = (byte)(decimal & 255);
  }

  // This gives a 16 bit integer (between +/- 2^16)
  int threeByteToInt(byte DB1, byte DB2, byte DB3) {
    return ((int)(((((DB1 & 15) << 8) | DB2) << 8) | DB3));
  }

  float threeByteToVoltage(byte DB1, byte DB2, byte DB3) {
    int decimal;
    float v;

    decimal = threeByteToInt(DB1, DB2, DB3);

    if (decimal <= 524287) {
      v = decimal * full_scale / 524287;
    } else {
      v = -(1048576 - decimal) * full_scale / 524288;
    }
    return v;
  }
};