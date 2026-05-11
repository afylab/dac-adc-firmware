#include "Peripherals/DAC/DACChannel.h"


DACChannel::DACChannel(int cs_pin, int channel_index) : commsController(cs_pin) {
    this->cs_pin = cs_pin;
    this->channel_index = channel_index;
    offset_error = 0.0;
    gain_error = 1.0;
    gain_error_inverse = 1.0;
    voltage_upper_bound = full_scale * gain_error + offset_error;
    voltage_lower_bound = -full_scale * gain_error + offset_error;
  }


  
  void DACChannel::setChannelIndex(int index) {
    this->channel_index = index;
  }


  
  int DACChannel::getChannelIndex() const {
    return channel_index;
  }



  // initialize is the command INITIALIZE, setup is called in main::setup
  void DACChannel::initialize() {
    byte bytesToSend[3] = {
        32, 0,
        2};  // Write to control register; Reserved byte; Unclamp DAC from GND
    commsController.transferDAC(bytesToSend, 3);
    setVoltage(0.0);
  }



  // initialize is the command INITIALIZE, setup is called in main::setup
  void DACChannel::setup() {
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }



  float DACChannel::setVoltage(float v) {
    byte b1;
    byte b2;
    byte b3;
    
    if (v > DACLimits::upper_voltage_limit[channel_index] || v < DACLimits::lower_voltage_limit[channel_index]) {
      return NAN;
    }

    voltageToDecimal(v * gain_error_inverse - offset_error, &b1, &b2, &b3);

    byte bytesToSend[3] = {b1, b2, b3};

    if (!commsController.transferDAC(bytesToSend,
                                     3)) {  // send command byte to DAC; MS data bits,
                                            // DAC2; LS 8 data bits, DAC2
      return NAN;
    }

    digitalWrite(ldac, LOW);
    digitalWrite(ldac, HIGH);

    return gain_error * (threeByteToVoltage(b1, b2, b3) + offset_error);
  }



  bool DACChannel::setVoltageNoTransactionNoLdac(float v) {
    byte b1;
    byte b2;
    byte b3;

    if (v > DACLimits::upper_voltage_limit[channel_index] || v < DACLimits::lower_voltage_limit[channel_index]) {
      return false;
    }

    voltageToDecimal(v * gain_error_inverse - offset_error, &b1, &b2, &b3);

    byte bytesToSend[3] = {b1, b2, b3};

    return commsController.transferDACNoTransaction(bytesToSend,
                             3);  // send command byte to DAC; MS data bits,
                                  // DAC2; LS 8 data bits, DAC2
  }



  void DACChannel::setCalibration(float offset, float gain) {
    this->offset_error = offset;
    this->gain_error = gain;
    this->gain_error_inverse = 1.0 / gain;
    voltage_upper_bound = full_scale * gain_error + offset_error;
    voltage_lower_bound = -full_scale * gain_error + offset_error;
  }



  void DACChannel::setFullScale(float full_scale) {
    this->full_scale = full_scale;
    voltage_upper_bound = full_scale * gain_error + offset_error;
    voltage_lower_bound = -full_scale * gain_error + offset_error;
  }



  float DACChannel::getHardwareLowerBound() { return voltage_lower_bound; }



  float DACChannel::getHardwareUpperBound() { return voltage_upper_bound; }



  float DACChannel::getOffsetError() { return offset_error; }


  float DACChannel::getGainError() { return gain_error; }



  float DACChannel::sendCode(int decimal) {
    byte b1;
    byte b2;
    byte b3;

    intToThreeBytes(decimal, &b1, &b2, &b3);

    byte bytesToSend[3] = {b1, b2, b3};

    if (!commsController.transferDAC(bytesToSend,
                                     3)) {  // send command byte to DAC; MS data bits,
                                            // DAC2; LS 8 data bits, DAC2
      return NAN;
    }

    digitalWrite(ldac, LOW);

    digitalWrite(ldac, HIGH);

    return gain_error * (threeByteToVoltage(b1, b2, b3) + offset_error);
  }



  float DACChannel::getVoltage() {
    byte bytesToSend[3] = {144, 0, 0};
    byte data[3]= {0, 0, 0};
    if (!commsController.transferDAC(bytesToSend, 3)) {
      return NAN;
    }
    // delayMicroseconds(2);
    if (!commsController.transferDAC(data, 3)) {
      return NAN;
    }

    float voltage = threeByteToVoltage(data[0], data[1], data[2]);
    return gain_error * (voltage + offset_error);
  }

  void DACChannel::voltageToDecimal(float v, byte *DB1, byte *DB2, byte *DB3) {
    int decimal;
    if (v >= 0) {
      decimal = v * 524287 / full_scale;
    } else {
      decimal = v * 524288 / full_scale + 1048576;
    }
    intToThreeBytes(decimal, DB1, DB2, DB3);
  }



  void DACChannel::intToThreeBytes(int decimal, byte *DB1, byte *DB2, byte *DB3) {
    *DB1 = (byte)((decimal >> 16) | 16);
    *DB2 = (byte)((decimal >> 8) & 255);
    *DB3 = (byte)(decimal & 255);
  }



  // This gives a 16 bit integer (between +/- 2^16)
  int DACChannel::threeByteToInt(byte DB1, byte DB2, byte DB3) {
    return ((int)(((((DB1 & 15) << 8) | DB2) << 8) | DB3));
  }



  float DACChannel::threeByteToVoltage(byte DB1, byte DB2, byte DB3) {
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
