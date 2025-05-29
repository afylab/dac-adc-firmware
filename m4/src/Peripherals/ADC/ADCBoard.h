// Thank you to CU Denver for the convenient AD7734 macros used here
// https://github.com/ucd-squidlab/AD7734Lib

#pragma once

#include <Arduino.h>
#include <Peripherals/PeripheralCommsController.h>

#include "Utils/shared_memory.h"

#include "Config.h"

// ADC symbols
// All tables & pages reference AD7734 Data Sheet Rev B (4 Channels, AD7732 is 2
// Channels) Communications Register, Table 11 Summery

// Read/Write bit (R, W), Table 11
#define READ 1 << 6
#define WRITE 0 << 6

// ADC register addresses, Table 11
#define ADDR_COM 0x0
#define ADDR_IO 0x1
#define ADDR_REVISION 0x2
#define ADDR_TEST 0x3
#define ADDR_ADCSTATUS 0x4
#define ADDR_CHECKSUM 0x5
#define ADDR_ADCZEROSCALECAL 0x6
#define ADDR_ADCFULLSCALE 0x7

#define DUMP_MODE 1 << 3

// Address macro functions, returns address for desired register of selected
// channel (0-3), Table 11
#define ADDR_CHANNELDATA(adc_channel) (0x8 + adc_channel)
#define ADDR_CHANNELZEROSCALECAL(adc_channel) (0x10 + adc_channel)
#define ADDR_CHANNELFULLSCALECAL(adc_channel) (0x18 + adc_channel)
#define ADDR_CHANNELSTATUS(adc_channel) (0x20 + adc_channel)
#define ADDR_CHANNELSETUP(adc_channel) (0x28 + adc_channel)
#define ADDR_CHANNELCONVERSIONTIME(adc_channel) (0x30 + adc_channel)
#define ADDR_MODE(adc_channel) (0x38 + adc_channel)

#define BIT_MODE16 0 << 1
#define BIT_MODE24 1 << 1

// SELECT ADC RESOLUTION HERE
#define BIT_MODE BIT_MODE24

// Operational Mode Register, Table 12
// mode bits (MD2, MD1, MD0 bits)
#define IDLE_MODE 0 << 5 | BIT_MODE
#define CONT_CONV_MODE 1 << 5 | BIT_MODE
#define SINGLE_CONV_MODE 2 << 5 | BIT_MODE
#define PWR_DOWN_MODE 3 << 5 | BIT_MODE
#define ZERO_SCALE_SELF_CAL_MODE 4 << 5 | BIT_MODE
#define CH_ZERO_SCALE_SYS_CAL_MODE 6 << 5 | BIT_MODE
#define CH_FULL_SCALE_SYS_CAL_MODE 7 << 5 | BIT_MODE
#define CH_EN_CONT_CONV 1 << 3

#define ADCRES16 65535.0
#define ADCRES24 16777215.0

// full scale range, can take 4 different values
#define FSR 20.0
#define ADC2DOUBLE16(vin) (FSR * ((double)vin - (ADCRES16 / 2.0)) / ADCRES16)
#define ADC2DOUBLE24(vin) (FSR * ((double)vin - (ADCRES24 / 2.0)) / ADCRES24)

// SELECT ADC RESOLUTION HERE
#define ADC2DOUBLE(vin) ADC2DOUBLE24(vin)

class ADCBoard {
 private:
  int cs_pin;
  int data_ready_pin;
  int reset_pin;
  PeripheralCommsController commsController;

  void waitDataReady() {
    int count = 0;
    while (digitalRead(data_ready_pin) == HIGH && count < 20000) {
      count = count + 1;
      delay(1);
    }
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, LOW);
    #endif
  }

 public:
  bool data_ready = false;

  ADCBoard(int cs_pin, int data_ready_pin, int reset_pin)
      : cs_pin(cs_pin), data_ready_pin(data_ready_pin), reset_pin(reset_pin), commsController(cs_pin) {}

  void setup() {
    pinMode(reset_pin, OUTPUT);

    pinMode(data_ready_pin, INPUT);
    //attachInterrupt(digitalPinToInterrupt(data_ready_pin), this->RDY_ISR, FALLING);

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    // Resets ADC on startup.
    digitalWrite(reset_pin, HIGH);
    digitalWrite(reset_pin, LOW);
    delay(5);
    digitalWrite(reset_pin, HIGH);

    #ifdef __NEW_DAC_ADC__
    pinMode(adc_sync, OUTPUT);
    digitalWrite(adc_sync, LOW);

    //Set I/O Register such that P1 bit is set as input and SYNC pin function is enabled
    byte data[2];
    data[0] = WRITE | ADDR_IO;
    data[1] = 0b00010001;
    commsController.transferADC(data, 2);
    #endif
  }

  void RDY_ISR () {
    setReadyFlag();
  }

  void initialize() {}

  int getDataReadyPin() const { return data_ready_pin; }

  void setReadyFlag() { data_ready = true; }
  void clearReadyFlag() { data_ready = false; }

  double readVoltage(int channel_index) {
    startSingleConversion(channel_index);
    waitDataReady();
    uint32_t data = getConversionData(channel_index);
    return ADC2DOUBLE(data);
  }

  // return ADC status register, pg. 16
  uint8_t getADCStatus() {
    byte data[2];
    data[0] = READ | ADDR_ADCSTATUS;

    commsController.transferADC(data, 2);
    return data[1];
  }

  void setRDYFN() {
    //first read contents of IO register
    byte new_io_reg[2];
    new_io_reg[0] = WRITE | ADDR_IO;
    new_io_reg[1] = 0b00011001;

    commsController.transferADC(new_io_reg, 2);
  }

  void unsetRDYFN() {
    //first read contents of IO register
    byte new_io_reg[2];
    new_io_reg[0] = WRITE | ADDR_IO;
    new_io_reg[1] = 0b00010001;

    commsController.transferADC(new_io_reg, 2);
  }

  void channelSetup(int adc_channel, uint8_t flags) {
    byte data[2];
    data[0] = WRITE | ADDR_CHANNELSETUP(adc_channel);
    data[1] = flags;
    commsController.transferADC(data, 2);
  }

  // tells the ADC to start a single conversion on the passed channel
  void startSingleConversion(int adc_channel) {
    byte data[2];
    // setup communication register for writing operation to the mode register
    data[0] = WRITE | ADDR_MODE(adc_channel);
    // setup mode register
    data[1] = SINGLE_CONV_MODE;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, LOW);
    #endif
    commsController.transferADC(data, 2);
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #endif

    // data is ready when _rdy goes low
  }

  // tells the ADC to start a continous conversion on the passed channel
  void startContinuousConversion(int adc_channel) {
    uint8_t data_array[4];

    // address the channel setup register and write to it
    data_array[0] = WRITE | ADDR_CHANNELSETUP(adc_channel);
    data_array[1] = CH_EN_CONT_CONV;

    // address the channel mode register and write to it
    data_array[2] = WRITE | ADDR_MODE(adc_channel);
    data_array[3] = CONT_CONV_MODE; // | 1 << 2; //includes setting continuous read mode

    // send off command
    commsController.transferADC(data_array, 4);

    // data is ready when _rdy goes low
  }

  void setConversionTime(int adc_channel, int chop, int fw) {
    byte chop_byte = chop == 1 ? 0x80 : 0x00;
    byte send = chop_byte | static_cast<byte>(fw);

    byte data[2];
    data[0] = WRITE | ADDR_CHANNELCONVERSIONTIME(adc_channel);
    data[1] = send;

    commsController.transferADC(data, 2);
  }

  uint32_t getConversionData(int adc_channel) {
    byte data[4];
    data[0] = READ | ADDR_CHANNELDATA(adc_channel);

    commsController.transferADC(data, 4);

    uint32_t upper = data[1];
    uint32_t lower = data[2];
    uint32_t last = data[3];

    uint32_t result = upper << 16 | lower << 8 | last;

    return result;
  }

  uint32_t getConversionDataNoTransaction(int adc_channel) {
    byte data[4];

    // setup communication register for reading channel data
    data[0] = READ | ADDR_CHANNELDATA(adc_channel);
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;

    // write to the communication register
    // read upper and lower bytes of channel data register (16 bit mode)
    commsController.transferADCNoTransaction(data, 4);

    uint32_t upper = data[1];
    uint32_t lower = data[2];
    uint32_t last = data[3];

    uint32_t result = upper << 16 | lower << 8 | last;

    return result;
    //return 0;
  }

  std::vector<double> continuousConvert(int channel_index, uint32_t period_us,
                                        uint32_t duration) {
    std::vector<double> data;
    uint32_t num_samples = duration / period_us;
    startContinuousConversion(channel_index);
    for (uint32_t i = 0; i < num_samples; i++) {
      data.push_back(ADC2DOUBLE(getConversionData(channel_index)));
      delayMicroseconds(period_us);
    }
    idleMode(channel_index);
    return data;
  }

  void idleMode(int adc_channel) {
    byte data[2];
    data[0] = WRITE | ADDR_MODE(adc_channel);
    data[1] = IDLE_MODE;
    commsController.transferADC(data, 2);
  }

  bool isChannelActive(int adc_channel) {
    uint8_t status = getADCStatus();
    return (status & (1 << adc_channel)) != 0;
  }

  void reset() {
    digitalWrite(reset_pin, HIGH);
    digitalWrite(reset_pin, LOW);
    delay(5);
    digitalWrite(reset_pin, HIGH);

    commsController.transferADC(0x28);
    commsController.transferADC(0);
    commsController.transferADC(0x2A);
    commsController.transferADC(0);
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      idleMode(i);
    }
  }

  uint8_t talkADC(byte command) {
    uint8_t comm = commsController.transferADC(command);
    return comm;
  }

  // chop = true
  float setConversionTime(int channel, float time_us) {
    return setConversionTimeFloat(channel, time_us,
                                  isMoreThanOneChannelActive());
  }

  // chop = true
  float setConversionTimeFloat(int channel, float time_us,
                               bool moreThanOneChannelActive) {
    return setConversionTime(
        channel, true,
        calculateFilterWord(time_us, true, moreThanOneChannelActive),
        moreThanOneChannelActive);
  }

  float setConversionTime(int channel, bool chop, byte fw,
                          bool moreThanOneChannelActive) {
    if ((fw > 127) || (chop && fw < 2) || (!chop && fw < 3)) {
      return -1;
    }

    byte chop_byte = chop ? 0b10000000 : 0b00000000;
    byte send = chop_byte | fw;

    byte data[2];
    data[0] = WRITE | ADDR_CHANNELCONVERSIONTIME(channel);
    data[1] = send;

    commsController.transferADC(data, 2);

    delayMicroseconds(100);

    // could've done the calculation with user-given values but it's good to
    // check
    return getConversionTime(channel, moreThanOneChannelActive);
  }

  float setConversionTimeFW(int channel, int filter_word) {
    return setConversionTime(channel, true, filter_word,
                            isMoreThanOneChannelActive());
  }

  float getConversionTime(int channel) {
    return getConversionTime(channel, isMoreThanOneChannelActive());
  }

  float getConversionTime(int channel, bool moreThanOneChannelActive) {
    byte data[2];
    data[0] = READ | ADDR_CHANNELCONVERSIONTIME(channel);
    commsController.transferADC(data, 2);

    return calculateConversionTime(data[1], moreThanOneChannelActive);
  }

  float calculateConversionTime(byte b, bool moreThanOneChannelActive) {
    // convert to actual conversion time
    byte received_fw = b & 0b01111111;
    bool received_chop = b & 0b10000000;
    if (received_chop) {
      if (moreThanOneChannelActive) {  // FW range is 2 to 127
        return (received_fw * 128.0 + 249.0) / 6.144;
      } else {
        return (received_fw * 128.0 + 248.0) / 6.144;
      }
    } else {  // FW range is 3 to 127
      if (moreThanOneChannelActive) {
        return (received_fw * 64.0 + 206.0) / 6.144;
      } else {
        return (received_fw * 64.0 + 207.0) / 6.144;
      }
    }

    return -1;
  }

  byte calculateFilterWord(float time_us, bool chop,
                           bool moreThanOneChannelActive) {
    byte out;
    if (chop) {
      if (moreThanOneChannelActive) {
        out = static_cast<byte>(round((time_us * 6.144 - 249.0) / 128.0));
      } else {
        out = static_cast<byte>(round((time_us * 6.144 - 248.0) / 128.0));
      }
      if (out < 2) return 2;
    } else {
      if (moreThanOneChannelActive) {
        out = static_cast<byte>(round((time_us * 6.144 - 206.0) / 64.0));
      } else {
        out = static_cast<byte>(round((time_us * 6.144 - 207.0) / 64.0));
      }
      if (out < 3) return 3;
    }
    if (out > 127) return 127;
    return out;
  }

  bool isMoreThanOneChannelActive() {
    return (getADCStatus() & 0b00001111) != 0;
  }

  void zeroScaleSelfCalibration() {
    byte data[2];
    data[0] = WRITE | ADDR_MODE(0);  // channel is zero but this is system-wide
    data[1] = ZERO_SCALE_SELF_CAL_MODE;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #endif
    commsController.transferADC(data, 2);
    waitDataReady();
  }

  void zeroScaleChannelSystemSelfCalibration(int channel) {
    byte data[2];
    data[0] = WRITE | ADDR_MODE(channel);
    data[1] = CH_ZERO_SCALE_SYS_CAL_MODE;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #endif
    commsController.transferADC(data, 2);
    waitDataReady();
  }

  void fullScaleChannelSystemSelfCalibration(int channel) {
    byte data[2];
    data[0] = WRITE | ADDR_MODE(channel);
    data[1] = CH_FULL_SCALE_SYS_CAL_MODE;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #endif
    commsController.transferADC(data, 2);
    waitDataReady();
  }
};

