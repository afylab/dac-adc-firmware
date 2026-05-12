// Thank you to CU Denver for the convenient AD7734 macros used here
// https://github.com/ucd-squidlab/AD7734Lib

#pragma once

#include <Arduino.h>
#include <Peripherals/PeripheralCommsController.h>

#include <vector>

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
 public:
  bool chopEnabled = true;
 private:
  int cs_pin;
  int data_ready_pin;
  int reset_pin;
  int board_idx;
  PeripheralCommsController commsController;

  void waitDataReady();

 public:
  bool data_ready = false;

  ADCBoard(int cs_pin, int data_ready_pin, int reset_pin, int board_idx);

  void setup();

  void RDY_ISR ();

  void initialize();

  uint32_t getZeroScaleCalibration(int adc_channel);

  uint32_t getFullScaleCalibration(int adc_channel);

  void setZeroScaleCalibration(int adc_channel, uint32_t value);

  void setFullScaleCalibration(int adc_channel, uint32_t value);

  void resetToPreviousConversionTimes();

  int getDataReadyPin() const;
  int getBoardIndex() const;

  void setReadyFlag();
  void clearReadyFlag();

  double readVoltage(int channel_index);

  // return ADC status register, pg. 16
  uint8_t getADCStatus();

  void setRDYFN();

  void unsetRDYFN();

  void channelSetup(int adc_channel, uint8_t flags);

  // tells the ADC to start a single conversion on the passed channel
  void startSingleConversion(int adc_channel);

  // tells the ADC to start a continous conversion on the passed channel
  void startContinuousConversion(int adc_channel);

  uint8_t getRevisionRegister();

  void setConversionTime(int adc_channel, int chop, int fw);

  uint32_t getConversionData(int adc_channel);

  uint32_t getConversionDataNoTransaction(int adc_channel);

  std::vector<double> continuousConvert(int channel_index, uint32_t period_us,
                                        uint32_t duration);

  void idleMode(int adc_channel);

  bool isChannelActive(int adc_channel);

  void hardReset();

  void restoreCalibrationFromFlash();

  void reset();

  uint8_t talkADC(byte command);

  float setConversionTime(int channel, float time_us);

  float setConversionTimeFloat(int channel, float time_us,
                               bool moreThanOneChannelActive);

  float setConversionTime(int channel, bool chop, byte fw,
                          bool moreThanOneChannelActive);

  float setConversionTimeFW(int channel, int filter_word);

  float getConversionTime(int channel);

  float getConversionTime(int channel, bool moreThanOneChannelActive);

  float calculateConversionTime(byte b, bool moreThanOneChannelActive);

  byte calculateFilterWord(float time_us, bool chop,
                           bool moreThanOneChannelActive);

  bool isMoreThanOneChannelActive();

  void zeroScaleSelfCalibration();

  void zeroScaleChannelSystemSelfCalibration(int channel);

  void fullScaleChannelSystemSelfCalibration(int channel);
};
