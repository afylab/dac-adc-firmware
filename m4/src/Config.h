#pragma once
#include <Arduino.h>
#include <SPI.h>

#define NUM_CHANNELS_PER_DAC_BOARD 4
#define NUM_CHANNELS_PER_ADC_BOARD 4

#ifdef __OLD_SHIELD__
#warning "OLD_SHIELD is active"
#define NUM_ADC_BOARDS 2
#define NUM_DAC_CHANNELS 4
const int adc_cs_pins[NUM_ADC_BOARDS] = {48, 42}; // SYNC for both 24-bit ADCs
const int dac_cs_pins[NUM_DAC_CHANNELS] = {24, 26, 38, 36}; // CS for 4x 20-bit DAC channels
#define ldac 22 // LDAC pin shared across all AD5791 -- used to synchronize DAC voltage output
const int reset[NUM_ADC_BOARDS] = {46, 44}; // reset pins on ADC
const int drdy[NUM_ADC_BOARDS] = {50, 40}; // data_ready pin for both ADCs -- used as input to indicate ADC conversion has completed
#define led 7 // indicator LED
#define data_pin 6 // data indicator LED
#define err 11 // error indicator LED
const static SPISettings DAC_SPI_SETTINGS(4000000, MSBFIRST, SPI_MODE1);
const static SPISettings ADC_SPI_SETTINGS(4000000, MSBFIRST, SPI_MODE3);
#else
#warning "NEW_SHIELD is active"
#define NUM_ADC_BOARDS 2
#define NUM_DAC_CHANNELS 16
const int adc_cs_pins[NUM_ADC_BOARDS] = {39,40};//,41,42};
const int dac_cs_pins[NUM_DAC_CHANNELS] = {23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38};
#define ldac 22
const int reset[NUM_ADC_BOARDS] = {43,44};//,45,46};
const int drdy[NUM_ADC_BOARDS] = {47,48};//,49,50};

#define GPIO_0 52
#define GPIO_1 53
#define GPIO_2 5
#define GPIO_3 4



#ifdef __NEW_DAC_ADC__
#define adc_sync 51
#endif

#define led 7 // indicator LED
#define data_pin 6 // data indicator LED
#define err 11 // error indicator LED
const static SPISettings DAC_SPI_SETTINGS(22000000, MSBFIRST, SPI_MODE1 ); 
const static SPISettings ADC_SPI_SETTINGS(8000000, MSBFIRST, SPI_MODE0);
#endif

// Global DAC voltage limits for fast clamping
namespace DACLimits {
  inline float upper_voltage_limit = 10.0;
  inline float lower_voltage_limit = -10.0;
  
  // Super fast inline clamp function using ternary operators
  inline float clampVoltage(float v) {
    return (v > upper_voltage_limit) ? upper_voltage_limit : 
           ((v < lower_voltage_limit) ? lower_voltage_limit : v);
  }
}

