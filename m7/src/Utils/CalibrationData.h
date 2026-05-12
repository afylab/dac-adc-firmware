#pragma once

#include <stdint.h>

#include "Config.h"

#define NUM_DAC_CALIBRATION_CHANNELS NUM_DAC_CHANNELS
#define NUM_ADC_CALIBRATION_CHANNELS (NUM_ADC_BOARDS * NUM_CHANNELS_PER_ADC_BOARD)

struct CalibrationData {
  float gain[NUM_DAC_CALIBRATION_CHANNELS];
  float offset[NUM_DAC_CALIBRATION_CHANNELS];

  bool adcCalibrated;
  uint32_t adc_offset[NUM_ADC_CALIBRATION_CHANNELS];
  uint32_t adc_gain[NUM_ADC_CALIBRATION_CHANNELS];

  CalibrationData() : adcCalibrated(false) {
    for (int i = 0; i < NUM_DAC_CALIBRATION_CHANNELS; ++i) {
      gain[i] = 1.0f;
      offset[i] = 0.0f;
    }
    for (int i = 0; i < NUM_ADC_CALIBRATION_CHANNELS; ++i) {
      adc_offset[i] = 0x800000u; // Default ADC offset
      adc_gain[i] = 0x200000u;   // Default ADC gain
    }
  }
};
