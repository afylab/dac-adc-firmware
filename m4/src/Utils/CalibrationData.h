#pragma once

#define NUM_CHANNELS 16

struct CalibrationData {
  float gain[NUM_CHANNELS];
  float offset[NUM_CHANNELS];

  bool adcCalibrated;
  uint32_t adc_offset[NUM_CHANNELS];
  uint32_t adc_gain[NUM_CHANNELS];

  CalibrationData() : adcCalibrated(false) {
    for (int i = 0; i < NUM_CHANNELS; ++i) {
      gain[i] = 1.0f;
      offset[i] = 0.0f;
      adc_offset[i] = 0x800000u; // Default ADC offset
      adc_gain[i] = 0x200000u;   // Default ADC gain
    }
  }
};