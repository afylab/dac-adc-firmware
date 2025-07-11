#pragma once

struct CalibrationData {
  float gain[16];
  float offset[16];

  bool adcCalibrated = false; // Flag to indicate if ADC calibration is done
  uint32_t adc_offset[16];
  uint32_t adc_gain[16];
};