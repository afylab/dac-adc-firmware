#pragma once

#include <Arduino.h>

#include "stm32h7xx.h"

struct TimingUtil {
  static volatile uint8_t adcFlag;
  static volatile bool dacFlag;

  static void resetTimers();
  static void stopAndResetAdcTimer();
  static void startAdcTimer();
  static void setupTimerOnlyDac(uint32_t period_us);
  static void setupTimersOnlyADC(uint32_t adc_period_us);
  static void setupTimersTimeSeries(uint32_t dac_period_us,
                                    uint32_t adc_period_us);
  static void setupTimersDacLed(uint64_t period_us, uint64_t phase_shift_us);
  static void disableDacInterrupt();
  static void disableAdcInterrupt();

#ifdef __NEW_DAC_ADC__
  template <int boardIndex>
  static void adcSyncISR() {
    adcFlag |= 1 << boardIndex;
    __SEV();
  }
#endif
};
