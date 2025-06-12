#pragma once

#include "stm32h7xx.h"
#include "Config.h"

struct TimingUtil {
  inline static volatile uint8_t adcFlag = 0;
  inline static volatile bool dacFlag = false;

  inline static void resetTimers() {
    // Disable all interrupts
    __disable_irq();

    // Reset TIM1 and TIM8
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM8_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();
    __HAL_RCC_TIM8_RELEASE_RESET();

    // Disable TIM1 and TIM8 clocks
    __HAL_RCC_TIM1_CLK_DISABLE();
    __HAL_RCC_TIM8_CLK_DISABLE();

    // Disable and clear all relevant interrupts
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
    NVIC_DisableIRQ(TIM8_CC_IRQn);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
    NVIC_ClearPendingIRQ(TIM8_CC_IRQn);

    // Reset flags
    adcFlag = 0;
    dacFlag = false;

    // Re-enable interrupts
    __enable_irq();
    delayMicroseconds(5);
  }

  inline static void setupTimerOnlyDac(uint32_t period_us) {
    resetTimers();

    // Enable TIM1 clock
    __HAL_RCC_TIM1_CLK_ENABLE();

    uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
    
    uint64_t total_ticks_dac = (period_us * timerClock) / 1000000;

    uint16_t psc_dac;
    uint16_t arr_dac;

    if (total_ticks_dac <= 65536) {
        psc_dac = 0;                // No prescaling
        arr_dac = total_ticks_dac - 1;    // Full resolution within 16 bits
    } else {
        // Compute the minimal prescaler (PSC+1) needed so that ARR <= 65535
        uint32_t prescaler_dac = (total_ticks_dac + 65536 - 1) / 65536;  // Rounds up division
        psc_dac = prescaler_dac - 1;   // Because PSC register = (PSC+1) - 1
        arr_dac = (total_ticks_dac / prescaler_dac) - 1;  // ARR counts from 0 to ARR, hence subtract 1
    }

    // Configure TIM1
    TIM1->PSC = psc_dac;
    TIM1->ARR = arr_dac;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->DIER |= TIM_DIER_UIE;

    // Enable interrupts
    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    // Start timer
    TIM1->CR1 |= TIM_CR1_CEN;
  }

  inline static void setupTimersOnlyADC(uint32_t adc_period_us) {
    resetTimers();

    // Enable TIM8 clock
    __HAL_RCC_TIM8_CLK_ENABLE();

    uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
    
    uint64_t total_ticks_adc = (adc_period_us * timerClock) / 1000000;

    uint16_t psc_adc;
    uint16_t arr_adc;

    if (total_ticks_adc <= 65536) {
        psc_adc = 0;                // No prescaling
        arr_adc = total_ticks_adc - 1;    // Full resolution within 16 bits
    } else {
        // Compute the minimal prescaler (PSC+1) needed so that ARR <= 65535
        uint32_t prescaler_adc = (total_ticks_adc + 65536 - 1) / 65536;  // Rounds up division
        psc_adc = prescaler_adc - 1;   // Because PSC register = (PSC+1) - 1
        arr_adc = (total_ticks_adc / prescaler_adc) - 1;  // ARR counts from 0 to ARR, hence subtract 1
    }

    // Configure TIM8
    TIM8->PSC = psc_adc;
    TIM8->ARR = arr_adc;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->DIER |= TIM_DIER_UIE;

    TIM8->EGR |= 0x01;
    TIM8->SR &= ~TIM_SR_UIF;

    // Enable interrupts
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3);
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    // Start timer
    TIM8->CR1 |= TIM_CR1_CEN;
  }

  inline static void setupTimersTimeSeries(uint32_t dac_period_us, uint32_t adc_period_us) {
    //******************************************************************/
    // setupTimersTimeSeries takes in two periods in microseconds and 
    // setups two individual timers to trigger interrupts based on their 
    // respective counter over flow.  The two timers run indpendent of
    // each other, but are synchronized to the same clock source.
    // The first timer (TIM1) is used for DAC updates, and the second timer
    // (TIM8) is used for ADC sampling. 
    //******************************************************************/
    resetTimers();

    // Enable TIM1 and TIM8 clocks
   __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    // Get the timer clock frequency
    uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
    
    // Calculate total timer ticks for the desired periods (in µs)
    uint64_t total_ticks_dac = (dac_period_us * timerClock) / 1000000;

    uint16_t psc_dac;
    uint16_t arr_dac;

    // Determine prescaler and 16-bit auto-reload value for TIM1 (DAC)
    // If total_ticks_dac is less than or equal to 65536, no prescaling is needed and we can use the full 16-bit resolution of the timer.
    // Otherwise, we need to compute the minimal prescaler (clock division = (PSC+1)) so that ARR <= 65535 and adjust the ARR value accordingly.
    
    if (total_ticks_dac <= 65536) {
        psc_dac = 0;                // No prescaling
        arr_dac = total_ticks_dac - 1;    // Full resolution within 16 bits
    } else {
        // Compute the minimal prescaler (PSC+1) needed so that ARR <= 65535
        uint32_t prescaler_dac = (total_ticks_dac + 65536 - 1) / 65536;  // Rounds up division
        psc_dac = prescaler_dac - 1;   // Because PSC register = (PSC+1) - 1
        arr_dac = (total_ticks_dac / prescaler_dac) - 1;  // ARR counts from 0 to ARR, hence subtract 1
    }

    // Determine prescaler and 16-bit auto-reload value for TIM8 (ADC)
    uint64_t total_ticks_adc = (adc_period_us * timerClock) / 1000000;

    uint16_t psc_adc;
    uint16_t arr_adc;

    if (total_ticks_adc <= 65536) {
        psc_adc = 0;                // No prescaling
        arr_adc = total_ticks_adc - 1;    // Full resolution within 16 bits
    } else {
        // Compute the minimal prescaler (PSC+1) needed so that ARR <= 65535
        uint32_t prescaler_adc = (total_ticks_adc + 65536 - 1) / 65536;  // Rounds up division
        psc_adc = prescaler_adc - 1;   // Because PSC register = (PSC+1) - 1
        arr_adc = (total_ticks_adc / prescaler_adc) - 1;  // ARR counts from 0 to ARR, hence subtract 1
    }

    // Configure TIM1
    TIM1->PSC = psc_dac;
    TIM1->ARR = arr_dac;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->DIER |= TIM_DIER_UIE;

    // Clear update interrupt flag and enable update event generation flags for TIM1
    TIM1->EGR |= 0x01;
    TIM1->SR &= ~TIM_SR_UIF;

    // Configure TIM8
    TIM8->PSC = psc_adc;
    TIM8->ARR = arr_adc;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->DIER |= TIM_DIER_UIE;

    // Clear update interrupt flag and enable update event generation flags for TIM8
    TIM8->EGR |= 0x01;
    TIM8->SR &= ~TIM_SR_UIF;

    // Enable interrupts
    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3);
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    // Start timers
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->CR1 |= TIM_CR1_CEN;
  }

  inline static void setupTimersDacLed(uint64_t period_us, uint64_t phase_shift_us) {
    // Reset timers and clear prior configuration
    resetTimers();
    // Enable clocks for TIM1 (master) and TIM8 (slave)
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
    
    // Calculate total timer ticks for the desired period (in µs)
    uint64_t total_ticks = (period_us * timerClock) / 1000000;

    uint16_t psc;
    uint16_t arr;

    if (total_ticks <= 65536) {
        psc = 0;                // No prescaling
        arr = total_ticks - 1;    // Full resolution within 16 bits
    } else {
        // Compute the minimal prescaler (PSC+1) needed so that ARR <= 65535
        uint32_t prescaler = (total_ticks + 65536 - 1) / 65536;  // Rounds up division
        psc = prescaler - 1;   // Because PSC register = (PSC+1) - 1
        arr = (total_ticks / prescaler) - 1;  // ARR counts from 0 to ARR, hence subtract 1
    }

    TIM1->PSC = psc;  // Set prescaler
    TIM1->ARR = arr;  // Set auto-reload value

    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->CNT = 0;  // Start at 0

    // Clear previous MMS bits then set master mode to “Update event” (TRGO)
    TIM1->CR2 &= ~TIM_CR2_MMS;
    TIM1->CR2 |= TIM_CR2_MMS_1;
    TIM1->DIER |= TIM_DIER_UIE;  // (Enable update interrupt if needed)

    // --- Configure TIM8 as Slave (for ADC triggering) ---
    TIM8->PSC = psc;
    TIM8->ARR = arr;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->CNT = 0;  // Start at 0

    // --- Set up TIM8 slave mode to use TIM1’s trigger ---
    // Clear TS bits to select ITR0 (which is 0 on STM32H7 devices)
    TIM8->SMCR &= ~TIM_SMCR_TS;
    // Clear the SMS bits...
    TIM8->SMCR &= ~TIM_SMCR_SMS;
    // ...and set SMS bits to 0b100 (Reset Mode)
    TIM8->SMCR |= TIM_SMCR_SMS_2;  // (Assuming SMS_2 represents the bit for value 4)
    // --- Configure phase shift if requested ---
    if (phase_shift_us > 0 && phase_shift_us < period_us) {
      // Use Channel 1 compare event for phase-shifted ADC trigger
      uint32_t timerPhaseShift = (phase_shift_us * (TIM8->ARR + 1)) / period_us;
      TIM8->CCR1 = timerPhaseShift;  // Set to the proper timer tick count for the phase shift
      // Set Channel 1 to PWM mode 1: clear then set OC1M bits
      TIM8->CCMR1 &= ~TIM_CCMR1_OC1M;
      TIM8->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; 
      TIM8->CCER |= TIM_CCER_CC1E;      // Enable CC1 output
      // Enable only the compare interrupt (disable UIE if not needed)
      TIM8->DIER |= TIM_DIER_CC1IE;
    } else {
      // No phase shift: use the update interrupt for ADC triggering.
      TIM8->DIER |= TIM_DIER_UIE;
    }
    // Clear update interrupt flag and enable update event generation flags for TIM1
    TIM1->EGR |= 0x01;
    TIM1->SR &= ~TIM_SR_UIF;
    TIM1->EGR |= 0x02;
    TIM1->SR &= ~TIM_SR_CC1IF;

    //Clear update interrupt flag and enable update event generation flags for TIM8
    TIM8->EGR |= 0x01;
    TIM8->SR &= ~TIM_SR_UIF;
    TIM8->EGR |= 0x02;
    TIM8->SR &= ~TIM_SR_CC1IF;

    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    NVIC_SetPriority(TIM8_CC_IRQn, 3);
    NVIC_EnableIRQ(TIM8_CC_IRQn);

    // Start the slave timer first so it’s waiting for TIM1’s trigger.
    TIM8->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CEN;
  }

  inline static void disableDacInterrupt() {
    TIM1->DIER &= ~TIM_DIER_UIE;
    NVIC_DisableIRQ(TIM1_UP_IRQn);
  }

  inline static void disableAdcInterrupt() {
    TIM8->DIER &= ~TIM_DIER_UIE;
    NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
    NVIC_DisableIRQ(TIM8_CC_IRQn);
  }

  #ifdef __NEW_DAC_ADC__
  template<int boardIndex>
  inline static void adcSyncISR() {
    adcFlag |= 1 << boardIndex;
    //digitalWrite(adc_sync, LOW);
    __SEV();
  }
  #endif
};

extern "C" void TIM1_UP_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR &= ~TIM_SR_UIF;
    digitalWrite(ldac, LOW);
    digitalWrite(ldac, HIGH);
    TimingUtil::dacFlag = true;
    __SEV();
  }
}

extern "C" void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_UIF) {
    TIM8->SR &= ~TIM_SR_UIF;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #else
    TimingUtil::adcFlag = 1;
    __SEV();
    #endif
  }
}

extern "C" void TIM8_CC_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_CC1IF) {
    TIM8->SR &= ~TIM_SR_CC1IF;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #else
    TimingUtil::adcFlag = 1;
    __SEV();
    #endif
  }
}