#include "Utils/TimingUtil.h"

#include "Config.h"
#include "Utils/FastGpio.h"

volatile uint8_t TimingUtil::adcFlag = 0;
volatile bool TimingUtil::dacFlag = false;

namespace {
void enableTimerClock(uint32_t clock_enable_bits) {
  RCC->APB2ENR |= clock_enable_bits;
  const uint32_t apb2enr = RCC->APB2ENR;
  (void)apb2enr;
  __DMB();
}
}

void TimingUtil::resetTimers() {
  __disable_irq();

  __HAL_RCC_TIM1_FORCE_RESET();
  __HAL_RCC_TIM8_FORCE_RESET();
  __HAL_RCC_TIM1_RELEASE_RESET();
  __HAL_RCC_TIM8_RELEASE_RESET();

  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_TIM8_CLK_DISABLE();

  NVIC_DisableIRQ(TIM1_UP_IRQn);
  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  NVIC_DisableIRQ(TIM8_CC_IRQn);
  NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
  NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
  NVIC_ClearPendingIRQ(TIM8_CC_IRQn);

  adcFlag = 0;
  dacFlag = false;

  __enable_irq();
  delayMicroseconds(5);
}

void TimingUtil::stopAndResetAdcTimer() {
  TIM8->CR1 &= ~TIM_CR1_CEN;
  TIM8->CNT = 0;
}

void TimingUtil::startAdcTimer() {
  TIM8->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::setupTimerOnlyDac(uint32_t period_us) {
  resetTimers();

  enableTimerClock(RCC_APB2ENR_TIM1EN);

  uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
  uint64_t total_ticks_dac = (period_us * timerClock) / 1000000;

  uint16_t psc_dac;
  uint16_t arr_dac;

  if (total_ticks_dac <= 65536) {
    psc_dac = 0;
    arr_dac = total_ticks_dac - 1;
  } else {
    uint32_t prescaler_dac = (total_ticks_dac + 65536 - 1) / 65536;
    psc_dac = prescaler_dac - 1;
    arr_dac = (total_ticks_dac / prescaler_dac) - 1;
  }

  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->PSC = psc_dac;
  TIM1->ARR = arr_dac;
  TIM1->CR1 = TIM_CR1_ARPE;
  TIM1->DIER |= TIM_DIER_UIE;

  NVIC_SetPriority(TIM1_UP_IRQn, 2);
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  TIM1->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::setupTimersOnlyADC(uint32_t adc_period_us) {
  resetTimers();

  enableTimerClock(RCC_APB2ENR_TIM8EN);

  uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
  uint64_t total_ticks_adc = (adc_period_us * timerClock) / 1000000;

  uint16_t psc_adc;
  uint16_t arr_adc;

  if (total_ticks_adc <= 65536) {
    psc_adc = 0;
    arr_adc = total_ticks_adc - 1;
  } else {
    uint32_t prescaler_adc = (total_ticks_adc + 65536 - 1) / 65536;
    psc_adc = prescaler_adc - 1;
    arr_adc = (total_ticks_adc / prescaler_adc) - 1;
  }

  TIM8->CR1 &= ~TIM_CR1_CEN;
  TIM8->PSC = psc_adc;
  TIM8->ARR = arr_adc;
  TIM8->CR1 = TIM_CR1_ARPE;
  TIM8->DIER |= TIM_DIER_UIE;

  TIM8->EGR |= 0x01;
  TIM8->SR &= ~TIM_SR_UIF;

  NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3);
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

  TIM8->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::setupTimersTimeSeries(uint32_t dac_period_us,
                                       uint32_t adc_period_us) {
  resetTimers();

  enableTimerClock(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN);

  uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
  uint64_t total_ticks_dac = (dac_period_us * timerClock) / 1000000;

  uint16_t psc_dac;
  uint16_t arr_dac;

  if (total_ticks_dac <= 65536) {
    psc_dac = 0;
    arr_dac = total_ticks_dac - 1;
  } else {
    uint32_t prescaler_dac = (total_ticks_dac + 65536 - 1) / 65536;
    psc_dac = prescaler_dac - 1;
    arr_dac = (total_ticks_dac / prescaler_dac) - 1;
  }

  uint64_t total_ticks_adc = (adc_period_us * timerClock) / 1000000;

  uint16_t psc_adc;
  uint16_t arr_adc;

  if (total_ticks_adc <= 65536) {
    psc_adc = 0;
    arr_adc = total_ticks_adc - 1;
  } else {
    uint32_t prescaler_adc = (total_ticks_adc + 65536 - 1) / 65536;
    psc_adc = prescaler_adc - 1;
    arr_adc = (total_ticks_adc / prescaler_adc) - 1;
  }

  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->PSC = psc_dac;
  TIM1->ARR = arr_dac;
  TIM1->CR1 = TIM_CR1_ARPE;
  TIM1->DIER |= TIM_DIER_UIE;

  TIM1->EGR |= 0x01;
  TIM1->SR &= ~TIM_SR_UIF;

  TIM8->CR1 &= ~TIM_CR1_CEN;
  TIM8->PSC = psc_adc;
  TIM8->ARR = arr_adc;
  TIM8->CR1 = TIM_CR1_ARPE;
  TIM8->DIER |= TIM_DIER_UIE;

  TIM8->EGR |= 0x01;
  TIM8->SR &= ~TIM_SR_UIF;

  NVIC_SetPriority(TIM1_UP_IRQn, 2);
  NVIC_EnableIRQ(TIM1_UP_IRQn);
  NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3);
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

  TIM1->CR1 |= TIM_CR1_CEN;
  TIM8->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::setupTimersDacLed(uint64_t period_us,
                                   uint64_t phase_shift_us) {
  resetTimers();

  enableTimerClock(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN);

  uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
  uint64_t total_ticks = (period_us * timerClock) / 1000000;

  uint16_t psc;
  uint16_t arr;

  if (total_ticks <= 65536) {
    psc = 0;
    arr = total_ticks - 1;
  } else {
    uint32_t prescaler = (total_ticks + 65536 - 1) / 65536;
    psc = prescaler - 1;
    arr = (total_ticks / prescaler) - 1;
  }

  TIM1->PSC = psc;
  TIM1->ARR = arr;
  TIM1->CR1 = TIM_CR1_ARPE;
  TIM1->CNT = 0;

  TIM1->CR2 &= ~TIM_CR2_MMS;
  TIM1->CR2 |= TIM_CR2_MMS_1;
  TIM1->DIER |= TIM_DIER_UIE;

  TIM8->PSC = psc;
  TIM8->ARR = arr;
  TIM8->CR1 = TIM_CR1_ARPE;
  TIM8->CNT = 0;

  TIM8->SMCR &= ~TIM_SMCR_TS;
  TIM8->SMCR &= ~TIM_SMCR_SMS;
  TIM8->SMCR |= TIM_SMCR_SMS_3;

  if (phase_shift_us > 0 && phase_shift_us < period_us) {
    uint32_t timerPhaseShift = (phase_shift_us * (TIM8->ARR + 1)) / period_us;
    TIM8->CCR1 = timerPhaseShift;
    TIM8->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM8->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
    TIM8->CCER |= TIM_CCER_CC1E;
    TIM8->DIER |= TIM_DIER_CC1IE;
  } else {
    TIM8->DIER |= TIM_DIER_UIE;
  }

  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->EGR |= 0x01;
  TIM1->SR &= ~TIM_SR_UIF;
  TIM1->EGR |= 0x02;
  TIM1->SR &= ~TIM_SR_CC1IF;

  TIM8->CR1 &= ~TIM_CR1_CEN;
  TIM8->EGR |= 0x01;
  TIM8->SR &= ~TIM_SR_UIF;
  TIM8->EGR |= 0x02;
  TIM8->SR &= ~TIM_SR_CC1IF;

  NVIC_SetPriority(TIM1_UP_IRQn, 2);
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  NVIC_SetPriority(TIM8_CC_IRQn, 3);
  NVIC_EnableIRQ(TIM8_CC_IRQn);

  TIM1->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::disableDacInterrupt() {
  TIM1->DIER &= ~TIM_DIER_UIE;
  NVIC_DisableIRQ(TIM1_UP_IRQn);
}

void TimingUtil::disableAdcInterrupt() {
  TIM8->DIER &= ~TIM_DIER_UIE;
  TIM8->DIER &= ~TIM_DIER_CC1IE;
  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  NVIC_DisableIRQ(TIM8_CC_IRQn);
}

extern "C" void TIM1_UP_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR &= ~TIM_SR_UIF;
    FastGpio::pulseLowHigh(ldac);
    TimingUtil::dacFlag = true;
    __SEV();
  }
}

extern "C" void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_UIF) {
    TIM8->SR &= ~TIM_SR_UIF;
#ifdef __NEW_DAC_ADC__
    FastGpio::digitalWrite(adc_sync, true);
#else
    TimingUtil::adcFlag = true;
    __SEV();
#endif
  }
}

extern "C" void TIM8_CC_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_CC1IF) {
    TIM8->SR &= ~TIM_SR_CC1IF;
#ifdef __NEW_DAC_ADC__
    FastGpio::digitalWrite(adc_sync, true);
#else
    TimingUtil::adcFlag = true;
    __SEV();
#endif
  }
}
