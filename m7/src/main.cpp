#include <Arduino.h>

#include "Config.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/God.h"
#include "Peripherals/God2D.h"
#include "Peripherals/PeripheralCommsController.h"
#include "UserIOHandler.h"
#include "Utils/flash.h"
#include "Utils/shared_memory.h"

#if defined(ARDUINO_GIGA) || defined(CORE_STM32H7)
#include "stm32h7xx.h"
#else
#error "This code is intended for STM32H7 based boards like Arduino Giga."
#endif

constexpr char kFlashWriteFailure[] =
    "Failed to write calibration data to flash!";

#define return_if_not_ok(x) \
  do {                      \
    int ret = x;            \
    if (ret != HAL_OK)      \
      return;               \
  } while (0);

static void configureSharedMemoryMpu() {
  HAL_MPU_Disable();

  MPU_Region_InitTypeDef MPU_InitStruct;
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = SHARED_MEMORY_ADDRESS;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER15;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  __DSB();
  __ISB();
}

static void prepareM4UsbClock() {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_HSI48_ENABLE();
  __HAL_RCC_CRS_CLK_ENABLE();

  uint32_t start = HAL_GetTick();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY) == RESET &&
         (HAL_GetTick() - start) < 10) {
  }

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  RCC_CRSInitTypeDef CRSInitStruct = {};
  CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB2;
  CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  CRSInitStruct.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
  CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&CRSInitStruct);

  HAL_PWREx_EnableUSBVoltageDetector();
}

void enableM4() {
  FLASH_OBProgramInitTypeDef OBInit;

  OBInit.Banks = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);
  if (OBInit.USERConfig & FLASH_OPTSR_BCM4) {
    OBInit.OptionType = OPTIONBYTE_USER;
    OBInit.USERType = OB_USER_BCM4;
    OBInit.USERConfig = 0;
    return_if_not_ok(HAL_FLASH_OB_Unlock());
    return_if_not_ok(HAL_FLASH_Unlock());
    return_if_not_ok(HAL_FLASHEx_OBProgram(&OBInit));
    return_if_not_ok(HAL_FLASH_OB_Launch());
    return_if_not_ok(HAL_FLASH_OB_Lock());
    return_if_not_ok(HAL_FLASH_Lock());
    NVIC_SystemReset();
    return;
  }

  prepareM4UsbClock();

  LL_SYSCFG_SetCM4BootAddress0(CM4_BINARY_START >> 16);
  LL_RCC_ForceCM4Boot();
}

static CalibrationData loadCalibrationData() {
  CalibrationData calibration_data;
  if (!readCalibrationFromFlash(calibration_data)) {
    for (size_t i = 0; i < NUM_DAC_CHANNELS; ++i) {
      calibration_data.gain[i] = 1.0f;
      calibration_data.offset[i] = 0.0f;
      calibration_data.adc_offset[i] = 0x800000;
      calibration_data.adc_gain[i] = 0x200000;
    }
    calibration_data.adcCalibrated = false;
    return calibration_data;
  }

  for (size_t i = 0; i < NUM_DAC_CHANNELS; ++i) {
    if (calibration_data.gain[i] < 0.5f || calibration_data.gain[i] > 1.5f) {
      calibration_data.gain[i] = 1.0f;
    }
    if (calibration_data.offset[i] < -1.0f ||
        calibration_data.offset[i] > 1.0f) {
      calibration_data.offset[i] = 0.0f;
    }
  }

  return calibration_data;
}

static void setupWorker() {
  UserIOHandler::setup();

  PeripheralCommsController::setup();

  for (int i : dac_cs_pins) {
    DACController::addChannel(i);
  }

  for (int i = 0; i < NUM_ADC_BOARDS; i++) {
    ADCController::addBoard(adc_cs_pins[i], drdy[i], reset[i], i);
  }

  DACController::setup();
  ADCController::setup();

  God::setup();
  God2D::setup();

  while (!isCalibrationDataReady()) {
    delay(1);
  }

  CalibrationData calibration_data;
  readCalibrationData(calibration_data);

  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    DACController::applyCalibration(i, calibration_data.offset[i],
                                    calibration_data.gain[i]);
  }

  if (!calibration_data.adcCalibrated) {
    ADCController::hardResetAllADCBoards();
    for (int i = 0; i < NUM_ADC_BOARDS * NUM_CHANNELS_PER_ADC_BOARD; i++) {
      uint32_t zeroScaleCalibration =
          ADCController::getChZeroScaleCalibration(i).getMessage().toInt();
      uint32_t fullScaleCalibration =
          ADCController::getChFullScaleCalibration(i).getMessage().toInt();

      calibration_data.adc_offset[i] = zeroScaleCalibration;
      calibration_data.adc_gain[i] = fullScaleCalibration;
      calibration_data.adcCalibrated = true;
    }
    updateCalibrationData(calibration_data);
  } else {
    for (int i = 0; i < NUM_ADC_BOARDS * NUM_CHANNELS_PER_ADC_BOARD; i++) {
      ADCController::applyChZeroScaleCalibration(i,
                                                 calibration_data.adc_offset[i]);

      if (calibration_data.adc_gain[i] != 0) {
        ADCController::applyChFullScaleCalibration(i,
                                                   calibration_data.adc_gain[i]);
      }
    }
  }
}

void setup() {
  configureSharedMemoryMpu();

  if (!initSharedMemory()) {
    while (1) {
      delay(1000);
    }
  }

  enableM4();

  CalibrationData calibration_data = loadCalibrationData();
  publishCalibrationData(calibration_data);

  setupWorker();
}

void loop() {
  UserIOHandler::handleUserIO();

  if (isCalibrationDataUpdated()) {
    CalibrationData calibration_data;
    readCalibrationData(calibration_data);
    if (!writeCalibrationToFlash(calibration_data)) {
      sendTextToGateway(kFlashWriteFailure, sizeof(kFlashWriteFailure));
    }
    clearCalibrationDataUpdated();
  }
}
