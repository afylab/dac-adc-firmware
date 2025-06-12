#include <Arduino.h>

#include "Utils/shared_memory.h"
#include "Utils/flash.h"

#define NUM_DAC_CHANNELS 16

// Add STM32H7 register access for M7 DMA initialization
#if defined(ARDUINO_GIGA) || defined(CORE_STM32H7)
#include "stm32h7xx.h"
#else
#error "This code is intended for STM32H7 based boards like Arduino Giga."
#endif

// DMAMUX Request IDs for SPI
#define SPI1_TX_DMA 38
#define SPI1_RX_DMA 37
#define SPI5_TX_DMA 86
#define SPI5_RX_DMA 85

// Fallback definitions if not available in headers
#ifndef RCC_AHB1ENR_DMAMUX1EN
#define RCC_AHB1ENR_DMAMUX1EN (1U << 2) // Bit 2 in AHB1ENR for DMAMUX1
#endif

#ifndef RCC_APB1LENR_SPI5EN
#define RCC_APB1LENR_SPI5EN (1U << 20) // Bit 20 in APB1LENR for SPI5
#endif

#define return_if_not_ok(x) \
  do                        \
  {                         \
    int ret = x;            \
    if (ret != HAL_OK)      \
      return;               \
  } while (0);

void initDmaForM4() {
  // Serial.println("M7: Initializing DMA for M4 core...");
  
  // 1. Enable Clocks for DMA, DMAMUX, SPI1, SPI5
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
  SET_BIT(RCC->APB1LENR, RCC_APB1LENR_SPI5EN);
  
  // Ensure clocks are enabled
  volatile uint32_t dummy_read = RCC->AHB1ENR;
  (void)dummy_read;
  delay(1);
  
  // 2. Configure DMAMUX for SPI1 (DAC)
  DMAMUX1_Channel0->CCR = SPI1_TX_DMA; // SPI1 TX -> DMA1 Stream 0
  DMAMUX1_Channel1->CCR = SPI1_RX_DMA; // SPI1 RX -> DMA1 Stream 1
  
  // 3. Configure DMAMUX for SPI5 (ADC on new shield)
  DMAMUX1_Channel2->CCR = SPI5_TX_DMA; // SPI5 TX -> DMA1 Stream 2  
  DMAMUX1_Channel3->CCR = SPI5_RX_DMA; // SPI5 RX -> DMA1 Stream 3
  
  // 4. Initialize DMA streams (basic setup, M4 will configure per-transfer)
  // DMA1 Stream 0 (SPI1 TX)
  DMA1_Stream0->CR &= ~DMA_SxCR_EN;
  while(DMA1_Stream0->CR & DMA_SxCR_EN);
  DMA1->LIFCR = 0x3F; // Clear all flags for stream 0
  
  // DMA1 Stream 1 (SPI1 RX) 
  DMA1_Stream1->CR &= ~DMA_SxCR_EN;
  while(DMA1_Stream1->CR & DMA_SxCR_EN);
  DMA1->LIFCR = (0x3F << 6); // Clear all flags for stream 1
  
  // DMA1 Stream 2 (SPI5 TX)
  DMA1_Stream2->CR &= ~DMA_SxCR_EN;
  while(DMA1_Stream2->CR & DMA_SxCR_EN);
  DMA1->LIFCR = (0x3F << 16); // Clear all flags for stream 2
  
  // DMA1 Stream 3 (SPI5 RX)
  DMA1_Stream3->CR &= ~DMA_SxCR_EN;
  while(DMA1_Stream3->CR & DMA_SxCR_EN);
  DMA1->LIFCR = (0x3F << 22); // Clear all flags for stream 3
  
  // 5. Set flag in shared memory that DMA is ready
  shared_memory->isBootComplete = true; // Reuse this flag for DMA ready
  
  // Serial.println("M7: DMA initialization complete. M4 can now use DMA.");
}

void enableM4()
{
  HAL_MPU_Disable();

  // Disable caching for the shared memory region.
  MPU_Region_InitTypeDef MPU_InitStruct;
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = D3_SRAM_BASE;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER15;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // Enable the MPU
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  // If CM4 is already booted, disable auto-boot and reset.
  FLASH_OBProgramInitTypeDef OBInit;

  OBInit.Banks = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);
  if (OBInit.USERConfig & FLASH_OPTSR_BCM4)
  {
    OBInit.OptionType = OPTIONBYTE_USER;
    OBInit.USERType = OB_USER_BCM4;
    OBInit.USERConfig = 0;
    return_if_not_ok(HAL_FLASH_OB_Unlock());
    return_if_not_ok(HAL_FLASH_Unlock());
    return_if_not_ok(HAL_FLASHEx_OBProgram(&OBInit));
    return_if_not_ok(HAL_FLASH_OB_Launch());
    return_if_not_ok(HAL_FLASH_OB_Lock());
    return_if_not_ok(HAL_FLASH_Lock());
    printf("CM4 autoboot disabled\n");
    NVIC_SystemReset();
    return;
  }

  // Classic boot, just set the address and we are ready to go
  LL_SYSCFG_SetCM4BootAddress0(CM4_BINARY_START >> 16);
  LL_RCC_ForceCM4Boot();
}

typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;


void setup()
{
  enableM4();

  if (!initSharedMemory())
  {
    while (1)
    {
      Serial.println("Failed to initialize shared memory");
      delay(1000);
    }
  }
  CalibrationData calibrationData;
  if (!readCalibrationFromFlash(calibrationData))
  {
    Serial.println("Failed to read calibration data from flash. Using default values!");
    for (size_t i = 0; i < NUM_DAC_CHANNELS; ++i)
    {
      calibrationData.gain[i] = 1.0f;
      calibrationData.offset[i] = 0.0f;
      calibrationData.adc_offset[i] = 0x800000; // Default ADC offset
      calibrationData.adc_gain[i] = 0x200000; // Default ADC gain
      
    }
  }
  else
  {
    // Validate calibration data and fix if corrupted
    bool calibration_corrupted = false;
    for (size_t i = 0; i < NUM_DAC_CHANNELS; ++i)
    {
      // Check if gain is outside reasonable range [0.5, 1.5]
      if (calibrationData.gain[i] < 0.5f || calibrationData.gain[i] > 1.5f)
      {
        calibrationData.gain[i] = 1.0f;
        calibration_corrupted = true;
      }
      
      // Check if offset is outside reasonable range [-1.0, 1.0]
      if (calibrationData.offset[i] < -1.0f || calibrationData.offset[i] > 1.0f)
      {
        calibrationData.offset[i] = 0.0f;
        calibration_corrupted = true;
      }
    }
    
    if (calibration_corrupted)
    {
      Serial.println("Calibration data was corrupted and has been reset to defaults.");
    }
  }

  initDmaForM4();
  m7SendCalibrationData(calibrationData);
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    String command_lower = command;
    command_lower.toLowerCase();
    if (command_lower == "stop")
    {
      setStopFlag(true);
    }
    else
    {
      m7SendChar(command.c_str(), command.length());
    }
  }
  if (m7HasCharMessage())
  {
    char response[CHAR_BUFFER_SIZE];
    size_t size;
    if (m7ReceiveChar(response, size))
    {
      if (size > 0)
      {
        size--; // Decrease size to exclude the last character
      }
      Serial.write(response, size);
      Serial.println();
    }
  }
  if (m7HasFloatMessage())
  {
    float response[FLOAT_BUFFER_SIZE];
    size_t size;
    if (m7ReceiveFloat(response, size))
    {
      for (size_t i = 0; i < size; ++i)
      {
        Serial.print(response[i], 8);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  if (m7HasVoltageMessage())
  {
    double response[VOLTAGE_BUFFER_SIZE];
    size_t size;
    if (m7ReceiveVoltage(response, size))
    {
      for (size_t i = 0; i < size; ++i)
      {
        binaryFloat send;
        send.floatingPoint = static_cast<float>(response[i]);
        Serial.write(send.binary, 4);
        // Serial.println(response[i], 8);
      }
    }
  }
  if (isCalibrationUpdated())
  {
    CalibrationData calibrationData;
    m7ReceiveCalibrationData(calibrationData);
    if (!writeCalibrationToFlash(calibrationData))
    {
      Serial.println("Failed to write calibration data to flash!");
    }
  }
}