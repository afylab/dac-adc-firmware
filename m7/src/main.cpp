#include <Arduino.h>

#include "Utils/shared_memory.h"
#include "Utils/flash.h"

#define NUM_DAC_CHANNELS 16


#define return_if_not_ok(x) \
  do                        \
  {                         \
    int ret = x;            \
    if (ret != HAL_OK)      \
      return;               \
  } while (0);

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
    }
  }
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