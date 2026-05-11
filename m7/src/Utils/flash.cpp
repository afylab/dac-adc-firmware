#include <Arduino.h>

#ifdef abs
#define DAC_ADC_RESTORE_ARDUINO_ABS
#undef abs
#endif
#include "QSPIFBlockDevice.h"
#ifdef DAC_ADC_RESTORE_ARDUINO_ABS
#define abs(x) ((x)>0?(x):-(x))
#undef DAC_ADC_RESTORE_ARDUINO_ABS
#endif

#include "Utils/flash.h"

#include <cstring>

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }
  crc ^= 0xFFFFFFFF;
  return crc;
}

bool writeCalibrationToFlash(const CalibrationData &data) {
  QSPIFBlockDevice qspi(QSPI_SO0, QSPI_SO1, QSPI_SO2, QSPI_SO3, QSPI_SCK, QSPI_CS,
                        QSPIF_POLARITY_MODE_1, 40000000);

  int err = qspi.init();
  if (err != 0) {
    Serial.println("FAILURE: QSPI flash init failed");
    return false;
  }

  uint8_t dataBuf[sizeof(CalibrationData)];
  memcpy(dataBuf, &data, sizeof(CalibrationData));

  uint32_t crc = calculateCRC32(dataBuf, sizeof(CalibrationData));

  const size_t totalSize = sizeof(CalibrationData) + sizeof(crc);
  uint8_t writeBuf[totalSize];
  memcpy(writeBuf, dataBuf, sizeof(CalibrationData));
  memcpy(writeBuf + sizeof(CalibrationData), &crc, sizeof(crc));

  size_t eraseSize = qspi.get_erase_size(CALIBRATION_FLASH_ADDR);
  uint32_t alignedAddr = CALIBRATION_FLASH_ADDR - (CALIBRATION_FLASH_ADDR % eraseSize);
  err = qspi.erase(alignedAddr, eraseSize);
  if (err != 0) {
    Serial.println("FAILURE: QSPI flash erase failed");
    qspi.deinit();
    return false;
  }

  err = qspi.program(writeBuf, CALIBRATION_FLASH_ADDR, totalSize);
  if (err != 0) {
    Serial.println("FAILURE: QSPI flash program failed");
    qspi.deinit();
    return false;
  }

  qspi.deinit();
  return true;
}

bool readCalibrationFromFlash(CalibrationData &data) {
  QSPIFBlockDevice qspi(QSPI_SO0, QSPI_SO1, QSPI_SO2, QSPI_SO3, QSPI_SCK, QSPI_CS,
                        QSPIF_POLARITY_MODE_1, 40000000);

  int err = qspi.init();
  if (err != 0) {
    Serial.println("FAILURE: QSPI flash init failed");
    return false;
  }

  const size_t totalSize = sizeof(CalibrationData) + sizeof(uint32_t);
  uint8_t readBuf[totalSize];

  err = qspi.read(readBuf, CALIBRATION_FLASH_ADDR, totalSize);
  if (err != 0) {
    Serial.println("FAILURE: QSPI flash read failed");
    qspi.deinit();
    return false;
  }

  uint8_t rawData[sizeof(CalibrationData)];
  memcpy(rawData, readBuf, sizeof(CalibrationData));

  uint32_t storedCrc;
  memcpy(&storedCrc, readBuf + sizeof(CalibrationData), sizeof(storedCrc));

  uint32_t calcCrc = calculateCRC32(rawData, sizeof(CalibrationData));
  if (calcCrc != storedCrc) {
    Serial.println("FAILURE: Calibration data CRC mismatch (data is corrupted or not written)");
    qspi.deinit();
    return false;
  }

  memcpy(&data, rawData, sizeof(CalibrationData));
  qspi.deinit();
  return true;
}
