#pragma once

#include <cstddef>
#include <cstdint>

#include "CalibrationData.h"

// Fixed flash memory address for storing calibration (15 MB offset in 16 MB QSPI flash).
// This address is chosen to avoid the region used by WiFi firmware (first 1MB) and OTA (next 13MB).
static constexpr uint32_t CALIBRATION_FLASH_ADDR = 15 * 1024 * 1024;

uint32_t calculateCRC32(const uint8_t *data, size_t length);
bool writeCalibrationToFlash(const CalibrationData &data);
bool readCalibrationFromFlash(CalibrationData &data);
