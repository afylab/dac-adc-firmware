#pragma once

#include <Arduino.h>

#include "Peripherals/OperationResult.h"

struct UserIOHandler {
  static void setup();
  static OperationResult getFirmwareVersion();
  static OperationResult nop();
  static OperationResult getEnvironment();
  static OperationResult id();
  static OperationResult rdy();

  __attribute__((section(".serial_number")))
  static const char serial_number[29];

  static OperationResult serialNumber();
  static bool readCommandLine(String& out);
  static void handleUserIO();
};
