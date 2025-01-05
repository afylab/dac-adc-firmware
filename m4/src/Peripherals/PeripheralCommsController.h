#pragma once

#include <Arduino.h>
#include "SPI.h"

class PeripheralCommsController {

 public:
  static bool spiInitialized;

  PeripheralCommsController() {}

  static void setup() {
    if (!spiInitialized) {
      SPI.begin();
      SPI.beginTransaction(DAC_SPI_SETTINGS);
      spiInitialized = true;
    }
  }

  static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }

  static void dataLedOff() { /*digitalWrite(led, LOW);*/ }

  byte receiveByte() {
    return SPI.transfer(0);
  }

  void beginTransaction() { SPI.beginTransaction(DAC_SPI_SETTINGS); }

  void endTransaction() { SPI.endTransaction(); }

  void transfer(void* buf, size_t count) { SPI.transfer(buf, count); }

  uint8_t transfer(uint8_t data) { return SPI.transfer(data); }
};

bool PeripheralCommsController::spiInitialized = false;
