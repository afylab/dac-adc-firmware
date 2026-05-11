#pragma once

#include <Arduino.h>

class PeripheralCommsController {
 private:
  static bool spiInitialized;
  static bool dmaReady;
  static bool useDma;
  int cs_pin;

  static uint8_t __attribute__((aligned(32))) dma_tx_buffer[64];
  static uint8_t __attribute__((aligned(32))) dma_rx_buffer[64];

  static void waitForDmaInit();
  uint8_t performDmaTransfer(bool is_dac, uint8_t* tx_buffer,
                             uint8_t* rx_buffer, size_t count);

 public:
  explicit PeripheralCommsController(int cs_pin);

  static void setup();
  void transferDAC(void* buf, size_t count);
  void transferADC(void* buf, size_t count);
  uint8_t transferDAC(uint8_t data);
  uint8_t transferADC(uint8_t data);
  void transferDACNoTransaction(void* buf, size_t count);
  void transferADCNoTransaction(void* buf, size_t count);
  uint8_t transferDACNoTransaction(uint8_t data);
  uint8_t transferADCNoTransaction(uint8_t data);
  static void beginDacTransaction();
  static void beginAdcTransaction();
  static void dataLedOn();
  static void dataLedOff();
  static void endTransaction();
};
