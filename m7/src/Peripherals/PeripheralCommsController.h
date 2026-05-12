#pragma once

#include <Arduino.h>

#include "Peripherals/OperationResult.h"

class PeripheralCommsController {
 public:
  struct DmaDiagnostics {
    uint32_t spi_base;
    uint32_t spi_sr;
    uint32_t spi_cfg1;
    uint32_t spi_cfg2;
    uint32_t spi_cr1;
    uint32_t spi_cr2;
    uint32_t spi_ier;
    uint32_t spi_i2scfgr;
    uint32_t dma_lisr;
    uint32_t tx_cr;
    uint32_t rx_cr;
    uint32_t tx_m0ar;
    uint32_t rx_m0ar;
    uint32_t tx_ndtr;
    uint32_t rx_ndtr;
    uint32_t failure_stage;
    uint32_t count;
    bool is_dac;
    bool transfer_ok;
  };

 private:
  static bool spiInitialized;
  static bool dmaReady;
  static bool lastTransferOk;
  static DmaDiagnostics lastDiagnostics;
  int cs_pin;

  static constexpr size_t kDmaBufferSize = 64;
  static constexpr uint32_t kDmaTransferTimeout = 100000;

  static uint8_t __attribute__((aligned(32))) dma_tx_buffer[kDmaBufferSize];
  static uint8_t __attribute__((aligned(32))) dma_rx_buffer[kDmaBufferSize];

  static bool waitForDmaInit();
  bool performDmaTransfer(bool is_dac, uint8_t* tx_buffer,
                          uint8_t* rx_buffer, size_t count);

 public:
  explicit PeripheralCommsController(int cs_pin);

  static void setup();
  bool transferDAC(void* buf, size_t count);
  bool transferADC(void* buf, size_t count);
  uint8_t transferDAC(uint8_t data);
  uint8_t transferADC(uint8_t data);
  bool transferDACNoTransaction(void* buf, size_t count);
  bool transferADCNoTransaction(void* buf, size_t count);
  uint8_t transferDACNoTransaction(uint8_t data);
  uint8_t transferADCNoTransaction(uint8_t data);
  static bool lastTransferSucceeded();
  static OperationResult getDiagnostics();
  static void dataLedOn();
  static void dataLedOff();
};
