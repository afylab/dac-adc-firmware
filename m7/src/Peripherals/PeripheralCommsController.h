#pragma once

#include <Arduino.h>

#include "Peripherals/OperationResult.h"

class PeripheralCommsController {
 public:
  struct SpiDiagnostics {
    uint32_t failure_stage;
    uint32_t count;
    uint32_t completed_transfers;
    uint32_t failed_transfers;
    uint32_t timeout_us;
    int cs_pin;
    int start_result;
    int callback_event;
    uint8_t spi_mode;
    bool is_dac;
    bool initialized;
    bool transfer_ok;
    bool timeout;
    bool callback_seen;
    bool deferred_errors;
    bool sticky_error;
    bool bus_shared;
  };

 private:
  static bool spiInitialized;
  static bool lastTransferOk;
  static bool deferSpiErrors;
  static bool stickySpiError;
  static uint8_t deferSpiErrorDepth;
  static SpiDiagnostics lastDiagnostics;
  static SpiDiagnostics firstStickyDiagnostics;
  int cs_pin;

  static constexpr size_t kSpiBufferSize = 64;
  static uint8_t __attribute__((aligned(32))) tx_buffer[kSpiBufferSize];
  static uint8_t __attribute__((aligned(32))) rx_buffer[kSpiBufferSize];

  bool performMbedTransfer(bool is_dac, uint8_t* tx, uint8_t* rx,
                           size_t count);

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
  static void beginDeferredSpiErrors();
  static OperationResult endDeferredSpiErrors();
  static void cancelDeferredSpiErrors();
  static bool hasDeferredSpiError();
  static void dataLedOn();
  static void dataLedOff();
};
