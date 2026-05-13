#include "Peripherals/PeripheralCommsController.h"

#include <cstring>
#include <new>

#include "Config.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "drivers/SPIMaster.h"
#include "hal/dma_api.h"
#include "hal/spi_api.h"
#include "Utils/FastGpio.h"

#ifndef DEVICE_SPI_ASYNCH
#define DEVICE_SPI_ASYNCH 0
#endif

#if !DEVICE_SPI_ASYNCH
#error "PeripheralCommsController requires Mbed async SPI support (DEVICE_SPI_ASYNCH=1)."
#endif

bool PeripheralCommsController::spiInitialized = false;
bool PeripheralCommsController::lastTransferOk = true;
bool PeripheralCommsController::deferSpiErrors = false;
bool PeripheralCommsController::stickySpiError = false;
uint8_t PeripheralCommsController::deferSpiErrorDepth = 0;
PeripheralCommsController::SpiDiagnostics
    PeripheralCommsController::lastDiagnostics = {};
PeripheralCommsController::SpiDiagnostics
    PeripheralCommsController::firstStickyDiagnostics = {};
uint8_t PeripheralCommsController::tx_buffer
    [PeripheralCommsController::kSpiBufferSize] __attribute__((aligned(32)));
uint8_t PeripheralCommsController::rx_buffer
    [PeripheralCommsController::kSpiBufferSize] __attribute__((aligned(32)));

namespace {
constexpr uint32_t kFailureNone = 0;
constexpr uint32_t kFailureNotInitialized = 1;
constexpr uint32_t kFailureCountTooLarge = 2;
constexpr uint32_t kFailureStart = 3;
constexpr uint32_t kFailureTimeout = 4;
constexpr uint32_t kFailureEvent = 5;
constexpr uint32_t kFailureDmaUsage = 6;
constexpr uint32_t kSpiTransferTimeoutUs = 50000;

struct SpiBusState {
  mbed::SPI* spi;
  uint32_t frequency_hz;
  uint8_t mode;
  bool configured;
  bool dma_usage_set;
  bool shared;
};

alignas(mbed::SPI) uint8_t dacSpiStorage[sizeof(mbed::SPI)];
#ifdef __NEW_SHIELD__
alignas(mbed::SPI) uint8_t adcSpiStorage[sizeof(mbed::SPI)];
#endif

mbed::SPI* dacSpi = nullptr;
mbed::SPI* adcSpi = nullptr;
SpiBusState dacBus{nullptr, 0, 0, false, false, false};
#ifdef __NEW_SHIELD__
SpiBusState adcBus{nullptr, 0, 0, false, false, false};
#endif

volatile bool asyncTransferDone = false;
volatile int asyncTransferEvent = 0;
uint32_t completedTransfers = 0;
uint32_t failedTransfers = 0;

void asyncTransferCallback(int event) {
  asyncTransferEvent = event;
  asyncTransferDone = true;
}

uint32_t transferFrequency(bool isDac) {
  return isDac ? DAC_SPI_FREQUENCY_HZ : ADC_SPI_FREQUENCY_HZ;
}

uint8_t transferMode(bool isDac) {
  return isDac ? DAC_SPI_MODE : ADC_SPI_MODE;
}

SpiBusState& busForTransfer(bool isDac) {
#ifdef __NEW_SHIELD__
  return isDac ? dacBus : adcBus;
#else
  (void)isDac;
  return dacBus;
#endif
}

void constructSpiBuses() {
  if (dacSpi == nullptr) {
    dacSpi = new (dacSpiStorage) mbed::SPI(PD_7, PG_9, PB_3, NC);
    dacBus.spi = dacSpi;
#ifndef __NEW_SHIELD__
    adcSpi = dacSpi;
    dacBus.shared = true;
#endif
  }

#ifdef __NEW_SHIELD__
  if (adcSpi == nullptr) {
    adcSpi = new (adcSpiStorage) mbed::SPI(PJ_10, PJ_11, PH_6, NC);
    adcBus.spi = adcSpi;
  }
#endif
}

bool configureDmaUsage(SpiBusState& bus) {
  if (bus.spi == nullptr) {
    return false;
  }
  if (bus.dma_usage_set) {
    return true;
  }
  if (bus.spi->set_dma_usage(DMA_USAGE_ALWAYS) != 0) {
    return false;
  }
  bus.dma_usage_set = true;
  return true;
}

bool configureBusForTransfer(bool isDac) {
  SpiBusState& bus = busForTransfer(isDac);
  if (!configureDmaUsage(bus)) {
    return false;
  }

  const uint32_t frequency = transferFrequency(isDac);
  const uint8_t mode = transferMode(isDac);
  if (!bus.configured || bus.frequency_hz != frequency || bus.mode != mode) {
    bus.spi->format(8, mode);
    bus.spi->frequency(frequency);
    bus.frequency_hz = frequency;
    bus.mode = mode;
    bus.configured = true;
  }
  return true;
}

void clearCallerBuffer(uint8_t* tx, uint8_t* rx, size_t count) {
  if (rx != nullptr) {
    memset(rx, 0, count);
  } else if (tx != nullptr) {
    memset(tx, 0, count);
  }
}

PeripheralCommsController::SpiDiagnostics makeDiagnostics(
    bool initialized, bool isDac, int csPin, size_t count,
    uint32_t failureStage, bool transferOk, bool timeout, bool callbackSeen,
    int startResult, int callbackEvent, bool deferredErrors,
    bool stickyError) {
  const SpiBusState& bus = busForTransfer(isDac);
  PeripheralCommsController::SpiDiagnostics diagnostics{};
  diagnostics.failure_stage = failureStage;
  diagnostics.count = static_cast<uint32_t>(count);
  diagnostics.completed_transfers = completedTransfers;
  diagnostics.failed_transfers = failedTransfers;
  diagnostics.timeout_us = kSpiTransferTimeoutUs;
  diagnostics.cs_pin = csPin;
  diagnostics.start_result = startResult;
  diagnostics.callback_event = callbackEvent;
  diagnostics.spi_mode = transferMode(isDac);
  diagnostics.is_dac = isDac;
  diagnostics.initialized = initialized;
  diagnostics.transfer_ok = transferOk;
  diagnostics.timeout = timeout;
  diagnostics.callback_seen = callbackSeen;
  diagnostics.deferred_errors = deferredErrors;
  diagnostics.sticky_error = stickyError;
  diagnostics.bus_shared = bus.shared;
  return diagnostics;
}

String diagnosticsToString(
    const PeripheralCommsController::SpiDiagnostics& diagnostics) {
  String out = "backend=mbed_async_dma";
  out += " ok=" + String(diagnostics.transfer_ok ? 1 : 0);
  out += " init=" + String(diagnostics.initialized ? 1 : 0);
  out += " st=" + String(diagnostics.failure_stage);
  out += " dac=" + String(diagnostics.is_dac ? 1 : 0);
  out += " shared=" + String(diagnostics.bus_shared ? 1 : 0);
  out += " cs=" + String(diagnostics.cs_pin);
  out += " n=" + String(diagnostics.count);
  out += " mode=" + String(diagnostics.spi_mode);
  out += " hz=" + String(transferFrequency(diagnostics.is_dac));
  out += " start=" + String(diagnostics.start_result);
  out += " event=" + String(diagnostics.callback_event);
  out += " cb=" + String(diagnostics.callback_seen ? 1 : 0);
  out += " timeout=" + String(diagnostics.timeout ? 1 : 0);
  out += " timeout_us=" + String(diagnostics.timeout_us);
  out += " defer=" + String(diagnostics.deferred_errors ? 1 : 0);
  out += " sticky=" + String(diagnostics.sticky_error ? 1 : 0);
  out += " done=" + String(diagnostics.completed_transfers);
  out += " failed=" + String(diagnostics.failed_transfers);
  return out;
}
}

PeripheralCommsController::PeripheralCommsController(int cs_pin)
    : cs_pin(cs_pin) {}

bool PeripheralCommsController::performMbedTransfer(bool is_dac, uint8_t* tx,
                                                    uint8_t* rx,
                                                    size_t count) {
  lastTransferOk = false;

  if (count == 0) {
    lastTransferOk = true;
    lastDiagnostics =
        makeDiagnostics(spiInitialized, is_dac, cs_pin, count, kFailureNone,
                        true, false, false, 0, 0, deferSpiErrors,
                        stickySpiError);
    return true;
  }

  if (!spiInitialized) {
    failedTransfers++;
    clearCallerBuffer(tx, rx, count);
    lastDiagnostics = makeDiagnostics(false, is_dac, cs_pin, count,
                                      kFailureNotInitialized, false, false,
                                      false, -1, 0, deferSpiErrors,
                                      stickySpiError);
    if (deferSpiErrors) {
      if (!stickySpiError) {
        firstStickyDiagnostics = lastDiagnostics;
        firstStickyDiagnostics.sticky_error = true;
      }
      stickySpiError = true;
      lastDiagnostics.sticky_error = true;
      return true;
    }
    return false;
  }

  if (count > kSpiBufferSize) {
    failedTransfers++;
    clearCallerBuffer(tx, rx, count);
    lastDiagnostics = makeDiagnostics(true, is_dac, cs_pin, count,
                                      kFailureCountTooLarge, false, false,
                                      false, -1, 0, deferSpiErrors,
                                      stickySpiError);
    if (deferSpiErrors) {
      if (!stickySpiError) {
        firstStickyDiagnostics = lastDiagnostics;
        firstStickyDiagnostics.sticky_error = true;
      }
      stickySpiError = true;
      lastDiagnostics.sticky_error = true;
      return true;
    }
    return false;
  }

  if (!configureBusForTransfer(is_dac)) {
    failedTransfers++;
    clearCallerBuffer(tx, rx, count);
    lastDiagnostics = makeDiagnostics(true, is_dac, cs_pin, count,
                                      kFailureDmaUsage, false, false, false,
                                      -1, 0, deferSpiErrors, stickySpiError);
    if (deferSpiErrors) {
      if (!stickySpiError) {
        firstStickyDiagnostics = lastDiagnostics;
        firstStickyDiagnostics.sticky_error = true;
      }
      stickySpiError = true;
      lastDiagnostics.sticky_error = true;
      return true;
    }
    return false;
  }

  if (tx != nullptr) {
    memcpy(tx_buffer, tx, count);
  } else {
    memset(tx_buffer, 0, count);
  }
  memset(rx_buffer, 0, count);

  asyncTransferDone = false;
  asyncTransferEvent = 0;

  SpiBusState& bus = busForTransfer(is_dac);
  FastGpio::digitalWrite(cs_pin, false);
  const int startResult = bus.spi->transfer<uint8_t>(
      tx_buffer, static_cast<int>(count), rx_buffer, static_cast<int>(count),
      mbed::event_callback_t(asyncTransferCallback), SPI_EVENT_ALL);

  if (startResult != 0) {
    FastGpio::digitalWrite(cs_pin, true);
    failedTransfers++;
    clearCallerBuffer(tx, rx, count);
    lastDiagnostics = makeDiagnostics(true, is_dac, cs_pin, count,
                                      kFailureStart, false, false, false,
                                      startResult, 0, deferSpiErrors,
                                      stickySpiError);
    if (deferSpiErrors) {
      if (!stickySpiError) {
        firstStickyDiagnostics = lastDiagnostics;
        firstStickyDiagnostics.sticky_error = true;
      }
      stickySpiError = true;
      lastDiagnostics.sticky_error = true;
      return true;
    }
    return false;
  }

  const uint32_t startUs = micros();
  while (!asyncTransferDone) {
    if (static_cast<uint32_t>(micros() - startUs) >=
        kSpiTransferTimeoutUs) {
      break;
    }
  }

  const bool timeout = !asyncTransferDone;
  const bool callbackSeen = asyncTransferDone;
  const int callbackEvent = asyncTransferEvent;
  if (timeout) {
    bus.spi->abort_all_transfers();
  }
  FastGpio::digitalWrite(cs_pin, true);

  const bool eventOk = callbackSeen &&
                       ((callbackEvent & SPI_EVENT_COMPLETE) != 0) &&
                       ((callbackEvent &
                         (SPI_EVENT_ERROR | SPI_EVENT_RX_OVERFLOW)) == 0);
  const bool transferOk = !timeout && eventOk;

  if (!transferOk) {
    failedTransfers++;
    clearCallerBuffer(tx, rx, count);
    lastDiagnostics = makeDiagnostics(
        true, is_dac, cs_pin, count,
        timeout ? kFailureTimeout : kFailureEvent, false, timeout,
        callbackSeen, startResult, callbackEvent, deferSpiErrors,
        stickySpiError);
    if (deferSpiErrors) {
      if (!stickySpiError) {
        firstStickyDiagnostics = lastDiagnostics;
        firstStickyDiagnostics.sticky_error = true;
      }
      stickySpiError = true;
      lastDiagnostics.sticky_error = true;
      return true;
    }
    return false;
  }

  if (rx != nullptr) {
    memcpy(rx, rx_buffer, count);
  } else if (tx != nullptr) {
    memcpy(tx, rx_buffer, count);
  }

  completedTransfers++;
  lastTransferOk = true;
  lastDiagnostics =
      makeDiagnostics(true, is_dac, cs_pin, count, kFailureNone, true, false,
                      callbackSeen, startResult, callbackEvent, deferSpiErrors,
                      stickySpiError);
  return true;
}

void PeripheralCommsController::setup() {
  if (spiInitialized) {
    return;
  }

  constructSpiBuses();
  bool initialized = configureBusForTransfer(true);
#ifdef __NEW_SHIELD__
  initialized = configureBusForTransfer(false) && initialized;
#endif

  spiInitialized = initialized;
  if (!initialized) {
    failedTransfers++;
    lastDiagnostics = makeDiagnostics(false, true, -1, 0, kFailureDmaUsage,
                                      false, false, false, -1, 0, false,
                                      false);
  } else {
    lastDiagnostics = makeDiagnostics(true, true, -1, 0, kFailureNone, true,
                                      false, false, 0, 0, false, false);
  }

  registerMemberFunction(getDiagnostics, "SPI_DIAG");
}

bool PeripheralCommsController::transferDAC(void* buf, size_t count) {
  return performMbedTransfer(true, static_cast<uint8_t*>(buf),
                             static_cast<uint8_t*>(buf), count);
}

bool PeripheralCommsController::transferADC(void* buf, size_t count) {
  return performMbedTransfer(false, static_cast<uint8_t*>(buf),
                             static_cast<uint8_t*>(buf), count);
}

uint8_t PeripheralCommsController::transferDAC(uint8_t data) {
  uint8_t tx_byte = data;
  bool ok = performMbedTransfer(true, &tx_byte, &tx_byte, 1);
  return ok ? tx_byte : 0;
}

uint8_t PeripheralCommsController::transferADC(uint8_t data) {
  uint8_t tx_byte = data;
  bool ok = performMbedTransfer(false, &tx_byte, &tx_byte, 1);
  return ok ? tx_byte : 0;
}

bool PeripheralCommsController::transferDACNoTransaction(void* buf,
                                                        size_t count) {
  return performMbedTransfer(true, static_cast<uint8_t*>(buf),
                             static_cast<uint8_t*>(buf), count);
}

bool PeripheralCommsController::transferADCNoTransaction(void* buf,
                                                        size_t count) {
  return performMbedTransfer(false, static_cast<uint8_t*>(buf),
                             static_cast<uint8_t*>(buf), count);
}

uint8_t PeripheralCommsController::transferDACNoTransaction(uint8_t data) {
  uint8_t tx_byte = data;
  return performMbedTransfer(true, &tx_byte, &tx_byte, 1) ? tx_byte : 0;
}

uint8_t PeripheralCommsController::transferADCNoTransaction(uint8_t data) {
  uint8_t tx_byte = data;
  return performMbedTransfer(false, &tx_byte, &tx_byte, 1) ? tx_byte : 0;
}

bool PeripheralCommsController::lastTransferSucceeded() {
  return lastTransferOk;
}

OperationResult PeripheralCommsController::getDiagnostics() {
  return OperationResult::Success(diagnosticsToString(lastDiagnostics));
}

void PeripheralCommsController::beginDeferredSpiErrors() {
  if (deferSpiErrorDepth == 0) {
    stickySpiError = false;
    firstStickyDiagnostics = {};
    lastDiagnostics.sticky_error = false;
  }
  deferSpiErrorDepth++;
  deferSpiErrors = true;
  lastDiagnostics.deferred_errors = true;
}

OperationResult PeripheralCommsController::endDeferredSpiErrors() {
  if (deferSpiErrorDepth > 1) {
    deferSpiErrorDepth--;
    return OperationResult::Success();
  }
  if (deferSpiErrorDepth == 0) {
    return OperationResult::Success();
  }

  const bool hadStickyError = stickySpiError;
  const SpiDiagnostics firstFailure = firstStickyDiagnostics;
  deferSpiErrorDepth = 0;
  deferSpiErrors = false;
  stickySpiError = false;
  firstStickyDiagnostics = {};
  lastDiagnostics.deferred_errors = false;
  lastDiagnostics.sticky_error = hadStickyError;

  if (!hadStickyError) {
    return OperationResult::Success();
  }

  return OperationResult::Failure("SPI transfer failed during ramp " +
                                  diagnosticsToString(firstFailure));
}

void PeripheralCommsController::cancelDeferredSpiErrors() {
  if (deferSpiErrorDepth > 1) {
    deferSpiErrorDepth--;
    return;
  }
  deferSpiErrorDepth = 0;
  deferSpiErrors = false;
  stickySpiError = false;
  firstStickyDiagnostics = {};
  lastDiagnostics.deferred_errors = false;
  lastDiagnostics.sticky_error = false;
}

bool PeripheralCommsController::hasDeferredSpiError() {
  return stickySpiError;
}

void PeripheralCommsController::dataLedOn() {}

void PeripheralCommsController::dataLedOff() {}
