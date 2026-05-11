#include "Peripherals/PeripheralCommsController.h"

#include <cstring>

#include "Config.h"
#include "SPI.h"
#include "Utils/shared_memory.h"

#if defined(ARDUINO_GIGA) || defined(CORE_STM32H7)
#include "stm32h7xx.h"
#else
#error "This code is intended for STM32H7 based boards like the Arduino Giga."
#endif

#ifndef SCB_CleanInvalidateDCache_by_Addr
static inline void fallback_cache_clean_invalidate(void* addr, size_t size) {
  (void)addr;
  (void)size;
  __asm volatile("dsb sy" : : : "memory");
  __asm volatile("isb sy" : : : "memory");
}
#define SCB_CleanInvalidateDCache_by_Addr(addr, size) \
  fallback_cache_clean_invalidate(addr, size)
#endif

#ifndef SCB_InvalidateDCache_by_Addr
static inline void fallback_cache_invalidate(void* addr, size_t size) {
  (void)addr;
  (void)size;
  __asm volatile("dsb sy" : : : "memory");
  __asm volatile("isb sy" : : : "memory");
}
#define SCB_InvalidateDCache_by_Addr(addr, size) \
  fallback_cache_invalidate(addr, size)
#endif

#ifndef SPI1_BASE
#define SPI1_BASE 0x40013000UL
#endif
#ifndef SPI5_BASE
#define SPI5_BASE 0x40015000UL
#endif

bool PeripheralCommsController::spiInitialized = false;
bool PeripheralCommsController::dmaReady = false;
bool PeripheralCommsController::useDma = true;
uint8_t PeripheralCommsController::dma_tx_buffer[64] __attribute__((aligned(32)));
uint8_t PeripheralCommsController::dma_rx_buffer[64] __attribute__((aligned(32)));

PeripheralCommsController::PeripheralCommsController(int cs_pin)
    : cs_pin(cs_pin) {
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
}

void PeripheralCommsController::waitForDmaInit() {
  if (dmaReady) {
    return;
  }

  uint32_t start_time = millis();
  while (!isBootComplete()) {
    delay(1);
    if (millis() - start_time > 3000) {
      return;
    }
  }

  dmaReady = true;
}

uint8_t PeripheralCommsController::performDmaTransfer(bool is_dac,
                                                      uint8_t* tx_buffer,
                                                      uint8_t* rx_buffer,
                                                      size_t count) {
  if (!useDma || count == 0) {
    return 0;
  }

  if (tx_buffer) {
    memcpy(dma_tx_buffer, tx_buffer, count);
  } else {
    memset(dma_tx_buffer, 0, count);
  }

  SCB_CleanInvalidateDCache_by_Addr(dma_tx_buffer, ((count + 31) / 32) * 32);
  SCB_InvalidateDCache_by_Addr(dma_rx_buffer, ((count + 31) / 32) * 32);

  DMA_Stream_TypeDef* tx_stream;
  DMA_Stream_TypeDef* rx_stream;
  SPI_TypeDef* spi_regs;

  if (is_dac) {
    tx_stream = DMA1_Stream0;
    rx_stream = DMA1_Stream1;
    spi_regs = reinterpret_cast<SPI_TypeDef*>(SPI1_BASE);
  } else {
#ifdef __NEW_SHIELD__
    tx_stream = DMA1_Stream2;
    rx_stream = DMA1_Stream3;
    spi_regs = reinterpret_cast<SPI_TypeDef*>(SPI5_BASE);
#else
    tx_stream = DMA1_Stream0;
    rx_stream = DMA1_Stream1;
    spi_regs = reinterpret_cast<SPI_TypeDef*>(SPI1_BASE);
#endif
  }

  digitalWrite(cs_pin, LOW);

  spi_regs->CR1 &= ~SPI_CR1_SPE;
  tx_stream->CR &= ~DMA_SxCR_EN;
  rx_stream->CR &= ~DMA_SxCR_EN;

  while ((tx_stream->CR & DMA_SxCR_EN) || (rx_stream->CR & DMA_SxCR_EN)) {
  }

  if (tx_stream == DMA1_Stream0) {
    DMA1->LIFCR = 0x3F;
  } else if (tx_stream == DMA1_Stream2) {
    DMA1->LIFCR = (0x3F << 16);
  }

  if (rx_stream == DMA1_Stream1) {
    DMA1->LIFCR = (0x3F << 6);
  } else if (rx_stream == DMA1_Stream3) {
    DMA1->LIFCR = (0x3F << 22);
  }

  rx_stream->PAR = reinterpret_cast<uint32_t>(&spi_regs->RXDR);
  rx_stream->M0AR = reinterpret_cast<uint32_t>(dma_rx_buffer);
  rx_stream->NDTR = count;
  rx_stream->CR = DMA_SxCR_PL_1 | DMA_SxCR_MINC;
  rx_stream->FCR = 0;

  tx_stream->PAR = reinterpret_cast<uint32_t>(&spi_regs->TXDR);
  tx_stream->M0AR = reinterpret_cast<uint32_t>(dma_tx_buffer);
  tx_stream->NDTR = count;
  tx_stream->CR = DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
  tx_stream->FCR = 0;

  spi_regs->CFG1 |= SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN;

  rx_stream->CR |= DMA_SxCR_EN;
  tx_stream->CR |= DMA_SxCR_EN;

  spi_regs->CR1 |= SPI_CR1_SPE;
  spi_regs->CR1 |= SPI_CR1_CSTART;

  uint32_t timeout = 100000;
  while ((tx_stream->NDTR > 0 || rx_stream->NDTR > 0) && timeout > 0) {
    __DMB();
    timeout--;
  }

  spi_regs->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_CSTART);
  spi_regs->CFG1 &= ~(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
  tx_stream->CR &= ~DMA_SxCR_EN;
  rx_stream->CR &= ~DMA_SxCR_EN;

  digitalWrite(cs_pin, HIGH);

  SCB_InvalidateDCache_by_Addr(dma_rx_buffer, ((count + 31) / 32) * 32);

  if (rx_buffer) {
    memcpy(rx_buffer, dma_rx_buffer, count);
  } else if (tx_buffer) {
    memcpy(tx_buffer, dma_rx_buffer, count);
  }

  return (count == 1) ? dma_rx_buffer[0] : 0;
}

void PeripheralCommsController::setup() {
  if (spiInitialized) {
    return;
  }

  SPI.begin();
#ifdef __NEW_SHIELD__
  SPI.beginTransaction(DAC_SPI_SETTINGS);
  SPI1.begin();
  SPI1.beginTransaction(ADC_SPI_SETTINGS);
#endif
  spiInitialized = true;
  waitForDmaInit();
}

void PeripheralCommsController::transferDAC(void* buf, size_t count) {
#ifndef __NEW_SHIELD__
  SPI.beginTransaction(DAC_SPI_SETTINGS);
#endif
  performDmaTransfer(true, static_cast<uint8_t*>(buf), static_cast<uint8_t*>(buf),
                     count);
#ifndef __NEW_SHIELD__
  SPI.endTransaction();
#endif
}

void PeripheralCommsController::transferADC(void* buf, size_t count) {
#ifndef __NEW_SHIELD__
  SPI.beginTransaction(ADC_SPI_SETTINGS);
#endif
  performDmaTransfer(false, static_cast<uint8_t*>(buf),
                     static_cast<uint8_t*>(buf), count);
#ifndef __NEW_SHIELD__
  SPI.endTransaction();
#endif
}

uint8_t PeripheralCommsController::transferDAC(uint8_t data) {
#ifndef __NEW_SHIELD__
  SPI.beginTransaction(DAC_SPI_SETTINGS);
#endif
  uint8_t tx_byte = data;
  uint8_t result = performDmaTransfer(true, &tx_byte, &tx_byte, 1);
#ifndef __NEW_SHIELD__
  SPI.endTransaction();
#endif
  return result;
}

uint8_t PeripheralCommsController::transferADC(uint8_t data) {
#ifndef __NEW_SHIELD__
  SPI.beginTransaction(ADC_SPI_SETTINGS);
#endif
  uint8_t tx_byte = data;
  uint8_t result = performDmaTransfer(false, &tx_byte, &tx_byte, 1);
#ifndef __NEW_SHIELD__
  SPI.endTransaction();
#endif
  return result;
}

void PeripheralCommsController::transferDACNoTransaction(void* buf,
                                                        size_t count) {
  performDmaTransfer(true, static_cast<uint8_t*>(buf), static_cast<uint8_t*>(buf),
                     count);
}

void PeripheralCommsController::transferADCNoTransaction(void* buf,
                                                        size_t count) {
  performDmaTransfer(false, static_cast<uint8_t*>(buf),
                     static_cast<uint8_t*>(buf), count);
}

uint8_t PeripheralCommsController::transferDACNoTransaction(uint8_t data) {
  uint8_t tx_byte = data;
  return performDmaTransfer(true, &tx_byte, &tx_byte, 1);
}

uint8_t PeripheralCommsController::transferADCNoTransaction(uint8_t data) {
  uint8_t tx_byte = data;
  return performDmaTransfer(false, &tx_byte, &tx_byte, 1);
}

void PeripheralCommsController::beginDacTransaction() {
#ifndef __NEW_SHIELD__
  SPI.beginTransaction(DAC_SPI_SETTINGS);
#endif
}

void PeripheralCommsController::beginAdcTransaction() {
#ifndef __NEW_SHIELD__
  SPI.beginTransaction(ADC_SPI_SETTINGS);
#endif
}

void PeripheralCommsController::dataLedOn() {
  /* digitalWrite(led, HIGH); */
}

void PeripheralCommsController::dataLedOff() {
  /* digitalWrite(led, LOW); */
}

void PeripheralCommsController::endTransaction() {
  SPI.endTransaction();
}
