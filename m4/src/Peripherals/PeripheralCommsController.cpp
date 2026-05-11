#include "Peripherals/PeripheralCommsController.h"

#include <cstring>

#include "Config.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
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
bool PeripheralCommsController::lastTransferOk = true;
PeripheralCommsController::DmaDiagnostics PeripheralCommsController::lastDiagnostics = {};
uint8_t PeripheralCommsController::dma_tx_buffer[PeripheralCommsController::kDmaBufferSize] __attribute__((aligned(32)));
uint8_t PeripheralCommsController::dma_rx_buffer[PeripheralCommsController::kDmaBufferSize] __attribute__((aligned(32)));

namespace {
constexpr uint32_t kDmaDisableTimeout = 100000;
constexpr uint32_t kSpiEotTimeout = 100000;
constexpr uint32_t kSpiClearFlags =
    SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC | SPI_IFCR_OVRC |
    SPI_IFCR_CRCEC | SPI_IFCR_TIFREC | SPI_IFCR_MODFC | SPI_IFCR_TSERFC |
    SPI_IFCR_SUSPC;
constexpr uint32_t kSpiErrorFlags =
    SPI_SR_UDR | SPI_SR_OVR | SPI_SR_CRCE | SPI_SR_TIFRE | SPI_SR_MODF;
constexpr uint32_t kSpi1Base = SPI1_BASE;
constexpr uint32_t kSpi5Base = SPI5_BASE;
constexpr uintptr_t kM4D2SramAliasBase = 0x10000000UL;
constexpr uintptr_t kDmaD2SramAliasBase = 0x30000000UL;
constexpr uintptr_t kD2SramAliasSize = 0x48000UL;

struct TransferConfig {
  DMA_Stream_TypeDef* tx_stream;
  DMA_Stream_TypeDef* rx_stream;
  SPI_TypeDef* spi_regs;
  uint32_t frequency_hz;
  uint8_t spi_mode;
};

struct BaudCacheEntry {
  uint32_t spi_base;
  uint32_t target_hz;
  uint32_t baud_bits;
  bool valid;
};

void enableSpiGpioClocks() {
  RCC->AHB4ENR |=
      RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOGEN;
#ifdef __NEW_SHIELD__
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOHEN | RCC_AHB4ENR_GPIOJEN;
#endif
  const uint32_t ahb4enr = RCC->AHB4ENR;
  (void)ahb4enr;
  __DMB();
}

void configureAlternateFunction(GPIO_TypeDef* port, uint32_t pin,
                                uint32_t alternate_function) {
  const uint32_t mode_shift = pin * 2U;
  const uint32_t afr_index = pin / 8U;
  const uint32_t afr_shift = (pin % 8U) * 4U;

  port->MODER = (port->MODER & ~(0x3UL << mode_shift)) |
                (0x2UL << mode_shift);
  port->OTYPER &= ~(1UL << pin);
  port->OSPEEDR |= 0x3UL << mode_shift;
  port->PUPDR &= ~(0x3UL << mode_shift);
  port->AFR[afr_index] =
      (port->AFR[afr_index] & ~(0xFUL << afr_shift)) |
      (alternate_function << afr_shift);
}

void configureSpiPins() {
  constexpr uint32_t kSpiAlternateFunction = 5;

  enableSpiGpioClocks();

  configureAlternateFunction(GPIOB, 3, kSpiAlternateFunction);  // SPI1 SCK
  configureAlternateFunction(GPIOD, 7, kSpiAlternateFunction);  // SPI1 MOSI
  configureAlternateFunction(GPIOG, 9, kSpiAlternateFunction);  // SPI1 MISO

#ifdef __NEW_SHIELD__
  configureAlternateFunction(GPIOH, 6, kSpiAlternateFunction);  // SPI5 SCK
  configureAlternateFunction(GPIOJ, 10, kSpiAlternateFunction); // SPI5 MOSI
  configureAlternateFunction(GPIOJ, 11, kSpiAlternateFunction); // SPI5 MISO
#endif
}

void resetSpiPeripheral(SPI_TypeDef* spi_regs) {
  spi_regs->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_CSTART);
  if (reinterpret_cast<uint32_t>(spi_regs) == kSpi1Base) {
    SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
    CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
  } else if (reinterpret_cast<uint32_t>(spi_regs) == kSpi5Base) {
    SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI5RST);
    CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI5RST);
  }
}

void configureSpiHardware() {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
#ifdef __NEW_SHIELD__
  RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
#endif
  const uint32_t apb2enr = RCC->APB2ENR;
  (void)apb2enr;
  __DMB();
  configureSpiPins();
  resetSpiPeripheral(reinterpret_cast<SPI_TypeDef*>(SPI1_BASE));
#ifdef __NEW_SHIELD__
  resetSpiPeripheral(reinterpret_cast<SPI_TypeDef*>(SPI5_BASE));
#endif
}

uint32_t cacheLineSize(size_t count) {
  return static_cast<uint32_t>(((count + 31) / 32) * 32);
}

uint32_t dmaAddress(const void* ptr) {
  uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
  if (addr >= kM4D2SramAliasBase &&
      addr < (kM4D2SramAliasBase + kD2SramAliasSize)) {
    addr = kDmaD2SramAliasBase + (addr - kM4D2SramAliasBase);
  }
  return static_cast<uint32_t>(addr);
}

void clearCallerBuffer(uint8_t* tx_buffer, uint8_t* rx_buffer, size_t count) {
  if (rx_buffer) {
    memset(rx_buffer, 0, count);
  } else if (tx_buffer) {
    memset(tx_buffer, 0, count);
  }
}

bool waitForStreamsDisabled(DMA_Stream_TypeDef* tx_stream,
                            DMA_Stream_TypeDef* rx_stream) {
  uint32_t timeout = kDmaDisableTimeout;
  while ((tx_stream->CR & DMA_SxCR_EN) || (rx_stream->CR & DMA_SxCR_EN)) {
    if (timeout-- == 0) {
      return false;
    }
    __DMB();
  }
  return true;
}

uint32_t dmaClearMask(DMA_Stream_TypeDef* stream) {
  if (stream == DMA1_Stream0) {
    return DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 |
           DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
  } else if (stream == DMA1_Stream1) {
    return DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 |
           DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;
  } else if (stream == DMA1_Stream2) {
    return DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2 |
           DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;
  } else if (stream == DMA1_Stream3) {
    return DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3 |
           DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3;
  }

  return 0;
}

uint32_t dmaErrorFlags(DMA_Stream_TypeDef* stream) {
  // Only abort on fatal DMA errors. FEIF can assert in this direct-mode byte
  // DMA path while the TX stream is otherwise progressing normally.
  if (stream == DMA1_Stream0) {
    return DMA1->LISR & (DMA_LISR_DMEIF0 | DMA_LISR_TEIF0);
  } else if (stream == DMA1_Stream1) {
    return DMA1->LISR & (DMA_LISR_DMEIF1 | DMA_LISR_TEIF1);
  } else if (stream == DMA1_Stream2) {
    return DMA1->LISR & (DMA_LISR_DMEIF2 | DMA_LISR_TEIF2);
  } else if (stream == DMA1_Stream3) {
    return DMA1->LISR & (DMA_LISR_DMEIF3 | DMA_LISR_TEIF3);
  }

  return 0;
}

void clearDmaFlags(DMA_Stream_TypeDef* stream) {
  const uint32_t mask = dmaClearMask(stream);
  if (mask != 0) {
    DMA1->LIFCR = mask;
  }
}

uint32_t spiKernelClockHz(SPI_TypeDef* spi_regs) {
#if defined(RCC_PERIPHCLK_SPI123) && defined(RCC_PERIPHCLK_SPI45)
  const uint32_t periph_clk =
      (reinterpret_cast<uint32_t>(spi_regs) == kSpi1Base)
          ? HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123)
          : HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
  if (periph_clk != 0) {
    return periph_clk;
  }
#endif

  if (reinterpret_cast<uint32_t>(spi_regs) == kSpi5Base) {
    return HAL_RCC_GetPCLK1Freq();
  }
  return HAL_RCC_GetPCLK2Freq();
}

uint32_t spiBaudRateBits(SPI_TypeDef* spi_regs, uint32_t target_hz) {
  static BaudCacheEntry cache[3] = {};
  const uint32_t spi_base = reinterpret_cast<uint32_t>(spi_regs);
  for (const BaudCacheEntry& entry : cache) {
    if (entry.valid && entry.spi_base == spi_base &&
        entry.target_hz == target_hz) {
      return entry.baud_bits;
    }
  }

  const uint32_t kernel_hz = spiKernelClockHz(spi_regs);
  uint32_t baud_bits = 7U << SPI_CFG1_MBR_Pos;

  if (kernel_hz != 0 && target_hz != 0) {
    uint32_t divider = 2;
    for (uint32_t mbr = 0; mbr <= 7; mbr++) {
      if ((kernel_hz / divider) <= target_hz) {
        baud_bits = mbr << SPI_CFG1_MBR_Pos;
        break;
      }
      divider <<= 1;
    }
  }

  for (BaudCacheEntry& entry : cache) {
    if (!entry.valid) {
      entry.spi_base = spi_base;
      entry.target_hz = target_hz;
      entry.baud_bits = baud_bits;
      entry.valid = true;
      break;
    }
  }

  return baud_bits;
}

uint32_t spiModeBits(uint8_t spi_mode) {
  uint32_t cfg2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR;
  if ((spi_mode & 0x01U) != 0) {
    cfg2 |= SPI_CFG2_CPHA;
  }
  if ((spi_mode & 0x02U) != 0) {
    cfg2 |= SPI_CFG2_CPOL;
  }
  return cfg2;
}

void drainSpiRxFifo(SPI_TypeDef* spi_regs) {
  uint32_t guard = kDmaDisableTimeout;
  while ((spi_regs->SR & (SPI_SR_RXP | SPI_SR_RXWNE)) != 0 && guard-- > 0) {
    volatile uint8_t unused =
        *reinterpret_cast<volatile uint8_t*>(&spi_regs->RXDR);
    (void)unused;
  }
}

void configureSpiForDma(SPI_TypeDef* spi_regs, uint32_t frequency_hz,
                        uint8_t spi_mode, size_t count) {
  spi_regs->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_CSTART);
  drainSpiRxFifo(spi_regs);
  spi_regs->IFCR = kSpiClearFlags;
  spi_regs->IER = 0;
  spi_regs->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

  const uint32_t cfg1 =
      (7U << SPI_CFG1_DSIZE_Pos) | spiBaudRateBits(spi_regs, frequency_hz);
  const uint32_t cfg2 = spiModeBits(spi_mode);
  if ((spi_regs->CFG1 & ~(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN)) != cfg1) {
    spi_regs->CFG1 = cfg1;
  }
  if (spi_regs->CFG2 != cfg2) {
    spi_regs->CFG2 = cfg2;
  }
  spi_regs->CR1 = SPI_CR1_SSI;
  spi_regs->CR2 =
      (static_cast<uint32_t>(count) << SPI_CR2_TSIZE_Pos) & SPI_CR2_TSIZE;
}

bool waitForSpiEndOfTransfer(SPI_TypeDef* spi_regs) {
  uint32_t timeout = kSpiEotTimeout;
  while ((spi_regs->SR & SPI_SR_EOT) == 0) {
    if ((spi_regs->SR & kSpiErrorFlags) != 0) {
      return false;
    }
    if (timeout-- == 0) {
      return false;
    }
    __DMB();
  }
  return true;
}

void disableSpiDma(SPI_TypeDef* spi_regs, DMA_Stream_TypeDef* tx_stream,
                   DMA_Stream_TypeDef* rx_stream) {
  spi_regs->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_CSTART);
  spi_regs->CFG1 &= ~(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
  tx_stream->CR &= ~DMA_SxCR_EN;
  rx_stream->CR &= ~DMA_SxCR_EN;
  waitForStreamsDisabled(tx_stream, rx_stream);
  clearDmaFlags(tx_stream);
  clearDmaFlags(rx_stream);
  drainSpiRxFifo(spi_regs);
  spi_regs->IFCR = kSpiClearFlags;
}

TransferConfig transferConfig(bool is_dac) {
  if (is_dac) {
    return {DMA1_Stream0, DMA1_Stream1, reinterpret_cast<SPI_TypeDef*>(SPI1_BASE),
            DAC_SPI_FREQUENCY_HZ, DAC_SPI_MODE};
  }

#ifdef __NEW_SHIELD__
  return {DMA1_Stream2, DMA1_Stream3, reinterpret_cast<SPI_TypeDef*>(SPI5_BASE),
          ADC_SPI_FREQUENCY_HZ, ADC_SPI_MODE};
#else
  return {DMA1_Stream0, DMA1_Stream1, reinterpret_cast<SPI_TypeDef*>(SPI1_BASE),
          ADC_SPI_FREQUENCY_HZ, ADC_SPI_MODE};
#endif
}
}

void recordDiagnostics(PeripheralCommsController::DmaDiagnostics& diagnostics,
                       const TransferConfig& config, bool is_dac,
                       size_t count, uint32_t failure_stage,
                       bool transfer_ok) {
  diagnostics.spi_base = reinterpret_cast<uint32_t>(config.spi_regs);
  diagnostics.spi_sr = config.spi_regs->SR;
  diagnostics.spi_cfg1 = config.spi_regs->CFG1;
  diagnostics.spi_cfg2 = config.spi_regs->CFG2;
  diagnostics.spi_cr1 = config.spi_regs->CR1;
  diagnostics.spi_cr2 = config.spi_regs->CR2;
  diagnostics.spi_ier = config.spi_regs->IER;
  diagnostics.spi_i2scfgr = config.spi_regs->I2SCFGR;
  diagnostics.dma_lisr = DMA1->LISR;
  diagnostics.tx_cr = config.tx_stream->CR;
  diagnostics.rx_cr = config.rx_stream->CR;
  diagnostics.tx_m0ar = config.tx_stream->M0AR;
  diagnostics.rx_m0ar = config.rx_stream->M0AR;
  diagnostics.tx_ndtr = config.tx_stream->NDTR;
  diagnostics.rx_ndtr = config.rx_stream->NDTR;
  diagnostics.failure_stage = failure_stage;
  diagnostics.count = static_cast<uint32_t>(count);
  diagnostics.is_dac = is_dac;
  diagnostics.transfer_ok = transfer_ok;
}

PeripheralCommsController::PeripheralCommsController(int cs_pin)
    : cs_pin(cs_pin) {
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
}

bool PeripheralCommsController::waitForDmaInit() {
  if (dmaReady) {
    return true;
  }

  uint32_t start_time = millis();
  while (!isBootComplete()) {
    delay(1);
    if (millis() - start_time > 3000) {
      dmaReady = false;
      return false;
    }
  }

  dmaReady = true;
  return true;
}

bool PeripheralCommsController::performDmaTransfer(bool is_dac,
                                                   uint8_t* tx_buffer,
                                                   uint8_t* rx_buffer,
                                                   size_t count) {
  lastTransferOk = false;
  const TransferConfig config = transferConfig(is_dac);
  if (count == 0) {
    lastTransferOk = true;
    recordDiagnostics(lastDiagnostics, config, is_dac, count, 0, true);
    return true;
  }
  if (!dmaReady) {
    dmaReady = waitForDmaInit();
  }
  if (!dmaReady || count > kDmaBufferSize) {
    clearCallerBuffer(tx_buffer, rx_buffer, count);
    recordDiagnostics(lastDiagnostics, config, is_dac, count, 1, false);
    return false;
  }

  if (tx_buffer) {
    memcpy(dma_tx_buffer, tx_buffer, count);
  } else {
    memset(dma_tx_buffer, 0, count);
  }

  const uint32_t aligned_count = cacheLineSize(count);
  SCB_CleanInvalidateDCache_by_Addr(dma_tx_buffer, aligned_count);
  SCB_InvalidateDCache_by_Addr(dma_rx_buffer, aligned_count);

  config.spi_regs->CR1 &= ~SPI_CR1_SPE;
  config.tx_stream->CR &= ~DMA_SxCR_EN;
  config.rx_stream->CR &= ~DMA_SxCR_EN;

  if (!waitForStreamsDisabled(config.tx_stream, config.rx_stream)) {
    dmaReady = false;
    clearCallerBuffer(tx_buffer, rx_buffer, count);
    recordDiagnostics(lastDiagnostics, config, is_dac, count, 2, false);
    return false;
  }

  clearDmaFlags(config.tx_stream);
  clearDmaFlags(config.rx_stream);
  configureSpiForDma(config.spi_regs, config.frequency_hz, config.spi_mode,
                     count);

  config.rx_stream->PAR = reinterpret_cast<uint32_t>(&config.spi_regs->RXDR);
  config.rx_stream->M0AR = dmaAddress(dma_rx_buffer);
  config.rx_stream->NDTR = static_cast<uint32_t>(count);
  config.rx_stream->CR = DMA_SxCR_PL | DMA_SxCR_MINC;
  config.rx_stream->FCR = 0;

  config.tx_stream->PAR = reinterpret_cast<uint32_t>(&config.spi_regs->TXDR);
  config.tx_stream->M0AR = dmaAddress(dma_tx_buffer);
  config.tx_stream->NDTR = static_cast<uint32_t>(count);
  config.tx_stream->CR = DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
  config.tx_stream->FCR = 0;

  config.spi_regs->CFG1 |= SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN;

  digitalWrite(cs_pin, LOW);

  config.rx_stream->CR |= DMA_SxCR_EN;
  config.tx_stream->CR |= DMA_SxCR_EN;
  __DMB();

  config.spi_regs->CR1 |= SPI_CR1_SPE;
  config.spi_regs->CR1 |= SPI_CR1_CSTART;

  uint32_t timeout = kDmaTransferTimeout;
  while ((config.tx_stream->NDTR > 0 || config.rx_stream->NDTR > 0) &&
         timeout > 0) {
    if (dmaErrorFlags(config.tx_stream) != 0 ||
        dmaErrorFlags(config.rx_stream) != 0 ||
        (config.spi_regs->SR & kSpiErrorFlags) != 0) {
      break;
    }
    __DMB();
    timeout--;
  }

  const bool dmaComplete =
      (config.tx_stream->NDTR == 0 && config.rx_stream->NDTR == 0);
  const bool spiComplete =
      dmaComplete ? waitForSpiEndOfTransfer(config.spi_regs) : false;
  const bool transferComplete =
      dmaComplete && spiComplete &&
      dmaErrorFlags(config.tx_stream) == 0 &&
      dmaErrorFlags(config.rx_stream) == 0 &&
      (config.spi_regs->SR & kSpiErrorFlags) == 0;

  if (!transferComplete) {
    recordDiagnostics(lastDiagnostics, config, is_dac, count, 3, false);
  }

  disableSpiDma(config.spi_regs, config.tx_stream, config.rx_stream);

  digitalWrite(cs_pin, HIGH);

  if (!transferComplete) {
    dmaReady = false;
    clearCallerBuffer(tx_buffer, rx_buffer, count);
    return false;
  }

  SCB_InvalidateDCache_by_Addr(dma_rx_buffer, aligned_count);

  if (rx_buffer) {
    memcpy(rx_buffer, dma_rx_buffer, count);
  } else if (tx_buffer) {
    memcpy(tx_buffer, dma_rx_buffer, count);
  }

  lastTransferOk = true;
  recordDiagnostics(lastDiagnostics, config, is_dac, count, 0, true);
  return true;
}

void PeripheralCommsController::setup() {
  if (spiInitialized) {
    return;
  }

  configureSpiHardware();
  spiInitialized = true;
  dmaReady = waitForDmaInit();
  registerMemberFunction(getDiagnostics, "SPI_DIAG");
}

bool PeripheralCommsController::transferDAC(void* buf, size_t count) {
  return performDmaTransfer(true, static_cast<uint8_t*>(buf),
                            static_cast<uint8_t*>(buf), count);
}

bool PeripheralCommsController::transferADC(void* buf, size_t count) {
  return performDmaTransfer(false, static_cast<uint8_t*>(buf),
                            static_cast<uint8_t*>(buf), count);
}

uint8_t PeripheralCommsController::transferDAC(uint8_t data) {
  uint8_t tx_byte = data;
  bool ok = performDmaTransfer(true, &tx_byte, &tx_byte, 1);
  return ok ? tx_byte : 0;
}

uint8_t PeripheralCommsController::transferADC(uint8_t data) {
  uint8_t tx_byte = data;
  bool ok = performDmaTransfer(false, &tx_byte, &tx_byte, 1);
  return ok ? tx_byte : 0;
}

bool PeripheralCommsController::transferDACNoTransaction(void* buf,
                                                        size_t count) {
  return performDmaTransfer(true, static_cast<uint8_t*>(buf),
                            static_cast<uint8_t*>(buf), count);
}

bool PeripheralCommsController::transferADCNoTransaction(void* buf,
                                                        size_t count) {
  return performDmaTransfer(false, static_cast<uint8_t*>(buf),
                            static_cast<uint8_t*>(buf), count);
}

uint8_t PeripheralCommsController::transferDACNoTransaction(uint8_t data) {
  uint8_t tx_byte = data;
  return performDmaTransfer(true, &tx_byte, &tx_byte, 1) ? tx_byte : 0;
}

uint8_t PeripheralCommsController::transferADCNoTransaction(uint8_t data) {
  uint8_t tx_byte = data;
  return performDmaTransfer(false, &tx_byte, &tx_byte, 1) ? tx_byte : 0;
}

bool PeripheralCommsController::lastTransferSucceeded() {
  return lastTransferOk;
}

OperationResult PeripheralCommsController::getDiagnostics() {
  String out = "ok=" + String(lastDiagnostics.transfer_ok ? 1 : 0);
  out += " st=" + String(lastDiagnostics.failure_stage);
  out += " dac=" + String(lastDiagnostics.is_dac ? 1 : 0);
  out += " n=" + String(lastDiagnostics.count);
  out += " sr=" + String(lastDiagnostics.spi_sr, HEX);
  out += " cfg1=" + String(lastDiagnostics.spi_cfg1, HEX);
  out += " cfg2=" + String(lastDiagnostics.spi_cfg2, HEX);
  out += " cr1=" + String(lastDiagnostics.spi_cr1, HEX);
  out += " cr2=" + String(lastDiagnostics.spi_cr2, HEX);
  out += " ier=" + String(lastDiagnostics.spi_ier, HEX);
  out += " i2s=" + String(lastDiagnostics.spi_i2scfgr, HEX);
  out += " lisr=" + String(lastDiagnostics.dma_lisr, HEX);
  out += " txcr=" + String(lastDiagnostics.tx_cr, HEX);
  out += " rxcr=" + String(lastDiagnostics.rx_cr, HEX);
  out += " txn=" + String(lastDiagnostics.tx_ndtr);
  out += " rxn=" + String(lastDiagnostics.rx_ndtr);
  out += " d2=" + String(RCC->D2CCIP1R, HEX);
  out += " mux=" + String(DMAMUX1_Channel0->CCR, HEX) + "/" +
         String(DMAMUX1_Channel1->CCR, HEX) + "/" +
         String(DMAMUX1_Channel2->CCR, HEX) + "/" +
         String(DMAMUX1_Channel3->CCR, HEX);
  return OperationResult::Success(out);
}

void PeripheralCommsController::dataLedOn() {
  /* digitalWrite(led, HIGH); */
}

void PeripheralCommsController::dataLedOff() {
  /* digitalWrite(led, LOW); */
}
