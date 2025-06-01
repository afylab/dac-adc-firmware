#pragma once

#include <Arduino.h>
#include "SPI.h"
#include "Config.h"
#include "Utils/shared_memory.h"

#if defined(ARDUINO_GIGA) || defined(CORE_STM32H7)
#include "stm32h7xx.h"
#else
#error "This code is intended for STM32H7 based boards like the Arduino Giga."
#endif

#define CACHE_LINE_SIZE 32

// Fallback definitions if not available
#ifndef SCB_CleanInvalidateDCache_by_Addr
static inline void fallback_cache_clean_invalidate(void* addr, size_t size) {
    (void)addr; (void)size;
    __asm volatile ("dsb sy" : : : "memory");
    __asm volatile ("isb sy" : : : "memory");
}
#define SCB_CleanInvalidateDCache_by_Addr(addr, size) fallback_cache_clean_invalidate(addr, size)
#endif

#ifndef SCB_InvalidateDCache_by_Addr
static inline void fallback_cache_invalidate(void* addr, size_t size) {
    (void)addr; (void)size;
    __asm volatile ("dsb sy" : : : "memory");
    __asm volatile ("isb sy" : : : "memory");
}
#define SCB_InvalidateDCache_by_Addr(addr, size) fallback_cache_invalidate(addr, size)
#endif

// SPI base addresses (fallback if not defined)
#ifndef SPI1_BASE
#define SPI1_BASE 0x40013000UL
#endif
#ifndef SPI5_BASE  
#define SPI5_BASE 0x40015000UL
#endif

class PeripheralCommsController {

  private:
    inline static bool spiInitialized = false;
    inline static bool dmaReady = false;
    inline static bool useDma = true; // Enable DMA with proper implementation
    int cs_pin;

    // DMA buffers - must be 32-byte aligned for cache coherency
    static uint8_t __attribute__((aligned(32))) dma_tx_buffer[64];
    static uint8_t __attribute__((aligned(32))) dma_rx_buffer[64];

    // Wait for M7 to initialize DMA (only once)
    static void waitForDmaInit() {
        if (dmaReady) return;
        
        uint32_t start_time = millis();
        while (!isBootComplete()) {
            delay(1); // Short delay instead of busy wait
            if (millis() - start_time > 3000) { // 3 second timeout
                return;
            }
        }
        
        dmaReady = true;
    }

    // High-performance DMA transfer
    uint8_t performDmaTransfer(bool is_dac, uint8_t* tx_buffer, uint8_t* rx_buffer, size_t count) {
        if (count == 0) return 0;
        
        // Copy data to aligned DMA buffers
        if (tx_buffer) {
            memcpy(dma_tx_buffer, tx_buffer, count);
        } else {
            memset(dma_tx_buffer, 0, count);
        }
        
        // Clean cache for TX buffer before DMA
        SCB_CleanInvalidateDCache_by_Addr(dma_tx_buffer, ((count + 31) / 32) * 32);
        SCB_InvalidateDCache_by_Addr(dma_rx_buffer, ((count + 31) / 32) * 32);
        
        // Select DMA streams and SPI based on configuration
        DMA_Stream_TypeDef* tx_stream;
        DMA_Stream_TypeDef* rx_stream;
        SPI_TypeDef* spi_regs;
        
        if (is_dac) {
            tx_stream = DMA1_Stream0; // SPI1 TX
            rx_stream = DMA1_Stream1; // SPI1 RX  
            spi_regs = (SPI_TypeDef*)SPI1_BASE;
        } else {
            #ifdef __NEW_SHIELD__
            tx_stream = DMA1_Stream2; // SPI5 TX
            rx_stream = DMA1_Stream3; // SPI5 RX
            spi_regs = (SPI_TypeDef*)SPI5_BASE;
            #else
            tx_stream = DMA1_Stream0; // SPI1 TX (shared)
            rx_stream = DMA1_Stream1; // SPI1 RX (shared)
            spi_regs = (SPI_TypeDef*)SPI1_BASE;
            #endif
        }
        
        // Chip Select LOW
        digitalWrite(cs_pin, LOW);
        
        // Disable SPI and DMA streams
        spi_regs->CR1 &= ~SPI_CR1_SPE;
        tx_stream->CR &= ~DMA_SxCR_EN;
        rx_stream->CR &= ~DMA_SxCR_EN;
        
        // Wait for disable to complete
        while ((tx_stream->CR & DMA_SxCR_EN) || (rx_stream->CR & DMA_SxCR_EN));
        
        // Clear DMA flags
        if (tx_stream == DMA1_Stream0) DMA1->LIFCR = 0x3F;
        else if (tx_stream == DMA1_Stream2) DMA1->LIFCR = (0x3F << 16);
        
        if (rx_stream == DMA1_Stream1) DMA1->LIFCR = (0x3F << 6);
        else if (rx_stream == DMA1_Stream3) DMA1->LIFCR = (0x3F << 22);
        
        // Configure RX DMA
        rx_stream->PAR = (uint32_t)&spi_regs->RXDR;
        rx_stream->M0AR = (uint32_t)dma_rx_buffer;
        rx_stream->NDTR = count;
        rx_stream->CR = DMA_SxCR_PL_1 | DMA_SxCR_MINC; // High priority, memory increment
        rx_stream->FCR = 0; // Direct mode
        
        // Configure TX DMA  
        tx_stream->PAR = (uint32_t)&spi_regs->TXDR;
        tx_stream->M0AR = (uint32_t)dma_tx_buffer;
        tx_stream->NDTR = count;
        tx_stream->CR = DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0; // High priority, memory increment, mem-to-periph
        tx_stream->FCR = 0; // Direct mode
        
        // Configure SPI for DMA
        spi_regs->CFG1 |= SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN;
        
        // Enable DMA streams
        rx_stream->CR |= DMA_SxCR_EN;
        tx_stream->CR |= DMA_SxCR_EN;
        
        // Enable SPI and start transfer
        spi_regs->CR1 |= SPI_CR1_SPE;
        spi_regs->CR1 |= SPI_CR1_CSTART;
        
        // Wait for transfer completion with timeout
        uint32_t timeout = 1000; // 1ms timeout per byte
        while ((tx_stream->NDTR > 0 || rx_stream->NDTR > 0) && timeout > 0) {
            delayMicroseconds(1);
            timeout--;
        }
        
        // Quick cleanup
        spi_regs->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_CSTART);
        spi_regs->CFG1 &= ~(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
        tx_stream->CR &= ~DMA_SxCR_EN;
        rx_stream->CR &= ~DMA_SxCR_EN;
        
        digitalWrite(cs_pin, HIGH);
        
        // Invalidate cache for received data
        SCB_InvalidateDCache_by_Addr(dma_rx_buffer, ((count + 31) / 32) * 32);
        
        // Copy results back
        if (rx_buffer) {
            memcpy(rx_buffer, dma_rx_buffer, count);
        } else if (tx_buffer) {
            memcpy(tx_buffer, dma_rx_buffer, count);
        }
        
        return (count == 1) ? dma_rx_buffer[0] : 0;
    }
    
  public:
      PeripheralCommsController(int cs_pin) : cs_pin(cs_pin) {
        pinMode(cs_pin, OUTPUT);
        digitalWrite(cs_pin, HIGH);
      }

      #ifdef __NEW_SHIELD__
      static void setup() {
        if (!spiInitialized) {
          SPI.begin();
          SPI.beginTransaction(DAC_SPI_SETTINGS);
          spiInitialized = true;
          SPI1.begin();
          SPI1.beginTransaction(ADC_SPI_SETTINGS);
          
          // Initialize DMA on first setup
          waitForDmaInit();
        }
      }
      void transferDAC(void* buf, size_t count) {
        performDmaTransfer(true, (uint8_t*)buf, (uint8_t*)buf, count);
      }
      void transferADC(void* buf, size_t count) {
        performDmaTransfer(false, (uint8_t*)buf, (uint8_t*)buf, count);
      }
      uint8_t transferDAC(uint8_t data) {
        uint8_t tx_byte = data;
        return performDmaTransfer(true, &tx_byte, &tx_byte, 1);
      }
      uint8_t transferADC(uint8_t data) {
        uint8_t tx_byte = data;
        return performDmaTransfer(false, &tx_byte, &tx_byte, 1);
      }

      void transferDACNoTransaction(void* buf, size_t count) {
        performDmaTransfer(true, (uint8_t*)buf, (uint8_t*)buf, count);
      }
      void transferADCNoTransaction(void* buf, size_t count) {
        performDmaTransfer(false, (uint8_t*)buf, (uint8_t*)buf, count);
      }
      uint8_t transferDACNoTransaction(uint8_t data) {
        uint8_t tx_byte = data;
        return performDmaTransfer(true, &tx_byte, &tx_byte, 1);
      }
      uint8_t transferADCNoTransaction(uint8_t data) {
        uint8_t tx_byte = data;
        return performDmaTransfer(false, &tx_byte, &tx_byte, 1);
      }

      static void beginDacTransaction() { }
      static void beginAdcTransaction() { }

      #else
      static void setup() {
        if (!spiInitialized) {
          SPI.begin();
          spiInitialized = true;
          
          waitForDmaInit();
        }
      }
      void transferDAC(void* buf, size_t count) {
        SPI.beginTransaction(DAC_SPI_SETTINGS);
        performDmaTransfer(true, (uint8_t*)buf, (uint8_t*)buf, count);
        SPI.endTransaction();
      }
      void transferADC(void* buf, size_t count) {
        SPI.beginTransaction(ADC_SPI_SETTINGS);
        performDmaTransfer(false, (uint8_t*)buf, (uint8_t*)buf, count);
        SPI.endTransaction();
      }
      uint8_t transferDAC(uint8_t data) {
        SPI.beginTransaction(DAC_SPI_SETTINGS);
        uint8_t tx_byte = data;
        uint8_t result = performDmaTransfer(true, &tx_byte, &tx_byte, 1);
        SPI.endTransaction();
        return result;
      }
      uint8_t transferADC(uint8_t data) {
        SPI.beginTransaction(ADC_SPI_SETTINGS);
        uint8_t tx_byte = data;
        uint8_t result = performDmaTransfer(false, &tx_byte, &tx_byte, 1);
        SPI.endTransaction();
        return result;
      }

      void transferDACNoTransaction(void* buf, size_t count) {
        performDmaTransfer(true, (uint8_t*)buf, (uint8_t*)buf, count);
      }
      void transferADCNoTransaction(void* buf, size_t count) {
        performDmaTransfer(false, (uint8_t*)buf, (uint8_t*)buf, count);
      }
      uint8_t transferDACNoTransaction(uint8_t data) {
        uint8_t tx_byte = data;
        uint8_t result = performDmaTransfer(true, &tx_byte, &tx_byte, 1);
        return result;
      }
      uint8_t transferADCNoTransaction(uint8_t data) {
        uint8_t tx_byte = data;
        uint8_t result = performDmaTransfer(false, &tx_byte, &tx_byte, 1);
        return result;
      }

      static void beginDacTransaction() {
        SPI.beginTransaction(DAC_SPI_SETTINGS);
      }
      static void beginAdcTransaction() {
        SPI.beginTransaction(ADC_SPI_SETTINGS);
      }

      #endif

      static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }

      static void dataLedOff() { /*digitalWrite(led, LOW);*/ }

      static void endTransaction() { SPI.endTransaction(); }

  
};

uint8_t PeripheralCommsController::dma_tx_buffer[64] __attribute__((aligned(32)));
uint8_t PeripheralCommsController::dma_rx_buffer[64] __attribute__((aligned(32)));