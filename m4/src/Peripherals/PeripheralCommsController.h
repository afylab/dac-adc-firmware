#pragma once

#include <Arduino.h>
#include "SPI.h"
#include "Config.h"
#include "Utils/shared_memory.h"

// Required for STM32H7xx register definitions
#if defined(ARDUINO_GIGA) || defined(CORE_STM32H7)
#include "stm32h7xx.h"
#else
#error "This code is intended for STM32H7 based boards like Arduino Giga."
#endif

// Cache line size for STM32H7
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
    int cs_pin;

    // Wait for M7 to initialize DMA (only once)
    static void waitForDmaInit() {
        if (dmaReady) return;
        
        // Send debug message
        const char* msg = "M4: Waiting for M7 DMA init...\n";
        m4SendChar(msg, strlen(msg));
        
        uint32_t start_time = millis();
        while (!isBootComplete()) {
            delay(1); // Short delay instead of busy wait
            if (millis() - start_time > 3000) { // 3 second timeout
                const char* timeout_msg = "M4: TIMEOUT waiting for M7 DMA!\n";
                m4SendChar(timeout_msg, strlen(timeout_msg));
                return;
            }
        }
        
        dmaReady = true;
        uint32_t wait_time = millis() - start_time;
        char time_msg[100];
        snprintf(time_msg, sizeof(time_msg), "M4: DMA ready after %lu ms\n", wait_time);
        m4SendChar(time_msg, strlen(time_msg));
    }

    // High-performance DMA transfer
    uint8_t performDmaTransfer(bool is_dac, uint8_t* tx_buffer, uint8_t* rx_buffer, size_t count) {
        if (count == 0) return 0;
        
        // Ensure DMA is ready
        if (!dmaReady) {
            waitForDmaInit();
            if (!dmaReady) {
                // Fallback to blocking SPI if DMA not ready
                return performFallbackSpi(is_dac, tx_buffer, rx_buffer, count);
            }
        }
        
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
        
        // Cache management for DMA coherency
        uint32_t tx_addr_aligned = (uint32_t)tx_buffer & ~(CACHE_LINE_SIZE - 1);
        uint32_t tx_len_aligned = ((count + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
        SCB_CleanInvalidateDCache_by_Addr((void*)tx_addr_aligned, tx_len_aligned);
        
        if (rx_buffer && rx_buffer != tx_buffer) {
            uint32_t rx_addr_aligned = (uint32_t)rx_buffer & ~(CACHE_LINE_SIZE - 1);
            uint32_t rx_len_aligned = ((count + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
            SCB_InvalidateDCache_by_Addr((void*)rx_addr_aligned, rx_len_aligned);
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
        rx_stream->M0AR = (uint32_t)(rx_buffer ? rx_buffer : tx_buffer);
        rx_stream->NDTR = count;
        rx_stream->CR = DMA_SxCR_PL_1 | DMA_SxCR_MINC; // High priority, memory increment
        rx_stream->FCR = 0; // Direct mode
        
        // Configure TX DMA  
        tx_stream->PAR = (uint32_t)&spi_regs->TXDR;
        tx_stream->M0AR = (uint32_t)tx_buffer;
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
        
        // Efficient polling with shorter timeout
        uint32_t timeout_cycles = count * 10; // Adaptive timeout based on transfer size
        uint32_t cycles = 0;
        
        while ((tx_stream->NDTR > 0 || rx_stream->NDTR > 0) && cycles < timeout_cycles) {
            cycles++;
            if (cycles % 100 == 0) {
                // Check for SPI errors every 100 cycles
                if (spi_regs->SR & (SPI_SR_OVR | SPI_SR_UDR)) break;
            }
        }
        
        // Quick cleanup
        spi_regs->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_CSTART);
        tx_stream->CR &= ~DMA_SxCR_EN;
        rx_stream->CR &= ~DMA_SxCR_EN;
        
        digitalWrite(cs_pin, HIGH);
        
        // Cache invalidation for received data
        if (rx_buffer) {
            uint32_t rx_addr_aligned = (uint32_t)rx_buffer & ~(CACHE_LINE_SIZE - 1);
            uint32_t rx_len_aligned = ((count + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
            SCB_InvalidateDCache_by_Addr((void*)rx_addr_aligned, rx_len_aligned);
        }
        
        return (count == 1 && rx_buffer) ? rx_buffer[0] : 0;
    }
    
    // Fallback SPI implementation
    uint8_t performFallbackSpi(bool is_dac, uint8_t* tx_buffer, uint8_t* rx_buffer, size_t count) {
        digitalWrite(cs_pin, LOW);
        
        uint8_t result = 0;
        if (count == 1) {
            result = SPI.transfer(tx_buffer ? tx_buffer[0] : 0);
            if (rx_buffer) rx_buffer[0] = result;
        } else {
            for (size_t i = 0; i < count; i++) {
                uint8_t tx_byte = tx_buffer ? tx_buffer[i] : 0;
                uint8_t rx_byte = SPI.transfer(tx_byte);
                if (rx_buffer) rx_buffer[i] = rx_byte;
                if (tx_buffer && !rx_buffer) tx_buffer[i] = rx_byte;
            }
        }
        
        digitalWrite(cs_pin, HIGH);
        return result;
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
          
          // Initialize DMA on first setup
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
        SPI.beginTransaction(DAC_SPI_SETTINGS);
        performDmaTransfer(true, (uint8_t*)buf, (uint8_t*)buf, count);
        SPI.endTransaction();
      }
      void transferADCNoTransaction(void* buf, size_t count) {
        SPI.beginTransaction(ADC_SPI_SETTINGS);
        performDmaTransfer(false, (uint8_t*)buf, (uint8_t*)buf, count);
        SPI.endTransaction();
      }
      uint8_t transferDACNoTransaction(uint8_t data) {
        SPI.beginTransaction(DAC_SPI_SETTINGS);
        uint8_t tx_byte = data;
        uint8_t result = performDmaTransfer(true, &tx_byte, &tx_byte, 1);
        SPI.endTransaction();
        return result;
      }
      uint8_t transferADCNoTransaction(uint8_t data) {
        SPI.beginTransaction(ADC_SPI_SETTINGS);
        uint8_t tx_byte = data;
        uint8_t result = performDmaTransfer(false, &tx_byte, &tx_byte, 1);
        SPI.endTransaction();
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