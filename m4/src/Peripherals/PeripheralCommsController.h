#pragma once

#include <Arduino.h>
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

// Fallback definitions if not available in headers
#ifndef RCC_AHB1ENR_DMAMUX1EN
#define RCC_AHB1ENR_DMAMUX1EN (1U << 2) // Bit 2 in AHB1ENR for DMAMUX1
#endif

#ifndef RCC_APB1LENR_SPI5EN
#define RCC_APB1LENR_SPI5EN (1U << 20) // Bit 20 in APB1LENR for SPI5
#endif

#ifndef RCC_AHB4ENR_GPIOAEN
#define RCC_AHB4ENR_GPIOAEN (1U << 0) // Bit 0 in AHB4ENR for GPIOA
#endif

#ifndef RCC_AHB4ENR_GPIOFEN
#define RCC_AHB4ENR_GPIOFEN (1U << 5) // Bit 5 in AHB4ENR for GPIOF
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

    // Low-level SPI initialization without Arduino library
    static void initializeSpiHardware() {
        // Enable clocks for SPI peripherals
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);   // Enable SPI1 clock
        #ifdef __NEW_SHIELD__
        SET_BIT(RCC->APB1LENR, RCC_APB1LENR_SPI5EN); // Enable SPI5 clock
        #endif
        
        // Enable GPIO clocks (GPIOA for SPI1, GPIOF for SPI5)
        SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN);
        #ifdef __NEW_SHIELD__
        SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOFEN);
        #endif
        
        // Configure SPI1 GPIO pins (PA5=SCK, PA6=MISO, PA7=MOSI)
        // Set alternate function mode (AF5 for SPI1)
        GPIOA->MODER &= ~((3UL << 10) | (3UL << 12) | (3UL << 14)); // Clear bits
        GPIOA->MODER |= (2UL << 10) | (2UL << 12) | (2UL << 14);    // Set AF mode
        GPIOA->AFR[0] &= ~((15UL << 20) | (15UL << 24) | (15UL << 28)); // Clear AF
        GPIOA->AFR[0] |= (5UL << 20) | (5UL << 24) | (5UL << 28);   // Set AF5
        GPIOA->OSPEEDR |= (3UL << 10) | (3UL << 12) | (3UL << 14);  // High speed
        
        #ifdef __NEW_SHIELD__
        // Configure SPI5 GPIO pins (PF7=SCK, PF8=MISO, PF9=MOSI)
        // Set alternate function mode (AF5 for SPI5)
        GPIOF->MODER &= ~((3UL << 14) | (3UL << 16) | (3UL << 18)); // Clear bits
        GPIOF->MODER |= (2UL << 14) | (2UL << 16) | (2UL << 18);    // Set AF mode
        GPIOF->AFR[0] &= ~(15UL << 28);                              // Clear AF for PF7
        GPIOF->AFR[0] |= (5UL << 28);                                // Set AF5 for PF7
        GPIOF->AFR[1] &= ~((15UL << 0) | (15UL << 4));              // Clear AF for PF8,PF9
        GPIOF->AFR[1] |= (5UL << 0) | (5UL << 4);                   // Set AF5 for PF8,PF9
        GPIOF->OSPEEDR |= (3UL << 14) | (3UL << 16) | (3UL << 18);  // High speed
        #endif
        
        // Configure SPI1 registers
        SPI_TypeDef* spi1 = (SPI_TypeDef*)SPI1_BASE;
        spi1->CR1 = 0; // Reset
        spi1->CFG1 = 0; // Reset
        spi1->CFG2 = 0; // Reset
        
        // CFG1: 8-bit data, prescaler /32 (for ~5MHz from 160MHz clock)
        spi1->CFG1 = (7 << SPI_CFG1_DSIZE_Pos) |  // 8-bit data size
                     (4 << SPI_CFG1_MBR_Pos);      // Prescaler /32
        
        // CFG2: Master mode, software CS, MSB first
        spi1->CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_SSOE;
        
        // CR1: Enable SPI 
        spi1->CR1 = SPI_CR1_SPE;
        
        #ifdef __NEW_SHIELD__
        // Configure SPI5 registers (similar to SPI1)
        SPI_TypeDef* spi5 = (SPI_TypeDef*)SPI5_BASE;
        spi5->CR1 = 0; // Reset
        spi5->CFG1 = 0; // Reset  
        spi5->CFG2 = 0; // Reset
        
        // CFG1: 8-bit data, prescaler /32
        spi5->CFG1 = (7 << SPI_CFG1_DSIZE_Pos) |  // 8-bit data size
                     (4 << SPI_CFG1_MBR_Pos);      // Prescaler /32
        
        // CFG2: Master mode, software CS, MSB first
        spi5->CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_SSOE;
        
        // CR1: Enable SPI
        spi5->CR1 = SPI_CR1_SPE;
        #endif
    }

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
          initializeSpiHardware();
          spiInitialized = true;
          
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
          initializeSpiHardware();
          spiInitialized = true;
          
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

      #endif

      static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }

      static void dataLedOff() { /*digitalWrite(led, LOW);*/ }

      static void endTransaction() { }

  
};

// Static buffer definitions
uint8_t PeripheralCommsController::dma_tx_buffer[64] __attribute__((aligned(32)));
uint8_t PeripheralCommsController::dma_rx_buffer[64] __attribute__((aligned(32)));