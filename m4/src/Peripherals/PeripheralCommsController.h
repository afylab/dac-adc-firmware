#pragma once

#include <Arduino.h>
#include "SPI.h"
#include "Config.h"

// Required for STM32H7xx register definitions (SPI_TypeDef, DMA_TypeDef, etc.)
// The Arduino Giga core should provide a path to this.
#if defined(ARDUINO_GIGA) || defined(CORE_STM32H7)
#include "stm32h7xx.h"
#else
#error "This code is intended for STM32H7 based boards like Arduino Giga."
#endif

// DMAMUX Request IDs (from user-provided example)
typedef enum {
  DMAMUX1_REQ_GEN0 = 1, DMAMUX1_REQ_GEN1 = 2, DMAMUX1_REQ_GEN2 = 3, DMAMUX1_REQ_GEN3 = 4, DMAMUX1_REQ_GEN4 = 5,
  DMAMUX1_REQ_GEN5 = 6, DMAMUX1_REQ_GEN6 = 7, DMAMUX1_REQ_GEN7 = 8, ADC1_DMA = 9, ADC2_DMA = 10,
  TIM1_CH1 = 11, TIM1_CH2 = 12, TIM1_CH3 = 13, TIM1_CH4 = 14, TIM1_UP = 15,
  TIM1_TRIG = 16, TIM1_COM = 17, TIM2_CH1 = 18, TIM2_CH2 = 19, TIM2_CH3 = 20,
  TIM2_CH4 = 21, TIM2_UP = 22, TIM3_CH1 = 23, TIM3_CH2 = 24, TIM3_CH3 = 25,
  TIM3_CH4 = 26, TIM3_UP = 27, TIM3_TRIG = 28, TIM4_CH1 = 29, TIM4_CH2 = 30,
  TIM4_CH3 = 31, TIM4_UP = 32, I2C1_RX_DMA = 33, I2C1_TX_DMA = 34, I2C2_RX_DMA = 35,
  I2C2_TX_DMA = 36, SPI1_RX_DMA = 37, SPI1_TX_DMA = 38, SPI2_RX_DMA = 39, SPI2_TX_DMA = 40,
  USART1_RX_DMA = 41, USART1_TX_DMA = 42, USART2_RX_DMA = 43, USART2_TX_DMA = 44, USART3_RX_DMA = 45,
  USART3_TX_DMA = 46, TIM8_CH1 = 47, TIM8_CH2 = 48, TIM8_CH3 = 49, TIM8_CH4 = 50,
  TIM8_UP = 51, TIM8_TRIG = 52, TIM8_COM = 53, RESERVED_DMAMUX_54 = 54, TIM5_CH1 = 55,
  TIM5_CH2 = 56, TIM5_CH3 = 57, TIM5_CH4 = 58, TIM5_UP = 59, TIM5_TRIG = 60,
  SPI3_RX_DMA = 61, SPI3_TX_DMA = 62, UART4_RX_DMA = 63, UART4_TX_DMA = 64, UART5_RX_DMA = 65,
  UART5_TX_DMA = 66, DAC_CH1_DMA = 67, DAC_CH2_DMA = 68, TIM6_UP = 69, TIM7_UP = 70,
  USART6_RX_DMA = 71, USART6_TX_DMA = 72, I2C3_RX_DMA = 73, I2C3_TX_DMA = 74, DCMI_DMA = 75,
  CRYP_IN_DMA = 76, CRYP_OUT_DMA = 77, HASH_IN_DMA = 78, UART7_RX_DMA = 79, UART7_TX_DMA = 80,
  UART8_RX_DMA = 81, UART8_TX_DMA = 82, SPI4_RX_DMA = 83, SPI4_TX_DMA = 84, SPI5_RX_DMA = 85,
  SPI5_TX_DMA = 86, SAI1A_DMA = 87, SAI1B_DMA = 88, SAI2A_DMA = 89, SAI2B_DMA = 90,
  SWPMI_RX_DMA = 91, SWPMI_TX_DMA = 92, SPDIFRX_DAT_DMA = 93, SPDIFRX_CTRL_DMA = 94, HR_REQ_1 = 95,
  HR_REQ_2 = 96, HR_REQ_3 = 97, HR_REQ_4 = 98, HR_REQ_5 = 99, HR_REQ_6 = 100,
  DFSDM1_DMA0 = 101, DFSDM1_DMA1 = 102, DFSDM1_DMA2 = 103, DFSDM1_DMA3 = 104, TIM15_CH1 = 105,
  TIM15_UP = 106, TIM15_TRIG = 107, TIM15_COM = 108, TIM16_CH1 = 109, TIM16_UP = 110,
  TIM17_CH1 = 111, TIM17_UP = 112, SAI3_A_DMA = 113, SAI3_B_DMA = 114, ADC3_DMA = 115
} DMAMUX1_CxCR_DMAREQ_ID;

// Cache line size, typically 32 bytes for STM32H7
#define CACHE_LINE_SIZE 32

// Forward declare the class
class PeripheralCommsController;

// Structure to hold DMA configuration for a given SPI interface (DAC or ADC)
struct SpiDmaHardwareConfig {
    SPI_TypeDef* spi_regs;
    DMA_TypeDef* dma_regs; // DMA1 or DMA2

    DMA_Stream_TypeDef* tx_dma_stream;
    uint8_t tx_dma_stream_number; // 0-7 for status/clear logic
    DMAMUX_Channel_TypeDef* tx_dmamux_channel_regs;
    DMAMUX1_CxCR_DMAREQ_ID tx_dmamux_req_id;
    IRQn_Type tx_dma_irqn;

    DMA_Stream_TypeDef* rx_dma_stream;
    uint8_t rx_dma_stream_number; // 0-7
    DMAMUX_Channel_TypeDef* rx_dmamux_channel_regs;
    DMAMUX1_CxCR_DMAREQ_ID rx_dmamux_req_id;
    IRQn_Type rx_dma_irqn;

    volatile bool* tx_complete_flag;
    volatile bool* rx_complete_flag;
    volatile bool* error_flag;
};

// Global DMA status flags
static volatile bool dac_dma_tx_complete = false;
static volatile bool dac_dma_rx_complete = false;
static volatile bool dac_dma_error = false;

#ifdef __NEW_SHIELD__
static volatile bool adc_dma_tx_complete = false;
static volatile bool adc_dma_rx_complete = false;
static volatile bool adc_dma_error = false;
#else // Old shield ADC uses DAC DMA resources, so shares flags
#define adc_dma_tx_complete dac_dma_tx_complete
#define adc_dma_rx_complete dac_dma_rx_complete
#define adc_dma_error dac_dma_error
#endif

// Static configurations for DAC and ADC DMA (initialized in setup)
static SpiDmaHardwareConfig dac_hw_config;
#ifdef __NEW_SHIELD__
static SpiDmaHardwareConfig adc_hw_config;
#endif

// Fallback macro definitions if not available
#ifndef RCC_AHB1ENR_DMAMUX1EN
#define RCC_AHB1ENR_DMAMUX1EN (1U << 2) // Bit 2 in AHB1ENR for DMAMUX1
#endif

#ifndef RCC_APB1LENR_SPI5EN
#define RCC_APB1LENR_SPI5EN (1U << 20) // Bit 20 in APB1LENR for SPI5
#endif

#ifndef SPI_CR1_CRCEN
#define SPI_CR1_CRCEN (1U << 13) // Bit 13 in CR1 for CRC enable
#endif

// Fallback SPI base addresses if not defined (STM32H747XI values)
#ifndef SPI1_BASE
#define SPI1_BASE 0x40013000UL
#endif

#ifndef SPI5_BASE
#define SPI5_BASE 0x40015000UL
#endif

// Fallback cache functions if CMSIS ones aren't available
#ifndef SCB_CleanInvalidateDCache_by_Addr
static inline void fallback_cache_clean_invalidate(void* addr, size_t size) {
    (void)addr; (void)size; // Suppress unused warnings
    __asm volatile ("dsb sy" : : : "memory");  // Data Synchronization Barrier
    __asm volatile ("isb sy" : : : "memory");  // Instruction Synchronization Barrier
}
#define SCB_CleanInvalidateDCache_by_Addr(addr, size) fallback_cache_clean_invalidate(addr, size)
#endif

#ifndef SCB_InvalidateDCache_by_Addr
static inline void fallback_cache_invalidate(void* addr, size_t size) {
    (void)addr; (void)size; // Suppress unused warnings
    __asm volatile ("dsb sy" : : : "memory");  // Data Synchronization Barrier
    __asm volatile ("isb sy" : : : "memory");  // Instruction Synchronization Barrier
}
#define SCB_InvalidateDCache_by_Addr(addr, size) fallback_cache_invalidate(addr, size)
#endif

class PeripheralCommsController {
public:
    // DMA Stream Status/Clear helpers - moved to public for ISR access
    static uint8_t getDmaStreamIrqStatus(DMA_TypeDef* dma_regs, uint8_t stream_idx) {
        uint32_t status_reg_val;
        uint8_t relevant_bits_offset;

        if (stream_idx < 4) { // Streams 0-3 use LISR
            status_reg_val = dma_regs->LISR;
            if (stream_idx == 0) relevant_bits_offset = 0;
            else if (stream_idx == 1) relevant_bits_offset = 6;
            else if (stream_idx == 2) relevant_bits_offset = 16;
            else relevant_bits_offset = 22; // stream 3
        } else { // Streams 4-7 use HISR
            status_reg_val = dma_regs->HISR;
            if (stream_idx == 4) relevant_bits_offset = 0;
            else if (stream_idx == 5) relevant_bits_offset = 6;
            else if (stream_idx == 6) relevant_bits_offset = 16;
            else relevant_bits_offset = 22; // stream 7
        }
        return (status_reg_val >> relevant_bits_offset) & 0x3F; // 6 status bits per stream
    }

    static void clearDmaStreamIrqFlags(DMA_TypeDef* dma_regs, uint8_t stream_idx, uint8_t flags_to_clear) {
        uint32_t clear_val = ((uint32_t)flags_to_clear) & 0x3F; // Ensure only 6 bits

        if (stream_idx < 4) { // Streams 0-3 use LIFCR
            if (stream_idx == 0) dma_regs->LIFCR = (clear_val << 0);
            else if (stream_idx == 1) dma_regs->LIFCR = (clear_val << 6);
            else if (stream_idx == 2) dma_regs->LIFCR = (clear_val << 16);
            else dma_regs->LIFCR = (clear_val << 22); // stream 3
        } else { // Streams 4-7 use HIFCR
            if (stream_idx == 4) dma_regs->HIFCR = (clear_val << 0);
            else if (stream_idx == 5) dma_regs->HIFCR = (clear_val << 6);
            else if (stream_idx == 6) dma_regs->HIFCR = (clear_val << 16);
            else dma_regs->HIFCR = (clear_val << 22); // stream 7
        }
    }

private:
    int cs_pin;
    bool is_adc; // To differentiate which config to use for old shield

    // Helper to get SPI kernel clock frequency (placeholder, needs accurate implementation for Giga)
    static uint32_t getSPIKernelClockHz(SPI_TypeDef* spi_instance) {
        // For now, return a reasonable default since we can't easily determine
        // the exact peripheral clock without diving deep into RCC configuration
        return 200000000; // 200MHz is a reasonable assumption for STM32H7 peripheral clock
    }

    static void configureSpiPeripheral(SPI_TypeDef* spi, const SPISettings& settings) {
        // Disable SPI before configuration
        spi->CR1 &= ~SPI_CR1_SPE;
        while(spi->CR1 & SPI_CR1_SPE); // Ensure SPE is cleared
        
        // Disable CRC if the bit is defined
        #ifdef SPI_CR1_CRCEN
        spi->CR1 &= ~SPI_CR1_CRCEN;
        #endif

        uint32_t cfg1_val = 0;
        uint32_t cfg2_val = 0;

        // Data size: 8-bit transfers
        cfg1_val |= (7U << SPI_CFG1_DSIZE_Pos); // 8-bit data

        // Baud rate
        uint32_t spi_ker_ck = getSPIKernelClockHz(spi);
        if (spi_ker_ck == 0) { 
            spi_ker_ck = 200000000; // Fallback
        }
        
        // Get clock frequency from SPISettings using the correct method
        uint32_t clock_freq = settings.getClockFreq(); // Use getClockFreq() instead of getClock()
        if (clock_freq == 0) {
             clock_freq = 1000000; // Default to 1MHz if not set or zero
        }

        uint32_t baud_rate_prescaler_val = spi_ker_ck / clock_freq;

        uint32_t mbr = 0;
        if (baud_rate_prescaler_val <= 2) mbr = 0b000;      // /2
        else if (baud_rate_prescaler_val <= 4) mbr = 0b001; // /4
        else if (baud_rate_prescaler_val <= 8) mbr = 0b010; // /8
        else if (baud_rate_prescaler_val <= 16) mbr = 0b011;// /16
        else if (baud_rate_prescaler_val <= 32) mbr = 0b100;// /32
        else if (baud_rate_prescaler_val <= 64) mbr = 0b101;// /64
        else if (baud_rate_prescaler_val <= 128) mbr = 0b110;// /128
        else mbr = 0b111;                                   // /256
        
        cfg1_val |= (mbr << SPI_CFG1_MBR_Pos);

        // SPI Mode (CPOL and CPHA) - use correct method name
        uint8_t spi_mode = settings.getDataMode(); // Use getDataMode() instead of getMode()
        if (spi_mode == SPI_MODE0) { /*CPOL=0, CPHA=0 by default*/ }
        else if (spi_mode == SPI_MODE1) { cfg2_val |= SPI_CFG2_CPHA; } // CPHA=1
        else if (spi_mode == SPI_MODE2) { cfg2_val |= SPI_CFG2_CPOL; } // CPOL=1
        else if (spi_mode == SPI_MODE3) { cfg2_val |= SPI_CFG2_CPOL | SPI_CFG2_CPHA; } // CPOL=1, CPHA=1

        // Bit order (MSBFIRST is typical)
        if (settings.getBitOrder() == LSBFIRST) {
            cfg2_val |= SPI_CFG2_LSBFRST;
        }
        
        // Master mode
        cfg2_val |= SPI_CFG2_MASTER;
        // Software slave management
        cfg2_val |= SPI_CFG2_AFCNTR;
        cfg2_val |= SPI_CFG2_SSOM; 

        // FIFO threshold: Set to 1 data item
        cfg1_val = (cfg1_val & ~SPI_CFG1_FTHLV_Msk) | (0U << SPI_CFG1_FTHLV_Pos);

        // Apply configurations
        spi->CFG1 = cfg1_val;
        spi->CFG2 = cfg2_val;

        // Enable SPI DMA for TX and RX
        spi->CFG1 |= SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN;
    }

    // Internal DMA transfer logic
    uint8_t performSpiDmaTransfer(const SpiDmaHardwareConfig& config,
                                  const SPISettings& spi_settings,
                                  uint8_t* tx_buffer, uint8_t* rx_buffer, size_t count) {
        
        if (count == 0) return 0; // Nothing to transfer

        // 0. Reset completion and error flags
        *config.tx_complete_flag = false;
        *config.rx_complete_flag = false;
        *config.error_flag = false;

        // 1. Configure SPI peripheral (baud, mode etc.)
        configureSpiPeripheral(config.spi_regs, spi_settings);

        // 2. Chip Select LOW
        digitalWrite(cs_pin, LOW);
        
        // 3. Cache Maintenance
        // Ensure TX buffer is written to RAM from cache
        if (tx_buffer) {
            // Align address and size to cache line
            uint32_t tx_addr_aligned = (uint32_t)tx_buffer & ~(CACHE_LINE_SIZE - 1);
            uint32_t tx_len_aligned = (((uint32_t)tx_buffer + count - tx_addr_aligned + CACHE_LINE_SIZE -1) / CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
            SCB_CleanInvalidateDCache_by_Addr((void*)tx_addr_aligned, tx_len_aligned);
        }
        // Invalidate RX buffer in cache before DMA write
        if (rx_buffer) {
            uint32_t rx_addr_aligned = (uint32_t)rx_buffer & ~(CACHE_LINE_SIZE - 1);
            uint32_t rx_len_aligned = (((uint32_t)rx_buffer + count - rx_addr_aligned + CACHE_LINE_SIZE -1) / CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
            SCB_InvalidateDCache_by_Addr((void*)rx_addr_aligned, rx_len_aligned);
        }

        // 4. Configure RX DMA Stream (must be done before TX if full duplex)
        // Disable stream before configuration
        config.rx_dma_stream->CR &= ~DMA_SxCR_EN;
        while (config.rx_dma_stream->CR & DMA_SxCR_EN) { /* Wait for disable */ }
        clearDmaStreamIrqFlags(config.dma_regs, config.rx_dma_stream_number, 0x3F); // Clear all flags

        config.rx_dma_stream->PAR = (uint32_t)&config.spi_regs->RXDR; // Peripheral address (SPI RX Data Register)
        config.rx_dma_stream->M0AR = (uint32_t)(rx_buffer ? rx_buffer : tx_buffer); // Memory address (RX buffer, or dummy if no RX buf)
        config.rx_dma_stream->NDTR = count; // Number of data items

        uint32_t rx_cr_val = 0;
        rx_cr_val |= DMA_SxCR_PL_1; // Priority high (0b10)
        rx_cr_val |= (0 << DMA_SxCR_MSIZE_Pos); // Memory data size: 8-bit (byte)
        rx_cr_val |= (0 << DMA_SxCR_PSIZE_Pos); // Peripheral data size: 8-bit (byte)
        rx_cr_val |= DMA_SxCR_MINC; // Memory increment mode
        // DIR: Peripheral-to-memory (0b00) - already default
        rx_cr_val |= DMA_SxCR_TCIE; // Transfer complete interrupt enable
        rx_cr_val |= DMA_SxCR_TEIE; // Transfer error interrupt enable
        // rx_cr_val |= DMA_SxCR_CIRC; // Circular mode (not for single shot)
        config.rx_dma_stream->CR = rx_cr_val;
        config.rx_dma_stream->FCR = 0; // Direct mode, no FIFO interrupts (optional: configure FIFO)

        // 5. Configure TX DMA Stream
        config.tx_dma_stream->CR &= ~DMA_SxCR_EN;
        while (config.tx_dma_stream->CR & DMA_SxCR_EN) { /* Wait for disable */ }
        clearDmaStreamIrqFlags(config.dma_regs, config.tx_dma_stream_number, 0x3F);

        config.tx_dma_stream->PAR = (uint32_t)&config.spi_regs->TXDR; // Peripheral address (SPI TX Data Register)
        config.tx_dma_stream->M0AR = (uint32_t)tx_buffer; // Memory address (TX buffer)
        config.tx_dma_stream->NDTR = count;

        uint32_t tx_cr_val = 0;
        tx_cr_val |= DMA_SxCR_PL_1; // Priority high
        tx_cr_val |= (0 << DMA_SxCR_MSIZE_Pos); // Mem 8-bit
        tx_cr_val |= (0 << DMA_SxCR_PSIZE_Pos); // Periph 8-bit
        tx_cr_val |= DMA_SxCR_MINC; // Mem increment
        tx_cr_val |= DMA_SxCR_DIR_0; // Direction: Memory-to-peripheral (0b01)
        tx_cr_val |= DMA_SxCR_TCIE; // Transfer complete interrupt
        tx_cr_val |= DMA_SxCR_TEIE; // Transfer error interrupt
        config.tx_dma_stream->CR = tx_cr_val;
        config.tx_dma_stream->FCR = 0; // Direct mode

        // 6. Enable DMA Streams (RX first, then TX)
        config.rx_dma_stream->CR |= DMA_SxCR_EN;
        config.tx_dma_stream->CR |= DMA_SxCR_EN;
        
        // 7. Enable SPI peripheral and start transfer
        config.spi_regs->CR1 |= SPI_CR1_SPE;    // Enable SPI
        config.spi_regs->IFCR = 0xFFFFFFFF;      // Clear all SPI flags before starting
        config.spi_regs->CR1 |= SPI_CR1_CSTART; // Start transfer

        // 8. Wait for DMA completion (both TX and RX, or error)
        // Timeout for safety (e.g. 100ms)
        uint32_t timeout_start = millis();
        while (!(*config.tx_complete_flag && *config.rx_complete_flag) && !*config.error_flag) {
            if (millis() - timeout_start > 100) {
                *config.error_flag = true; // Signal timeout error
                // TODO: more robust error handling - disable DMA/SPI
                break;
            }
            // Yield or delay slightly if in a tight loop in a bare-metal like environment
            // For Arduino, this busy wait might be okay for short transfers.
        }
        
        // 9. Disable SPI, DMA streams after transfer
        config.spi_regs->CR1 &= ~SPI_CR1_SPE; // Disable SPI
        // It's good practice to ensure CSTART is not pending if master
        if ( (config.spi_regs->CR1 & SPI_CR1_CSTART) && (config.spi_regs->SR & SPI_SR_EOT)) {
             // if EOT is set, SPI is done. If still busy, we might need to wait for TXC or similar
        }
        config.spi_regs->IFCR = 0xFFFFFFFF; // Clear SPI flags

        config.tx_dma_stream->CR &= ~DMA_SxCR_EN; // Disable TX DMA
        config.rx_dma_stream->CR &= ~DMA_SxCR_EN; // Disable RX DMA
        
        // Clear potential leftover flags in DMA status registers
        clearDmaStreamIrqFlags(config.dma_regs, config.tx_dma_stream_number, 0x3F);
        clearDmaStreamIrqFlags(config.dma_regs, config.rx_dma_stream_number, 0x3F);

        // 10. Chip Select HIGH
        digitalWrite(cs_pin, HIGH);

        // Optional: Invalidate RX buffer again to ensure CPU sees DMA'd data
        if (rx_buffer && !*config.error_flag) {
             uint32_t rx_addr_aligned = (uint32_t)rx_buffer & ~(CACHE_LINE_SIZE - 1);
             uint32_t rx_len_aligned = (((uint32_t)rx_buffer + count - rx_addr_aligned + CACHE_LINE_SIZE -1) / CACHE_LINE_SIZE) * CACHE_LINE_SIZE;
             SCB_InvalidateDCache_by_Addr((void*)rx_addr_aligned, rx_len_aligned);
        }
        
        if (*config.error_flag) {
            // Handle error - for now, return 0 or specific error code
            return 0; // Or some error indicator
        }
        
        // For single byte transfer, return received byte
        if (count == 1 && rx_buffer) {
            return rx_buffer[0];
        }
        return (rx_buffer && count > 0) ? rx_buffer[0] : 0; // Or just void for buffer transfers
    }

    // Helper to initialize a SpiDmaHardwareConfig struct
    static void initSingleDmaStreamConfig(DMA_Stream_TypeDef* stream, uint8_t stream_num,
                                          DMA_TypeDef* dma_ctrl, DMAMUX_Channel_TypeDef* mux_ch,
                                          DMAMUX1_CxCR_DMAREQ_ID req_id, IRQn_Type irqn) {
        // DMAMUX configuration
        mux_ch->CCR = req_id;

        // DMA Stream basic config
        stream->CR &= ~DMA_SxCR_EN;
        while(stream->CR & DMA_SxCR_EN);

        clearDmaStreamIrqFlags(dma_ctrl, stream_num, 0x3F);

        NVIC_SetPriority(irqn, 2);
        NVIC_EnableIRQ(irqn);
    }

public:
    PeripheralCommsController(int cs, bool for_adc_old_shield = false) : cs_pin(cs), is_adc(for_adc_old_shield) {
        pinMode(cs_pin, OUTPUT);
        digitalWrite(cs_pin, HIGH);
    }

    static void setup() {
        // 1. Enable Clocks for DMA, DMAMUX, SPI1, SPI5
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;

        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
        #ifdef __NEW_SHIELD__
        SET_BIT(RCC->APB1LENR, RCC_APB1LENR_SPI5EN);
        #endif
        
        volatile uint32_t dummy_read = RCC->AHB1ENR;
        (void)dummy_read;
        delay(1);

        // Initialize DAC DMA Configuration - use SPI peripheral base addresses
        dac_hw_config.spi_regs = (SPI_TypeDef*)SPI1_BASE; // Use SPI1_BASE for direct register access
        dac_hw_config.dma_regs = DMA1;
        
        dac_hw_config.tx_dma_stream = DMA1_Stream0;
        dac_hw_config.tx_dma_stream_number = 0;
        dac_hw_config.tx_dmamux_channel_regs = DMAMUX1_Channel0;
        dac_hw_config.tx_dmamux_req_id = SPI1_TX_DMA;
        dac_hw_config.tx_dma_irqn = DMA1_Stream0_IRQn;
        dac_hw_config.tx_complete_flag = &dac_dma_tx_complete;

        dac_hw_config.rx_dma_stream = DMA1_Stream1;
        dac_hw_config.rx_dma_stream_number = 1;
        dac_hw_config.rx_dmamux_channel_regs = DMAMUX1_Channel1;
        dac_hw_config.rx_dmamux_req_id = SPI1_RX_DMA;
        dac_hw_config.rx_dma_irqn = DMA1_Stream1_IRQn;
        dac_hw_config.rx_complete_flag = &dac_dma_rx_complete;
        dac_hw_config.error_flag = &dac_dma_error;

        initSingleDmaStreamConfig(dac_hw_config.tx_dma_stream, dac_hw_config.tx_dma_stream_number, dac_hw_config.dma_regs, dac_hw_config.tx_dmamux_channel_regs, dac_hw_config.tx_dmamux_req_id, dac_hw_config.tx_dma_irqn);
        initSingleDmaStreamConfig(dac_hw_config.rx_dma_stream, dac_hw_config.rx_dma_stream_number, dac_hw_config.dma_regs, dac_hw_config.rx_dmamux_channel_regs, dac_hw_config.rx_dmamux_req_id, dac_hw_config.rx_dma_irqn);

        #ifdef __NEW_SHIELD__
        // Initialize ADC DMA Configuration
        adc_hw_config.spi_regs = (SPI_TypeDef*)SPI5_BASE; // Use SPI5_BASE for direct register access
        adc_hw_config.dma_regs = DMA1;

        adc_hw_config.tx_dma_stream = DMA1_Stream2;
        adc_hw_config.tx_dma_stream_number = 2;
        adc_hw_config.tx_dmamux_channel_regs = DMAMUX1_Channel2;
        adc_hw_config.tx_dmamux_req_id = SPI5_TX_DMA;
        adc_hw_config.tx_dma_irqn = DMA1_Stream2_IRQn;
        adc_hw_config.tx_complete_flag = &adc_dma_tx_complete;
        
        adc_hw_config.rx_dma_stream = DMA1_Stream3;
        adc_hw_config.rx_dma_stream_number = 3;
        adc_hw_config.rx_dmamux_channel_regs = DMAMUX1_Channel3;
        adc_hw_config.rx_dmamux_req_id = SPI5_RX_DMA;
        adc_hw_config.rx_dma_irqn = DMA1_Stream3_IRQn;
        adc_hw_config.rx_complete_flag = &adc_dma_rx_complete;
        adc_hw_config.error_flag = &adc_dma_error;

        initSingleDmaStreamConfig(adc_hw_config.tx_dma_stream, adc_hw_config.tx_dma_stream_number, adc_hw_config.dma_regs, adc_hw_config.tx_dmamux_channel_regs, adc_hw_config.tx_dmamux_req_id, adc_hw_config.tx_dma_irqn);
        initSingleDmaStreamConfig(adc_hw_config.rx_dma_stream, adc_hw_config.rx_dma_stream_number, adc_hw_config.dma_regs, adc_hw_config.rx_dmamux_channel_regs, adc_hw_config.rx_dmamux_req_id, adc_hw_config.rx_dma_irqn);
        #endif
    }

    void transferDAC(void* buf, size_t count) {
        performSpiDmaTransfer(dac_hw_config, DAC_SPI_SETTINGS, (uint8_t*)buf, (uint8_t*)buf, count);
    }

    uint8_t transferDAC(uint8_t data) {
        uint8_t tx_byte = data;
        uint8_t rx_byte = 0;
        performSpiDmaTransfer(dac_hw_config, DAC_SPI_SETTINGS, &tx_byte, &rx_byte, 1);
        return rx_byte;
    }

    void transferADC(void* buf, size_t count) {
        #ifdef __NEW_SHIELD__
        performSpiDmaTransfer(adc_hw_config, ADC_SPI_SETTINGS, (uint8_t*)buf, (uint8_t*)buf, count);
        #else
        performSpiDmaTransfer(dac_hw_config, ADC_SPI_SETTINGS, (uint8_t*)buf, (uint8_t*)buf, count);
        #endif
    }

    uint8_t transferADC(uint8_t data) {
        uint8_t tx_byte = data;
        uint8_t rx_byte = 0;
        #ifdef __NEW_SHIELD__
        performSpiDmaTransfer(adc_hw_config, ADC_SPI_SETTINGS, &tx_byte, &rx_byte, 1);
        #else
        performSpiDmaTransfer(dac_hw_config, ADC_SPI_SETTINGS, &tx_byte, &rx_byte, 1);
        #endif
        return rx_byte;
    }

    // NoTransaction versions - these don't manage SPI transactions themselves
    // They use the DMA implementation but expect the caller to manage any transaction state
    void transferDACNoTransaction(void* buf, size_t count) {
        performSpiDmaTransfer(dac_hw_config, DAC_SPI_SETTINGS, (uint8_t*)buf, (uint8_t*)buf, count);
    }

    uint8_t transferDACNoTransaction(uint8_t data) {
        uint8_t tx_byte = data;
        uint8_t rx_byte = 0;
        performSpiDmaTransfer(dac_hw_config, DAC_SPI_SETTINGS, &tx_byte, &rx_byte, 1);
        return rx_byte;
    }

    void transferADCNoTransaction(void* buf, size_t count) {
        #ifdef __NEW_SHIELD__
        performSpiDmaTransfer(adc_hw_config, ADC_SPI_SETTINGS, (uint8_t*)buf, (uint8_t*)buf, count);
        #else
        performSpiDmaTransfer(dac_hw_config, ADC_SPI_SETTINGS, (uint8_t*)buf, (uint8_t*)buf, count);
        #endif
    }

    uint8_t transferADCNoTransaction(uint8_t data) {
        uint8_t tx_byte = data;
        uint8_t rx_byte = 0;
        #ifdef __NEW_SHIELD__
        performSpiDmaTransfer(adc_hw_config, ADC_SPI_SETTINGS, &tx_byte, &rx_byte, 1);
        #else
        performSpiDmaTransfer(dac_hw_config, ADC_SPI_SETTINGS, &tx_byte, &rx_byte, 1);
        #endif
        return rx_byte;
    }

    // Transaction control functions for backwards compatibility
    // These are used in older shield configurations where SPI transactions need explicit management
    static void beginDacTransaction() {
        #ifdef __NEW_SHIELD__
        // For new shield, each DAC has its own SPI so no explicit transaction management needed
        // The DMA implementation handles everything
        #else
        // For old shield, multiple devices share SPI so we could configure here if needed
        // But with DMA implementation, the SPI configuration is done per-transfer
        // So these are essentially no-ops now
        #endif
    }

    static void beginAdcTransaction() {
        #ifdef __NEW_SHIELD__
        // For new shield, each ADC has its own SPI so no explicit transaction management needed
        #else
        // For old shield, multiple devices share SPI so we could configure here if needed
        // But with DMA implementation, the SPI configuration is done per-transfer
        #endif
    }

    // Existing utility methods
    static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }
    static void dataLedOff() { /*digitalWrite(led, LOW);*/ }
    static void endTransaction() { }
};

// Generic DMA ISR handler
// This needs to be adapted based on which stream corresponds to which config's flags
inline void handle_dma_interrupt(DMA_TypeDef* dma_regs, uint8_t stream_idx, volatile bool* p_tx_flag, volatile bool* p_rx_flag, volatile bool* p_error_flag) {
    uint8_t irq_status = PeripheralCommsController::getDmaStreamIrqStatus(dma_regs, stream_idx);
    
    if (irq_status & (DMA_LISR_TEIF0 << (stream_idx < 4 ? (stream_idx * 6 + (stream_idx > 1 ? 4 : 0)) : ((stream_idx-4) * 6 + ((stream_idx-4) > 1 ? 4 : 0) )) )) { // TEIF (Transfer Error)
        *p_error_flag = true;
         PeripheralCommsController::clearDmaStreamIrqFlags(dma_regs, stream_idx, DMA_LIFCR_CTEIF0); // Clear TEIF
    }
    if (irq_status & (DMA_LISR_FEIF0 << (stream_idx < 4 ? (stream_idx * 6 + (stream_idx > 1 ? 4 : 0)) : ((stream_idx-4) * 6 + ((stream_idx-4) > 1 ? 4 : 0) )) )) { // FEIF (FIFO Error)
        *p_error_flag = true; // Treat FIFO error as a transfer error
        PeripheralCommsController::clearDmaStreamIrqFlags(dma_regs, stream_idx, DMA_LIFCR_CFEIF0); // Clear FEIF
    }
    if (irq_status & (DMA_LISR_TCIF0 << (stream_idx < 4 ? (stream_idx * 6 + (stream_idx > 1 ? 4 : 0)) : ((stream_idx-4) * 6 + ((stream_idx-4) > 1 ? 4 : 0) )) )) { // TCIF (Transfer Complete)
        // This logic is a bit tricky: one stream for TX, one for RX.
        // If this ISR is for the TX stream, set tx_complete_flag.
        // If this ISR is for the RX stream, set rx_complete_flag.
        // The current SpiDmaHardwareConfig passes pointers to the *pair* of flags.
        // We need to know if this stream_idx belongs to a TX or RX path.
        
        // A simpler way for now: If it's Stream0 or Stream2 (our TX streams), set TX flag.
        // If Stream1 or Stream3 (our RX streams), set RX flag.
        if (stream_idx == dac_hw_config.tx_dma_stream_number) *dac_hw_config.tx_complete_flag = true;
        else if (stream_idx == dac_hw_config.rx_dma_stream_number) *dac_hw_config.rx_complete_flag = true;
        #ifdef __NEW_SHIELD__
        else if (stream_idx == adc_hw_config.tx_dma_stream_number) *adc_hw_config.tx_complete_flag = true;
        else if (stream_idx == adc_hw_config.rx_dma_stream_number) *adc_hw_config.rx_complete_flag = true;
        #endif
        PeripheralCommsController::clearDmaStreamIrqFlags(dma_regs, stream_idx, DMA_LIFCR_CTCIF0); // Clear TCIF
    }
     // Also clear DMEIF (Direct Mode Error) and HTIF (Half Transfer) if enabled/occurs
    PeripheralCommsController::clearDmaStreamIrqFlags(dma_regs, stream_idx, DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CHTIF0);
}

// DMA ISRs
extern "C" {
void DMA1_Stream0_IRQHandler(void) { // DAC TX (SPI1_TX)
    handle_dma_interrupt(DMA1, 0, &dac_dma_tx_complete, &dac_dma_rx_complete, &dac_dma_error);
}

void DMA1_Stream1_IRQHandler(void) { // DAC RX (SPI1_RX)
    handle_dma_interrupt(DMA1, 1, &dac_dma_tx_complete, &dac_dma_rx_complete, &dac_dma_error);
}

#ifdef __NEW_SHIELD__
void DMA1_Stream2_IRQHandler(void) { // ADC TX (SPI5_TX)
    handle_dma_interrupt(DMA1, 2, &adc_dma_tx_complete, &adc_dma_rx_complete, &adc_dma_error);
}

void DMA1_Stream3_IRQHandler(void) { // ADC RX (SPI5_RX)
    handle_dma_interrupt(DMA1, 3, &adc_dma_tx_complete, &adc_dma_rx_complete, &adc_dma_error);
}
#endif
} // extern "C"

// Redefine adc_dma flags if not __NEW_SHIELD__ to avoid compilation error on access
#ifndef __NEW_SHIELD__
#undef adc_dma_tx_complete
#undef adc_dma_rx_complete
#undef adc_dma_error
// These will point to dac flags as defined earlier
static volatile bool& adc_dma_tx_complete = dac_dma_tx_complete;
static volatile bool& adc_dma_rx_complete = dac_dma_rx_complete;
static volatile bool& adc_dma_error = dac_dma_error;
#endif 