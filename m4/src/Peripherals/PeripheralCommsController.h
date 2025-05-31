#pragma once

#include <Arduino.h>
#include "SPI.h"
// Include STM32 HAL headers before Arduino to avoid conflicts
#include "stm32h7xx_hal.h"
#include "Config.h"

// DMA-based SPI communication controller for Arduino Giga M4
class PeripheralCommsController {
public:
    // Make DMA handles public for interrupt access
    inline static DMA_HandleTypeDef hdma_spi1_tx;
    inline static DMA_HandleTypeDef hdma_spi1_rx;
    inline static DMA_HandleTypeDef hdma_spi6_tx; 
    inline static DMA_HandleTypeDef hdma_spi6_rx;
    
private:
    inline static bool spiInitialized = false;
    inline static bool dmaInitialized = false;
    
    // SPI handles - Arduino SPI maps to SPI1, Arduino SPI1 maps to SPI6
    inline static SPI_HandleTypeDef hspi1; // Arduino SPI (DAC)
    inline static SPI_HandleTypeDef hspi6; // Arduino SPI1 (ADC on new shield)
    
    // Transfer completion flags
    inline static volatile bool dacTxComplete = true;
    inline static volatile bool dacRxComplete = true;
    inline static volatile bool adcTxComplete = true;
    inline static volatile bool adcRxComplete = true;
    
    // CS pin for this instance
    int cs_pin;
    
    // Buffer management for DMA transfers
    inline static uint8_t dacTxBuffer[32];
    inline static uint8_t dacRxBuffer[32];
    inline static uint8_t adcTxBuffer[32];
    inline static uint8_t adcRxBuffer[32];

public:
    PeripheralCommsController(int cs_pin) : cs_pin(cs_pin) {}
    
    static void setup() {
        if (!spiInitialized) {
            initializeSPI();
            spiInitialized = true;
        }
        if (!dmaInitialized) {
            initializeDMA();
            dmaInitialized = true;
        }
    }
    
    // DAC transfer functions
    void transferDAC(void* buf, size_t count) {
        transferDACDMA(buf, count, true);
    }
    
    void transferDACNoTransaction(void* buf, size_t count) {
        transferDACDMA(buf, count, false);
    }
    
    uint8_t transferDAC(uint8_t data) {
        uint8_t txData = data;
        uint8_t rxData = 0;
        transferDACDMA(&txData, 1, true);
        return rxData;
    }
    
    uint8_t transferDACNoTransaction(uint8_t data) {
        uint8_t txData = data;
        uint8_t rxData = 0;
        transferDACDMA(&txData, 1, false);
        return rxData;
    }
    
    // ADC transfer functions
    void transferADC(void* buf, size_t count) {
        transferADCDMA(buf, count, true);
    }
    
    void transferADCNoTransaction(void* buf, size_t count) {
        transferADCDMA(buf, count, false);
    }
    
    uint8_t transferADC(uint8_t data) {
        uint8_t txData = data;
        uint8_t rxData = 0;
        transferADCDMA(&txData, 1, true);
        return rxData;
    }
    
    uint8_t transferADCNoTransaction(uint8_t data) {
        uint8_t txData = data;
        uint8_t rxData = 0;
        transferADCDMA(&txData, 1, false);
        return rxData;
    }
    
    // Transaction control functions
    #ifdef __NEW_SHIELD__
    static void beginDacTransaction() {
        // For new shield, DAC transactions are already set up
    }
    
    static void beginAdcTransaction() {
        // For new shield, ADC transactions are already set up  
    }
    #else
    static void beginDacTransaction() {
        // Begin DAC SPI transaction
        HAL_SPI_StateTypeDef state = HAL_SPI_GetState(&hspi1);
        if (state == HAL_SPI_STATE_READY) {
            // Configure for DAC settings
            configureSPIForDAC();
        }
    }
    
    static void beginAdcTransaction() {
        // Begin ADC SPI transaction - use SPI1 for old shield
        HAL_SPI_StateTypeDef state = HAL_SPI_GetState(&hspi1);
        if (state == HAL_SPI_STATE_READY) {
            // Configure for ADC settings
            configureSPIForADC();
        }
    }
    #endif
    
    static void endTransaction() {
        // Wait for any pending DMA transfers to complete
        waitForAllTransfersComplete();
    }
    
    // LED control functions
    static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }
    static void dataLedOff() { /*digitalWrite(led, LOW);*/ }
    
    // Wait for transfer completion
    static void waitForDACTransfer() {
        while (!dacTxComplete || !dacRxComplete) {
            __WFI(); // Wait for interrupt
        }
    }
    
    static void waitForADCTransfer() {
        while (!adcTxComplete || !adcRxComplete) {
            __WFI(); // Wait for interrupt
        }
    }
    
    static void waitForAllTransfersComplete() {
        waitForDACTransfer();
        waitForADCTransfer();
    }

private:
    static void initializeSPI() {
        // Enable SPI clocks
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_SPI6_CLK_ENABLE();
        
        // Configure SPI1 for DAC (Arduino SPI) - use STM32 HAL peripheral base
        hspi1.Instance = (SPI_TypeDef *)SPI1_BASE;
        #ifdef __NEW_SHIELD__
        hspi1.Init.Mode = SPI_MODE_MASTER;
        hspi1.Init.Direction = SPI_DIRECTION_2LINES;
        hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
        hspi1.Init.NSS = SPI_NSS_SOFT;
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 20MHz target
        hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        hspi1.Init.CRCPolynomial = 0x0;
        hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
        hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
        hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
        hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
        hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
        hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
        hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
        hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
        #else
        hspi1.Init.Mode = SPI_MODE_MASTER;
        hspi1.Init.Direction = SPI_DIRECTION_2LINES;
        hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
        hspi1.Init.NSS = SPI_NSS_SOFT;
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // 4MHz target
        hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        #endif
        
        if (HAL_SPI_Init(&hspi1) != HAL_OK) {
            // Error handling
        }
        
        // Configure SPI6 for ADC (Arduino SPI1) - use STM32 HAL peripheral base  
        hspi6.Instance = (SPI_TypeDef *)SPI6_BASE;
        #ifdef __NEW_SHIELD__
        hspi6.Init.Mode = SPI_MODE_MASTER;
        hspi6.Init.Direction = SPI_DIRECTION_2LINES;
        hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
        hspi6.Init.NSS = SPI_NSS_SOFT;
        hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // 8MHz target
        hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        hspi6.Init.CRCPolynomial = 0x0;
        hspi6.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
        hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
        hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
        hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
        hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
        hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
        hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
        hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
        #else
        hspi6.Init.Mode = SPI_MODE_MASTER;
        hspi6.Init.Direction = SPI_DIRECTION_2LINES;
        hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi6.Init.CLKPolarity = SPI_POLARITY_HIGH;
        hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
        hspi6.Init.NSS = SPI_NSS_SOFT;
        hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // 4MHz target
        hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        #endif
        
        if (HAL_SPI_Init(&hspi6) != HAL_OK) {
            // Error handling
        }
    }
    
    static void initializeDMA() {
        // Enable DMA clocks
        __HAL_RCC_DMA1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();
        
        // Configure DMA for SPI1 TX (DAC)
        hdma_spi1_tx.Instance = DMA1_Stream1;
        hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
        hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi1_tx.Init.Mode = DMA_NORMAL;
        hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        
        if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK) {
            // Error handling
        }
        
        __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);
        
        // Configure DMA for SPI1 RX (DAC)
        hdma_spi1_rx.Instance = DMA1_Stream2;
        hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
        hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi1_rx.Init.Mode = DMA_NORMAL;
        hdma_spi1_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        
        if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK) {
            // Error handling
        }
        
        __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);
        
        // Configure DMA for SPI6 TX (ADC)
        hdma_spi6_tx.Instance = DMA2_Stream1;
        hdma_spi6_tx.Init.Request = DMA_REQUEST_SPI6_TX;
        hdma_spi6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi6_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi6_tx.Init.Mode = DMA_NORMAL;
        hdma_spi6_tx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_spi6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        
        if (HAL_DMA_Init(&hdma_spi6_tx) != HAL_OK) {
            // Error handling
        }
        
        __HAL_LINKDMA(&hspi6, hdmatx, hdma_spi6_tx);
        
        // Configure DMA for SPI6 RX (ADC)
        hdma_spi6_rx.Instance = DMA2_Stream2;
        hdma_spi6_rx.Init.Request = DMA_REQUEST_SPI6_RX;
        hdma_spi6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi6_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi6_rx.Init.Mode = DMA_NORMAL;
        hdma_spi6_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_spi6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        
        if (HAL_DMA_Init(&hdma_spi6_rx) != HAL_OK) {
            // Error handling
        }
        
        __HAL_LINKDMA(&hspi6, hdmarx, hdma_spi6_rx);
        
        // Enable DMA interrupts
        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
        
        HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
        
        HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
        
        HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    }
    
    static void configureSPIForDAC() {
        #ifndef __NEW_SHIELD__
        // Reconfigure SPI1 for DAC settings
        hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // 4MHz
        HAL_SPI_Init(&hspi1);
        #endif
    }
    
    static void configureSPIForADC() {
        #ifndef __NEW_SHIELD__
        // Reconfigure SPI1 for ADC settings (old shield uses same SPI)
        hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
        hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // 4MHz
        HAL_SPI_Init(&hspi1);
        #endif
    }
    
    void transferDACDMA(void* buf, size_t count, bool useTransaction) {
        // Auto-initialize if not done yet
        if (!spiInitialized || !dmaInitialized) {
            setup();
        }
        
        if (count > sizeof(dacTxBuffer)) {
            // Handle error - buffer too large
            return;
        }
        
        // Copy data to DMA buffer (required for DMA coherency)
        memcpy(dacTxBuffer, buf, count);
        
        dacTxComplete = false;
        dacRxComplete = false;
        
        digitalWrite(cs_pin, LOW);
        
        if (HAL_SPI_TransmitReceive_DMA(&hspi1, dacTxBuffer, dacRxBuffer, count) != HAL_OK) {
            // Error handling
            dacTxComplete = true;
            dacRxComplete = true;
            digitalWrite(cs_pin, HIGH);
            return;
        }
        
        // Wait for transfer completion
        waitForDACTransfer();
        
        digitalWrite(cs_pin, HIGH);
        
        // Copy received data back to original buffer
        memcpy(buf, dacRxBuffer, count);
    }
    
    void transferADCDMA(void* buf, size_t count, bool useTransaction) {
        // Auto-initialize if not done yet
        if (!spiInitialized || !dmaInitialized) {
            setup();
        }
        
        if (count > sizeof(adcTxBuffer)) {
            // Handle error - buffer too large
            return;
        }
        
        // Copy data to DMA buffer (required for DMA coherency)
        memcpy(adcTxBuffer, buf, count);
        
        adcTxComplete = false;
        adcRxComplete = false;
        
        digitalWrite(cs_pin, LOW);
        
        #ifdef __NEW_SHIELD__
        if (HAL_SPI_TransmitReceive_DMA(&hspi6, adcTxBuffer, adcRxBuffer, count) != HAL_OK) {
        #else
        if (HAL_SPI_TransmitReceive_DMA(&hspi1, adcTxBuffer, adcRxBuffer, count) != HAL_OK) {
        #endif
            // Error handling
            adcTxComplete = true;
            adcRxComplete = true;
            digitalWrite(cs_pin, HIGH);
            return;
        }
        
        // Wait for transfer completion
        waitForADCTransfer();
        
        digitalWrite(cs_pin, HIGH);
        
        // Copy received data back to original buffer
        memcpy(buf, adcRxBuffer, count);
    }

public:
    // DMA completion callbacks
    static void dacTxCompleteCallback() {
        dacTxComplete = true;
    }
    
    static void dacRxCompleteCallback() {
        dacRxComplete = true;
    }
    
    static void adcTxCompleteCallback() {
        adcTxComplete = true;
    }
    
    static void adcRxCompleteCallback() {
        adcRxComplete = true;
    }
};

// DMA interrupt handlers
extern "C" {
    void DMA1_Stream1_IRQHandler(void) {
        HAL_DMA_IRQHandler(&PeripheralCommsController::hdma_spi1_tx);
    }
    
    void DMA1_Stream2_IRQHandler(void) {
        HAL_DMA_IRQHandler(&PeripheralCommsController::hdma_spi1_rx);
    }
    
    void DMA2_Stream1_IRQHandler(void) {
        HAL_DMA_IRQHandler(&PeripheralCommsController::hdma_spi6_tx);
    }
    
    void DMA2_Stream2_IRQHandler(void) {
        HAL_DMA_IRQHandler(&PeripheralCommsController::hdma_spi6_rx);
    }
}

// HAL callback functions
extern "C" {
    void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
        if (hspi->Instance == (SPI_TypeDef *)SPI1_BASE) {
            PeripheralCommsController::dacTxCompleteCallback();
        } else if (hspi->Instance == (SPI_TypeDef *)SPI6_BASE) {
            PeripheralCommsController::adcTxCompleteCallback();
        }
    }
    
    void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
        if (hspi->Instance == (SPI_TypeDef *)SPI1_BASE) {
            PeripheralCommsController::dacRxCompleteCallback();
        } else if (hspi->Instance == (SPI_TypeDef *)SPI6_BASE) {
            PeripheralCommsController::adcRxCompleteCallback();
        }
    }
    
    void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
        if (hspi->Instance == (SPI_TypeDef *)SPI1_BASE) {
            PeripheralCommsController::dacTxCompleteCallback();
            PeripheralCommsController::dacRxCompleteCallback();
        } else if (hspi->Instance == (SPI_TypeDef *)SPI6_BASE) {
            PeripheralCommsController::adcTxCompleteCallback();
            PeripheralCommsController::adcRxCompleteCallback();
        }
    }
}