#include <Arduino.h>

#include "Config.h"
#include "FunctionRegistry/FunctionRegistry.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/God.h"
#include "Peripherals/PeripheralCommsController.h"
#include <vector>
#include "UserIOHandler.h"

#include "Utils/shared_memory.h"

#include "Peripherals/God2D.h"


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  if (!initSharedMemory()) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // timing debug stuff
  // // Enable trace and debug blocks (TRCENA bit in DEMCR)
  //CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  // // Clear the counter
  //DWT->CYCCNT = 0;
  // // Enable the cycle counter
  //DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  UserIOHandler::setup();

  PeripheralCommsController::setup();


  for (int i : dac_cs_pins) {
    DACController::addChannel(i);
  }

  for (int i=0; i<NUM_ADC_BOARDS; i++) {
    ADCController::addBoard(adc_cs_pins[i], drdy[i], reset[i], i);
  }

  DACController::setup();
  ADCController::setup();

  God::setup();
  God2D::setup();
  
  // wait for DMA initialization 
  while (!isBootComplete());

  // Wait for calibration data to be sent
  while (!isCalibrationReady());

  //load calibration data from Flash
  CalibrationData calibrationData;
  m4ReceiveCalibrationData(calibrationData);

  // Set calibrations for DAC
  for (int i=0; i<NUM_DAC_CHANNELS; i++) {
    DACController::setCalibration(i, calibrationData.offset[i], calibrationData.gain[i]);
  }

  // Set calibrations for ADC
  if (!calibrationData.adcCalibrated) {
    // If ADC is not calibrated, reset ADC to all initial values
    ADCController::hardResetAllADCBoards();
    for (int i = 0; i < NUM_ADC_BOARDS * NUM_CHANNELS_PER_ADC_BOARD; i++) {
      uint32_t zeroScaleCalibration = ADCController::getChZeroScaleCalibration(i).getMessage().toInt();
      uint32_t fullScaleCalibration = ADCController::getChFullScaleCalibration(i).getMessage().toInt();

      calibrationData.adc_offset[i] = zeroScaleCalibration;
      calibrationData.adc_gain[i] = fullScaleCalibration;

      calibrationData.adcCalibrated = true;
    }
    m4SendCalibrationData(calibrationData);
  } else {
    for (int i=0; i<NUM_ADC_BOARDS * NUM_CHANNELS_PER_ADC_BOARD; i++) {
      ADCController::setChZeroScaleCalibration(i, calibrationData.adc_offset[i]);
      ADCController::setChFullScaleCalibration(i, calibrationData.adc_gain[i]);
    }
  }
}

void loop() {
  UserIOHandler::handleUserIO();
}