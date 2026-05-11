#include "Peripherals/ADC/ADCBoard.h"



  void ADCBoard::waitDataReady() {
    int count = 0;
    while (digitalRead(data_ready_pin) == HIGH && count < 20000) {
      count = count + 1;
      delay(1);
    }
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, LOW);
    #endif
  }



  ADCBoard::ADCBoard(int cs_pin, int data_ready_pin, int reset_pin, int board_idx)
      : cs_pin(cs_pin),
        data_ready_pin(data_ready_pin),
        reset_pin(reset_pin),
        board_idx(board_idx),
        commsController(cs_pin) {}



  void ADCBoard::setup() {
    pinMode(reset_pin, OUTPUT);

    pinMode(data_ready_pin, INPUT);
    //attachInterrupt(digitalPinToInterrupt(data_ready_pin), this->RDY_ISR, FALLING);

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    // Resets ADC on startup.
    digitalWrite(reset_pin, HIGH);
    digitalWrite(reset_pin, LOW);
    delay(5);
    digitalWrite(reset_pin, HIGH);

    #ifdef __NEW_DAC_ADC__
    pinMode(adc_sync, OUTPUT);
    digitalWrite(adc_sync, LOW);

    //Set I/O Register such that P1 bit is set as input and SYNC pin function is enabled
    byte data[2];
    data[0] = WRITE | ADDR_IO;
    data[1] = 0b00010001;
    commsController.transferADC(data, 2);
    #endif
  }



  void ADCBoard::RDY_ISR () {
    setReadyFlag();
  }



  void ADCBoard::initialize() {
    reset();
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      setConversionTime(i, 500);
    }
  }



  uint32_t ADCBoard::getZeroScaleCalibration(int adc_channel) {
    byte data[4] = {0, 0, 0, 0};
    data[0] = READ | ADDR_CHANNELZEROSCALECAL(adc_channel);

    commsController.transferADC(data, 4);
    uint32_t result = data[1] << 16 | data[2] << 8 | data[3];
    return result;
  }



  uint32_t ADCBoard::getFullScaleCalibration(int adc_channel) {
    byte data[4] = {0, 0, 0, 0};
    data[0] = READ | ADDR_CHANNELFULLSCALECAL(adc_channel);

    commsController.transferADC(data, 4);
    uint32_t result = data[1] << 16 | data[2] << 8 | data[3];
    return result;
  }



  void ADCBoard::setZeroScaleCalibration(int adc_channel, uint32_t value) {
    byte data[4] = {0, 0, 0, 0};
    data[0] = WRITE | ADDR_CHANNELZEROSCALECAL(adc_channel);
    data[1] = 0xFF & (value >> 16);
    data[2] = 0xFF & (value >> 8);
    data[3] = 0xFF & value;

    commsController.transferADC(data, 4);
  }



  void ADCBoard::setFullScaleCalibration(int adc_channel, uint32_t value) {
    byte data[4] = {0, 0, 0, 0};
    data[0] = WRITE | ADDR_CHANNELFULLSCALECAL(adc_channel);
    data[1] = 0xFF & (value >> 16);
    data[2] = 0xFF & (value >> 8);
    data[3] = 0xFF & value;

    commsController.transferADC(data, 4);
  }



  void ADCBoard::resetToPreviousConversionTimes() {
    uint32_t zeroScaleCalibrations[NUM_CHANNELS_PER_ADC_BOARD];
    uint32_t fullScaleCalibrations[NUM_CHANNELS_PER_ADC_BOARD];

    float conversion_times[NUM_CHANNELS_PER_ADC_BOARD];
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      conversion_times[i] = getConversionTime(i); 
    }

    for(int i = 0; i<NUM_CHANNELS_PER_ADC_BOARD; i++) {
      zeroScaleCalibrations[i] = getZeroScaleCalibration(i);
      fullScaleCalibrations[i] = getFullScaleCalibration(i);
    }
    
    reset();

    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      setZeroScaleCalibration(i, zeroScaleCalibrations[i]);
      setFullScaleCalibration(i, fullScaleCalibrations[i]);
    }
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      setConversionTime(i, conversion_times[i]);
    }
  }



  int ADCBoard::getDataReadyPin() const { return data_ready_pin; }


  int ADCBoard::getBoardIndex() const { return board_idx; }



  void ADCBoard::setReadyFlag() { data_ready = true; }


  void ADCBoard::clearReadyFlag() { data_ready = false; }



  double ADCBoard::readVoltage(int channel_index) {
    startSingleConversion(channel_index);
    waitDataReady();
    uint32_t data = getConversionData(channel_index);
    return ADC2DOUBLE(data);
  }



  // return ADC status register, pg. 16
  uint8_t ADCBoard::getADCStatus() {
    byte data[2] = {0, 0};
    data[0] = READ | ADDR_ADCSTATUS;

    commsController.transferADC(data, 2);
    return data[1];
  }



  void ADCBoard::setRDYFN() {
    //first read contents of IO register
    byte new_io_reg[2];
    new_io_reg[0] = WRITE | ADDR_IO;
    new_io_reg[1] = 0b00011001;

    commsController.transferADC(new_io_reg, 2);
  }



  void ADCBoard::unsetRDYFN() {
    //first read contents of IO register
    byte new_io_reg[2];
    new_io_reg[0] = WRITE | ADDR_IO;
    new_io_reg[1] = 0b00010001;

    commsController.transferADC(new_io_reg, 2);
  }



  void ADCBoard::channelSetup(int adc_channel, uint8_t flags) {
    byte data[2];
    data[0] = WRITE | ADDR_CHANNELSETUP(adc_channel);
    data[1] = flags;
    commsController.transferADC(data, 2);
  }



  // tells the ADC to start a single conversion on the passed channel
  void ADCBoard::startSingleConversion(int adc_channel) {
    byte data[2];
    // setup communication register for writing operation to the mode register
    data[0] = WRITE | ADDR_MODE(adc_channel);
    // setup mode register
    data[1] = SINGLE_CONV_MODE;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, LOW);
    #endif
    commsController.transferADC(data, 2);
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #endif

    // data is ready when _rdy goes low
  }



  // tells the ADC to start a continous conversion on the passed channel
  void ADCBoard::startContinuousConversion(int adc_channel) {
    uint8_t data_array[4];

    // address the channel setup register and write to it
    data_array[0] = WRITE | ADDR_CHANNELSETUP(adc_channel);
    data_array[1] = CH_EN_CONT_CONV;

    // address the channel mode register and write to it
    data_array[2] = WRITE | ADDR_MODE(adc_channel);
    data_array[3] = CONT_CONV_MODE; // | 1 << 2; //includes setting continuous read mode

    // send off command
    commsController.transferADC(data_array, 4);

    // data is ready when _rdy goes low
  }



  uint8_t ADCBoard::getRevisionRegister() {
    byte data[2] = {0, 0};
    data[0] = READ | ADDR_REVISION;

    commsController.transferADC(data, 2);
    return data[1];
  }



  void ADCBoard::setConversionTime(int adc_channel, int chop, int fw) {
    byte chop_byte = chop == 1 ? 0x80 : 0x00;
    byte send = chop_byte | static_cast<byte>(fw);

    byte data[2];
    data[0] = WRITE | ADDR_CHANNELCONVERSIONTIME(adc_channel);
    data[1] = send;

    commsController.transferADC(data, 2);
  }



  uint32_t ADCBoard::getConversionData(int adc_channel) {
    byte data[4] = {0, 0, 0, 0};
    data[0] = READ | ADDR_CHANNELDATA(adc_channel);

    commsController.transferADC(data, 4);

    uint32_t upper = data[1];
    uint32_t lower = data[2];
    uint32_t last = data[3];

    uint32_t result = upper << 16 | lower << 8 | last;

    return result;
  }



  uint32_t ADCBoard::getConversionDataNoTransaction(int adc_channel) {
    byte data[4];

    // setup communication register for reading channel data
    data[0] = READ | ADDR_CHANNELDATA(adc_channel);
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;

    // write to the communication register
    // read upper and lower bytes of channel data register (16 bit mode)
    commsController.transferADCNoTransaction(data, 4);

    uint32_t upper = data[1];
    uint32_t lower = data[2];
    uint32_t last = data[3];

    uint32_t result = upper << 16 | lower << 8 | last;

    return result;
    //return 0;
  }



  std::vector<double> ADCBoard::continuousConvert(int channel_index, uint32_t period_us,
                                        uint32_t duration) {
    std::vector<double> data;
    uint32_t num_samples = duration / period_us;
    startContinuousConversion(channel_index);
    for (uint32_t i = 0; i < num_samples; i++) {
      data.push_back(ADC2DOUBLE(getConversionData(channel_index)));
      delayMicroseconds(period_us);
    }
    idleMode(channel_index);
    return data;
  }



  void ADCBoard::idleMode(int adc_channel) {
    byte data[2];
    data[0] = WRITE | ADDR_MODE(adc_channel);
    data[1] = IDLE_MODE;
    commsController.transferADC(data, 2);
  }



  bool ADCBoard::isChannelActive(int adc_channel) {
    uint8_t status = getADCStatus();
    return (status & (1 << adc_channel)) != 0;
  }



  void ADCBoard::hardReset() {
    digitalWrite(reset_pin, HIGH);
    digitalWrite(reset_pin, LOW);
    delay(5);
    digitalWrite(reset_pin, HIGH);

    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      idleMode(i);
    }
  }



  void ADCBoard::restoreCalibrationFromFlash() {
    if (!isCalibrationReady()) {
      return;
    }

    CalibrationData data;
    m4ReceiveCalibrationData(data);

    if (!data.adcCalibrated) {
      return;
    }

    int boardIndex = getBoardIndex();

    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      setZeroScaleCalibration(i, data.adc_offset[NUM_CHANNELS_PER_ADC_BOARD * boardIndex + i]);

      uint32_t fullScale = data.adc_gain[NUM_CHANNELS_PER_ADC_BOARD * boardIndex + i];
      if (fullScale != 0) {
        setFullScaleCalibration(i, fullScale);
      }
    }
  }



  void ADCBoard::reset() {
    digitalWrite(reset_pin, HIGH);
    digitalWrite(reset_pin, LOW);
    delay(5);
    digitalWrite(reset_pin, HIGH);

    // commsController.transferADC(0x28);
    // commsController.transferADC(0);
    // commsController.transferADC(0x2A);
    // commsController.transferADC(0);
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      idleMode(i);
    }

    //Set I/O Register such that P1 bit is set as input and SYNC pin function is enabled
    #ifdef __NEW_DAC_ADC__
    byte data[2];
    data[0] = WRITE | ADDR_IO;
    data[1] = 0b00010001;
    commsController.transferADC(data, 2);
    #endif

    restoreCalibrationFromFlash();
  }



  uint8_t ADCBoard::talkADC(byte command) {
    uint8_t comm = commsController.transferADC(command);
    return comm;
  }



  float ADCBoard::setConversionTime(int channel, float time_us) {
    return setConversionTimeFloat(channel, time_us,
                                  isMoreThanOneChannelActive());
  }



  float ADCBoard::setConversionTimeFloat(int channel, float time_us,
                               bool moreThanOneChannelActive) {
    return setConversionTime(
        channel, chopEnabled,
        calculateFilterWord(time_us, true, moreThanOneChannelActive),
        moreThanOneChannelActive);
  }



  float ADCBoard::setConversionTime(int channel, bool chop, byte fw,
                          bool moreThanOneChannelActive) {
    if ((fw > 127) || (chop && fw < 2) || (!chop && fw < 3)) {
      return -1;
    }

    byte chop_byte = chop ? 0b10000000 : 0b00000000;
    byte send = chop_byte | fw;

    byte data[2];
    data[0] = WRITE | ADDR_CHANNELCONVERSIONTIME(channel);
    data[1] = send;

    commsController.transferADC(data, 2);

    // could've done the calculation with user-given values but it's good to check
    float time_us = getConversionTime(channel, moreThanOneChannelActive);

    delayMicroseconds(100);

    return time_us;
  }



  float ADCBoard::setConversionTimeFW(int channel, int filter_word) {
    return setConversionTime(channel, true, filter_word,
                            isMoreThanOneChannelActive());
  }



  float ADCBoard::getConversionTime(int channel) {
    return getConversionTime(channel, isMoreThanOneChannelActive());
  }



  float ADCBoard::getConversionTime(int channel, bool moreThanOneChannelActive) {
    byte data[2] = {0, 0};
    data[0] = READ | ADDR_CHANNELCONVERSIONTIME(channel);
    commsController.transferADC(data, 2);

    return calculateConversionTime(data[1], moreThanOneChannelActive);
  }



  float ADCBoard::calculateConversionTime(byte b, bool moreThanOneChannelActive) {
    // convert to actual conversion time
    byte received_fw = b & 0b01111111;
    bool received_chop = b & 0b10000000;
    if (received_chop) {
      if (moreThanOneChannelActive) {  // FW range is 2 to 127
        return (received_fw * 128.0 + 249.0) / 6.144;
      } else {
        return (received_fw * 128.0 + 248.0) / 6.144;
      }
    } else {  // FW range is 3 to 127
      if (moreThanOneChannelActive) {
        return (received_fw * 64.0 + 206.0) / 6.144;
      } else {
        return (received_fw * 64.0 + 207.0) / 6.144;
      }
    }

    return -1;
  }



  byte ADCBoard::calculateFilterWord(float time_us, bool chop,
                           bool moreThanOneChannelActive) {
    byte out;
    if (chop) {
      if (moreThanOneChannelActive) {
        out = static_cast<byte>(round((time_us * 6.144 - 249.0) / 128.0));
      } else {
        out = static_cast<byte>(round((time_us * 6.144 - 248.0) / 128.0));
      }
      if (out < 2) return 2;
    } else {
      if (moreThanOneChannelActive) {
        out = static_cast<byte>(round((time_us * 6.144 - 206.0) / 64.0));
      } else {
        out = static_cast<byte>(round((time_us * 6.144 - 207.0) / 64.0));
      }
      if (out < 3) return 3;
    }
    if (out > 127) return 127;
    return out;
  }



  bool ADCBoard::isMoreThanOneChannelActive() {
    return (getADCStatus() & 0b00001111) != 0;
  }



  void ADCBoard::zeroScaleSelfCalibration() {
    byte data[2];
    data[0] = WRITE | ADDR_MODE(0);  // channel is zero but this is system-wide
    data[1] = ZERO_SCALE_SELF_CAL_MODE;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #endif
    commsController.transferADC(data, 2);
    waitDataReady();
  }



  void ADCBoard::zeroScaleChannelSystemSelfCalibration(int channel) {
    byte data[2];
    data[0] = WRITE | ADDR_MODE(channel);
    data[1] = CH_ZERO_SCALE_SYS_CAL_MODE;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #endif
    commsController.transferADC(data, 2);
    waitDataReady();

    // wait for the data ready flag to be set then store the calibration data in flash 
    int boardIndex = getBoardIndex();

    uint32_t zeroScaleCalibration = getZeroScaleCalibration(channel);

    CalibrationData calibrationData;
    m4ReceiveCalibrationData(calibrationData);
    calibrationData.adc_offset[NUM_CHANNELS_PER_ADC_BOARD * boardIndex + channel] = zeroScaleCalibration;
    m4SendCalibrationData(calibrationData);
    
  }



  void ADCBoard::fullScaleChannelSystemSelfCalibration(int channel) {
    byte data[2];
    data[0] = WRITE | ADDR_MODE(channel);
    data[1] = CH_FULL_SCALE_SYS_CAL_MODE;
    #ifdef __NEW_DAC_ADC__
    digitalWrite(adc_sync, HIGH);
    #endif
    commsController.transferADC(data, 2);
    waitDataReady();

    // wait for the data ready flag to be set then store the calibration data in flash
    int boardIndex = getBoardIndex();

    uint32_t fullScaleCalibration = getFullScaleCalibration(channel);

    CalibrationData calibrationData;
    m4ReceiveCalibrationData(calibrationData);
    calibrationData.adc_gain[NUM_CHANNELS_PER_ADC_BOARD * boardIndex + channel] = fullScaleCalibration;
    m4SendCalibrationData(calibrationData);
  }
