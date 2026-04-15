#pragma once
#include <Arduino.h>
#include "Utils/CalibrationData.h"

#define CHAR_BUFFER_SIZE 256
#define FLOAT_BUFFER_SIZE 256
#define VOLTAGE_BUFFER_SIZE 2048
#define MAX_MESSAGE_SIZE 256

constexpr uint8_t CHAR_FRAME_TYPE_NORMAL = 0x01;
constexpr uint8_t CHAR_FRAME_TYPE_FRAGMENT = 0x02;
constexpr uint8_t CHAR_FRAGMENT_VERSION = 1;
constexpr size_t CHAR_FRAGMENT_HEADER_SIZE = 9;

struct CharCircularBuffer {
  char buffer[CHAR_BUFFER_SIZE];
  volatile uint32_t read_index;
  volatile uint32_t write_index;
};

struct FloatCircularBuffer {
  float buffer[FLOAT_BUFFER_SIZE];
  volatile uint32_t read_index;
  volatile uint32_t write_index;
};

struct VoltageCircularBuffer {
  double buffer[VOLTAGE_BUFFER_SIZE];
  volatile uint32_t read_index;
  volatile uint32_t write_index;
};

struct SharedMemory {
  CharCircularBuffer m4_to_m7_char_buffer;
  CharCircularBuffer m7_to_m4_char_buffer;

  FloatCircularBuffer m4_to_m7_float_buffer;

  VoltageCircularBuffer m4_to_m7_voltage_buffer;

  volatile bool stop_flag;

  volatile bool isCalibrationUpdated;
  volatile bool isBootComplete;
  volatile bool isCalibrationReady;

  CalibrationData calibrationData;
};

extern SharedMemory* shared_memory;

bool initSharedMemory();


void m4SendCalibrationData(const CalibrationData& data);
void m4ReceiveCalibrationData(CalibrationData& data);
bool isCalibrationUpdated();
bool isBootComplete();
bool isCalibrationReady();


void setStopFlag(bool value);
bool getStopFlag();

bool m4SendChar(const char* data, size_t length);
bool m4ReceiveChar(char* data, size_t& length);
bool m4HasCharMessage();

bool m4SendFloat(const float* data, size_t length);

bool m4SendVoltage(const double* data, size_t length);