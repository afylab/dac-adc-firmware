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
  CharCircularBuffer gateway_to_worker_char_buffer;
  CharCircularBuffer worker_to_gateway_char_buffer;

  FloatCircularBuffer worker_to_gateway_float_buffer;

  VoltageCircularBuffer worker_to_gateway_voltage_buffer;

  volatile bool stop_requested;

  volatile bool calibration_updated;
  volatile bool worker_dma_ready;
  volatile bool calibration_ready;

  CalibrationData calibration_data;
};

constexpr uintptr_t SHARED_MEMORY_ADDRESS = 0x10040000UL;
constexpr size_t SHARED_MEMORY_REGION_SIZE = 32 * 1024;
static_assert(sizeof(SharedMemory) <= SHARED_MEMORY_REGION_SIZE,
              "SharedMemory must fit in the reserved shared SRAM window");

extern SharedMemory* shared_memory;

bool initSharedMemory();

void requestWorkerStop();

bool sendCommandToWorker(const char* data, size_t length);
bool receiveTextFromWorker(char* data, size_t& length);
bool hasTextFromWorker();

bool receiveFloatResponseFromWorker(float* data, size_t& length);
bool hasFloatResponseFromWorker();

bool receiveVoltageFrameFromWorker(double* data, size_t& length);
bool hasVoltageFrameFromWorker();
