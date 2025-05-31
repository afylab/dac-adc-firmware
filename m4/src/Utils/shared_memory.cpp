#include "shared_memory.h"
#include <SDRAM.h>

// Adjust the SDRAM base if needed for GIGA R1
#define SDRAM_START_ADDRESS 0x38000000

// Global pointer to the shared memory struct in SDRAM
SharedMemory* shared_memory = nullptr;

bool initSharedMemory() {
  // Initialize the SDRAM
  SDRAM.begin(SDRAM_START_ADDRESS);

  // Map our SharedMemory struct onto the start of SDRAM
  shared_memory = reinterpret_cast<SharedMemory*>(SDRAM_START_ADDRESS);

  // Zero out ring buffer indices
  shared_memory->m4_to_m7_char_buffer.read_index = 0;
  shared_memory->m4_to_m7_char_buffer.write_index = 0;
  shared_memory->m7_to_m4_char_buffer.read_index = 0;
  shared_memory->m7_to_m4_char_buffer.write_index = 0;
  shared_memory->m4_to_m7_float_buffer.read_index = 0;
  shared_memory->m4_to_m7_float_buffer.write_index = 0;
  shared_memory->m4_to_m7_voltage_buffer.read_index = 0;
  shared_memory->m4_to_m7_voltage_buffer.write_index = 0;

  // Reset stop_flag
  shared_memory->stop_flag = false;

  shared_memory->isCalibrationUpdated = false;
  shared_memory->isBootComplete = false;
  shared_memory->isCalibrationReady = false;

  return true;
}

void m4SendCalibrationData(const CalibrationData& data) {
  memcpy(&shared_memory->calibrationData, &data, sizeof(CalibrationData));
  shared_memory->isCalibrationUpdated = true;
}

void m4ReceiveCalibrationData(CalibrationData& data) {
  memcpy(&data, &shared_memory->calibrationData, sizeof(CalibrationData));
}

void m7SendCalibrationData(const CalibrationData& data) {
  memcpy(&shared_memory->calibrationData, &data, sizeof(CalibrationData));
  shared_memory->isCalibrationReady = true;
}

void m7ReceiveCalibrationData(CalibrationData& data) {
  memcpy(&data, &shared_memory->calibrationData, sizeof(CalibrationData));
  shared_memory->isCalibrationUpdated = false;
}

bool isCalibrationUpdated() {
  return shared_memory->isCalibrationUpdated;
}

bool isBootComplete() {
  return shared_memory->isBootComplete;
}

bool isCalibrationReady() {
  return shared_memory->isCalibrationReady;
}

// Set/get stop flag
void setStopFlag(bool value) { shared_memory->stop_flag = value; }
bool getStopFlag() { return shared_memory->stop_flag; }

// ---------------------------------------------------------------------------
//  Utilities to safely write/read a 32-bit length into the char buffer
//  (avoids unaligned 32-bit access and handles wraparound cleanly).
// ---------------------------------------------------------------------------
static void writeUint32ToCharBuffer(CharCircularBuffer* buffer,
                                    uint32_t& index,
                                    uint32_t value) {
  for (int i = 0; i < 4; i++) {
    buffer->buffer[index] = static_cast<char>((value >> (8 * i)) & 0xFF);
    index = (index + 1) % CHAR_BUFFER_SIZE; // advance by 1 byte
  }
}

static uint32_t readUint32FromCharBuffer(CharCircularBuffer* buffer,
                                         uint32_t& index) {
  uint32_t value = 0;
  for (int i = 0; i < 4; i++) {
    value |= (static_cast<unsigned char>(buffer->buffer[index]) << (8 * i));
    index = (index + 1) % CHAR_BUFFER_SIZE; // advance by 1 byte
  }
  return value;
}

// ---------------------------------------------------------------------------
//  Char buffer operations
// ---------------------------------------------------------------------------
static bool charBufferSend(CharCircularBuffer* buffer, const char* data,
                           size_t length) {
  // Enforce max message size
  if (length > MAX_MESSAGE_SIZE) return false;

  // Compute available space in the ring buffer
  // We need to store 4 bytes for 'length' + the actual payload
  uint32_t available_space =
      (buffer->read_index - buffer->write_index - 1 + CHAR_BUFFER_SIZE) %
      CHAR_BUFFER_SIZE;

  // If not enough space, fail
  if (length + 4 > available_space) {
    return false;
  }

  // Write the 4-byte message length, byte by byte
  uint32_t write_index = buffer->write_index;
  writeUint32ToCharBuffer(buffer, write_index, static_cast<uint32_t>(length));

  // Write the actual message bytes
  for (size_t i = 0; i < length; i++) {
    buffer->buffer[write_index] = data[i];
    write_index = (write_index + 1) % CHAR_BUFFER_SIZE;
  }

  // Update the official write_index
  buffer->write_index = write_index;
  return true;
}

static bool charBufferReceive(CharCircularBuffer* buffer, char* data,
                              size_t& length) {
  // If buffer empty, no messages
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }

  // Read the 4-byte length
  uint32_t read_index = buffer->read_index;
  uint32_t msg_length = readUint32FromCharBuffer(buffer, read_index);

  // Check if caller's buffer is large enough
  if (msg_length > length) {
    // Let the caller know the required size
    length = msg_length;
    return false;
  }

  // Copy the message into 'data'
  for (size_t i = 0; i < msg_length; i++) {
    data[i] = buffer->buffer[read_index];
    read_index = (read_index + 1) % CHAR_BUFFER_SIZE;
  }

  // Update read_index
  buffer->read_index = read_index;

  // Tell caller how many bytes we read
  length = msg_length;
  return true;
}

static bool charBufferHasMessage(CharCircularBuffer* buffer) {
  return (buffer->read_index != buffer->write_index);
}

// ---------------------------------------------------------------------------
//  Float buffer operations (unchanged from original)
//  - The first float is used to store the message length
//  - This may also run into alignment issues if you read/write from the "wrong"
//    boundary, but we preserve your original approach below.
// ---------------------------------------------------------------------------
static bool floatBufferSend(FloatCircularBuffer* buffer, const float* data,
                            size_t length) {
  if (length > MAX_MESSAGE_SIZE) return false;

  // How many float-slots are free?
  uint32_t available_space =
      (buffer->read_index - buffer->write_index - 1 + FLOAT_BUFFER_SIZE) %
      FLOAT_BUFFER_SIZE;

  // We store an extra float for 'length', then the payload floats
  if (length + 1 > available_space) return false;

  uint32_t write_index = buffer->write_index;

  // Store "length" as the first float
  buffer->buffer[write_index] = static_cast<float>(length);
  write_index = (write_index + 1) % FLOAT_BUFFER_SIZE;

  // Store the payload
  for (size_t i = 0; i < length; ++i) {
    buffer->buffer[write_index] = data[i];
    write_index = (write_index + 1) % FLOAT_BUFFER_SIZE;
  }

  buffer->write_index = write_index;
  return true;
}

static bool floatBufferReceive(FloatCircularBuffer* buffer, float* data,
                               size_t& length) {
  // Check if empty
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }

  uint32_t read_index = buffer->read_index;

  // The first float is "message length"
  float msg_length_f = buffer->buffer[read_index];
  uint32_t msg_length = static_cast<uint32_t>(msg_length_f);
  read_index = (read_index + 1) % FLOAT_BUFFER_SIZE;

  if (msg_length > length) {
    length = msg_length;
    return false;
  }

  // Copy the floats into data
  for (size_t i = 0; i < msg_length; i++) {
    data[i] = buffer->buffer[read_index];
    read_index = (read_index + 1) % FLOAT_BUFFER_SIZE;
  }

  buffer->read_index = read_index;
  length = msg_length;
  return true;
}

static bool floatBufferHasMessage(FloatCircularBuffer* buffer) {
  return (buffer->read_index != buffer->write_index);
}

// ---------------------------------------------------------------------------
//  Voltage buffer operations
// ---------------------------------------------------------------------------
static bool voltageBufferSend(VoltageCircularBuffer* buffer,
                              const double* data, size_t length) {
  if (length > MAX_MESSAGE_SIZE) return false;

  // How many double-slots are free?
  uint32_t available_space =
      (buffer->read_index - buffer->write_index - 1 + VOLTAGE_BUFFER_SIZE) %
      VOLTAGE_BUFFER_SIZE;

  // We do not store a "length" here; just push doubles
  if (length > available_space) return false;

  for (size_t i = 0; i < length; ++i) {
    buffer->buffer[buffer->write_index] = data[i];
    buffer->write_index = (buffer->write_index + 1) % VOLTAGE_BUFFER_SIZE;
  }

  return true;
}

static bool voltageBufferReceive(VoltageCircularBuffer* buffer,
                                 double* data, size_t& length) {
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }

  // How many floats are available?
  size_t available =
      (buffer->write_index - buffer->read_index + VOLTAGE_BUFFER_SIZE) %
      VOLTAGE_BUFFER_SIZE;

  // We'll read min(length, available)
  size_t to_read = (length < available) ? length : available;

  for (size_t i = 0; i < to_read; ++i) {
    data[i] = buffer->buffer[buffer->read_index];
    buffer->read_index = (buffer->read_index + 1) % VOLTAGE_BUFFER_SIZE;
  }

  length = to_read;
  return true;
}

static bool voltageBufferHasMessage(VoltageCircularBuffer* buffer) {
  // In this design, "HasMessage" just means not empty
  return buffer->read_index != buffer->write_index;
}

// ---------------------------------------------------------------------------
//  M4 char functions
// ---------------------------------------------------------------------------
bool m4SendChar(const char* data, size_t length) {
  return charBufferSend(&shared_memory->m4_to_m7_char_buffer, data, length);
}
bool m4ReceiveChar(char* data, size_t& length) {
  return charBufferReceive(&shared_memory->m7_to_m4_char_buffer, data, length);
}
bool m4HasCharMessage() {
  return charBufferHasMessage(&shared_memory->m7_to_m4_char_buffer);
}

// M4 float functions
bool m4SendFloat(const float* data, size_t length) {
  return floatBufferSend(&shared_memory->m4_to_m7_float_buffer, data, length);
}
// Stub: not implemented in your original snippet
bool m4ReceiveFloat(float* data, size_t& length) {
  length = 0;
  return false; // or implement properly if needed
}
bool m4HasFloatMessage() {
  return false; // or implement if you have a separate M7->M4 float buffer
}

// M4 voltage functions
bool m4SendVoltage(const double* data, size_t length) {
  return voltageBufferSend(&shared_memory->m4_to_m7_voltage_buffer, data, length);
}
// Stub: not implemented in your original snippet
bool m4ReceiveVoltage(double* data, size_t& length) {
  length = 0;
  return false; // or implement properly if needed
}
bool m4HasVoltageMessage() {
  return false; // or implement if there's an M7->M4 voltage buffer
}

// ---------------------------------------------------------------------------
//  M7 char functions
// ---------------------------------------------------------------------------
bool m7SendChar(const char* data, size_t length) {
  return charBufferSend(&shared_memory->m7_to_m4_char_buffer, data, length);
}
bool m7ReceiveChar(char* data, size_t& length) {
  return charBufferReceive(&shared_memory->m4_to_m7_char_buffer, data, length);
}
bool m7HasCharMessage() {
  return charBufferHasMessage(&shared_memory->m4_to_m7_char_buffer);
}

// ---------------------------------------------------------------------------
//  M7 float functions
// ---------------------------------------------------------------------------
bool m7ReceiveFloat(float* data, size_t& length) {
  return floatBufferReceive(&shared_memory->m4_to_m7_float_buffer, data, length);
}
bool m7HasFloatMessage() {
  return floatBufferHasMessage(&shared_memory->m4_to_m7_float_buffer);
}

// ---------------------------------------------------------------------------
//  M7 voltage functions
// ---------------------------------------------------------------------------
bool m7ReceiveVoltage(double* data, size_t& length) {
  return voltageBufferReceive(&shared_memory->m4_to_m7_voltage_buffer, data, length);
}
bool m7HasVoltageMessage() {
  return voltageBufferHasMessage(&shared_memory->m4_to_m7_voltage_buffer);
}
