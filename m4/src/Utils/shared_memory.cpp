#include "shared_memory.h"
#include <SDRAM.h>

#define SDRAM_START_ADDRESS 0x38000000

SharedMemory* shared_memory = nullptr;

bool initSharedMemory() {
  SDRAM.begin(SDRAM_START_ADDRESS);

  shared_memory = reinterpret_cast<SharedMemory*>(SDRAM_START_ADDRESS);

  shared_memory->m4_to_m7_char_buffer.read_index = 0;
  shared_memory->m4_to_m7_char_buffer.write_index = 0;
  shared_memory->m7_to_m4_char_buffer.read_index = 0;
  shared_memory->m7_to_m4_char_buffer.write_index = 0;
  shared_memory->m4_to_m7_float_buffer.read_index = 0;
  shared_memory->m4_to_m7_float_buffer.write_index = 0;
  shared_memory->m4_to_m7_voltage_buffer.read_index = 0;
  shared_memory->m4_to_m7_voltage_buffer.write_index = 0;

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
  // NOTE: With a 256-byte ring, and a 4-byte length header, and leaving one slot empty, the true maximum payload per message is 251.
  constexpr size_t kMaxCharPayload = (CHAR_BUFFER_SIZE > 5) ? (CHAR_BUFFER_SIZE - 5) : 0;
  if (length > kMaxCharPayload) return false;

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

static bool floatBufferSend(FloatCircularBuffer* buffer, const float* data,
                            size_t length) {
  // We store one extra float for 'length' and keep one slot empty to
  // distinguish full vs empty.
  constexpr size_t kMaxFloatPayload = (FLOAT_BUFFER_SIZE > 2) ? (FLOAT_BUFFER_SIZE - 2) : 0;
  if (length > kMaxFloatPayload) return false;

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

// ---------------------------------------------------------------------------
//  M4 char functions
// ---------------------------------------------------------------------------
bool m4SendChar(const char* data, size_t length) {
  constexpr size_t kCharFrameMaxPayload =
      (CHAR_BUFFER_SIZE > 5) ? (CHAR_BUFFER_SIZE - 5) : 0;
  constexpr size_t kNormalFrameOverhead = 1;

  auto write_le16 = [](uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  };
  auto write_le32 = [](uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
  };

  auto send_blocking = [&](const char* frame, size_t frame_len) -> bool {
    uint32_t start_ms = millis();
    while (!charBufferSend(&shared_memory->m4_to_m7_char_buffer, frame,
                           frame_len)) {
      if (millis() - start_ms > 5000) {
        return false;
      }
      delay(1);
    }
    return true;
  };

  if (kCharFrameMaxPayload <= kNormalFrameOverhead) {
    return false;
  }

  if (length <= (kCharFrameMaxPayload - kNormalFrameOverhead)) {
    uint8_t frame[kCharFrameMaxPayload];
    frame[0] = CHAR_FRAME_TYPE_NORMAL;
    memcpy(&frame[1], data, length);
    return send_blocking(reinterpret_cast<const char*>(frame), length + 1);
  }

  if (kCharFrameMaxPayload <= CHAR_FRAGMENT_HEADER_SIZE) {
    return false;
  }

  const size_t max_chunk = kCharFrameMaxPayload - CHAR_FRAGMENT_HEADER_SIZE;
  uint16_t seq = 0;
  size_t offset = 0;

  while (offset < length) {
    const size_t remaining = length - offset;
    const size_t chunk = (remaining < max_chunk) ? remaining : max_chunk;
    const bool is_first = (offset == 0);
    const bool is_last = (offset + chunk == length);

    uint8_t frame[kCharFrameMaxPayload];
    frame[0] = CHAR_FRAME_TYPE_FRAGMENT;
    frame[1] = static_cast<uint8_t>((is_first ? 0x01 : 0x00) |
                                    (is_last ? 0x02 : 0x00));
    frame[2] = CHAR_FRAGMENT_VERSION;
    write_le16(&frame[3], seq);
    write_le32(&frame[5], static_cast<uint32_t>(length));
    memcpy(&frame[CHAR_FRAGMENT_HEADER_SIZE], data + offset, chunk);

    if (!send_blocking(reinterpret_cast<const char*>(frame),
                       CHAR_FRAGMENT_HEADER_SIZE + chunk)) {
      return false;
    }

    offset += chunk;
    seq++;
  }

  return true;
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

// M4 voltage functions
bool m4SendVoltage(const double* data, size_t length) {
  return voltageBufferSend(&shared_memory->m4_to_m7_voltage_buffer, data, length);
}
