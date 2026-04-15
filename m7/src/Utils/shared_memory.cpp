#include "shared_memory.h"
#include <SDRAM.h>
#include <vector>

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

  if (length + 4 > available_space) {
    return false;
  }

  uint32_t write_index = buffer->write_index;
  writeUint32ToCharBuffer(buffer, write_index, static_cast<uint32_t>(length));

  for (size_t i = 0; i < length; i++) {
    buffer->buffer[write_index] = data[i];
    write_index = (write_index + 1) % CHAR_BUFFER_SIZE;
  }

  buffer->write_index = write_index;
  return true;
}

static bool charBufferReceive(CharCircularBuffer* buffer, char* data,
                              size_t& length) {
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }

  uint32_t read_index = buffer->read_index;
  uint32_t msg_length = readUint32FromCharBuffer(buffer, read_index);

  if (msg_length > length) {
    length = msg_length;
    return false;
  }

  for (size_t i = 0; i < msg_length; i++) {
    data[i] = buffer->buffer[read_index];
    read_index = (read_index + 1) % CHAR_BUFFER_SIZE;
  }

  buffer->read_index = read_index;

  length = msg_length;
  return true;
}

static bool charBufferHasMessage(CharCircularBuffer* buffer) {
  return (buffer->read_index != buffer->write_index);
}

static bool floatBufferReceive(FloatCircularBuffer* buffer, float* data,
                               size_t& length) {
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }

  uint32_t read_index = buffer->read_index;

  float msg_length_f = buffer->buffer[read_index];
  uint32_t msg_length = static_cast<uint32_t>(msg_length_f);
  read_index = (read_index + 1) % FLOAT_BUFFER_SIZE;

  if (msg_length > length) {
    length = msg_length;
    return false;
  }

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
static bool voltageBufferReceive(VoltageCircularBuffer* buffer,
                                 double* data, size_t& length) {
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }

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
  return buffer->read_index != buffer->write_index;
}

// ---------------------------------------------------------------------------
//  M7 char functions
// ---------------------------------------------------------------------------
bool m7SendChar(const char* data, size_t length) {
  // The underlying ring buffer is only CHAR_BUFFER_SIZE bytes and stores
  // an additional 4-byte length prefix per frame (see charBufferSend()).
  // That means the *true* maximum encoded frame size is (CHAR_BUFFER_SIZE - 5).
  //
  // Large commands (e.g. AWG with many points) can exceed this, so we fragment
  // them into multiple frames and let the M4 side reassemble.

  constexpr size_t kCharFrameMaxPayload = (CHAR_BUFFER_SIZE > 5) ? (CHAR_BUFFER_SIZE - 5) : 0; // 251 for 256B ring
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
    // Block until there's space; this prevents dropping long commands.
    // If the M4 stops draining the ring buffer, this would block here.
    uint32_t start_ms = millis();
    while (!charBufferSend(&shared_memory->m7_to_m4_char_buffer, frame, frame_len)) {
      // Avoid hard-deadlocking the M7 if the M4 is busy/crashed and not draining.
      if (millis() - start_ms > 5000) {
        return false;
      }
      delay(1);
    }
    return true;
  };

  // Fast path: fits in a single ring-buffer frame.
  if (kCharFrameMaxPayload <= kNormalFrameOverhead) {
    return false;
  }
  if (length <= (kCharFrameMaxPayload - kNormalFrameOverhead)) {
    uint8_t frame[kCharFrameMaxPayload];
    frame[0] = CHAR_FRAME_TYPE_NORMAL;
    memcpy(&frame[1], data, length);
    return send_blocking(reinterpret_cast<const char*>(frame), length + 1);
  }

  // Fragmented path: fixed header + chunked payload.
  if (kCharFrameMaxPayload <= CHAR_FRAGMENT_HEADER_SIZE) {
    // Should never happen with current sizes.
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
    // Flags
    frame[1] = static_cast<uint8_t>((is_first ? 0x01 : 0x00) | (is_last ? 0x02 : 0x00));
    // Version
    frame[2] = CHAR_FRAGMENT_VERSION;
    // Sequence
    write_le16(&frame[3], seq);
    // Total length of the full (reassembled) message
    write_le32(&frame[5], static_cast<uint32_t>(length));
    // Payload bytes
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
bool m7ReceiveChar(char* data, size_t& length) {
  struct CharReassemblyState {
    bool assembling = false;
    uint16_t next_seq = 0;
    uint32_t expected_total = 0;
    std::vector<char> assembled;
  };
  static CharReassemblyState state;

  const size_t output_capacity = length;
  char frame[CHAR_BUFFER_SIZE];
  size_t frame_length = sizeof(frame);
  if (!charBufferReceive(&shared_memory->m4_to_m7_char_buffer, frame,
                         frame_length)) {
    return false;
  }

  auto read_le16 = [](const uint8_t* p) -> uint16_t {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
  };
  auto read_le32 = [](const uint8_t* p) -> uint32_t {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
  };

  const uint8_t* u8 = reinterpret_cast<const uint8_t*>(frame);
  if (frame_length == 0) {
    length = 0;
    return false;
  }

  const uint8_t frame_type = u8[0];
  if (frame_type == CHAR_FRAME_TYPE_NORMAL) {
    const size_t payload_length = frame_length - 1;
    if (payload_length > output_capacity) {
      length = 0;
      return false;
    }

    state = CharReassemblyState{};
    memcpy(data, frame + 1, payload_length);
    length = payload_length;
    return true;
  }

  if (frame_type != CHAR_FRAME_TYPE_FRAGMENT ||
      frame_length < CHAR_FRAGMENT_HEADER_SIZE) {
    state = CharReassemblyState{};
    length = 0;
    return false;
  }

  const uint8_t flags = u8[1];
  const uint8_t version = u8[2];
  const uint16_t seq = read_le16(&u8[3]);
  const uint32_t total_len = read_le32(&u8[5]);
  const bool is_first = (flags & 0x01) != 0;
  const bool is_last = (flags & 0x02) != 0;

  if (version != CHAR_FRAGMENT_VERSION || total_len == 0) {
    state = CharReassemblyState{};
    length = 0;
    return false;
  }

  if (is_first) {
    state.assembling = true;
    state.next_seq = 0;
    state.expected_total = total_len;
    state.assembled.clear();
    state.assembled.reserve(total_len);
  }

  if (!state.assembling || seq != state.next_seq ||
      total_len != state.expected_total) {
    state = CharReassemblyState{};
    length = 0;
    return false;
  }

  const size_t payload_len =
      (frame_length > CHAR_FRAGMENT_HEADER_SIZE)
          ? (frame_length - CHAR_FRAGMENT_HEADER_SIZE)
          : 0;
  const size_t already = state.assembled.size();
  if (payload_len > 0 && already < state.expected_total) {
    const size_t remaining = state.expected_total - already;
    const size_t to_append =
        (payload_len < remaining) ? payload_len : remaining;
    state.assembled.insert(state.assembled.end(),
                           frame + CHAR_FRAGMENT_HEADER_SIZE,
                           frame + CHAR_FRAGMENT_HEADER_SIZE + to_append);
  }

  state.next_seq++;

  if (!(is_last && state.assembled.size() >= state.expected_total)) {
    length = 0;
    return false;
  }

  if (state.expected_total > output_capacity) {
    state = CharReassemblyState{};
    length = 0;
    return false;
  }

  memcpy(data, state.assembled.data(), state.expected_total);
  length = state.expected_total;
  state = CharReassemblyState{};
  return true;
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
