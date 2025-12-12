#pragma once

#include <Arduino.h>

#include <vector>

#include "FunctionRegistry/FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "UNKNOWN"
#endif

#ifndef STRINGIZE
#define STRINGIZE(x) #x
#define STRINGIZE_VALUE_OF(x) STRINGIZE(x)
#endif

struct UserIOHandler {
  static void setup() {
    registerMemberFunction(nop, "NOP");
    registerMemberFunction(id, "*IDN?");
    registerMemberFunction(rdy, "*RDY?");
    registerMemberFunction(serialNumber, "SERIAL_NUMBER");
    registerMemberFunction(getEnvironment, "GET_ENVIRONMENT");
    registerMemberFunction(getFirmwareVersion, "GET_FIRMWARE_VERSION");
  }

  static OperationResult getFirmwareVersion() {
    #ifdef __FIRMWARE_VERSION_STRING__
    return OperationResult::Success(STRINGIZE_VALUE_OF(__FIRMWARE_VERSION_STRING__));
    #endif
    return OperationResult::Success(String("Commit Hash: ") + STRINGIZE_VALUE_OF(__FIRMWARE_VERSION__));
  }

  static OperationResult nop() { return OperationResult::Success("NOP"); }

  static OperationResult getEnvironment() {
    String env;
    #if defined(__NEW_DAC_ADC__) && defined(__NEW_SHIELD__)
    env = "NEW_HARDWARE";
    #elif !defined(__NEW_DAC_ADC__) && defined(__NEW_SHIELD__)
    env = "NEW_SHIELD_OLD_DAC_ADC";
    #else
    env = "OLD_HARDWARE";
    #endif
    return OperationResult::Success(env);
  }

  static OperationResult id() {
    return OperationResult::Success("DAC-ADC_AD7734-AD5791");
  }
  static OperationResult rdy() { return OperationResult::Success("READY"); }


  // IMPORTANT: If you are modifying the serial number, it MUST have the format __SERIAL_NUMBER__ and then 11 characters.
  // I HIGHLY recommend using the following format: __SERIAL_NUMBER__ {2 characters representing the instrument} {4 digits for the year} {3 characters for the device ID}
  // The serial number ideally should be set to a default value here and then changed post-compile time using the firmware_uploader.py or patch_serial_number.py scripts.
  __attribute__((section(".serial_number")))
  inline static const char serial_number[29] = "__SERIAL_NUMBER__DA_2025_ABC";

  static OperationResult serialNumber() {
    return OperationResult::Success(serial_number + 17);
  }

  // Read one complete command line from shared memory.
  //
  // Returns true only when a full command is available in `out`.
  // For fragmented messages, this will return false until all fragments arrive.
  static bool readCommandLine(String& out) {
    // NOTE:
    // - The M7 forwards user commands to the M4 via the shared-memory char ring.
    // - That ring only supports ~251 payload bytes per frame.
    // - For large commands (e.g. AWG with many points), the M7 fragments the
    //   command into multiple frames with a small binary header.
    //
    // This function reassembles those fragments and returns the full command
    // line *only when a full command is ready*.

    out = "";
    if (!m4HasCharMessage()) return false;

    // Fragment reassembly state (persists across loop iterations).
    static bool assembling = false;
    static uint16_t next_seq = 0;
    static uint32_t expected_total = 0;
    static String assembled;

    auto read_le16 = [](const uint8_t* p) -> uint16_t {
      return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    };
    auto read_le32 = [](const uint8_t* p) -> uint32_t {
      return static_cast<uint32_t>(p[0]) |
             (static_cast<uint32_t>(p[1]) << 8) |
             (static_cast<uint32_t>(p[2]) << 16) |
             (static_cast<uint32_t>(p[3]) << 24);
    };

    char buffer[CHAR_BUFFER_SIZE];
    size_t size = sizeof(buffer);
    if (!m4ReceiveChar(buffer, size)) {
      return false;
    }

    constexpr size_t kFragHeaderSize = 12;
    const uint8_t* u8 = reinterpret_cast<const uint8_t*>(buffer);
    const bool is_frag = (size >= kFragHeaderSize &&
                          u8[0] == 'S' && u8[1] == 'M' && u8[2] == 'C' && u8[3] == '1');

    if (!is_frag) {
      // Legacy single-frame command.
      assembling = false;
      next_seq = 0;
      expected_total = 0;
      assembled = "";
      out = String(buffer, size);
      return true;
    } else {
      const uint8_t flags = u8[4];
      const uint8_t version = u8[5];
      const uint16_t seq = read_le16(&u8[6]);
      const uint32_t total_len = read_le32(&u8[8]);

      const bool is_first = (flags & 0x01) != 0;
      const bool is_last  = (flags & 0x02) != 0;

      if (version != 1 || total_len == 0) {
        // Bad header; drop assembly state.
        assembling = false;
        next_seq = 0;
        expected_total = 0;
        assembled = "";
        return false;
      }

      if (is_first) {
        assembling = true;
        next_seq = 0;
        expected_total = total_len;
        assembled = "";
        assembled.reserve(expected_total);
      }

      if (!assembling) {
        // We got a mid-stream fragment without a start; drop it.
        return false;
      }

      if (seq != next_seq || total_len != expected_total) {
        // Missing/out-of-order fragment; reset.
        assembling = false;
        next_seq = 0;
        expected_total = 0;
        assembled = "";
        return false;
      }

      const size_t payload_len = (size > kFragHeaderSize) ? (size - kFragHeaderSize) : 0;
      if (payload_len > 0) {
        // Append payload bytes (ASCII command text), but never exceed expected_total.
        const uint32_t already = static_cast<uint32_t>(assembled.length());
        if (already < expected_total) {
          const uint32_t remaining = expected_total - already;
          const uint32_t to_append = (payload_len < remaining) ? static_cast<uint32_t>(payload_len) : remaining;
          assembled.concat(reinterpret_cast<const char*>(&u8[kFragHeaderSize]),
                           static_cast<unsigned int>(to_append));
        }
      }

      next_seq++;

      if (!(is_last && assembled.length() >= expected_total)) {
        // Not complete yet.
        return false;
      }

      out = assembled;

      // Reset assembly state for next command.
      assembling = false;
      next_seq = 0;
      expected_total = 0;
      assembled = "";
      return true;
    }
  }

  ///*************************************************************************///
  /// handleUserIO is called in the main loop of the M4 processor (see main.cpp)
  /// The M4 processor cannot directly process information from UART, so in order
  /// to communicate with user serial input, the M7 takes the serial input and saves
  /// the command in a shared memory buffer.  Conversely, when the M4 needs to 
  /// communicate with the M7, it stores data in the shared memory buffer.
  ///*************************************************************************///

  static void handleUserIO() {
    // Drain as many frames as are available so the M7 doesn't block
    // when sending a large fragmented command. We only execute when a full
    // command has been reassembled by readCommandLine().
    while (m4HasCharMessage()) {
      String commandLine;
      if (!readCommandLine(commandLine)) {
        // Not enough fragments yet (or nothing to do).
        return;
      }

      commandLine.trim();
      if (commandLine.length() == 0) return;

      // Parse:
      //   COMMAND,arg1,arg2,arg3,...
      // without allocating one String per argument (important for large AWGs).
      int commaPos = commandLine.indexOf(',');
      String command = (commaPos == -1) ? commandLine : commandLine.substring(0, commaPos);
      command.trim();

      std::vector<float> args;
      if (commaPos != -1) {
        const char* p = commandLine.c_str() + commaPos + 1;
        while (*p != '\0') {
          // Skip whitespace
          while (*p == ' ' || *p == '\t') p++;
          if (*p == '\0') break;

          char* endPtr = nullptr;
          double v = strtod(p, &endPtr);
          if (endPtr == p) {
            m4SendChar("Invalid arguments!", 19);
            return;
          }
          args.push_back(static_cast<float>(v));

          p = endPtr;
          while (*p == ' ' || *p == '\t') p++;

          if (*p == ',') {
            p++; // next token
            continue;
          }
          if (*p == '\0' || *p == '\r' || *p == '\n') break;

          // Unexpected character between numbers.
          m4SendChar("Invalid arguments!", 19);
          return;
        }
      }

      OperationResult result = OperationResult::Failure("Something went wrong!");
      FunctionRegistry::ExecuteResult executeResult =
          FunctionRegistry::execute(command, args, result);

      switch (executeResult) {
        case FunctionRegistry::ExecuteResult::Success:
          if (result.hasMessage()) {
            size_t messageSize = result.getMessage().length() + 1;
            char* message = new char[messageSize];
            result.getMessage().toCharArray(message, messageSize);
            m4SendChar(message, messageSize);
            delete[] message;
          }
          break;
        case FunctionRegistry::ExecuteResult::ArgumentError:
          m4SendChar("FAILURE: Argument error", 24);
          break;
        case FunctionRegistry::ExecuteResult::FunctionNotFound:
          m4SendChar("FAILURE: Function not found", 28);
          break;
      }

      return; // execute only one full command per loop call
    }
  }
};