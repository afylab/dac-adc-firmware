#include "UserIOHandler.h"

#include <cstdlib>
#include <vector>

#include "FunctionRegistry/FunctionRegistry.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Utils/shared_memory.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "UNKNOWN"
#endif

#ifndef STRINGIZE
#define STRINGIZE(x) #x
#define STRINGIZE_VALUE_OF(x) STRINGIZE(x)
#endif

__attribute__((section(".serial_number")))
const char UserIOHandler::serial_number[29] = "__SERIAL_NUMBER__DA_2025_ABC";

namespace {
void resetFragmentAssembly(bool& assembling, uint16_t& next_seq,
                           uint32_t& expected_total, String& assembled) {
  assembling = false;
  next_seq = 0;
  expected_total = 0;
  assembled = "";
}

uint16_t readLe16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readLe32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
}  // namespace

void UserIOHandler::setup() {
  registerMemberFunction(nop, "NOP");
  registerMemberFunction(id, "*IDN?");
  registerMemberFunction(rdy, "*RDY?");
  registerMemberFunction(serialNumber, "SERIAL_NUMBER");
  registerMemberFunction(getEnvironment, "GET_ENVIRONMENT");
  registerMemberFunction(getFirmwareVersion, "GET_FIRMWARE_VERSION");
}

OperationResult UserIOHandler::getFirmwareVersion() {
#ifdef __FIRMWARE_VERSION_STRING__
  return OperationResult::Success(
      STRINGIZE_VALUE_OF(__FIRMWARE_VERSION_STRING__));
#endif
  return OperationResult::Success(String("Commit Hash: ") +
                                  STRINGIZE_VALUE_OF(__FIRMWARE_VERSION__));
}

OperationResult UserIOHandler::nop() {
  return OperationResult::Success("NOP");
}

OperationResult UserIOHandler::getEnvironment() {
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

OperationResult UserIOHandler::id() {
  return OperationResult::Success("DAC-ADC_AD7734-AD5791");
}

OperationResult UserIOHandler::rdy() {
  return OperationResult::Success("READY");
}

OperationResult UserIOHandler::serialNumber() {
  return OperationResult::Success(serial_number + 17);
}

bool UserIOHandler::readCommandLine(String& out) {
  out = "";
  if (!m4HasCharMessage()) {
    return false;
  }

  static bool assembling = false;
  static uint16_t next_seq = 0;
  static uint32_t expected_total = 0;
  static String assembled;

  char buffer[CHAR_BUFFER_SIZE];
  size_t size = sizeof(buffer);
  if (!m4ReceiveChar(buffer, size)) {
    return false;
  }

  const uint8_t* u8 = reinterpret_cast<const uint8_t*>(buffer);
  if (size == 0) {
    return false;
  }

  const uint8_t frame_type = u8[0];
  if (frame_type == CHAR_FRAME_TYPE_NORMAL) {
    resetFragmentAssembly(assembling, next_seq, expected_total, assembled);
    out = String(buffer + 1, size - 1);
    return true;
  }

  if (frame_type != CHAR_FRAME_TYPE_FRAGMENT ||
      size < CHAR_FRAGMENT_HEADER_SIZE) {
    resetFragmentAssembly(assembling, next_seq, expected_total, assembled);
    return false;
  }

  const uint8_t flags = u8[1];
  const uint8_t version = u8[2];
  const uint16_t seq = readLe16(&u8[3]);
  const uint32_t total_len = readLe32(&u8[5]);

  const bool is_first = (flags & 0x01) != 0;
  const bool is_last = (flags & 0x02) != 0;

  if (version != CHAR_FRAGMENT_VERSION || total_len == 0) {
    resetFragmentAssembly(assembling, next_seq, expected_total, assembled);
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
    return false;
  }

  if (seq != next_seq || total_len != expected_total) {
    resetFragmentAssembly(assembling, next_seq, expected_total, assembled);
    return false;
  }

  const size_t payload_len =
      (size > CHAR_FRAGMENT_HEADER_SIZE) ? (size - CHAR_FRAGMENT_HEADER_SIZE)
                                         : 0;
  if (payload_len > 0) {
    const uint32_t already = static_cast<uint32_t>(assembled.length());
    if (already < expected_total) {
      const uint32_t remaining = expected_total - already;
      const uint32_t to_append =
          (payload_len < remaining) ? static_cast<uint32_t>(payload_len)
                                    : remaining;
      assembled.concat(
          reinterpret_cast<const char*>(&u8[CHAR_FRAGMENT_HEADER_SIZE]),
          static_cast<unsigned int>(to_append));
    }
  }

  next_seq++;

  if (!(is_last && assembled.length() >= expected_total)) {
    return false;
  }

  out = assembled;
  resetFragmentAssembly(assembling, next_seq, expected_total, assembled);
  return true;
}

void UserIOHandler::handleUserIO() {
  while (m4HasCharMessage()) {
    String commandLine;
    if (!readCommandLine(commandLine)) {
      return;
    }

    commandLine.trim();
    if (commandLine.length() == 0) {
      return;
    }

    int commaPos = commandLine.indexOf(',');
    String command =
        (commaPos == -1) ? commandLine : commandLine.substring(0, commaPos);
    command.trim();

    std::vector<float> args;
    if (commaPos != -1) {
      const char* p = commandLine.c_str() + commaPos + 1;
      while (*p != '\0') {
        while (*p == ' ' || *p == '\t') {
          p++;
        }
        if (*p == '\0') {
          break;
        }

        char* endPtr = nullptr;
        double v = strtod(p, &endPtr);
        if (endPtr == p) {
          m4SendChar("Invalid arguments!", 19);
          return;
        }
        args.push_back(static_cast<float>(v));

        p = endPtr;
        while (*p == ' ' || *p == '\t') {
          p++;
        }

        if (*p == ',') {
          p++;
          continue;
        }
        if (*p == '\0' || *p == '\r' || *p == '\n') {
          break;
        }

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

    return;
  }
}
