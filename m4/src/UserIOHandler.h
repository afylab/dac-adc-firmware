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
    return OperationResult::Success(__FIRMWARE_VERSION_STRING__);
    #endif
    return OperationResult::Success(STRINGIZE_VALUE_OF(__FIRMWARE_VERSION__));
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

  static std::vector<String> query_memory() {
    std::vector<String> comm;
    if (!m4HasCharMessage()) {
      return comm;
    }

    char buffer[CHAR_BUFFER_SIZE];
    size_t size;
    if (!m4ReceiveChar(buffer, size)) {
      return comm;  // Return empty vector if we couldn't get the data
    }

    String input = String(buffer, size);  // Create String with exact size
    input.trim();

    int startPos = 0;
    int commaPos = input.indexOf(',');
    while (commaPos != -1) {
      comm.push_back(input.substring(startPos, commaPos));
      startPos = commaPos + 1;
      commaPos = input.indexOf(',', startPos);
    }
    comm.push_back(input.substring(startPos));

    return comm;
  }

  static bool isValidFloat(const String& str) {
    char* endPtr;
    strtod(str.c_str(), &endPtr);
    return endPtr != str.c_str() && *endPtr == '\0' && str.length() > 0;
  }

  ///*************************************************************************///
  /// handleUserIO is called in the main loop of the M4 processor (see main.cpp)
  /// The M4 processor cannot directly process information from UART, so in order
  /// to communicate with user serial input, the M7 takes the serial input and saves
  /// the command in a shared memory buffer.  Conversely, when the M4 needs to 
  /// communicate with the M7, it stores data in the shared SRAM buffer.
  ///*************************************************************************///

  static void handleUserIO() {
    std::vector<String> comm;
    if (m4HasCharMessage()) {
      comm = query_memory();

      if (comm.size() > 0) {
        String command = comm[0];
        std::vector<float> args;

        for (size_t i = 1; i < comm.size(); ++i) {
          if (!isValidFloat(comm[i])) {
            m4SendChar("Invalid arguments!", 19);
            return;
          }
          args.push_back(comm[i].toFloat());
        }
        OperationResult result =
            OperationResult::Failure("Something went wrong!");
        FunctionRegistry::ExecuteResult executeResult =
            FunctionRegistry::execute(command, args, result);

        switch (executeResult) {
          case FunctionRegistry::ExecuteResult::Success:
            if (result.hasMessage()) {
              size_t messageSize = result.getMessage().length() + 1;
              char* message = new char[messageSize];
              result.getMessage().toCharArray(message,
                                              messageSize);
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
      }
    }
  }
};