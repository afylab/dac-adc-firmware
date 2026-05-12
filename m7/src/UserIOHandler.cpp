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
const char UserIOHandler::serial_number[29] = "__SERIAL_NUMBER__DA_2026_ABC";

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
  if (!hasCommandFromGateway()) {
    return false;
  }

  char buffer[4096];
  size_t size = sizeof(buffer);
  if (!receiveCommandFromGateway(buffer, size)) {
    return false;
  }
  out = String(buffer, size);
  return true;
}

void UserIOHandler::handleUserIO() {
  while (hasCommandFromGateway()) {
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
          sendTextToGateway("Invalid arguments!", sizeof("Invalid arguments!"));
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

        sendTextToGateway("Invalid arguments!", sizeof("Invalid arguments!"));
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
          sendTextToGateway(message, messageSize);
          delete[] message;
        }
        break;
      case FunctionRegistry::ExecuteResult::ArgumentError:
        sendTextToGateway("FAILURE: Argument error",
                          sizeof("FAILURE: Argument error"));
        break;
      case FunctionRegistry::ExecuteResult::FunctionNotFound:
        sendTextToGateway("FAILURE: Function not found",
                          sizeof("FAILURE: Function not found"));
        break;
    }

    return;
  }
}
