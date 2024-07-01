#pragma once
#include "FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

#define REGISTER_MEMBER_FUNCTION_VECTOR(registry, className, functionName, \
                                        commandName)                       \
  registry.registerFunction(                                               \
      commandName,                                                         \
      [this](const std::vector<float>& args) -> OperationResult {          \
        return this->functionName(args);                                   \
      },                                                                   \
      -1)

#define REGISTER_MEMBER_FUNCTION_0(registry, className, functionName, \
                                   commandName)                       \
  registry.registerFunction(                                          \
      commandName,                                                    \
      [this](const std::vector<float>&) -> OperationResult {          \
        return this->functionName();                                  \
      },                                                              \
      0)

#define REGISTER_MEMBER_FUNCTION_1(registry, className, functionName, \
                                   commandName)                       \
  registry.registerFunction(                                          \
      commandName,                                                    \
      [this](const std::vector<float>& args) -> OperationResult {     \
        return this->functionName(static_cast<float>(args[0]));       \
      },                                                              \
      1)

#define REGISTER_MEMBER_FUNCTION_2(registry, className, functionName, \
                                   commandName)                       \
  registry.registerFunction(                                          \
      commandName,                                                    \
      [this](const std::vector<float>& args) -> OperationResult {     \
        return this->functionName(static_cast<float>(args[0]),        \
                                  static_cast<float>(args[1]));       \
      },                                                              \
      2)

#define REGISTER_MEMBER_FUNCTION_3(registry, className, functionName, \
                                   commandName)                       \
  registry.registerFunction(                                          \
      commandName,                                                    \
      [this](const std::vector<float>& args) -> OperationResult {     \
        return this->functionName(static_cast<float>(args[0]),        \
                                  static_cast<float>(args[1]),        \
                                  static_cast<float>(args[2]));       \
      },                                                              \
      3)

#define REGISTER_MEMBER_FUNCTION_4(registry, className, functionName,  \
                                   commandName)                        \
  registry.registerFunction(                                           \
      commandName,                                                     \
      [this](const std::vector<float>& args) -> OperationResult {      \
        return this->functionName(                                     \
            static_cast<float>(args[0]), static_cast<float>(args[1]),  \
            static_cast<float>(args[2]), static_cast<float>(args[3])); \
      },                                                               \
      4)