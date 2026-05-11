#include "FunctionRegistry/FunctionRegistry.h"

std::vector<FunctionRegistry::FunctionEntry> FunctionRegistry::functions;

FunctionRegistry::FunctionEntry::FunctionEntry(
    const String& n, std::function<OperationResult(const std::vector<float>&)> f,
    int ac)
    : name(n), func(f), argCount(ac) {}

FunctionRegistry::ExecuteResult FunctionRegistry::execute(
    const String& name, const std::vector<float>& args, OperationResult& result) {
  String upper_name = name;
  upper_name.toUpperCase();
  if (upper_name == "PRINT_FUNCTIONS") {
    String message = "Available functions, args: \nPRINT_FUNCTIONS, 0\n";
    for (const auto& entry : functions) {
      message += String(entry.name + ", " + entry.argCount + "\n");
    }
    message = message.substring(0, message.length() - 1);
    result = OperationResult::Success(message);
    return ExecuteResult::Success;
  }
  for (const auto& entry : functions) {
    if (entry.name == upper_name) {
      if (entry.argCount >= 0 &&
          static_cast<int>(entry.argCount) != static_cast<int>(args.size())) {
        result = OperationResult::Failure("Argument count mismatch");
        return ExecuteResult::ArgumentError;
      }
      result = entry.func(args);
      return ExecuteResult::Success;
    }
  }
  result = OperationResult::Failure("Function not found");
  return ExecuteResult::FunctionNotFound;
}
