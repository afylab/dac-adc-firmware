#pragma once

#include <cassert>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "FunctionRegistry/FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

namespace FunctionRegistryDetail {

template <typename T>
struct FunctionTraits;

template <typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType (*)(Args...)> {
  using return_type = ReturnType;
  using args_tuple = std::tuple<Args...>;
  static constexpr size_t arity = sizeof...(Args);
};

template <typename ClassType, typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType (ClassType::*)(Args...)> {
  using return_type = ReturnType;
  using args_tuple = std::tuple<Args...>;
  static constexpr size_t arity = sizeof...(Args);
};

template <typename ClassType, typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType (ClassType::*)(Args...) const> {
  using return_type = ReturnType;
  using args_tuple = std::tuple<Args...>;
  static constexpr size_t arity = sizeof...(Args);
};

template <typename Functor>
struct FunctionTraits : public FunctionTraits<decltype(&Functor::operator())> {
};

template <typename Function, typename Tuple, size_t... Is>
auto callFunctionHelper(Function function, const std::vector<float>& args,
                        std::index_sequence<Is...>)
    -> decltype(function(
        static_cast<typename std::tuple_element<Is, Tuple>::type>(args[Is])...)) {
  return function(
      static_cast<typename std::tuple_element<Is, Tuple>::type>(args[Is])...);
}

}  // namespace FunctionRegistryDetail

template <typename Function>
void registerMemberFunction(Function function, const String& commandName) {
  using Traits = FunctionRegistryDetail::FunctionTraits<Function>;
  constexpr int argCount = static_cast<int>(Traits::arity);

  using ArgsTuple = typename Traits::args_tuple;

  auto wrapper = [function](const std::vector<float>& args) -> OperationResult {
    assert(args.size() == Traits::arity &&
           "Incorrect number of arguments provided.");

    return FunctionRegistryDetail::callFunctionHelper<Function, ArgsTuple>(
        function, args, std::make_index_sequence<Traits::arity>{});
  };

  FunctionRegistry::registerFunction(commandName, wrapper, argCount);
}

template <typename Function>
void registerMemberFunctionVector(Function function,
                                  const String& commandName) {
  FunctionRegistry::registerFunction(commandName, function, -1);
}
