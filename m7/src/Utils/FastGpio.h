#pragma once

#include <Arduino.h>
#include "stm32h7xx.h"

namespace FastGpio {

struct Pin {
  GPIO_TypeDef* port;
  uint32_t mask;
};

inline void high(Pin pin) {
  pin.port->BSRR = pin.mask;
}

inline void low(Pin pin) {
  pin.port->BSRR = pin.mask << 16;
}

inline void write(Pin pin, bool value) {
  if (value) {
    high(pin);
  } else {
    low(pin);
  }
}

inline void pulseLowHigh(Pin pin) {
  low(pin);
  for (int i = 0; i < 16; ++i) {
    __NOP();
  }
  high(pin);
}

inline Pin fromArduinoPin(int pin) {
  switch (pin) {
    case 22:
      return {GPIOJ, 1u << 12};
    case 23:
      return {GPIOG, 1u << 13};
    case 24:
      return {GPIOG, 1u << 12};
    case 25:
      return {GPIOJ, 1u << 0};
    case 26:
      return {GPIOJ, 1u << 14};
    case 27:
      return {GPIOJ, 1u << 1};
    case 28:
      return {GPIOJ, 1u << 15};
    case 29:
      return {GPIOJ, 1u << 2};
    case 30:
      return {GPIOK, 1u << 3};
    case 36:
      return {GPIOK, 1u << 6};
    case 38:
      return {GPIOJ, 1u << 7};
    case 39:
      return {GPIOI, 1u << 14};
    case 40:
      return {GPIOE, 1u << 6};
    case 42:
      return {GPIOI, 1u << 15};
    case 43:
      return {GPIOI, 1u << 10};
    case 44:
      return {GPIOG, 1u << 10};
    case 46:
      return {GPIOH, 1u << 15};
    case 48:
      return {GPIOK, 1u << 0};
    case 51:
      return {GPIOE, 1u << 5};
    case 52:
      return {GPIOK, 1u << 2};
    default:
      return {nullptr, 0};
  }
}

inline void digitalWrite(int pin, bool value) {
  Pin fastPin = fromArduinoPin(pin);
  if (fastPin.port != nullptr) {
    write(fastPin, value);
    return;
  }
  ::digitalWrite(pin, value ? HIGH : LOW);
}

inline void pulseLowHigh(int pin) {
  Pin fastPin = fromArduinoPin(pin);
  if (fastPin.port != nullptr) {
    pulseLowHigh(fastPin);
    return;
  }
  ::digitalWrite(pin, LOW);
  for (int i = 0; i < 16; ++i) {
    __NOP();
  }
  ::digitalWrite(pin, HIGH);
}

}
