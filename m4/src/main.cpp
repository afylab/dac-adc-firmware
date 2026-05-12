#include <Arduino.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "mbed/drivers/usb/include/usb/USBCDC.h"

#include "Utils/shared_memory.h"

typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

namespace {

constexpr uint8_t kUsbStringDescriptor = 0x03;
volatile bool bootloader_touch_requested = false;

static const uint8_t kManufacturerDescriptor[] = {
    24, kUsbStringDescriptor, 's', 0, 'p', 0, '2', 0, ' ', 0, 'q', 0,
    'u', 0, 'a', 0, 'n', 0, 't', 0, 'u', 0, 'm', 0};

static const uint8_t kProductDescriptor[] = {
    30, kUsbStringDescriptor, 'G', 0, 'a', 0, 't', 0, 'e', 0, 'K', 0,
    'e', 0, 'e', 0, 'p', 0, 'e', 0, 'r', 0, ' ', 0, '1', 0, '.', 0,
    '0', 0};

constexpr size_t kSerialMarkerLength = 17;
constexpr size_t kDacSerialFieldLength = 12;

__attribute__((section(".serial_number"), used))
static const char kDacSerialNumber[kSerialMarkerLength + kDacSerialFieldLength] =
    "__SERIAL_NUMBER__DA_2025_ABC";

static const uint8_t* dacSerialDescriptor() {
  static uint8_t descriptor[2 + kDacSerialFieldLength * 2];
  static bool initialized = false;

  if (!initialized) {
    const char* serial = kDacSerialNumber + kSerialMarkerLength;
    size_t serial_length = 0;
    while (serial_length < kDacSerialFieldLength &&
           serial[serial_length] != '\0') {
      serial_length++;
    }

    descriptor[0] = 2 + serial_length * 2;
    descriptor[1] = kUsbStringDescriptor;

    for (size_t i = 0; i < serial_length; ++i) {
      descriptor[2 + i * 2] = static_cast<uint8_t>(serial[i]);
      descriptor[3 + i * 2] = 0;
    }

    initialized = true;
  }

  return descriptor;
}

static void resetToBootloaderDfu() {
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  RTC_HandleTypeDef rtc_handle = {};
  rtc_handle.Instance = RTC;
  HAL_RTCEx_BKUPWrite(&rtc_handle, RTC_BKP_DR0, 0xDF59);
  NVIC_SystemReset();
}

class GateKeeperUSBCDC : public USBCDC {
 public:
  using USBCDC::USBCDC;

 protected:
  const uint8_t* string_imanufacturer_desc() override {
    return kManufacturerDescriptor;
  }

  const uint8_t* string_iproduct_desc() override {
    return kProductDescriptor;
  }

  const uint8_t* string_iserial_desc() override {
    return dacSerialDescriptor();
  }

  void line_coding_changed(int baud, int bits, int parity, int stop) override {
    (void)bits;
    (void)parity;
    (void)stop;
    if (baud == 1200) {
      bootloader_touch_requested = true;
    }
  }
};

}  // namespace

static GateKeeperUSBCDC usb_cdc(false, 0x2341, 0x0266, 0x0101);

static void usbWrite(const uint8_t* data, uint32_t size) {
  if (!usb_cdc.ready()) {
    return;
  }
  usb_cdc.send(const_cast<uint8_t*>(data), size);
}

static void usbPrint(const char* data) {
  usbWrite(reinterpret_cast<const uint8_t*>(data), strlen(data));
}

static void handleHostCommand(const char* command) {
  String command_lower = command;
  command_lower.trim();
  command_lower.toLowerCase();
  if (command_lower == "stop") {
    requestWorkerStop();
  } else {
    sendCommandToWorker(command, strlen(command));
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  initSharedMemory();

  usb_cdc.init();
  usb_cdc.connect();
}

void loop() {
  if (bootloader_touch_requested) {
    usb_cdc.disconnect();
    delay(250);
    resetToBootloaderDfu();
  }

  static char command_buffer[4096];
  static size_t command_length = 0;

  uint8_t input[64];
  uint32_t actual = 0;
  usb_cdc.receive_nb(input, sizeof(input), &actual);
  for (uint32_t i = 0; i < actual; ++i) {
    const char c = static_cast<char>(input[i]);
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      command_buffer[command_length] = '\0';
      handleHostCommand(command_buffer);
      command_length = 0;
      continue;
    }
    if (command_length < sizeof(command_buffer) - 1) {
      command_buffer[command_length++] = c;
    }
  }

  if (hasTextFromWorker()) {
    char response[4096];
    size_t size = sizeof(response);
    if (receiveTextFromWorker(response, size)) {
      if (size > 0) {
        size--;
      }
      usbWrite(reinterpret_cast<const uint8_t*>(response), size);
      usbPrint("\r\n");
    }
  }

  if (hasFloatResponseFromWorker()) {
    float response[FLOAT_BUFFER_SIZE];
    size_t size = FLOAT_BUFFER_SIZE;
    if (receiveFloatResponseFromWorker(response, size)) {
      for (size_t i = 0; i < size; ++i) {
        char value[24];
        snprintf(value, sizeof(value), "%.8f ", response[i]);
        usbPrint(value);
      }
      usbPrint("\r\n");
    }
  }

  if (hasVoltageFrameFromWorker()) {
    double response[VOLTAGE_BUFFER_SIZE];
    size_t size = VOLTAGE_BUFFER_SIZE;
    if (receiveVoltageFrameFromWorker(response, size)) {
      for (size_t i = 0; i < size; ++i) {
        binaryFloat send;
        send.floatingPoint = static_cast<float>(response[i]);
        usbWrite(send.binary, 4);
      }
    }
  }
}
