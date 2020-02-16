#define INSTRUMENT_MODULE
#include "FreeRTOS.h"
#include "timing.h"
#include <counters.h>
#include <instrumentation/instrumentation_helper.h>
#include <leds.h>
#include <main.h>
#include <mathutils.h>
#include <modules/systemmodule.h>
#include <stm32f4xx.h>
namespace unav::modules {

SystemModule::SystemModule()
    : timer(), thermometer(&I2C_PORT, TEMP_SENSOR_ADDRESS) {}

void SystemModule::initialize() {
  subscribe(SystemModule::ModuleMessageId);
  BaseModule::initializeTask(osPriority::osPriorityAboveNormal, 1024);
  setup();
}

void SystemModule::moduleThreadStart() {
  auto c = xTaskGetTickCount();
  auto wait = 100;
  auto value = -123;

  while (true) {
    float v = thermometer.getTemperature();

    value = int32_t(v * 100);

    instrumentation_setCounter(perf_sys_temp, value);
    if (value > 0) {
      leds_setPattern(2, &leds_pattern_on);
    } else {
      leds_setPattern(2, &leds_pattern_on);
    }
    vTaskDelayUntil(&c, wait);
  }
}

void SystemModule::setup() {}

void SystemModule::checkMessages() {
  message_t *receivedMsg = nullptr;
  uint32_t transactionId = 0;
  if (waitMessage(&receivedMsg, 0)) {
    switch (receivedMsg->type) {
    case message_types_t::internal_motor_control: {
    } break;
    default:
      break;
    }
    if (transactionId) {
      sendAck(receivedMsg, transactionId);
    } else {
      releaseMessage(receivedMsg);
    }
  }
}

} // namespace unav::modules
