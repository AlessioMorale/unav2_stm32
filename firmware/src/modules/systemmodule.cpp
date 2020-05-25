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

SystemModule::SystemModule() : thermometer(&I2C_PORT, TEMP_SENSOR_ADDRESS), power(&I2C_PORT, POWER_MONITOR_ADDRESS), timer() {
}

void SystemModule::initialize() {
  subscribe(SystemModule::ModuleMessageId, SystemModule::ModuleName);
  BaseModule::initializeTask(osPriority::osPriorityAboveNormal, 1024);
  setup();
}

void SystemModule::moduleThreadStart() {
  auto c = xTaskGetTickCount();
  auto wait = 100;
  auto value = -123;
  power.setup(unav::drivers::Ina219ConfigGain::gain1_40MV, unav::drivers::Ina219ConfigBusAdcResolution::adc12BIT,
              unav::drivers::Ina219ConfigShuntAdcResolution::adc12BIT_128S_69MS, 20480u, 0.002f);
  while (true) {
    float v = thermometer.getTemperature();

    value = int32_t(v * 100);

    instrumentation_setCounter(perf_sys_temp, value);

    unav::drivers::PowerStatus_t p = power.getPowerStatus();

    value = int32_t(p.current * 100);
    instrumentation_setCounter(perf_sys_current, value);

    value = int32_t(p.voltage * 100);
    instrumentation_setCounter(perf_sys_voltage, value);

    vTaskDelayUntil(&c, wait);
  }
}

void SystemModule::setup() {
}

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
