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
#include <modules.h>
namespace unav::modules {

SystemModule::SystemModule() : thermometer(&I2C_PORT, TEMP_SENSOR_ADDRESS), power(&I2C_PORT, POWER_MONITOR_ADDRESS), timer() {
}

void SystemModule::initialize() {
  initializeTask(osPriority::osPriorityAboveNormal, SystemModule::ModuleName);
  setup();
}

void SystemModule::moduleThreadStart() {
  auto c = xTaskGetTickCount();
  auto wait = 100;
  auto value = -123;
  power.setup(unav::drivers::Ina219ConfigGain::gain1_40MV, unav::drivers::Ina219ConfigBusAdcResolution::adc12BIT,
              unav::drivers::Ina219ConfigShuntAdcResolution::adc12BIT_128S_69MS, 20480u, 0.002f);
  while (true) {
    auto msgsys = unav::Application::messaging.prepareMessage();
    systemstatus_content_t *systemstatus = &msgsys->systemstatus;
    msgsys->type = message_types_t::outbound_SystemStatus;

    systemstatus->temp = thermometer.getTemperature();
    unav::drivers::PowerStatus_t p = power.getPowerStatus();

    systemstatus->battery_current = p.current;
    systemstatus->battery_voltage = p.voltage;
    systemstatus->manager_status = static_cast<uint8_t>(unav::Modules::motorManagerModule->getStatus());
    systemstatus->controller_status = static_cast<uint8_t>(unav::Modules::motorControllerModule->getStatus());
    systemstatus->health_status = 0;
    systemstatus->system_status = static_cast<uint8_t>(unav::Modules::motorManagerModule->getStatus());

    unav::Application::messaging.sendMessage(msgsys);

    vTaskDelayUntil(&c, wait);
  }
}

void SystemModule::setup() {
}

template class BaseModule<SYSTEMSTACKSIZE>;

} // namespace unav::modules
