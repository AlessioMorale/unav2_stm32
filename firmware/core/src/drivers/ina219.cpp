#include "drivers/ina219.h"
#include "FreeRTOS.h"
#include <stm32f4xx.h>
#include <utils/endianess.h>
namespace unav::drivers {

Ina219::Ina219(I2C_HandleTypeDef *port, uint8_t deviceAddress) : i2cport{port}, address{deviceAddress} {
  static_assert(sizeof(Ina219ConfigReg_t) == 2);
}

void Ina219::setup(Ina219ConfigGain gain, Ina219ConfigBusAdcResolution busAdc, Ina219ConfigShuntAdcResolution shuntAdc, uint16_t calibration, float shunt) {

  calibrationConfig = calibration;
  shuntConfig = shunt;
  currentLsb = currentLsbFactor / ((float)calibration * shunt);
  powerLsb = powerCalibrationFactor * currentLsb;

  Ina219ConfigReg_t configRegister;
  configRegister.flags.reset = 1;

  writeUint16(Ina219Registers::configuration, (uint16_t)configRegister.registerValue);
  vTaskDelay(10);

  writeUint16(Ina219Registers::calibration, calibration);
  vTaskDelay(2);

  configRegister.flags.reset = 0;
  configRegister.flags.gain = (uint16_t)gain & 0x3;
  configRegister.flags.badc = (uint16_t)busAdc & 0xF;
  configRegister.flags.sadc = (uint16_t)shuntAdc & 0xF;
  configRegister.flags.bRange = 1;
  configRegister.flags.mode = (uint16_t)Ina219ConfigMode::sAndBvoltContinuous;

  uint16_t val = configRegister.registerValue;
  writeUint16(Ina219Registers::configuration, val);

  vTaskDelay(2);
}

PowerStatus_t Ina219::getPowerStatus() {
  PowerStatus_t powerStatus;
  uint16_t tmp;

  tmp = (uint16_t)(readUint16(Ina219Registers::busVoltage) >> 3);
  powerStatus.voltage = voltageLsb * tmp;

  tmp = readUint16(Ina219Registers::current);
  powerStatus.current = -1.0f * currentLsb * static_cast<int16_t>(tmp);
  return powerStatus;
}

void Ina219::writeUint16(Ina219Registers destination, uint16_t value) {
  uint16_t tmp = hton16(value);
  HAL_I2C_Mem_Write(i2cport, (uint16_t)address, static_cast<uint16_t>(destination), I2C_MEMADD_SIZE_8BIT, (uint8_t *)&tmp, 2, 10);
}

uint16_t Ina219::readUint16(Ina219Registers destination) {
  uint16_t ret{0};
  HAL_I2C_Mem_Read(i2cport, (uint16_t)address, static_cast<uint16_t>(destination), I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ret, 2, 10);
  return ntoh16(ret);
}

} // namespace unav::drivers
