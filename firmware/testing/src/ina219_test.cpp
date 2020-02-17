#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "drivers/ina219.h"
#include "mocks/i2cmocks.h"
#include "modules/system/systemhealthchecker.h"
#include <endian.h>
#include <iostream>

#include <CppUTest/MemoryLeakDetectorNewMacros.h>
TEST_GROUP(Drivers_Ina219) {
  void setup(){};
  void teardown() {
    i2c_registers.clear();
    i2c_response.clear();
  };
};

TEST(Drivers_Ina219, SensorSetup) {
  I2C_HandleTypeDef handle;
  unav::drivers::Ina219 ina219(&handle, 123);

  ina219.setup(unav::drivers::Ina219ConfigGain::gain1_40MV, unav::drivers::Ina219ConfigBusAdcResolution::adc12BIT,
               unav::drivers::Ina219ConfigShuntAdcResolution::adc12BIT_128S_69MS, 4092u, 0.002f);

  auto config = be16toh(i2c_registers[0x00]);
  const uint16_t expectedValue = 0b0010000111011111;
  const uint16_t expectedMask = 0b1011111111011111;
  CHECK((config & expectedMask) == expectedValue);
}

TEST(Drivers_Ina219, TestReading) {
  I2C_HandleTypeDef handle;
  unav::drivers::Ina219 ina219(&handle, 123);

  ina219.setup(unav::drivers::Ina219ConfigGain::gain1_40MV, unav::drivers::Ina219ConfigBusAdcResolution::adc12BIT,
               unav::drivers::Ina219ConfigShuntAdcResolution::adc12BIT_128S_69MS, 20480u, 0.002f);

  i2c_response[0x1] = htobe16(0x07D0);
  i2c_response[0x2] = htobe16(0x5D98);
  i2c_response[0x3] = htobe16(0x1766);
  i2c_response[0x4] = htobe16(0x2710);

  unav::drivers::PowerStatus_t status = ina219.getPowerStatus();
  DOUBLES_EQUAL(-10.0, status.current, 0.02);
  DOUBLES_EQUAL(12.0, status.voltage, 0.02);
}
