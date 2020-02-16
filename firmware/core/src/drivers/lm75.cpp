#include <drivers/lm75.h>
#include "i2c.h"

namespace unav::drivers {

typedef union {
  uint8_t Bytes[2];
  int16_t Word;
} twoBytesConverter_t;

Lm75::Lm75(I2C_HandleTypeDef *port, uint8_t address)
    : i2cport{port}, address{address} {}

void Lm75::setup() {
  //
}
int32_t Lm75::getTemperature() {
  twoBytesConverter_t Result;
  uint32_t temperature;
  uint8_t data[2];
  auto status =
      HAL_I2C_Mem_Read(i2cport, (uint16_t)address, (uint16_t)LM75_TEMP,
                       I2C_MEMADD_SIZE_8BIT, data, 2, 10);
  Result.Bytes[1] = data[0];
  Result.Bytes[0] = data[1];
  Result.Word /= 128;

  temperature = (Result.Word * 10u) / LM75_TEMP_CORR;
  return temperature;
}
}  // namespace unav::drivers
