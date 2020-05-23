#include <drivers/lm75.h>

namespace unav::drivers {

typedef union {
  uint8_t Bytes[2];
  int16_t Word;
} twoBytesConverter_t;

Lm75::Lm75(I2C_HandleTypeDef *port, uint8_t deviceAddress) : i2cport{port}, address{deviceAddress} {
}

void Lm75::setup() {
  //
}
float Lm75::getTemperature() {
  twoBytesConverter_t Result;
  float temperature;
  uint8_t data[2];
  HAL_I2C_Mem_Read(i2cport, (uint16_t)address, (uint16_t)Lm75Registers::temperature, I2C_MEMADD_SIZE_8BIT, data, 2, 10);
  Result.Bytes[1] = data[0];
  Result.Bytes[0] = data[1];
  Result.Word /= 128;

  temperature = (float)Result.Word * temperatureConversionValue;
  return temperature;
}
} // namespace unav::drivers
