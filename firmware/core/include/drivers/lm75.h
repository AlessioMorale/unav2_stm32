#include "stm32f4xx_hal.h"
#include <mathutils.h>
#include <stdint.h>
#include <utils/timer.h>
namespace unav::drivers {

enum class Lm75Registers : uint8_t {
  temperature = 0x00, //
  configuration = 0x01,
  hysteresys = 0x02,
  overtempshutdown = 0x03,
  deviceId = 0x07
};
class Lm75 {

public:
  Lm75(I2C_HandleTypeDef *i2c, uint8_t deviceAddress);
  float getTemperature();
  void setup();

  const float temperatureConversionValue = 0.5f;

protected:
  I2C_HandleTypeDef *i2cport;
  uint8_t address;

private:
  Lm75() = delete;
  Lm75(const Lm75 &) = delete;
  Lm75 &operator=(const Lm75 &) = delete;
  unav::utils::Timer timer;
};
} // namespace unav::drivers
