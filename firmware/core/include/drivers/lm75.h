#include <mathutils.h>
#include <stdint.h>
#include <utils/timer.h>
#include "stm32f4xx_hal.h"
namespace unav::drivers {
/**
 * This is a Encoder class to demonstrate features of the boiler plate.
 */
class Lm75 {
 public:
  /**
   * Default constructur for Encoder (does nothing).
   */
  Lm75(I2C_HandleTypeDef *i2c, uint8_t address);
  int32_t getTemperature();
  void setup();
  const uint8_t LM75_TEMP = 0x00;  // Temperature Register of LM75
  const uint8_t LM75_CONF = 0x01;  // Configuration Register of LM75
  const uint8_t LM75_HYST 0x02     //
      const uint8_t LM75_TOS =
          0x03;  // Over-temp Shutdown threshold Register of LM75
  const uint8_t LM75_DevID 0x07       // Product ID Register
      const int32_t LM75_TEMP_CORR 2  // LM75

      protected : I2C_HandleTypeDef *i2cport;
  uint8_t address;

 private:
  Lm75() = delete;
  Lm75(const Lm75 &) = delete;
  Lm75 &operator=(const Lm75 &) = delete;
  unav::utils::Timer timer;
};
}  // namespace unav::drivers
