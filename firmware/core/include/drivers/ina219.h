#include <mathutils.h>
#include <stdint.h>
#include <utils/timer.h>
#include "stm32f4xx_hal.h"
namespace unav::drivers {
/**
 * This is a Encoder class to demonstrate features of the boiler plate.
 */
typedef struct {
  float voltage;
  float current;

} power_status_t;
class Ina219 {
 public:
  /**
   * Default constructur for Encoder (does nothing).
   */
  Ina219(I2C_HandleTypeDef *i2c, uint8_t address);
  power_status_t getPowerStatus();
  void setup();

 protected:
  I2C_HandleTypeDef *i2cport;
  uint8_t address;

 private:
  Ina219() = delete;
  Ina219(const Ina219 &) = delete;
  Ina219 &operator=(const Ina219 &) = delete;

  unav::utils::Timer timer;
};
}  // namespace unav::drivers
