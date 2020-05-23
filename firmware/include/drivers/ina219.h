#include "stm32f4xx.h"
#include <stdint.h>

namespace unav::drivers {
/**
 * This is a Encoder class to demonstrate features of the boiler plate.
 */
typedef struct {
  float voltage;
  float current;
} PowerStatus_t;

typedef union {
  struct {
    uint16_t mode : 3;
    uint16_t sadc : 4;
    uint16_t badc : 4;
    uint16_t gain : 2;
    uint16_t bRange : 1;
    uint16_t resvd : 1;
    uint16_t reset : 1;
  } flags __attribute__((packed));
  uint16_t registerValue;
} Ina219ConfigReg_t;

enum class Ina219Registers : uint8_t {
  configuration = 0x00,
  shuntVoltage = 0x01,
  busVoltage = 0x02,
  power = 0x03,
  current = 0x04,
  calibration = 0x05,
};

enum class Ina219ConfigGain : uint16_t {
  gain1_40MV = 0x0,  // Gain 1, 40mV Range
  gain2_80MV = 0x1,  // Gain 2, 80mV Range
  gain4_160MV = 0x2, // Gain 4, 160mV Range
  gain8_320MV = 0x3, // Gain 8, 320mV Range
};

enum class Ina219ConfigBusAdcResolution : uint16_t {

  adc9BIT = 0x0,  // 9-bit bus res = 0..511
  adc10BIT = 0x1, // 10-bit bus res = 0..1023
  adc11BIT = 0x2, // 11-bit bus res = 0..2047
  adc12BIT = 0x3, // 12-bit bus res = 0..4097
};

enum class Ina219ConfigShuntAdcResolution : uint16_t {
  adc9BIT_1S_84US = 0x0,     // 1 x 9-bit shunt sample
  adc10BIT_1S_148US = 0x1,   // 1 x 10-bit shunt sample
  adc11BIT_1S_276US = 0x2,   // 1 x 11-bit shunt sample
  adc12BIT_1S_532US = 0x3,   // 1 x 12-bit shunt sample
  adc12BIT_2S_1060US = 0x9,  // 2 x 12-bit shunt samples averaged together
  adc12BIT_4S_2130US = 0xA,  // 4 x 12-bit shunt samples averaged together
  adc12BIT_8S_4260US = 0xB,  // 8 x 12-bit shunt samples averaged together
  adc12BIT_16S_8510US = 0xC, // 16 x 12-bit shunt samples averaged together
  adc12BIT_32S_17MS = 0xD,   // 32 x 12-bit shunt samples averaged together
  adc12BIT_64S_34MS = 0xE,   // 64 x 12-bit shunt samples averaged together
  adc12BIT_128S_69MS = 0xF,  // 128 x 12-bit shunt samples averaged together
};

enum class Ina219ConfigMode : uint16_t {
  powerdown = 0,
  sVoltTriggered = 0x1,
  bVoltTriggered = 0x2,
  sAndBvoltTriggered = 0x3,
  adcOff = 0x4,
  sVoltContinuous = 0x5,
  bVoltContinuous = 0x6,
  sAndBvoltContinuous = 0x7,
};

class Ina219 {
public:
  Ina219(I2C_HandleTypeDef *i2c, uint8_t address);
  PowerStatus_t getPowerStatus();
  void setup(Ina219ConfigGain gain, Ina219ConfigBusAdcResolution busAdc, Ina219ConfigShuntAdcResolution shuntAdc, uint16_t calibration, float shunt);

protected:
  I2C_HandleTypeDef *i2cport;
  uint8_t address;

  uint16_t calibrationConfig;
  float shuntConfig;

  float currentLsb;
  float powerLsb;
  const float currentLsbFactor = 0.04096f;
  const float voltageLsb = 0.004f; // LSB at full scale = 30V

  const float powerCalibrationFactor = 20;
  void writeUint16(Ina219Registers destination, uint16_t value);
  uint16_t readUint16(Ina219Registers source);

private:
  Ina219() = delete;
  Ina219(const Ina219 &) = delete;
  Ina219 &operator=(const Ina219 &) = delete;
};
} // namespace unav::drivers
