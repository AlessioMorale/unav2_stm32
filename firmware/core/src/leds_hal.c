#include <leds_hal.h>
volatile uint8_t leds_hal_ledvalue[NUM_LEDS] = {0};

/**
 * @brief runs the low level update of leds output based on required brightness.
 * Generate a software pwm output
 */
void leds_hal_updateLeds() {
  const GPIO_PinState ledsOnState[] = LED_ARRAY_OF_STATES_ON;
  const GPIO_PinState ledsOffState[] = LED_ARRAY_OF_STATES_OFF;
  const GPIO_TypeDef *ledsGpio[] = LED_ARRAY_OF_GPIO;
  const uint16_t ledspin[] = LED_ARRAY_OF_PIN;
  static uint8_t count = 0;

  count += UINT8_MAX / 20;
  for (int led = 0; led < NUM_LEDS; led++) {
    const GPIO_PinState pinstate = (count < leds_hal_ledvalue[led]) ? ledsOnState[led] : ledsOffState[led];
    HAL_GPIO_WritePin((GPIO_TypeDef *)ledsGpio[led], ledspin[led], pinstate);
  }
}