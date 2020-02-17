#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "stm32f4xx.h"
#include <iostream>
#include <mocks/i2cmocks.h>
std::map<uint16_t, uint16_t> i2c_registers;
std::map<uint16_t, uint16_t> i2c_response;

extern "C" {
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
                                    uint32_t Timeout) {
  (void)Timeout;
  (void)DevAddress;
  (void)MemAddSize;

  CHECK_FALSE_TEXT(hi2c == nullptr, "Null I2C handle");

  uint16_t val;
  if (Size == 1) {
    val = *(uint8_t *)pData;
  } else if (Size == 2) {
    val = *(uint16_t *)pData;
  }

  i2c_registers[MemAddress] = val;

  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
                                   uint32_t Timeout) {
  (void)Timeout;
  (void)DevAddress;
  (void)MemAddSize;

  uint16_t value = 0;
  CHECK_FALSE_TEXT(hi2c == nullptr, "Null I2C handle");

  value = i2c_response[MemAddress];

  switch (Size) {
  case 1:
    *pData = (uint8_t)value;
    break;
  case 2:
    *((uint16_t *)pData) = (uint16_t)value;
    break;
  default:
    CHECK(false);
  }
  return HAL_OK;
}
}
