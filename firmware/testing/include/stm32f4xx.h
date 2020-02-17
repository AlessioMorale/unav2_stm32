#pragma once
#include <stdint.h>
extern "C" {

typedef enum { HAL_OK = 0x00U, HAL_ERROR = 0x01U, HAL_BUSY = 0x02U, HAL_TIMEOUT = 0x03U } HAL_StatusTypeDef;

// I2C Support

#define I2C_MEMADD_SIZE_8BIT 0x00000001U
#define I2C_MEMADD_SIZE_16BIT 0x00000010U
typedef uint32_t I2C_HandleTypeDef;

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
                                    uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
                                   uint32_t Timeout);
}
