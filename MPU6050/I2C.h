#include <stm32f1xx_hal_def.h>
#include "stdint.h"

#ifndef __I2C_H_
#define __I2C_H_
HAL_StatusTypeDef i2c_write(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t length, uint8_t const *data);
HAL_StatusTypeDef i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length,
    uint8_t *data);
HAL_StatusTypeDef IICwriteBit(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t bitNum, uint8_t data);
HAL_StatusTypeDef IICwriteBits(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t bitStart, uint8_t length, uint8_t data);
#endif
