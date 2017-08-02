#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_hal_conf.h>
#include "stm32f4xx_hal.h"

#include "stdint.h"
#include <math.h>

#include "diag/Trace.h"

#ifndef __I2C_H_
#define __I2C_H_

#define STM32_HAL
#define MPU6050

#ifdef STM32_HAL
#define UNUSED(x) ((void)(x))

#define delay_ms  HAL_Delay

#define fabs        fabsf
#define min(a,b) ((a<b)?a:b)

static inline int reg_int_cb(struct int_param_s *int_param)
{
  UNUSED(int_param);
    return 0;
}

#define get_ms(timestamp) (*timestamp=HAL_GetTick())

#define __no_operation() (0)

#endif
#define hi2cMPU6050 hi2c1

#ifdef TRACE
#ifdef DIAG_TRACE_H_
#define log_i       trace_printf
#define log_e       trace_printf
#else
#define log_i       printf
#define log_e       printf
#endif
#else
#define log_i(...)     do {} while (0)
#define log_e(...)     do {} while (0)
#endif

HAL_StatusTypeDef i2c_write(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t length, uint8_t const *data);
HAL_StatusTypeDef i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length,
    uint8_t *data);
HAL_StatusTypeDef IICwriteBit(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t bitNum, uint8_t data);
HAL_StatusTypeDef IICwriteBits(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t bitStart, uint8_t length, uint8_t data);

#endif
