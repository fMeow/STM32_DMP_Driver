# STM32 DMP Driver

Implemented with HAL Lib.

Notice that I have ported two version of DMP driver, one in the master branch from TI implementation, another from Arduino I2Cdev. **I recommend the [I2Cdev](https://github.com/fMeow/STM32_DMP_Driver/tree/I2Cdev) one** for a more completed featured and better code organization, with the cost of binary size.

## Hardware I2C
1. Put ./Source/MPU6050 into your source folder, say, 'src' and place ./Include/MPU6050 into your header folder. ATTENTION: Keep header files in MPU6050 folder under your include path.
2.  Modify ./Include/MPU6050/I2C.h to meet your need.
    - Redefine hi2cMPU6050 to the I2C_HandleTypeDef object which communicates with MPU6050.
    - Decide which HAL header files to use. For STM32F4 user:

        ```cpp
        #include <stm32f4xx_hal_def.h>
        #include <stm32f4xx_hal_conf.h>
        #include "stm32f4xx_hal.h"
        ```

    - Enable or disable "diag/Trace.h". 

## Software I2c

## Cpp Class Inherited From I2CDev
