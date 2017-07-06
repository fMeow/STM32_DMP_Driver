/*
 * ArduinoWrapper.h
 *
 *  Created on: July 6, 2017
 *      Author: Guoli Lv
 */

#ifndef ARDUINOWRAPPER_H_
#define ARDUINOWRAPPER_H_



//Standard Libraries
#include <stm32f1xx_hal.h>

//TODO functions that need wrapper: millis(), Serial.print

#define millis() HAL_GetTick()
#define I2CDEV_DEFAULT_WRITE_TIMEOUT     100

#endif /* ARDUINOWRAPPER_H_ */
