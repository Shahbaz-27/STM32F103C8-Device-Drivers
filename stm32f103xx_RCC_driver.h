/*
 * stm32f103xx_RCC_driver.h
 *
 *  Created on: Sep 11, 2024
 *      Author: Dell
 */

#ifndef STM32F103XX_RCC_DRIVER_H_
#define STM32F103XX_RCC_DRIVER_H_

#include "stm32f103xx.h"


uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t  RCC_GetPLLOutputClock(void);






#endif /* STM32F103XX_RCC_DRIVER_H_ */
