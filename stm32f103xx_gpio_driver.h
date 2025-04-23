/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Aug 24, 2024
 *  Author: Shahbaz Ahmed
 */

#ifndef STM32F103XX_GPIO_DRIVER_H_
#define STM32F103XX_GPIO_DRIVER_H_
#include "stm32f103xx.h"

#define En  1

typedef struct{
vo	uint8_t GPIOx_pinNumber;
vo  uint8_t GPIOx_pinMode;
vo	uint8_t GPIOx_pinConfig;
	uint8_t GPIOx_CFG_INTERRUPT_EDGE_TYPE;
	uint8_t GPIOx_pinAltFuncMode;


}GPIOx_PinConfig_t;

typedef struct
{
	/*Note: The first member of this structure is actually a pointer of type structure names GPIOx_RegDef_t which is infact holding all registers of GPIO so thats why we need to access the base address in order to have all the registers in hand
	 * THe second member is NOT of pointer type but just a instance of the associated structure since that structure doesnt have any registers and so no memory allocation play in act*/

	GPIOx_RegDef_t *pGPIOx;
GPIOx_PinConfig_t GPIOx_PinConfig;
}GPIOx_Handle_t;


//@GPIO Modes
/*for MODE==00 all the CFG are in input mode*/

/*Note that the first 4 macros are in accordance to reference manual while the other 3 macros are custom based*/
#define GPIOx_CFG_INPUTANALOG 				0 //for mode==00 GPIO Input Analog
#define GPIOx_CFG_INPUTFLOAT 				1 //for mode==00 GPIO Input Floating
#define GPIOx_CFG_INPUTWITHPUPD 			2 //for mode==00 GPIO Input Pull-up/down
#define GPIOx_CFG_INPUTRESERVED 			3 //for mode==00 system reserved
#define GPIOx_CFG_INPUT_ITRISEEDGE 			4 //for mode==00 Interrupt mode for Rising Edge
#define GPIOx_CFG_INPUT_ITFALLEDGE 			5 //for mode==00 Interrupt mode for Falling Edge
#define GPIOx_CFG_INPUT_ITRISEFALLEDGE 		6 //for mode==00 Interrupt mode for both Rising and FallingEdge




/*for MODE !=00 all the CFG are in output mode*/
#define GPIOx_CFG_OUTPP				0 //for mode !=00 GPIO Output Push/Pull
#define GPIOx_CFG_OUTOD    	 		1 //for mode !=00 GPIO Output Open drain
#define GPIOx_CFG_ALTFNOUTPP		2 //for mode !=00 Alternate Fun Output Push/Pull
#define GPIOx_CFG_ALTFNOUTOD		3 //for mode !=00 Alternate Fun Output Open drain


//@GPIO Speeds
/*Macros for Mode bits in CRL/CRH*/
/*Note that the first 4 macros are in accordance to reference manual while the other 1 macro is custom based*/

#define GPIOx_MODE_INPUT	 		0 // here MODE=00 so in input mode(reset state)
#define GPIOx_MODE_OUTPUT10MHz 		1 // GPIO Output @10 MHz
#define GPIOx_MODE_OUTPUT02MHz 		2 // GPIO Output @2  MHz
#define GPIOx_MODE_OUTPUT50MHz 		3 // GPIO Output @50 MHz
#define GPIOx_MODE_INTERRUPT		4 // Custom defined


/*___________________Lets create Macros for Pin Numbers__________*/

//@GPIO Pin Numbers
#define GPIOx_PIN_NUM0  0
#define GPIOx_PIN_NUM1  1
#define GPIOx_PIN_NUM2  2
#define GPIOx_PIN_NUM3  3
#define GPIOx_PIN_NUM4  4
#define GPIOx_PIN_NUM5  5
#define GPIOx_PIN_NUM6  6
#define GPIOx_PIN_NUM7  7
#define GPIOx_PIN_NUM8  8
#define GPIOx_PIN_NUM9  9
#define GPIOx_PIN_NUM10 10
#define GPIOx_PIN_NUM11 11
#define GPIOx_PIN_NUM12 12
#define GPIOx_PIN_NUM13 13
#define GPIOx_PIN_NUM14 14
#define GPIOx_PIN_NUM15 15





/*Let's define some of the APIs(Application Programming Interfaces) for GPIO */
void GPIO_PeriClkCtrl	(GPIOx_RegDef_t *pGPIOx,uint8_t EnorDi);
void GPIO_Init	     	(GPIOx_Handle_t *pGPIOHandle);
void GPIO_DeInit     	(GPIOx_RegDef_t *pGPIOx);
uint8_t GPIO_ReadPin    (GPIOx_RegDef_t *pGPIOx,uint8_t pinNumber);
uint16_t GPIO_ReadPort  (GPIOx_RegDef_t *pGPIOx);
void GPIO_WritePin   	(GPIOx_RegDef_t *pGPIOx,uint8_t pinNumber,uint8_t value);
void GPIO_WritePort  	(GPIOx_RegDef_t *pGPIOx,uint8_t value);
void GPIO_TogglePin  	(GPIOx_RegDef_t *pGPIOx,uint8_t pinNumber);


void GPIO_IRQConfig  (uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig  (uint8_t IRQPriority,uint8_t IRQNumber);

void GPIO_IRQHandling(uint8_t pinNumber);





#endif /* STM32F103XX_GPIO_DRIVER_H_ */
