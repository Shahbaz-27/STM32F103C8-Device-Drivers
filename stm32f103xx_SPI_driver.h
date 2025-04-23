/*
 * stm32f103xx_SPI_driver.h
 *
 *  Created on: Aug 31, 2024
 *      Author: Dell
 */
#ifndef STM32F103XX_SPI_DRIVER_H_
#define STM32F103XX_SPI_DRIVER_H_

#include "stm32f103xx.h"


typedef struct{
	vo uint8_t SPI_deviceMode;
	vo uint8_t SPI_busConfig;
	vo uint8_t SPI_DFF;
	vo uint8_t SPI_CPHA;
	vo uint8_t SPI_CPOL;
	vo uint8_t SPI_SSM;
	vo uint8_t SPI_speed;
}SPIx_Config_t;



typedef struct{
	SPIx_RegDef_t *pSPIx;
	SPIx_Config_t SPIx_Config;
}SPIx_Handle_t;




/*Macros for defining the Configs for SPI Configs*/
#define SPI_DEVICE_SLAVE_MODE       0
#define SPI_DEVICE_MASTER_MODE      1


#define SPI_BUS_CONFIG_FULLDUPLEX   0
#define SPI_BUS_CONFIG_HALFDUPLEX   1
#define SPI_BUS_CONFIG_SIMPLEX_RX   2

#define SPI_CLKSPEED_DIV2           0
#define SPI_CLKSPEED_DIV4           1
#define SPI_CLKSPEED_DIV8           2
#define SPI_CLKSPEED_DIV16          3
#define SPI_CLKSPEED_DIV32          4
#define SPI_CLKSPEED_DIV64          5
#define SPI_CLKSPEED_DIV128         6
#define SPI_CLKSPEED_DIV256         7


#define SPI_DFF_8BITS                0
#define SPI_DFF_16BITS               1

#define SPI_CPOL_0					0
#define SPI_CPOL_1 					1

#define SPI_CPHA_FIRSTCLK			0
#define SPI_CPHA_SECONDCLK 			1


#define SPI_SSM_EN                      1
#define SPI_SSM_DI                      0

//Some Macros
/*_______Declarations for APIs required for SPI Driver Development_________*/
void SPI_PeriClkCtrl	(SPIx_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_Init	     	(SPIx_Handle_t *pSPIxHandle,SPIx_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_DeInit     	(SPIx_RegDef_t *pSPIx);
void SPI_sendData       (SPIx_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t length);
void SPI_receiveData    (SPIx_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t length);
void SPI_SSIConfig		(SPIx_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_PeripheralCtrl (SPIx_RegDef_t *pSPIx,uint8_t EnorDi);


void SPI_IRQConfig          (uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig  (uint8_t IRQPriority,uint8_t IRQNumber);
void SPI_IRQHandling        (SPIx_Handle_t *pSPIxHandle);

#endif /* STM32F103XX_SPI_DRIVER_H_ */
