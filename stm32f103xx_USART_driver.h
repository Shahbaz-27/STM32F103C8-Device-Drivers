/*
 * stm32f103xx_USART.h
 *
 *  Created on: Sep 5, 2024
 *      Author: Dell
 */

#ifndef STM32F103XX_USART_H_
#define STM32F103XX_USART_H_
#include "stm32f103xx.h"



typedef struct{
	 uint8_t USART_commMode; // for asking to configure in UART or USART
	 uint8_t USART_busConfig;
	 uint8_t USART_dataFrameSize;
	 uint8_t USART_parityBit;
	 uint32_t USART_baudRate;
	 uint8_t USART_stopBit;
	 uint8_t USART_HWFlowCtrl;

}USARTx_Config_t;


typedef struct{
USARTx_RegDef_t *pUSARTx;
USARTx_Config_t USARTx_Config;


}USARTx_Handle_t;

#define USART_COMM_MODE_RX		 		0
#define USART_COMM_MODE_TX		    	1
#define USART_COMM_MODE_RXTX	     	2
#define USART_BUS_CONFIG_HALFDUPLEX     1
#define USART_DATA_FRAME_8BITS		    0
#define USART_DATA_FRAME_9BITS		    1


#define USART_PARITYBIT_EVEN_EN			0
#define USART_PARITYBIT_ODD_EN			1
#define USART_PARITYBIT_DI				2

#define USART_HW_FLOW_CTRL_NONE			0
#define USART_HW_FLOW_CTRL_CTS			1
#define USART_HW_FLOW_CTRL_RTS			2
#define USART_HW_FLOW_CTRL_CTSRTS		3




#define USART_STOP_BIT_1				0
#define USART_STOP_BIT_HALF				1
#define USART_STOP_BIT_2				2
#define USART_STOP_BIT_ONE_HALF			3



/*@USART_Baud
*Possible options for USART_Baud
*/
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define USART_STD_BAUD_3M 					3000000






void USART_PeriClkCtrl(USARTx_RegDef_t *PUSARTx,uint8_t EnorDi);
void USART_Init(USARTx_Handle_t *pUSARTxHandle);
void USART_DeInit(USARTx_RegDef_t *pUSARTx);
void USART_sendData(USARTx_Handle_t *pUSARTxHandle,uint8_t *pTxBuffer,uint32_t length);
void USART_receiveData(USARTx_Handle_t *pUSARTxHandle,uint8_t *pRxBuffer,uint32_t length);
void USARTx_setBaudRate(USARTx_RegDef_t *pUSARTx,uint32_t baudRate);

void USART_IRQHanding();
void USART_IRQConfig();

#endif /* STM32F103XX_USART_H_ */
