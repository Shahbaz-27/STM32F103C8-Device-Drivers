/*
 * stm32f103xx_USART_driver.c
 *
 *  Created on: Sep 5, 2024
 *      Author: Dell
 */

#include "stm32f103xx.h"
#include "stm32f103xx_USART_driver.h"

void USART_PeriClkCtrl(USARTx_RegDef_t *pUSARTx,uint8_t EnorDi)
{
	if (EnorDi == En)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
	}



}
void USART_Init(USARTx_Handle_t *pUSARTxHandle)
{
	// Here we need to do initializations for USART
AFIO_CLK_EN();
uint32_t regmask=0;


	if(pUSARTxHandle->USARTx_Config.USART_commMode == USART_COMM_MODE_RX)
	{
		regmask |= (1<<USART_RX_EN);

	}
	else if(pUSARTxHandle->USARTx_Config.USART_commMode == USART_COMM_MODE_TX)
	{
		regmask |= (1<<USART_TX_EN);
	}
	else
	{
		regmask |= (1<<USART_RX_EN);
		regmask |= (1<<USART_TX_EN);
	}
	// NOW DOING FOR PARITY CONTROL

	if(pUSARTxHandle->USARTx_Config.USART_parityBit == USART_PARITYBIT_EVEN_EN)
	{

		regmask |= ( 1 << USART_PCE );
		// be default even parity is selected but still you can hardcore the value on the bit
		regmask &= ~( 1  << USART_PARITY_SEL );
	}
	else if(pUSARTxHandle->USARTx_Config.USART_parityBit == USART_PARITYBIT_ODD_EN)
	{

		regmask |= ( 1 << USART_PCE );
		regmask |= ( 1  << USART_PARITY_SEL );

	}
	else
	{
		regmask &= ~( 1 << USART_PCE );
	}


	regmask |= (pUSARTxHandle->USARTx_Config.USART_dataFrameSize << USART_WL);
	regmask |= (1<<USART_EN);
	pUSARTxHandle->pUSARTx->CR1 = regmask;

	// Reseting the temp mask variable to play with next registers
	regmask = 0;




	/*________These configs are now for CR2________*/
	// for the time being iam only considering UART nature
	regmask |= (pUSARTxHandle->USARTx_Config.USART_stopBit << USART_STOP);
	pUSARTxHandle->pUSARTx->CR2 = regmask;
	regmask = 0;


	/*________These configs are now for CR3________*/
	if (pUSARTxHandle->USARTx_Config.USART_HWFlowCtrl == USART_HW_FLOW_CTRL_CTS)
	{
			regmask |= (1<<USART_CTSE);
	}
	else if(pUSARTxHandle->USARTx_Config.USART_HWFlowCtrl == USART_HW_FLOW_CTRL_RTS)
	{
			regmask |= (1<<USART_RTSE);
	}
	else if(pUSARTxHandle->USARTx_Config.USART_HWFlowCtrl == USART_HW_FLOW_CTRL_CTSRTS)
	{
			regmask |= (1<<USART_CTSE);
			regmask |= (1<<USART_RTSE);
	}
	pUSARTxHandle->pUSARTx->CR3 = regmask;
	regmask = 0;
	uint32_t baudRate = pUSARTxHandle->USARTx_Config.USART_baudRate;
	USARTx_setBaudRate(pUSARTxHandle->pUSARTx,baudRate);
}





void USARTx_setBaudRate(USARTx_RegDef_t *pUSARTx,uint32_t baudRate)
{


	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartDiv;

	//variables to hold Mantissa and Fraction values
	uint32_t mantissaPart;
	uint32_t fractionPart;
	uint32_t regmask=0; // will be used to mask the bits on BRR Reg
// Now it is essential to know the value of system clk that is appearing on the peripheral buses
  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  /*Note: This bit feature is not avaiable in stm32f103xx

  if(pUSARTx->CR1 & (1 << TO DO))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = (PCLKx / (16 *BaudRate));
  }else
  {
	   //over sampling by 16
	   //TO DO
  }
  .*/

  //Calculate the Mantissa part

  usartDiv = (PCLKx * 100 / (16 * baudRate)) ;
  mantissaPart = usartDiv / 100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  regmask |= mantissaPart << 4;



  //Extract the fraction part
  fractionPart = (usartDiv - (mantissaPart * 100));
  // as we need the approximation for the fractional part of the USART DIV and the const for
  //Calculate the final fractional
  //OVER8 = 1 , over sampling by 8 as by default this is the case and we this over8 bit is not present in stm32f103xx , in this it is always oversampling by 8
  fractionPart = ((fractionPart * USART_OVER8) + 50) / 100;

  regmask |= fractionPart; // starting from LSB of the BRR reg






  /*
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  F_part = ((( F_part * TO DO)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * TO DO)+ 50) / 100) & ((uint8_t)0x0F);

   }
*/



  pUSARTx->BRR= regmask;
}





void USART_DeInit(USARTx_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)

	{
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}


}
void USART_sendData(USARTx_Handle_t *pUSARTxHandle,uint8_t *pTxBuffer,uint32_t length)
{
	volatile uint16_t  *pdata;
	for(uint32_t i=0;i<length;++i)
	{

		while( !(pUSARTxHandle->pUSARTx->SR & 1<<USART_TXE) );
		if(pUSARTxHandle->USARTx_Config.USART_dataFrameSize == USART_DATA_FRAME_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer; // pdata has now the address of the element of array
			pUSARTxHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTxHandle->USARTx_Config.USART_parityBit == USART_PARITYBIT_DI)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			// check for the parity control in case of 8 bit data transfer
			//This is 8bit data transfer
			if (pUSARTxHandle->USARTx_Config.USART_parityBit == USART_PARITYBIT_DI)
			{
				//complete 8 bits for user data
			pUSARTxHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			//Implement the code to increment the buffer address
				pTxBuffer++;
			}
			else
			{
				//7 bits for user data and one bit for parity
				pUSARTxHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0x7F);
				//Implement the code to increment the buffer address
					pTxBuffer++;
			}
		}

	}
	while(!(pUSARTxHandle->pUSARTx->SR & 1<<USART_TC));
}


void USART_receiveData(USARTx_Handle_t *pUSARTxHandle, uint8_t *pRxBuffer, uint32_t length)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < length; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while( !(pUSARTxHandle->pUSARTx->SR & 1<<USART_RXNE) );

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTxHandle->USARTx_Config.USART_dataFrameSize == USART_DATA_FRAME_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//Now, check are we using USART_ParityControl control or not
			if(pUSARTxHandle->USARTx_Config.USART_parityBit == USART_PARITYBIT_DI)
			{
				//No parity is used , so all 9bits will be of user data

				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTxHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTxHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//Now, check are we using USART_ParityControl control or not
			if(pUSARTxHandle->USARTx_Config.USART_parityBit==USART_PARITYBIT_DI)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUSARTxHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTxHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//Now , increment the pRxBuffer
			pRxBuffer++;
		}

	}
	if(*pRxBuffer == "ON")
	{
						GPIO_WritePin(GPIOA, GPIOx_PIN_NUM7, GPIOx_Pin_Set);
	}
	else
	{
						GPIO_WritePin(GPIOA, GPIOx_PIN_NUM5, GPIOx_Pin_Set);
	}
}





















