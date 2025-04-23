/*
 * stm32f103xx_SPI_driver.c
 *
 *  Created on: Aug 31, 2024
 *      Author: Dell
 */


#include "stm32f103xx.h"
#include "stm32f103xx_SPI_driver.h"



/*_______Definations for APIs required for SPI Driver Development_________*/
void SPI_PeriClkCtrl	(SPIx_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == En)
	{
	    if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
	    else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
	    else if (pSPIx == SPI3)
	    {
			SPI3_PCLK_EN();
		}

	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
	    else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
	    else if (pSPIx == SPI3)
	    {
			SPI3_PCLK_DI();
		}


	}

}

SPI_PeripheralCtrl(SPIx_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if (EnorDi==En)
	{

	pSPIx->CR1 |=(1<<SPIx_CR1_SPE);
	}
	else
	{
	pSPIx->CR1 &= ~(1<<SPIx_CR1_SPE);
	}
}
SPI_SSIConfig(SPIx_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if (EnorDi==En)
	{
	pSPIx->CR1 |=(1<<SPIx_CR1_SSI);
	}
	else
	{
	pSPIx->CR1 &= ~(1<<SPIx_CR1_SSI);
	}
}

void SPI_Init	     	(SPIx_Handle_t *pSPIxHandle,SPIx_RegDef_t *pSPIx,uint8_t EnorDi)
{
	// Enabling the clk
	 SPI_PeriClkCtrl	(pSPIx,EnorDi);
	//First for SPI_deviceMode
	// for masterconfiguration

	uint32_t regmask = 0;
  regmask |= (pSPIxHandle->SPIx_Config.SPI_deviceMode<<SPIx_CR1_MSTR);

  //For SPI Bus Configs
  if(pSPIxHandle->SPIx_Config.SPI_busConfig == SPI_BUS_CONFIG_FULLDUPLEX)
  {
	  regmask &=(1<<SPIx_CR1_BIDIMODE);

  }
  else if(pSPIxHandle->SPIx_Config.SPI_busConfig == SPI_BUS_CONFIG_HALFDUPLEX)
  {
	  regmask |=(1<<SPIx_CR1_BIDIMODE);

  }
  else if(pSPIxHandle->SPIx_Config.SPI_busConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
  {
	  if(pSPIxHandle->SPIx_Config.SPI_deviceMode == SPI_DEVICE_MASTER_MODE)
	  { //clear the BIDI bit to have 2 line transmission in order to produce clk from master side
  	  regmask &=(1<<SPIx_CR1_RXONLY);
  	  // set the RXONLY BIT
  	  regmask |=(1<<SPIx_CR1_RXONLY);
	  }
  }



  // for SPI speed configs possiblities

  regmask |= (pSPIxHandle->SPIx_Config.SPI_speed<<SPIx_CR1_BR);


  // For Data Frame Size bits ->DFF

 regmask |= (pSPIxHandle->SPIx_Config.SPI_DFF<<SPIx_CR1_DFF);



  // for CPOL and CPHA nature

  regmask |= (pSPIxHandle->SPIx_Config.SPI_CPOL<< SPIx_CR1_CPOL);


  regmask |= (pSPIxHandle->SPIx_Config.SPI_CPHA<< SPIx_CR1_CPHA);


  // Now about the Software Slave Management Feature


  regmask |= (pSPIxHandle->SPIx_Config.SPI_SSM<<SPIx_CR1_SSM);

  // Now masking the Entire content of regmask to the actual physical register
  pSPIxHandle->pSPIx->CR1 = regmask;

}


void SPI_DeInit    (SPIx_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI1)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI1)
	{
		SPI3_REG_RESET();
	}

}


void SPI_sendData       (SPIx_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t length)
{
	while(length != 0)
	{
		while((pSPIx->SR & 1<<SPIx_SR_TXE) == 0);

		if (pSPIx->CR1 & 1<<SPIx_CR1_DFF)
		{
			// for 16 bits
			pSPIx->DR = *((uint16_t*)pTxBuffer); // Deferencing and typecasting
			length--;// since 2 bytes are written in DR and we now want to reset the length
			length--;
			(uint16_t*)pTxBuffer++; // as pTxbuffer stores the addrees of the data to be transferred so that the address has to be incremented for pointing to next data item
		}
		else
		{
			pSPIx->DR = *pTxBuffer;
			length--; // since one byte is written in DR and we now want to reset the length
			pTxBuffer++; // as pTxbuffer stores the addrees of the data to be transferred so that the address has to be incremented for pointing to next data item

		}

	}

}

void SPI_receiveData    (SPIx_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t length)
{

	while(length != 0)
		{
		while(!(pSPIx->SR & 1<<SPIx_SR_RXNE));

			if (pSPIx->CR1 & 1<<SPIx_CR1_DFF)
			{
				// for 16 bits
				*((uint16_t*)pRxBuffer) = pSPIx->DR; // Deferencing and typecasting
				length--;// since 2 bytes are written in DR and we now want to reset the length
				length--;
				(uint16_t*)pRxBuffer++; // as pTxbuffer stores the addrees of the data to be transferred so that the address has to be incremented for pointing to next data item
			}
			else
			{
				*pRxBuffer=pSPIx->DR;
				length--; // since one byte is written in DR and we now want to reset the length
				pRxBuffer++; // as pTxbuffer stores the addrees of the data to be transferred so that the address has to be incremented for pointing to next data item

			}

		}


}

