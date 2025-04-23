/*
 * stm32f103xx_RCC_driver.c
 *
 *  Created on: Sep 11, 2024
 *      Author: Shahbaz
 */


#include "stm32f103xx_RCC_driver.h"

	uint16_t AHB_preScaler [8] = {2,4,8,16,64,128,256,512};
	uint8_t  APB1_preScaler[4] = { 2, 4 , 8, 16};
	uint8_t  APB2_preScaler[4] = { 2, 4 , 8, 16};
uint32_t RCC_GetPCLK1Value(void)
{


	uint32_t pclk1,sysClk;

	uint8_t clkSrc,temp,ahbp,apb1p;
	clkSrc = (RCC->CFGR >> 2) & 0x3;


	if(clkSrc == 0 )
	{
		sysClk = 8000000;// HSI from internal 8MHz Oscillator is selected as the sysClk by default
	}else if(clkSrc == 1)
	{
		sysClk = 8000000;
	}else if (clkSrc == 2)
	{
		sysClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if (temp<8)
	{
		 ahbp = 1;
	}
	else
	{
		ahbp = AHB_preScaler[temp-8];
	}



	//apb1
	temp = (RCC->CFGR >> 8) & 0xF;

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_preScaler[temp-4];
	}

	pclk1 =  (sysClk / ahbp) /apb1p;// this formula is derived from the clk tree

	return pclk1;
}






/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2,temp,sysClk;
	uint8_t apb2p,ahbp,clkSrc;

	clkSrc = (RCC->CFGR >> 2) & 0x3;

	if (clkSrc == 0)
	{
		sysClk = 8000000;
	}
	else if(clkSrc==1)
	{
		sysClk = 8000000;
	}
	else
	{
		sysClk = RCC_GetPLLOutputClock();
	}

	// for ahb prescalar
	temp = RCC->CFGR >> 8 &0xF;
	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_preScaler[temp - 8];
	}


	// for apb2 prescalar

	temp = RCC->CFGR >> 11 & 0x7;
	if(temp <4)
	{
		apb2p = 1;

	}
	else
	{
		apb2p = APB2_preScaler[temp - 4];
	}
	pclk2 = (sysClk / ahbp)/apb2p;
	return pclk2;
}



uint32_t  RCC_GetPLLOutputClock(void)
{

	return 0;
}
