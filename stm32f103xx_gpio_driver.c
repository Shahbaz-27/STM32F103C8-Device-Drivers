/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Aug 24, 2024
 *      Author: Shahbaz Ahmed
 */
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"

// This API is responsible for Enabling and Disabling the clk for a GPIO Port
void GPIO_PeriClkCtrl(GPIOx_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == En)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
	}

}





void GPIO_Init(GPIOx_Handle_t *pGPIOHandle) //The * symbol means this argument is a pointer to the structure, not the structure itself.
//This allows the function to modify the data in the structure, and also saves memory because we only pass the address of the structure instead of copying all its data.
{
	uint32_t temp = 0;
	uint8_t pinNumber = pGPIOHandle->GPIOx_PinConfig.GPIOx_pinNumber;

	// Determine if the pin is in input or output mode
    if (pGPIOHandle->GPIOx_PinConfig.GPIOx_pinMode == GPIOx_MODE_INPUT)
    {
	        // Input mode (MODE = 00)
	        // For pins 0-7 (use CRL register)
	  if (pinNumber <= 7)
	  {
		 // Reset the configuration bits (CFG) for the input type (Analog, Floating, etc.)
		  pGPIOHandle->pGPIOx->CRL &= ~(0x3 << ((4 * pinNumber) + 2));
		 // Set the configuration bits (CFG) for the input type (Analog, Floating, etc.)
 		 temp = (pGPIOHandle->GPIOx_PinConfig.GPIOx_pinConfig << ((4 * pinNumber) + 2));
  		 pGPIOHandle->pGPIOx->CRL |= temp;

	  }
	        // For pins 8-15 (use CRH register)
	  else
	  {
	      // Reset the configuration bits (CFG) for the input type (Analog, Floating, etc.)
	      pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (4 * (pinNumber-8))+2 );
	      // Set the configuration bits (CFG) for the input type (Analog, Floating, etc.)
	      temp = (pGPIOHandle->GPIOx_PinConfig.GPIOx_pinConfig << ((4 *(pinNumber-8)) + 2));
          // Write the configuration to the CRH register
           pGPIOHandle->pGPIOx->CRH |= temp;

	   }
	   temp=0;
	 }
    else if (pGPIOHandle->GPIOx_PinConfig.GPIOx_pinMode == GPIOx_MODE_INTERRUPT)
        {
    	  // Input mode (MODE = 00)
          // For pins 0-7 (use CRL register)
    	  if (pinNumber <= 7)
    	  {
    		    // Reset the configuration bits (CFG) for the input type (Analog, Floating, etc.)
    		    pGPIOHandle->pGPIOx->CRL &= ~(0x3 << ((4 * pinNumber) + 2));
    		    // Set the configuration bits (CFG) for the input type (Analog, Floating, etc.)
    		  	temp = (pGPIOHandle->GPIOx_PinConfig.GPIOx_pinConfig << ((4 * pinNumber) + 2));
      		    pGPIOHandle->pGPIOx->CRL |= temp;
      			uint8_t temp1= pGPIOHandle->GPIOx_PinConfig.GPIOx_pinNumber/4;
      			uint8_t temp2= pGPIOHandle->GPIOx_PinConfig.GPIOx_pinNumber % 4;
      			uint8_t portCode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);
      			AFIO_CLK_EN();
      			AFIO->EXTICR[temp1] = portCode << (temp2*4) ;

      				if(pGPIOHandle->GPIOx_PinConfig.GPIOx_CFG_INTERRUPT_EDGE_TYPE == GPIOx_CFG_INPUT_ITRISEEDGE)
      		     	{
      					EXTI->RTSR |= (1<<pinNumber);
      		     	}
      				else if(pGPIOHandle->GPIOx_PinConfig.GPIOx_CFG_INTERRUPT_EDGE_TYPE == GPIOx_CFG_INPUT_ITFALLEDGE)
      		     	{
      					EXTI->FTSR |= (1<<pinNumber);
      		     	}
      		     	else if(pGPIOHandle->GPIOx_PinConfig.GPIOx_CFG_INTERRUPT_EDGE_TYPE == GPIOx_CFG_INPUT_ITRISEFALLEDGE)
      		     	{
      		     		EXTI->FTSR |= (1<<pinNumber);
      		     		EXTI->RTSR |= (1<<pinNumber);
      		     	}
      		  EXTI->IMR |= (1<<pinNumber);
    	  }

    	  // For pins 8-15 (use CRH register)
    	  else
    	  {
    	      // Reset the configuration bits (CFG) for the input type (Analog, Floating, etc.)
    	      pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (4 * (pinNumber-8))+2 );
    	      // Set the configuration bits (CFG) for the input type (Analog, Floating, etc.)
    	      temp = (pGPIOHandle->GPIOx_PinConfig.GPIOx_pinConfig << ((4 *(pinNumber-8)) + 2));
              // Write the configuration to the CRH register
               pGPIOHandle->pGPIOx->CRH |= temp;
               	uint8_t temp1= pGPIOHandle->GPIOx_PinConfig.GPIOx_pinNumber/4;
               	uint8_t temp2= pGPIOHandle->GPIOx_PinConfig.GPIOx_pinNumber % 4;
               	uint8_t portCode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);
               	AFIO_CLK_EN();
               	AFIO->EXTICR[temp1] = portCode << (temp2*4) ;

               		if(pGPIOHandle->GPIOx_PinConfig.GPIOx_CFG_INTERRUPT_EDGE_TYPE == GPIOx_CFG_INPUT_ITRISEEDGE)
                    {
               	  	  EXTI->RTSR |= (1<<pinNumber);
                    }
               		else if(pGPIOHandle->GPIOx_PinConfig.GPIOx_CFG_INTERRUPT_EDGE_TYPE == GPIOx_CFG_INPUT_ITFALLEDGE)
                    {
               				EXTI->FTSR |= (1<<pinNumber);
                    }
                     else if(pGPIOHandle->GPIOx_PinConfig.GPIOx_CFG_INTERRUPT_EDGE_TYPE == GPIOx_CFG_INPUT_ITRISEFALLEDGE)
                    	{
                    		EXTI->FTSR |= (1<<pinNumber);
                    		EXTI->RTSR |= (1<<pinNumber);
                    	}
                 EXTI->IMR |= (1<<pinNumber);

    	   }
    	   temp=0;
    	 }
   else
   {
	// The else is to be executed only when MODE !=00
   // now to config bits for mode!=0 in output mode
	   if((pGPIOHandle->GPIOx_PinConfig.GPIOx_pinConfig == GPIOx_CFG_ALTFNOUTPP) || (pGPIOHandle->GPIOx_PinConfig.GPIOx_pinConfig == GPIOx_CFG_ALTFNOUTOD ))
	   {
		   AFIO_CLK_EN();
	   }

		   temp = (pGPIOHandle->GPIOx_PinConfig.GPIOx_pinConfig <<2) | (pGPIOHandle->GPIOx_PinConfig.GPIOx_pinMode); // the 4 bit data

	        if (pinNumber <= 7)
	       	{
	        	pGPIOHandle->pGPIOx->CRL &= ~(0xF << (4 * pinNumber) ) ;
	           // Write the configuration to the CRL register
	        	 temp = (temp <<  (4 * pinNumber ));
	        	 pGPIOHandle->pGPIOx->CRL |= temp;
	       	}
	     	    // For pins 8-15 (use CRH register)
	       	else
	        {
	        	pGPIOHandle->pGPIOx->CRH &= ~(0xF << (4 * (pinNumber-8)) ) ;
	           // Write the configuration to the CRH register
	        	  temp = (temp <<  (4 * (pinNumber-8) ));
	              pGPIOHandle->pGPIOx->CRH |= temp; // as we don't want to wipe out entire content of the register
	    	}
	   temp=0;
	 }
}



void GPIO_DeInit     (GPIOx_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}

}
uint8_t GPIO_ReadPin    (GPIOx_RegDef_t *pGPIOx,uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x00000001);
	return value;
}


uint16_t GPIO_ReadPort   (GPIOx_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}


void GPIO_WritePin   (GPIOx_RegDef_t *pGPIOx,uint8_t pinNumber,uint8_t value)
{
	if(value== GPIOx_Pin_Set)
	{
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<pinNumber);
	}

}


void GPIO_WritePort  (GPIOx_RegDef_t *pGPIOx,uint8_t value)
{
	pGPIOx->ODR = value;
}


void GPIO_TogglePin  (GPIOx_RegDef_t *pGPIOx,uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1<<pinNumber);
}



void GPIO_IRQConfig  (uint8_t IRQNumber,uint8_t EnorDi)
{
	if(EnorDi == En)
	{
		if(IRQNumber>=0 && IRQNumber <=31)
		{
			//program ISER0
			*NVIC_ISER0 =  (1<< IRQNumber);
		}
		else if(IRQNumber>=32 && IRQNumber <=63)
		{
			//program ISER1

			*NVIC_ISER1 |= (1<< IRQNumber % 32);
		}
		else if(IRQNumber>=64 && IRQNumber <=95)
		{
			//program ISER2

			*NVIC_ISER2 |= (1<< IRQNumber % 64);
		}
	}
	else
		if(IRQNumber>=0 && IRQNumber <=31)
		{
			//program ICER0
			*NVIC_ICER0 = (1<< IRQNumber);
		}
		else if(IRQNumber>=32 && IRQNumber <=63)
		{
			//program ICER1
			*NVIC_ICER1 |= (1<< IRQNumber %32);

		}
		else if(IRQNumber>=64 && IRQNumber <=95)
		{
			//program ICER2
			*NVIC_ICER2 |= (1<< IRQNumber % 64);

		}

}

void GPIO_IRQPriorityConfig  (uint8_t IRQPriority,uint8_t IRQNumber)
{
	 uint8_t iprx_reg = IRQNumber /4;// will tell the reg number
	 uint8_t iprx_sec = IRQNumber % 4;// will tell the sec number
	 uint8_t shiftAmount = (8 * iprx_sec) + (8-NO_PR_BITS_IMPLEMENTED);
	volatile uint32_t *priority_reg = (volatile uint32_t *)(NVIC_PR_BASE_ADDR + (iprx_reg * 4));
	*priority_reg |= (IRQPriority << shiftAmount);


}



void GPIO_IRQHandling(uint8_t pinNumber)
{
	// need to clear the interrupt flag
	if (EXTI->PR & (1<<pinNumber)) // it means that PR has 1 at this point i.e a pending interrupt signal
	{
		// so after confirmation we must clear the pending bit otherwise multiple interrupts will be trigerred
		EXTI->PR |= (1<<pinNumber);//writing 1 to PR clears it
	}

}
