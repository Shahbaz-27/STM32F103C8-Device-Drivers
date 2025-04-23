/*
 * stm32f103xx.h
 *
 *  Created on: Aug 24, 2024
 *      Author: Shahbaz Ahmed
 */
#ifndef STM32F103XX_H_
#define STM32F103XX_H_

#include <stdint.h>
#define vo volatile


/******Processor Specific Peripherals and Details********/


#define NVIC_PR_BASE_ADDR  ((vo uint32_t*) 0xE000E400)
#define NO_PR_BITS_IMPLEMENTED 4
/*For enabling the interrupt for specific IRQ number*/
#define NVIC_ISER0    ((vo uint32_t*)0xE000E100) // To Enable the Interrupt bits in NVIC //
#define NVIC_ISER1    ((vo uint32_t*)0xE000E104) // To Enable the Interrupt bits in NVIC //
#define NVIC_ISER2    ((vo uint32_t*)0xE000E108) // To Enable the Interrupt bits in NVIC //
#define NVIC_ISER3    ((vo uint32_t*)0xE000E10C) // To Enable the Interrupt bits in NVIC //

/*For disabling the interrupt for specific IRQ number*/

#define NVIC_ICER0    ((vo uint32_t*)0XE000E180) // To Disable the Interrupt bits in NVIC //
#define NVIC_ICER1    ((vo uint32_t*)0XE000E184) // To Disable the Interrupt bits in NVIC //
#define NVIC_ICER2    ((vo uint32_t*)0XE000E188) // To Disable the Interrupt bits in NVIC //
#define NVIC_ICER3    ((vo uint32_t*)0XE000E18C) // To Disable the Interrupt bits in NVIC //

/*The lines which goes from EXTI to NVIC*/
/*This information is fixed for every MCU in accordance to the vector table*/
#define IRQ_NO_EXTI_0   	6
#define IRQ_NO_EXTI_1   	7
#define IRQ_NO_EXTI_2   	8
#define IRQ_NO_EXTI_3   	9
#define IRQ_NO_EXTI_4   	10
#define IRQ_NO_EXTI_5_9   	23
#define IRQ_NO_EXTI_10_15   40

				/*__________________________________________*/


/********MCU Specific Peripehrals and Details************/

/*Note: The letter "U" written at the end of addresses is to tell the compiler that these values must be treated as unsigned*/

#define FLASH_BASE_ADDR 	  0x08000000U /*Base Address for Flash Memory(Main Memory)*/
#define SRAM_ADDR  			  0x20000000U /*Bas Address for SRAM*/
#define ROM_ADDR			  0x1FFFF000U /*Base Address for ROM(System Memory)*/

#define APB1_BASE_ADDR 	  	  0x40000000U
#define APB2_BASE_ADDR 	  	  0x40010000U
#define AHB_BASE_ADDR 	  	  0x40018000U

/*Now defining the  addresses of ONLY THE CONCERNED peripherals hanging on APB2 Bus*/
#define AFIO_BASE_ADDR 		(APB2_BASE_ADDR+0x0000)
#define EXTI_BASE_ADDR  	(APB2_BASE_ADDR+0x0400)
#define GPIOA_BASE_ADDR  	(APB2_BASE_ADDR+0x0800)
#define GPIOB_BASE_ADDR  	(APB2_BASE_ADDR+0x0C00)
#define GPIOC_BASE_ADDR  	(APB2_BASE_ADDR+0x1000)
#define ADC1_BASE_ADDR  	(APB2_BASE_ADDR+0x2400)
#define ADC2_BASE_ADDR  	(APB2_BASE_ADDR+0x2800)
#define TIM1_BASE_ADDR  	(APB2_BASE_ADDR+0x2C00)
#define SPI1_BASE_ADDR  	(APB2_BASE_ADDR+0x3000)
#define USART1_BASE_ADDR  	(APB2_BASE_ADDR+0x3800)


/*Now defining the  addresses of ONLY THE CONCERNED peripherals hanging on APB1 Bus*/
#define TIM2_BASE_ADDR  	(APB1_BASE_ADDR+0x0000)
#define TIM3_BASE_ADDR  	(APB1_BASE_ADDR+0x0400)
#define TIM4_BASE_ADDR  	(APB1_BASE_ADDR+0x0800)
#define SPI2_BASE_ADDR	    (APB1_BASE_ADDR+0x3800)
#define SPI3_BASE_ADDR	    (APB1_BASE_ADDR+0x3C00)
#define USART2_BASE_ADDR  	(APB1_BASE_ADDR+0x4400)
#define USART3_BASE_ADDR  	(APB1_BASE_ADDR+0x4800)
#define UART4_BASE_ADDR  	(APB1_BASE_ADDR+0x4C00)
#define UART5_BASE_ADDR  	(APB1_BASE_ADDR+0x5000)
#define I2C1_BASE_ADDR  	(APB1_BASE_ADDR+0x5400)
#define I2C2_BASE_ADDR  	(APB1_BASE_ADDR+0x5800)
#define I2C1_BASE_ADDR  	(APB1_BASE_ADDR+0x5400)
#define I2C2_BASE_ADDR  	(APB1_BASE_ADDR+0x5800)


/*Now defining the  addresses of ONLY THE CONCERNED peripherals hanging on AHB Bus*/
#define RCC_BASE_ADDR  		(AHB_BASE_ADDR+0x9000)


/*A Efficient way of writing the addresses is using the C structure */

/*Note: The keyword vo is defined for volatile at the start of file this is due to the fact that the value of these registers may change any time by any function so the term volatile tells the compiler to avoid memory optimization and check the contents of these registers which are most likely to change at any time*/
typedef struct
{
	/*Note:
	 * The order of the members of the structure must be synchronized with register map of manufactureR if at 0x00 offset CRl is present then placing any other member of the structure at first position will access the wrong register
    */

	 vo uint32_t CRL;
	 vo uint32_t CRH;
	 vo uint32_t IDR;
	 vo uint32_t ODR;
	 vo uint32_t BSRR;
	 vo uint32_t BRR;
	 vo uint32_t LCKR;


	 /*The structure definition tells the compiler the layout of registers (i.e., the offsets of each register).
The base address (0x40010800) tells the compiler where in memory the GPIO registers start.
When you access a member like CRL, the compiler calculates the address by adding the offset of CRL (from the structure) to the base address of GPIOA.
The compiler then generates code to write to that specific memory address.*/

}GPIOx_RegDef_t;

typedef struct
{
	 vo uint32_t CR;
	 vo uint32_t CFGR;
	    uint32_t CIR;
	 vo uint32_t APB2RSTR;
	 vo uint32_t APB1RSTR;
	 vo uint32_t AHBENR;
	 vo uint32_t APB2ENR;
	 vo uint32_t APB1ENR;
	    uint32_t BDCR;
	    uint32_t CSR;

}RCC_RegDef_t;




typedef struct
{
	    uint32_t EVCR;
	    uint32_t MAPR;
	 vo uint32_t EXTICR[3];
}AFIO_RegDef_t;


typedef struct
{
	 vo uint32_t IMR;
	 vo uint32_t EMR;
	 vo uint32_t RTSR;
	 vo uint32_t FTSR;
	    uint32_t SWIER;
	 vo uint32_t PR;
}EXTI_RegDef_t;



typedef struct{
	vo uint32_t CR1;
	vo uint32_t CR2;
	vo uint32_t SR;
	vo uint32_t DR;
	vo uint32_t CRCPR;
	vo uint32_t RXCRCR;
	vo uint32_t TXCRCR;


}SPIx_RegDef_t;

typedef struct{

	vo uint32_t SR;
	vo uint32_t DR;
	vo uint32_t BRR;
	vo uint32_t CR1;
	vo uint32_t CR2;
	vo uint32_t CR3;
	vo uint32_t GTPR;


}USARTx_RegDef_t;



/*These macros are essential to pass the base addresses of the relevant peripheral to the associated structure so that the structure can know to add the offsets of its members to the correct base addresses*/

/*Note: Any where in our source file we can use these macros to access entire peripherals registers like to access the ODR of port B just wriite GPIOB->ODR and we can play with the content*/

#define GPIOA 			((GPIOx_RegDef_t* )GPIOA_BASE_ADDR )
#define GPIOB 			((GPIOx_RegDef_t* )GPIOB_BASE_ADDR )
#define GPIOC			((GPIOx_RegDef_t* )GPIOC_BASE_ADDR )
#define RCC   			((RCC_RegDef_t*   )RCC_BASE_ADDR   )
#define AFIO  			((AFIO_RegDef_t*  )AFIO_BASE_ADDR  )
#define EXTI  			((EXTI_RegDef_t*  )EXTI_BASE_ADDR  )
#define SPI1			((SPIx_RegDef_t*  )SPI1_BASE_ADDR  )
#define SPI2			((SPIx_RegDef_t*  )SPI2_BASE_ADDR  )
#define SPI3			((SPIx_RegDef_t*  )SPI3_BASE_ADDR  )
#define USART1			((USARTx_RegDef_t*)USART1_BASE_ADDR)
#define USART2			((USARTx_RegDef_t*)USART2_BASE_ADDR)
#define USART3			((USARTx_RegDef_t*)USART3_BASE_ADDR)
#define UART4			((USARTx_RegDef_t*)UART4_BASE_ADDR)
#define UART5			((USARTx_RegDef_t*)UART5_BASE_ADDR)

/*Now writing some clock enable and disable macros for various peripherals*/

/*CLK EN MACROS*/
#define GPIOA_PCLK_EN()  (RCC->APB2ENR |= (1<<2))
#define GPIOB_PCLK_EN()  (RCC->APB2ENR |= (1<<3))
#define GPIOC_PCLK_EN()  (RCC->APB2ENR |= (1<<4))


#define ADC1_PCLK_EN()   (RCC->APB2ENR |= (1<<9))
#define ADC2_PCLK_EN()   (RCC->APB2ENR |= (1<<10))


#define TIM1_PCLK_EN()   (RCC->APB2ENR |= (1<<11)
#define TIM2_PCLK_EN()   (RCC->APB1ENR |= (1<<0))
#define TIM3_PCLK_EN()   (RCC->APB1ENR |= (1<<1))
#define TIM4_PCLK_EN()   (RCC->APB1ENR |= (1<<2))


#define SPI1_PCLK_EN()   (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()   (RCC->APB1ENR |= (1<<15))


#define USART1_PCLK_EN() (RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1<<20))




#define AFIO_CLK_EN()    (RCC->APB2ENR |= (1<<0))


/*CLK DI(DISABLE) MACROS*/

#define GPIOA_PCLK_DI()  (RCC->APB2ENR &= ~(1<<2))
#define GPIOB_PCLK_DI()  (RCC->APB2ENR &= ~(1<<3))
#define GPIOC_PCLK_DI()  (RCC->APB2ENR &= ~(1<<4))


#define ADC1_PCLK_DI()   (RCC->APB2ENR &= ~(1<<9))
#define ADC2_PCLK_DI()   (RCC->APB2ENR &= ~(1<<10))


#define TIM1_PCLK_DI()   (RCC->APB2ENR &= ~(1<<11))
#define TIM2_PCLK_DI()   (RCC->APB1ENR &= ~(1<<0))
#define TIM3_PCLK_DI()   (RCC->APB1ENR &= ~(1<<1))
#define TIM4_PCLK_DI()   (RCC->APB1ENR &= ~(1<<2))


#define SPI1_PCLK_DI()   (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()   (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()   (RCC->APB1ENR &= ~(1<<15))


#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1<<20))




#define AFIO_CLK_DI()    (RCC->APB2ENR &= ~(1<<0)) /*Thats actually the SYSCFG En bit in nucleo and discovery boards
it is responsible for pin remapping,memory remapping and external interrupt lines works*/



/*Note: "\" is used in c lanuage to tell the compiler that a single statement is written on multiple lines otherwise the compiler will treat each line separately leading to errors.*/
//There must not be character or comments after the backslash.
#define GPIO_BASE_ADDR_TO_CODE(x)  ((x==GPIOA)?0:\
								    (x==GPIOB)?1:\
								    (x==GPIOC)?2:0)



/*GPIOx complete peripheral Reset Macros*/

/*Note: Sometimes we need to execute multiple unique lines in a single shot without making a function so thats where while(0) statement comes it executes entirely only once.
 * Here we first reset by writting 1 and then immediately removed the 1 and placed 0 to avoid forever reset condition  */
#define GPIOA_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2)); }while(0)
#define GPIOB_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3)); }while(0)
#define GPIOC_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)

#define SPI1_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)




#define USART1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14)); }while(0)
#define USART2_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET() do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET() do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET() do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)




		// some macros for bit pos names
#define SPIx_CR1_CPHA   	0
#define SPIx_CR1_CPOL   	1
#define SPIx_CR1_MSTR   	2
#define SPIx_CR1_BR     	3
#define SPIx_CR1_SPE   		6
#define SPIx_CR1_LSBFIRST   7
#define SPIx_CR1_SSI   		8
#define SPIx_CR1_SSM   		9
#define SPIx_CR1_RXONLY    10
#define SPIx_CR1_DFF       11
#define SPIx_CR1_CRCNEXT   12
#define SPIx_CR1_CRCEN     13
#define SPIx_CR1_BIDIOE    14
#define SPIx_CR1_BIDIMODE  15



#define SPIx_CR2_RXDMAEN   0
#define SPIx_CR2_TXDMAEN   1
#define SPIx_CR2_SSOE 	   2
#define SPIx_CR2_ERRIE     5
#define SPIx_CR2_RXNEIE    6
#define SPIx_CR2_TXEIE     7

#define SPIx_SR_RXNE      0
#define SPIx_SR_TXE       1
#define SPIx_SR_CHSIDE    2
#define SPIx_SR_UDR    	  3
#define SPIx_SR_CRCERR    4
#define SPIx_SR_MODF      5
#define SPIx_SR_OVR       6
#define SPIx_SR_BSY       7


// macros for bit position names for USART peripheral reg
#define USART_PARITY_SEL   				9
#define USART_PCE						10
#define USART_EN    					13
#define USART_RX_EN						2
#define USART_TX_EN						3
#define USART_TXE						7
#define USART_RXNE						5
#define USART_TC 						6

#define USART_WL						12
#define USART_RTSE						8
#define USART_CTSE						9

#define USART_STOP						12




//Some generic MACROS
#define En  			1
#define Di  			0

#define Set 			En
#define Reset 			Di

#define GPIOx_Pin_Set 	Set
#define GPIOx_Pin_Reset Reset

#define USART_OVER8  	8

#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx_SPI_driver.h"
#include "stm32f103xx_USART_driver.h"
#include "stm32f103xx_RCC_driver.h"

/*************************______The End of Document(EOD)______****************/
#endif /* STM32F103XX_H_ */
