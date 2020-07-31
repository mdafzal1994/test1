 
 //#ifndef  INC_STM32F407xx_H_
 //#define   INC_STM32F407xx_H_
 
 #ifndef    _HAL_GPIO3
 #define   _HAL_GPIO3
 
 
 #include<stdint.h>   //  uint32_t  DEFINE UNDER THIS HEADER 
 #define _vo  volatile
	 
 //#include "startup_stm32f407xx.s"
 
 /*  ======= PROCESSOR SPECIFICATION DATAIL =============
 
     ARM CORTEX Mx  PROCESSOR NVIC ISERx REGISTER ADDD    */
 
  #define NVIC_ISER0       ((_vo uint32_t *)0xE000E100)  // BASE ADD  OF NVIC_ISER0
  #define NVIC_ISER1       ((_vo uint32_t *)0xE000E104)
  #define NVIC_ISER2       ((_vo uint32_t *)0xE000E108)
  #define NVIC_ISER3       ((_vo uint32_t *)0xE000E10C)
 
 /* ARM CORTEX Mx  PROCESSOR NVIC ICERx REGISTER ADDD    */
	#define NVIC_ICER0       ((_vo uint32_t *)0xE000E100)  // BASE ADD  OF NVIC_ICER0
  #define NVIC_ICER1       ((_vo uint32_t *)0xE000E104)
  #define NVIC_ICER2       ((_vo uint32_t *)0xE000E108)
  #define NVIC_ICER3       ((_vo uint32_t *)0xE000E10C)
 
 
 /*   arm cortex Mx processsor priority    register */
 
 #define NIVC_PR_BASE_ADDR   ((_vo uint32_t*)0XE000E400)
 
 #define NO_PR_BITS_IMPLEMENTED     4   // THIS FOR ST  ARM CORTEX Mx   BUT FOR TI IT IS 3
 
 
 
 /*  RETURN PORT CODE FOR GIVEN GPIO BASE ADD  //
   THIS CODE RETUR CODE (BTW  0 TO 4 )FOR A GIVEN GPIO BASE ADDD   */
 
 #define  GPIO_BASEADDR_TO_CODE(x)     ((x==GPIOA)?0:\
                                         (x==GPIOB)?1:\
                                         (x==GPIOC)?2:\
                                         (x==GPIOD)?3:\
                                         (x==GPIOE)?4:0)
 
 /******  STM32 specific Interrupt Numbers ******************************/
 
	#define IRQ_NO_EXTI0                   6      /*!< EXTI Line0 Interrupt                                              */
  #define IRQ_NO_EXTI1                  7      /*!< EXTI Line1 Interrupt                                              */
  #define IRQ_NO_EXTI2                  8      /*!< EXTI Line2 Interrupt                                              */
  #define IRQ_NO_EXTI3                  9      /*!< EXTI Line3 Interrupt                                              */
  #define IRQ_NO_EXTI4                  10     /*!< EXTI Line4 Interrupt  */
  #define IRQ_NO_EXTI9_5                23     /*!< External Line[9:5] Interrupts */ 
  #define IRQ_NO_EXTI15_10              40     /*!< External Line[15:10] Interrupts */
 
 #define NVIC_IRQ_PRI0    0
 #define NVIC_IRQ_PRI1    1
 #define NVIC_IRQ_PRI2    2
 #define NVIC_IRQ_PRI3    3
 #define NVIC_IRQ_PRI4    4
 #define NVIC_IRQ_PRI5    5
 //#define NVIC_IRQ_PRI6    6	
	
	#define NVIC_IRQ_PRI6    6
	#define NVIC_IRQ_PRI7    7
	#define NVIC_IRQ_PRI8    8
	#define NVIC_IRQ_PRI9    9
	#define NVIC_IRQ_PRI10    10
	#define NVIC_IRQ_PRI11    11
	#define NVIC_IRQ_PRI12   12
	#define NVIC_IRQ_PRI13   13
	#define NVIC_IRQ_PRI14    14
	#define NVIC_IRQ_PRI15    15
		
			
				
				
 
 /* geral macro  */
 #define  ENABLE          1
 #define  DISABLE         0
 #define  SET             ENABLE 
 #define  RESET           DISABLE 
 #define  GPIO_PIN_SET    SET
 #define  GPIO_PIN_RESET  RESET
 
 
/* BASE ADD OF FLASH AND SRAM */

 #define FLASH_BASEADDR        0x08000000U
 #define SRAM1_BASEADDR        0x20000000U
 #define SRAM2_BASEADDR        0x2001C000U
 #define ROM_BASEADDR          0x1FFF0000U
 #define SRAM                  SRAM1_BASEADDR
 
 /*  BASE ADD OF BUS */
 
 #define  AHB1PERIPH_BASE     			   0x40020000U
 #define  AHB2PERIPH_BASE    				   0x50000000U
 #define  PERIPH_BASE    	             0x40000000U
 #define  APB1PERIPH_BASE    				   PERIPH_BASE
 #define  APB2PERIPH_BASE     				 0x40010000U

 /* BASE  ADD OF PERIPHERALS WHICH HANGING AHB1 */
 
   #define GPIOA_BASEADDR      0x40020000U
	 #define GPIOB_BASEADDR      0x40020400U
   #define GPIOC_BASEADDR      0x40020800U
   #define GPIOD_BASEADDR      0x40020C00U
   #define GPIOE_BASEADDR      0x40021000U
	 
	 
 /* BASE  ADD OF PERIPHERALS WHICH HANGING APB1 */
   
	 #define I2C1_BASEADDR          0x40005400U
	 #define I2C2_BASEADDR          0x40005800U
	 #define I2C3_BASEADDR         (APB1PERIPH_BASE +0X5C00)
		
		#define SPI2_BASEADDR         (APB1PERIPH_BASE +0X3800)
		#define SPI3_BASEADDR         (APB1PERIPH_BASE +0X3C00)
		
		#define USART2_BASEADDR         (APB1PERIPH_BASE +0X4400)
		#define USART3_BASEADDR         (APB1PERIPH_BASE +0X4800)
	 
	 #define UART4_BASEADDR         (APB1PERIPH_BASE +0X4C00)
	 #define UART5_BASEADDR         (APB1PERIPH_BASE +0X5000)
	 
	 #define RCC_BASEADDR        (AHB1PERIPH_BASE +0X3800)  
	 
	 /* BASE  ADD OF PERIPHERALS WHICH HANGING APB2 BUS */
	 
	   #define EXTI_BASEADDR         (APB2PERIPH_BASE +0X3C00)
		 #define SYSCFG_BASEADDR         (APB2PERIPH_BASE +0X3800)
	   #define SPI1_BASEADDR         (APB2PERIPH_BASE +0X3000)
		 #define USART1_BASEADDR         (APB2PERIPH_BASE +0X1000)
     #define USART6_BASEADDR         (APB2PERIPH_BASE +0X1400)

/*   ========== PERIPHERL RESIGSTER DEFINATION  STRUCTURE ==========*/
    /* peripharal register of GPIO  */
		
		typedef struct 
		{
		  _vo	 uint32_t MODER;      /* GPIO port mode register  ADDRESS OFFSET : 0X00 */
			_vo  uint32_t OTYPER;    /* ADDRESS OFFSET : 0X04 */
			_vo  uint32_t OSPEEDR;
			_vo  uint32_t PUPDR;
			_vo  uint32_t IDR;
			_vo  uint32_t ODR;
			_vo  uint32_t BSRR;
			_vo  uint32_t LCKR;
			_vo  uint32_t AFR[2];
			  
		}GPIO_RegDef_t;   // HOW TO USE THIS STRUCTURE 
		                  /* GPIO_RegDef_t  *pGPIOA=(GPIO_Reg_Def_t*)GPIOA_BASEADDR
	                           */	
		
		/** 
  * @brief System configuration controller
  */

typedef struct
{
  _vo uint32_t     MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  _vo uint32_t     PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  _vo uint32_t     EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t         RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  _vo uint32_t     CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_RegDef_t;

	





		/** 
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  _vo uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  _vo uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  _vo uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  _vo uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  _vo uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  _vo uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
}EXTI_RegDef_t;
		

		/*   ========== PERIPHERL RESIGSTER DEFINATION  STRUCTURE ==========*/
    /* peripharal register oF RCC   */
		
		typedef struct 
		{
		  _vo	 uint32_t CR;      /* RCC clock control register ADDRESS OFFSET : 0X00 */
			_vo  uint32_t PLLCFGR;    /* ADDRESS OFFSET : 0X04 */
			_vo  uint32_t CFGR;
			_vo  uint32_t CIR;
			_vo  uint32_t AHB1RSTR;
			_vo  uint32_t AHB2RSTR;
			_vo  uint32_t AHB3RSTR;
			uint32_t    RESERVED0;
			_vo  uint32_t ABP1RSTR;
					_vo  uint32_t ABP2RSTR;
			uint32_t    RESERVED1[2];
				_vo  uint32_t AHB1ENR;
				_vo  uint32_t AHB2ENR;
			_vo  uint32_t AHB3ENR;
			uint32_t    RESERVED2;
			_vo  uint32_t APB1ENR;
			_vo  uint32_t APB2ENR;
			 
			uint32_t    RESERVED3[2];
			_vo  uint32_t AHB1LPENR;
			_vo  uint32_t AHB2LPENR;
			_vo  uint32_t AHB3LPENR;
			uint32_t    RESERVED4;
			
			_vo  uint32_t  APB1LPENR;
			_vo  uint32_t  APB2LPENR;
			uint32_t    RESERVED5[2];
			
			_vo  uint32_t  BDCR;
			_vo  uint32_t  CSR;
			uint32_t    RESERVED6[2];
			_vo  uint32_t  SSCGR;
			_vo  uint32_t  PLLI2SCFGR;
			_vo  uint32_t  PLLSAICFGR;
			_vo  uint32_t  DCKCFGR;
			_vo  uint32_t  CKGATENR;
			_vo  uint32_t  DCKCFGR2;     
			
			}RCC_RegDef_t;	
		
		
		
	/*   PERIPHERIAL DEFINETION  (PERIPHERIAL BASE ADDRESSS TEPECAST TO XXX REGDEF_T)    */
 
		#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
		#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
		#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
		#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
		#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
		
		#define RCC	   ((RCC_RegDef_t*)RCC_BASEADDR)
		
		#define EXTI	   ((EXTI_RegDef_t*)EXTI_BASEADDR)
			               
			#define SYSCFG	   (( SYSCFG_RegDef_t*)SYSCFG_BASEADDR )
		
	/* CLOCK ENABLE MACRO FOR GPIOx PERIPHERALS  */
			
			#define  GPIOA_PCLK_EN()    (RCC->AHB1ENR |=(1<<0))
		  #define  GPIOB_PCLK_EN()    (RCC->AHB1ENR |=(1<<1))
			#define  GPIOC_PCLK_EN()    (RCC->AHB1ENR |=(1<<2))
			#define  GPIOD_PCLK_EN()    (RCC->AHB1ENR |=(1<<3))
			#define  GPIOE_PCLK_EN()    (RCC->AHB1ENR |=(1<<4))
			
		/* CLOCK ENABLE MACRO FOR I2CXx PERIPHERALS  */	 
		
		  #define  I2C1_PCLK_EN()    (RCC->APB1ENR |=(1<<21))



    /* CLOCK ENABLE MACRO FOR SPIXx PERIPHERALS  */	 
		
		  #define  SPI1_PCLK_EN()    (RCC->APB2ENR |=(1<<12))
			
			
			/* CLOCK ENABLE MACRO FOR SPIXx PERIPHERALS  */	 
		
		  #define  SYSCFG_PCLK_EN()    (RCC->APB2ENR |=(1<<14))


/* CLOCK DIABLE MACRO FOR  PERIPHERALS=====================  */	 
		
		  
/* clock gpio reset clock */
#define GPIOA_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &=~(uint32_t)(1<<0));}while(0)
#define GPIOB_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &=~(uint32_t)(1<<1));}while(0)
#define GPIOC_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &=~(uint32_t)(1<<2));}while(0)
#define GPIOD_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &=~(uint32_t)(1<<3));}while(0)
#define GPIOE_REG_RESET()    do{(RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &=~(uint32_t)(1<<4));}while(0)

/* CLOCK DISABLE MACRO FOR GPIOx PERIPHERALS  */
			
			#define  GPIOA_PCLK_DI()    (RCC->AHB1ENR &=~(uint32_t)(1<<0))
		  #define  GPIOB_PCLK_DI()    (RCC->AHB1ENR &=~(uint32_t)(1<<1))
			#define  GPIOC_PCLK_DI()    (RCC->AHB1ENR &=~(uint32_t)(1<<2))
			#define  GPIOD_PCLK_DI()    (RCC->AHB1ENR &=~(uint32_t)(1<<3))
			#define  GPIOE_PCLK_DI()    (RCC->AHB1ENR &=~(uint32_t)(1<<4))
			
		/* CLOCK DISABLE MACRO FOR I2CXx PERIPHERALS  */	 
		
		  #define  I2C1_PCLK_DI()    (RCC->APB1ENR &=~(1<<21))



    /* CLOCK DISABLE MACRO FOR SPIXx PERIPHERALS  */	 
		
		  #define  SPI1_PCLK_DI()    (RCC->APB2ENR &=~(1<<12))
#include "stm32f407xx_gpio_driver.h"
    
#endif